#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import math
import sys

import rospy
from geometry_msgs.msg import (PoseStamped, PoseWithCovarianceStamped, Twist,
                               Vector3Stamped)
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, String


def isfinite(value):
    return not math.isnan(value) and not math.isinf(value)


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q):
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if norm < 1e-9 or not isfinite(norm):
        return None
    x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    return math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))


def rotate_offset(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return c*x - s*y, s*x + c*y


def target_and_velocity(leader_x, leader_y, leader_yaw, offset_x, offset_y,
                        leader_vx, leader_vy, leader_omega):
    dx, dy = rotate_offset(offset_x, offset_y, leader_yaw)
    # p_d = p_0 + R(theta_0)d; dp_d = dp_0 + omega_0 J R(theta_0)d
    return (leader_x + dx, leader_y + dy,
            leader_vx - leader_omega*dy,
            leader_vy + leader_omega*dx)


class YawAlignment(object):
    def __init__(self, auto_align, initial_uwb_yaw, configured_offset):
        self.auto_align = auto_align
        self.initial_uwb_yaw = initial_uwb_yaw
        self.offset = None if auto_align else configured_offset
        self.yaw = None
        self.stamp = None

    def update(self, raw_yaw, stamp):
        if self.offset is None:
            self.offset = wrap_angle(self.initial_uwb_yaw - raw_yaw)
        self.yaw = wrap_angle(raw_yaw + self.offset)
        self.stamp = stamp


class DisplacementFollower(object):
    def __init__(self):
        self.robot_name = rospy.get_param("~robot_name", "ugv1")
        self.offset_x = float(rospy.get_param("~offset_x", -0.8))
        self.offset_y = float(rospy.get_param("~offset_y", 0.55))
        self.offset_yaw = float(rospy.get_param("~offset_yaw", 0.0))

        self.rate = max(1.0, float(rospy.get_param("~control_rate", 20.0)))
        self.k_position = float(rospy.get_param("~k_position", 0.8))
        self.k_heading = float(rospy.get_param("~k_heading", 1.8))
        self.k_heading_sync = float(rospy.get_param("~k_heading_sync", 1.2))
        self.max_linear = abs(float(rospy.get_param("~max_linear", 0.15)))
        self.max_angular = abs(float(rospy.get_param("~max_angular", 0.8)))
        self.rotate_threshold = abs(float(
            rospy.get_param("~rotate_in_place_threshold", 0.7)))
        self.position_tolerance = abs(float(
            rospy.get_param("~position_tolerance", 0.10)))
        self.hold_exit_tolerance = float(rospy.get_param("~hold_exit_tolerance", 0.18))
        self.hold_enter_duration = float(rospy.get_param("~hold_enter_duration", 0.4))
        self.heading_blend_distance = float(rospy.get_param("~heading_blend_distance", 0.50))
        self.heading_blend_max = float(rospy.get_param("~heading_blend_max", 0.35))
        self.heading_tolerance = float(rospy.get_param("~heading_tolerance", math.radians(5.0)))
        self.hold_max_angular = float(rospy.get_param("~hold_max_angular", 0.35))
        values = (self.position_tolerance, self.hold_exit_tolerance,
                  self.hold_enter_duration, self.heading_blend_distance,
                  self.heading_blend_max, self.heading_tolerance, self.hold_max_angular)
        if (not all(isfinite(value) for value in values) or
                not 0.0 < self.position_tolerance < self.hold_exit_tolerance <= self.heading_blend_distance or
                not 0.0 <= self.heading_blend_max < 0.5 or
                self.hold_enter_duration < 0.0 or
                not 0.0 <= self.heading_tolerance < math.pi/2.0 or
                self.hold_max_angular <= 0.0):
            raise ValueError("Require 0 < position_tolerance < hold_exit_tolerance <= "
                             "heading_blend_distance, 0 <= heading_blend_max < 0.5, "
                             "hold_enter_duration >= 0, 0 <= heading_tolerance < pi/2, "
                             "hold_max_angular > 0; all must be finite")
        self.velocity_deadband = abs(float(
            rospy.get_param("~velocity_deadband", 0.02)))
        self.data_timeout = max(0.05, float(
            rospy.get_param("~data_timeout", 0.5)))

        self.leader_linear_deadband = abs(float(
            rospy.get_param("~leader_linear_deadband", 0.03)))
        self.leader_angular_deadband = abs(float(
            rospy.get_param("~leader_angular_deadband", 0.03)))

        auto_align = bool(rospy.get_param("~auto_align_yaw", True))
        initial_yaw = float(rospy.get_param("~initial_uwb_yaw", 0.0))
        self.leader_heading = YawAlignment(
            auto_align, initial_yaw,
            float(rospy.get_param("~leader_yaw_offset", 0.0)))
        self.self_heading = YawAlignment(
            auto_align, initial_yaw,
            float(rospy.get_param("~self_yaw_offset", 0.0)))

        self.enabled = bool(rospy.get_param("~enabled", False))
        self.leader_pose = None
        self.self_pose = None
        self.leader_pose_stamp = None
        self.self_pose_stamp = None
        self.leader_valid = False
        self.self_valid = False
        self.leader_twist = (0.0, 0.0, 0.0)
        self.leader_velocity_stamp = None
        self.holding = False
        self.hold_since = None

        # Callbacks may run as soon as a subscriber is constructed.
        self.cmd_pub = rospy.Publisher(
            rospy.get_param("~cmd_vel_topic", "/ugv1/cmd_vel"),
            Twist, queue_size=5)
        self.target_pub = rospy.Publisher("~target_pose", PoseStamped, queue_size=5)
        self.error_pub = rospy.Publisher("~tracking_error", Vector3Stamped, queue_size=5)
        self.leader_velocity_pub = rospy.Publisher(
            "~leader_velocity", Vector3Stamped, queue_size=5)
        self.desired_velocity_pub = rospy.Publisher(
            "~desired_velocity", Vector3Stamped, queue_size=5)
        self.heading_error_pub = rospy.Publisher(
            "~heading_error", Float64, queue_size=5)
        self.heading_sync_error_pub = rospy.Publisher(
            "~heading_sync_error", Float64, queue_size=5)
        self.heading_blend_pub = rospy.Publisher(
            "~heading_blend", Float64, queue_size=5)
        self.state_pub = rospy.Publisher("~state", String, queue_size=1, latch=True)

        rospy.on_shutdown(self.stop)
        self.publish_state("READY" if self.enabled else "DISABLED")
        rospy.loginfo("%s displacement follower started; offset=(%.2f, %.2f)",
                      self.robot_name, self.offset_x, self.offset_y)
        if auto_align:
            rospy.logwarn("Keep ugv0 and %s pointing at uwb yaw %.3f rad until yaw alignment is captured",
                          self.robot_name, initial_yaw)

        rospy.Subscriber(rospy.get_param("~leader_pose_topic", "/ugv0/uwb/pose"),
                         PoseStamped, self.leader_pose_cb, queue_size=20)
        rospy.Subscriber(rospy.get_param("~self_pose_topic", "/ugv1/uwb/pose"),
                         PoseStamped, self.self_pose_cb, queue_size=20)
        rospy.Subscriber(rospy.get_param("~leader_valid_topic", "/ugv0/uwb/valid"),
                         Bool, self.leader_valid_cb, queue_size=10)
        rospy.Subscriber(rospy.get_param("~self_valid_topic", "/ugv1/uwb/valid"),
                         Bool, self.self_valid_cb, queue_size=10)
        rospy.Subscriber(rospy.get_param("~leader_odom_topic", "/ugv0/odom_combined"),
                         PoseWithCovarianceStamped, self.leader_odom_cb, queue_size=20)
        rospy.Subscriber(rospy.get_param("~leader_velocity_odom_topic", "/ugv0/odom"),
                         Odometry, self.leader_velocity_odom_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param("~self_odom_topic", "/ugv1/odom_combined"),
                         PoseWithCovarianceStamped, self.self_odom_cb, queue_size=20)
        rospy.Subscriber("/five_ugv_formation/enable", Bool,
                         self.enable_cb, queue_size=1)
        rospy.Timer(rospy.Duration(1.0/self.rate), self.control_cb)

    @staticmethod
    def finite_pose(msg):
        return isfinite(msg.pose.position.x) and isfinite(msg.pose.position.y)

    def leader_pose_cb(self, msg):
        if not self.finite_pose(msg):
            return
        self.leader_pose = (float(msg.pose.position.x), float(msg.pose.position.y))
        self.leader_pose_stamp = rospy.Time.now()

    def self_pose_cb(self, msg):
        if not self.finite_pose(msg):
            return
        self.self_pose = (float(msg.pose.position.x), float(msg.pose.position.y))
        self.self_pose_stamp = rospy.Time.now()

    def leader_valid_cb(self, msg):
        self.leader_valid = bool(msg.data)

    def self_valid_cb(self, msg):
        self.self_valid = bool(msg.data)

    def leader_odom_cb(self, msg):
        raw_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        if raw_yaw is None:
            return
        self.leader_heading.update(raw_yaw, rospy.Time.now())

    def leader_velocity_odom_cb(self, msg):
        twist = msg.twist.twist
        velocity = (twist.linear.x, twist.linear.y, twist.angular.z)
        if not all(isfinite(value) for value in velocity):
            self.leader_velocity_stamp = None
            return
        # Odometry.twist is expressed in child_frame_id (the UGV0 body frame).
        self.leader_twist = velocity
        self.leader_velocity_stamp = rospy.Time.now()

    def self_odom_cb(self, msg):
        raw_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        if raw_yaw is not None:
            self.self_heading.update(raw_yaw, rospy.Time.now())

    def enable_cb(self, msg):
        self.enabled = bool(msg.data)
        if not self.enabled:
            self.stop()
        self.publish_state("READY" if self.enabled else "DISABLED")
        rospy.logwarn("%s formation control %s", self.robot_name,
                      "enabled" if self.enabled else "disabled")

    def fresh(self, stamp, now):
        return stamp is not None and (now - stamp).to_sec() <= self.data_timeout

    def data_ready(self, now):
        if not self.leader_valid or not self.self_valid:
            return False, "UWB_INVALID"
        required = (self.leader_pose_stamp, self.self_pose_stamp,
                    self.leader_heading.stamp, self.self_heading.stamp,
                    self.leader_velocity_stamp)
        if not all(self.fresh(stamp, now) for stamp in required):
            return False, "STALE_OR_MISSING_INPUT"
        return True, "FOLLOWING"

    def control_cb(self, _event):
        if not self.enabled:
            return
        now = rospy.Time.now()
        ready, state = self.data_ready(now)
        if not ready:
            self.stop()
            self.publish_state(state)
            rospy.logwarn_throttle(1.0, "%s stopped: %s", self.robot_name, state)
            return

        body_vx, body_vy, leader_omega = self.leader_twist
        if math.hypot(body_vx, body_vy) < self.leader_linear_deadband:
            body_vx, body_vy = 0.0, 0.0
        leader_vx, leader_vy = rotate_offset(
            body_vx, body_vy, self.leader_heading.yaw)
        if abs(leader_omega) < self.leader_angular_deadband:
            leader_omega = 0.0

        target_x, target_y, feed_x, feed_y = target_and_velocity(
            self.leader_pose[0], self.leader_pose[1], self.leader_heading.yaw,
            self.offset_x, self.offset_y, leader_vx, leader_vy, leader_omega)
        error_x = target_x - self.self_pose[0]
        error_y = target_y - self.self_pose[1]
        error_norm = math.hypot(error_x, error_y)
        velocity_x = feed_x + self.k_position*error_x
        velocity_y = feed_y + self.k_position*error_y
        speed = math.hypot(velocity_x, velocity_y)

        leader_stopped = body_vx == 0.0 and body_vy == 0.0 and leader_omega == 0.0
        if self.holding and (not leader_stopped or error_norm > self.hold_exit_tolerance):
            self.holding = False
            self.hold_since = None
        if not self.holding:
            if leader_stopped and error_norm <= self.position_tolerance:
                if self.hold_since is None or now < self.hold_since:
                    self.hold_since = now
                self.holding = (now - self.hold_since).to_sec() >= self.hold_enter_duration
            else:
                self.hold_since = None

        desired_yaw = wrap_angle(self.leader_heading.yaw + self.offset_yaw)
        sync_error = wrap_angle(desired_yaw - self.self_heading.yaw)
        # Continuous dead zone: zero correction within heading_tolerance.
        sync_correction = self.k_heading_sync * math.copysign(
            max(0.0, abs(sync_error) - self.heading_tolerance), sync_error)
        proximity = clamp((self.heading_blend_distance - error_norm) /
                          (self.heading_blend_distance - self.position_tolerance), 0.0, 1.0)
        blend = self.heading_blend_max * proximity*proximity*(3.0 - 2.0*proximity)
        state = "APPROACH" if error_norm < self.heading_blend_distance else "FOLLOWING"
        cmd = Twist()
        heading_error = 0.0
        if self.holding:
            state = "HOLD"
            blend = 1.0
            heading_error = sync_error
            limit = min(self.max_angular, self.hold_max_angular)
            cmd.angular.z = clamp(sync_correction, -limit, limit)
        elif speed >= self.velocity_deadband:
            motion_yaw = math.atan2(velocity_y, velocity_x)
            # Interpolate along the shortest arc. A weight < 0.5 retains progress
            # toward the velocity vector even when the desired headings oppose.
            target_yaw = wrap_angle(motion_yaw + blend*wrap_angle(desired_yaw - motion_yaw))
            heading_error = wrap_angle(target_yaw - self.self_heading.yaw)
            gain = (1.0 - blend)*self.k_heading + blend*self.k_heading_sync
            cmd.angular.z = clamp(blend*leader_omega + gain*heading_error,
                                  -self.max_angular, self.max_angular)
            if abs(heading_error) <= self.rotate_threshold:
                projection = velocity_x*math.cos(self.self_heading.yaw) + \
                             velocity_y*math.sin(self.self_heading.yaw)
                cmd.linear.x = clamp(projection, 0.0, self.max_linear)
        else:
            # At very low requested speed, its direction is poorly defined.
            blend = 1.0
            heading_error = sync_error
            cmd.angular.z = clamp(leader_omega + sync_correction,
                                  -min(self.max_angular, self.hold_max_angular),
                                  min(self.max_angular, self.hold_max_angular))

        self.cmd_pub.publish(cmd)
        self.publish_debug(now, target_x, target_y, error_x, error_y,
                           error_norm, desired_yaw,
                           leader_vx, leader_vy, leader_omega,
                           velocity_x, velocity_y, speed, heading_error, sync_error, blend)
        self.publish_state(state)
        rospy.loginfo_throttle(
            1.0, "%s %s error=%.3f m yaw_error=%.1f deg cmd=(%.3f, %.3f)",
            self.robot_name, state, error_norm, math.degrees(sync_error),
            cmd.linear.x, cmd.angular.z)

    def publish_debug(self, stamp, target_x, target_y, error_x, error_y,
                      error_norm, target_yaw, leader_vx, leader_vy,
                      leader_omega, desired_vx, desired_vy, desired_speed,
                      heading_error, sync_error, blend):
        target = PoseStamped()
        target.header.stamp = stamp
        target.header.frame_id = "uwb_map"
        target.pose.position.x = target_x
        target.pose.position.y = target_y
        target.pose.orientation.z = math.sin(0.5*target_yaw)
        target.pose.orientation.w = math.cos(0.5*target_yaw)
        self.target_pub.publish(target)

        error = Vector3Stamped()
        error.header = target.header
        error.vector.x = error_x
        error.vector.y = error_y
        error.vector.z = error_norm
        self.error_pub.publish(error)

        leader_velocity = Vector3Stamped()
        leader_velocity.header = target.header
        leader_velocity.vector.x = leader_vx
        leader_velocity.vector.y = leader_vy
        leader_velocity.vector.z = leader_omega
        self.leader_velocity_pub.publish(leader_velocity)

        desired_velocity = Vector3Stamped()
        desired_velocity.header = target.header
        desired_velocity.vector.x = desired_vx
        desired_velocity.vector.y = desired_vy
        desired_velocity.vector.z = desired_speed
        self.desired_velocity_pub.publish(desired_velocity)

        self.heading_error_pub.publish(Float64(data=heading_error))
        self.heading_sync_error_pub.publish(Float64(data=sync_error))
        self.heading_blend_pub.publish(Float64(data=blend))

    def publish_state(self, text):
        msg = String()
        msg.data = text
        self.state_pub.publish(msg)

    def stop(self):
        self.holding = False
        self.hold_since = None
        if hasattr(self, "cmd_pub"):
            self.cmd_pub.publish(Twist())


def self_test():
    try:
        from unittest.mock import Mock, patch
    except ImportError:
        from mock import Mock, patch

    assert isfinite(0.0) and not isfinite(float("nan"))
    assert not isfinite(float("inf")) and not isfinite(float("-inf"))

    x, y = rotate_offset(1.0, 0.0, math.pi/2.0)
    assert abs(x) < 1e-9 and abs(y - 1.0) < 1e-9
    target = target_and_velocity(2.0, 3.0, math.pi/2.0,
                                 -1.0, 0.0, 0.2, 0.0, 0.5)
    assert abs(target[0] - 2.0) < 1e-9
    assert abs(target[1] - 2.0) < 1e-9
    assert abs(target[2] - 0.7) < 1e-9
    assert abs(target[3]) < 1e-9
    alignment = YawAlignment(True, 0.0, 0.0)
    alignment.update(1.2, rospy.Time(1.0))
    alignment.update(1.5, rospy.Time(2.0))
    assert abs(alignment.yaw - 0.3) < 1e-9

    # Exercise callbacks and the control loop without connecting to a ROS graph.
    with patch.multiple(rospy, get_param=lambda name, default: default,
                        Subscriber=Mock(), Publisher=Mock(side_effect=lambda *a, **k: Mock()),
                        Timer=Mock(), on_shutdown=Mock(), loginfo=Mock(),
                        logwarn=Mock(), loginfo_throttle=Mock(), logwarn_throttle=Mock()), \
            patch.object(rospy.Time, "now", return_value=rospy.Time(10.0)) as clock:
        # A subscriber can invoke callbacks before its constructor returns.
        for initial_enabled, enable_message in ((False, True), (True, False)):
            def subscribe(topic, message_type, callback, **kwargs):
                if topic == "/five_ugv_formation/enable":
                    callback(Bool(data=enable_message))

            with patch.object(rospy, "Subscriber", side_effect=subscribe), \
                    patch.object(rospy, "get_param", side_effect=lambda name, default:
                                 initial_enabled if name == "~enabled" else default):
                startup = DisplacementFollower()
            assert startup.enabled == enable_message
            assert startup.state_pub.publish.call_args[0][0].data == (
                "READY" if enable_message else "DISABLED")
            if not enable_message:
                cmd = startup.cmd_pub.publish.call_args[0][0]
                assert cmd.linear.x == cmd.angular.z == 0.0

        def subscribe_with_data(topic, message_type, callback, **kwargs):
            msg = message_type()
            if message_type is Bool:
                msg.data = True
            elif message_type is PoseWithCovarianceStamped:
                msg.pose.pose.orientation.w = 1.0
            callback(msg)

        with patch.object(rospy, "Subscriber", side_effect=subscribe_with_data), \
                patch.object(rospy, "Timer", side_effect=lambda duration, callback: callback(None)):
            startup = DisplacementFollower()
        assert startup.state_pub.publish.call_args[0][0].data == "FOLLOWING"

        controller = DisplacementFollower()
        controller.enabled = True
        controller.leader_heading = YawAlignment(False, 0.0, math.pi/2.0)
        pose = PoseStamped()
        heading = PoseWithCovarianceStamped()
        heading.pose.pose.orientation.w = 1.0
        controller.leader_valid_cb(Bool(data=True))
        controller.self_valid_cb(Bool(data=True))

        def refresh_poses():
            controller.leader_pose_cb(pose)
            controller.self_pose_cb(pose)
            controller.leader_odom_cb(heading)
            controller.self_odom_cb(heading)

        def assert_stopped():
            controller.control_cb(None)
            cmd = controller.cmd_pub.publish.call_args[0][0]
            assert cmd.linear.x == 0.0 and cmd.angular.z == 0.0
            assert controller.state_pub.publish.call_args[0][0].data == "STALE_OR_MISSING_INPUT"

        refresh_poses()
        assert_stopped()  # All pose inputs are present, but velocity is missing.
        odom = Odometry()
        for raw_yaw, body, expected in (
                (0.0, (0.2, 0.1, -0.3), (-0.1, 0.2, -0.3)),
                (0.0, (-0.2, 0.0, 0.0), (0.0, -0.2, 0.0)),
                (-math.pi/2.0, (0.2, 0.0, 0.2), (0.2, 0.0, 0.2)),
                (0.0, (0.01, 0.01, 0.01), (0.0, 0.0, 0.0))):
            heading.pose.pose.orientation.z = math.sin(raw_yaw/2.0)
            heading.pose.pose.orientation.w = math.cos(raw_yaw/2.0)
            refresh_poses()
            odom.twist.twist.linear.x, odom.twist.twist.linear.y, \
                odom.twist.twist.angular.z = body
            controller.leader_velocity_odom_cb(odom)
            controller.control_cb(None)
            actual = controller.leader_velocity_pub.publish.call_args[0][0]
            assert actual.header.frame_id == "uwb_map"
            assert all(abs(a-b) < 1e-9 for a, b in zip(
                (actual.vector.x, actual.vector.y, actual.vector.z), expected))

        odom = Odometry()
        controller.leader_velocity_odom_cb(odom)
        for step in range(4):
            clock.return_value = rospy.Time(10.0 + 0.05*step)
            pose.pose.position.x = 0.05*step
            pose.pose.position.y = -0.02*step
            refresh_poses()
            controller.control_cb(None)
            actual = controller.leader_velocity_pub.publish.call_args[0][0].vector
            assert (actual.x, actual.y, actual.z) == (0.0, 0.0, 0.0)

        clock.return_value = rospy.Time(11.0)
        refresh_poses()
        assert_stopped()  # Fresh poses cannot conceal a velocity input dropout.
        controller.leader_velocity_odom_cb(odom)
        assert controller.data_ready(clock.return_value) == (True, "FOLLOWING")
        for field, value in ((odom.twist.twist.linear, float("nan")),
                             (odom.twist.twist.angular, float("inf"))):
            key = "x" if field is odom.twist.twist.linear else "z"
            setattr(field, key, value)
            controller.leader_velocity_odom_cb(odom)
            assert_stopped()
            setattr(field, key, 0.0)
            controller.leader_velocity_odom_cb(odom)
            assert controller.data_ready(clock.return_value) == (True, "FOLLOWING")

        controller.leader_heading = YawAlignment(False, 0.0, 0.0)
        controller.self_heading = YawAlignment(False, 0.0, 0.0)
        controller.offset_x = controller.offset_y = 0.0

        def step(t, ex, ey=0.0, yaw=0.0, body=(0.0, 0.0, 0.0), leader_yaw=0.0):
            clock.return_value = rospy.Time.from_sec(t)
            controller.leader_pose = (0.0, 0.0)
            controller.self_pose = (-ex, -ey)
            controller.leader_pose_stamp = controller.self_pose_stamp = clock.return_value
            controller.leader_heading.update(leader_yaw, clock.return_value)
            controller.self_heading.update(yaw, clock.return_value)
            odom.twist.twist.linear.x, odom.twist.twist.linear.y, \
                odom.twist.twist.angular.z = body
            controller.leader_velocity_odom_cb(odom)
            controller.control_cb(None)
            return (controller.state_pub.publish.call_args[0][0].data,
                    controller.cmd_pub.publish.call_args[0][0])

        assert step(20.0, 0.0, 1.0)[0] == "FOLLOWING"
        assert controller.heading_blend_pub.publish.call_args[0][0].data == 0.0
        far_error = controller.heading_error_pub.publish.call_args[0][0].data
        assert step(20.1, 0.0, 0.3)[0] == "APPROACH"
        assert 0.0 < controller.heading_blend_pub.publish.call_args[0][0].data < 0.35
        assert 0.0 < controller.heading_error_pub.publish.call_args[0][0].data < far_error
        # Crossing +/-pi follows the short arc, not a full revolution.
        angle = math.radians(179.0)
        _, cmd = step(20.2, 0.3*math.cos(angle), 0.3*math.sin(angle), angle,
                      leader_yaw=-angle)
        assert 0.0 < cmd.angular.z < 0.1

        assert step(30.0, 0.09, yaw=0.4)[0] == "APPROACH"
        assert step(30.2, 0.09, yaw=0.4)[0] == "APPROACH"
        state, cmd = step(30.5, 0.09, yaw=0.4)
        assert state == "HOLD" and cmd.linear.x == 0.0
        assert -controller.hold_max_angular <= cmd.angular.z < 0.0
        for t, error in ((30.6, 0.14), (30.7, 0.18), (30.8, 0.06)):
            state, cmd = step(t, error, yaw=0.04)
            assert state == "HOLD" and cmd.linear.x == cmd.angular.z == 0.0
        assert step(31.0, 0.19)[0] == "APPROACH"
        assert step(31.1, 0.08)[0] == "APPROACH"
        assert step(31.3, 0.12)[0] == "APPROACH"  # Resets entry confirmation.
        assert step(31.5, 0.08)[0] == "APPROACH"
        assert step(31.8, 0.08)[0] == "APPROACH"
        assert step(32.0, 0.08)[0] == "HOLD"
        assert step(32.1, 0.08, body=(0.1, 0.0, 0.0))[0] == "APPROACH"
        assert step(32.2, 0.08)[0] == "APPROACH"
        assert step(32.7, 0.08)[0] == "HOLD"
        assert step(32.8, 0.08, body=(0.0, 0.0, 0.1))[0] == "APPROACH"
        assert abs(controller.cmd_pub.publish.call_args[0][0].angular.z - 0.035) < 1e-9
        assert step(32.9, 0.08)[0] == "APPROACH"
        assert step(33.4, 0.08)[0] == "HOLD"
        controller.self_valid_cb(Bool(data=False))
        controller.control_cb(None)
        assert not controller.holding and controller.hold_since is None
        assert controller.cmd_pub.publish.call_args[0][0].linear.x == 0.0
        controller.self_valid_cb(Bool(data=True))
        assert step(34.0, 0.08)[0] == "APPROACH"
        assert step(34.5, 0.08)[0] == "HOLD"
        controller.enable_cb(Bool(data=False))
        controller.enable_cb(Bool(data=True))
        assert step(35.0, 0.08)[0] == "APPROACH"

        # Closed-loop unicycle checks include lateral/behind targets and UWB jitter.
        for start in ((-1.0, -0.4, 0.0), (0.4, 0.2, 0.0), (0.0, -0.35, 0.0)):
            controller.stop()
            x, y, yaw = start
            for i in range(1600):
                noise_x, noise_y = 0.012*math.sin(0.7*i), 0.012*math.cos(0.5*i)
                state, cmd = step(100.0 + 0.05*i, -x + noise_x, -y + noise_y, yaw)
                if state == "HOLD":
                    assert cmd.linear.x == 0.0
                x += 0.05*cmd.linear.x*math.cos(yaw)
                y += 0.05*cmd.linear.x*math.sin(yaw)
                yaw = wrap_angle(yaw + 0.05*cmd.angular.z)
            assert state == "HOLD", (start, x, y, yaw, state)
            assert math.hypot(x, y) < controller.hold_exit_tolerance
            assert abs(yaw) < controller.heading_tolerance + 0.01
    print("displacement_follower self-test passed")


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        self_test()
    else:
        rospy.init_node("displacement_follower")
        DisplacementFollower()
        rospy.spin()
