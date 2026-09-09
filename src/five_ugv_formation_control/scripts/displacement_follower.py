#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import math
import sys
import threading
from collections import deque

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


def smoothstep(value):
    value = clamp(value, 0.0, 1.0)
    return value*value*(3.0 - 2.0*value)


def advance_pose(x, y, yaw, velocity, dt):
    vx, vy, omega = velocity
    angle = omega*dt
    if abs(angle) < 1e-8:
        dx, dy = vx*dt, vy*dt
    else:
        s, c = math.sin(angle)/omega, (1.0-math.cos(angle))/omega
        dx, dy = s*vx-c*vy, c*vx+s*vy
    dx, dy = rotate_offset(dx, dy, yaw)
    return x+dx, y+dy, wrap_angle(yaw+angle)


class RobotState(object):
    """Source-stamped state, shared by leader and follower input handling."""
    def __init__(self, offset):
        self.offset = offset
        self.pose = self.pose_stamp = self.velocity_stamp = None
        self.valid, self.valid_stamp = False, None
        self.headings = deque(maxlen=500)
        self.velocity = self.filtered_velocity = (0.0, 0.0, 0.0)
        self.acceleration = (0.0, 0.0, 0.0)
        self.stationary_since = None
        self.stopped = False

    def update_velocity(self, velocity, stamp, tau, timeout):
        dt = (stamp-self.velocity_stamp).to_sec() if self.velocity_stamp else timeout+1.0
        previous = self.filtered_velocity
        if dt > timeout:
            self.filtered_velocity = velocity
            self.acceleration = (0.0, 0.0, 0.0)
            self.stopped, self.stationary_since = False, None
        else:
            alpha = dt/(tau+dt)
            self.filtered_velocity = tuple(a+alpha*(b-a) for a, b in zip(previous, velocity))
            self.acceleration = tuple((b-a)/dt for a, b in zip(previous, self.filtered_velocity))
        self.velocity, self.velocity_stamp = velocity, stamp

    def stationary(self, now, linear, angular, duration):
        speed, omega = math.hypot(*self.velocity[:2]), abs(self.velocity[2])
        if speed > 1.5*linear or omega > 1.5*angular:
            self.stopped, self.stationary_since = False, None
        elif not self.stopped:
            if speed <= linear and omega <= angular:
                if self.stationary_since is None or now < self.stationary_since:
                    self.stationary_since = now
                self.stopped = (now-self.stationary_since).to_sec() >= duration
            else:
                self.stationary_since = None
        return self.stopped

    def yaw_at(self, stamp, horizon):
        if not self.headings:
            return None
        if self.headings[0][0] < stamp < self.headings[-1][0]:
            tb, b = self.headings[-1]
            for ta, a in reversed(self.headings):
                if ta <= stamp:
                    return wrap_angle(a + wrap_angle(b-a)*(stamp-ta).to_sec()/(tb-ta).to_sec())
                tb, b = ta, a
        t, yaw = self.headings[0] if stamp < self.headings[0][0] else self.headings[-1]
        dt = (stamp-t).to_sec()
        if abs(dt) > horizon+1e-9:
            return None
        return wrap_angle(yaw+self.velocity[2]*dt)

    def align(self, horizon):
        if self.offset is None and self.stopped and self.pose[2] is not None:
            raw = self.yaw_at(self.pose_stamp, horizon)
            if raw is not None:
                # Same map reference as the leader path tracker, not launch yaw.
                self.offset = wrap_angle(self.pose[2]-raw)

    def predict(self, now, horizon):
        raw = self.yaw_at(self.pose_stamp, horizon)
        if raw is None:
            return None
        # ponytail: constant-twist prediction is bounded; use odom integration
        # if measured acceleration makes this short-horizon approximation inadequate.
        dt = min(horizon, (now-self.pose_stamp).to_sec())
        x, y, _ = advance_pose(self.pose[0], self.pose[1], raw+self.offset, self.velocity, dt)
        stamp, raw = self.headings[-1]
        yaw = raw+self.offset+self.velocity[2]*min(horizon, (now-stamp).to_sec())
        return x, y, wrap_angle(yaw)


class DisplacementFollower(object):
    def __init__(self):
        self.robot_name = rospy.get_param("~robot_name", "ugv1")
        self.offset_x = float(rospy.get_param("~offset_x", -0.8))
        self.offset_y = float(rospy.get_param("~offset_y", 0.55))
        self.offset_yaw = float(rospy.get_param("~offset_yaw", 0.0))

        self.lock = threading.RLock()
        self.rate = float(rospy.get_param("~control_rate", 20.0))
        self.k_position = float(rospy.get_param("~k_position", 0.5))
        self.k_lateral = float(rospy.get_param("~k_lateral", 8.0))
        self.k_heading = float(rospy.get_param("~k_heading", 1.2))
        self.k_heading_sync = float(rospy.get_param("~k_heading_sync", 1.2))
        self.max_linear = float(rospy.get_param("~max_linear", 0.15))
        if not isfinite(self.max_linear) or not 0.0 <= self.max_linear <= 0.5:
            raise ValueError("max_linear must be finite and between 0 and 0.5 m/s")
        self.max_angular = float(rospy.get_param("~max_angular", 0.8))
        self.rotate_threshold = float(rospy.get_param("~rotate_in_place_threshold", 0.95))
        self.rotate_exit = float(rospy.get_param("~rotate_exit_threshold", 0.55))
        self.max_acceleration = float(rospy.get_param("~max_acceleration", 0.15))
        self.max_angular_acceleration = float(rospy.get_param("~max_angular_acceleration", 0.8))
        self.position_tolerance = float(rospy.get_param("~position_tolerance", 0.10))
        self.hold_exit_tolerance = float(rospy.get_param("~hold_exit_tolerance", 0.18))
        self.hold_enter_duration = float(rospy.get_param("~hold_enter_duration", 0.4))
        self.approach_distance = float(rospy.get_param("~approach_distance", 0.50))
        self.heading_tolerance = float(rospy.get_param("~heading_tolerance", math.radians(5.0)))
        self.hold_max_angular = float(rospy.get_param("~hold_max_angular", 0.35))
        self.velocity_deadband = float(rospy.get_param("~velocity_deadband", 0.02))
        self.data_timeout = float(rospy.get_param("~data_timeout", 0.6))
        self.prediction_limit = float(rospy.get_param("~prediction_limit", 0.2))
        self.velocity_filter_tau = float(rospy.get_param("~velocity_filter_tau", 0.15))
        self.stationary_duration = float(rospy.get_param("~stationary_duration", 0.4))
        self.leader_linear_deadband = float(rospy.get_param("~leader_linear_deadband", 0.01))
        self.leader_angular_deadband = float(rospy.get_param("~leader_angular_deadband", 0.01))
        positive = (self.rate, self.k_position, self.k_lateral, self.k_heading,
                    self.k_heading_sync, self.max_angular, self.max_acceleration,
                    self.max_angular_acceleration, self.hold_max_angular,
                    self.velocity_deadband, self.data_timeout, self.prediction_limit,
                    self.stationary_duration, self.leader_linear_deadband, self.leader_angular_deadband)
        nonnegative = (self.hold_enter_duration, self.heading_tolerance, self.velocity_filter_tau)
        if (not all(isfinite(v) and v > 0 for v in positive) or
                not all(isfinite(v) and v >= 0 for v in nonnegative) or
                not 0 < self.position_tolerance < self.hold_exit_tolerance <= self.approach_distance < float('inf') or
                not 0 < self.rotate_exit < self.rotate_threshold < math.pi/2 or
                self.heading_tolerance >= math.pi/2 or self.rate < 1 or
                not all(isfinite(v) for v in (self.offset_x, self.offset_y, self.offset_yaw))):
            raise ValueError("Invalid formation gains, limits, timeouts or tolerance ordering")

        auto_align = bool(rospy.get_param("~auto_align_yaw", True))
        leader_offset = float(rospy.get_param("~leader_yaw_offset", 0.0))
        self_offset = float(rospy.get_param("~self_yaw_offset", 0.0))
        if not isfinite(leader_offset) or not isfinite(self_offset):
            raise ValueError("Yaw offsets must be finite")
        self.leader = RobotState(None if auto_align else leader_offset)
        self.follower = RobotState(None if auto_align else self_offset)

        self.enabled = bool(rospy.get_param("~enabled", False))
        self.holding = False
        self.hold_since = None
        self.aligning = False
        self.turn_direction = 1.0
        self.last_control = None
        self.last_cmd = (0.0, 0.0)

        # Callbacks may run as soon as a subscriber is constructed.
        self.cmd_pub = rospy.Publisher(
            rospy.get_param("~cmd_vel_topic", "/ugv1/cmd_vel"),
            Twist, queue_size=1)
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
        self.reference_angular_pub = rospy.Publisher("~reference_angular", Float64, queue_size=5)
        self.state_pub = rospy.Publisher("~state", String, queue_size=1, latch=True)

        self.limit_pub = rospy.Publisher("~max_linear", Float64, queue_size=1, latch=True)
        self.limit_pub.publish(Float64(data=self.max_linear))
        rospy.Subscriber("~set_max_linear", Float64, self.limit_cb, queue_size=1)

        rospy.on_shutdown(self.stop)
        self.publish_state("READY" if self.enabled else "DISABLED")
        rospy.loginfo("%s displacement follower started; offset=(%.2f, %.2f)",
                      self.robot_name, self.offset_x, self.offset_y)
        if auto_align:
            rospy.logwarn("Keep leader and %s stationary until UWB/odom yaw alignment completes",
                          self.robot_name)

        for role, robot, number in (("leader", self.leader, 0), ("self", self.follower, 1)):
            for name, suffix, kind, callback in (
                    ("pose", "uwb/pose", PoseStamped, self.pose_cb),
                    ("valid", "uwb/valid", Bool, self.valid_cb),
                    ("odom", "odom_combined", PoseWithCovarianceStamped, self.odom_cb),
                    ("velocity_odom", "odom", Odometry, self.velocity_cb)):
                rospy.Subscriber(rospy.get_param("~%s_%s_topic" % (role, name),
                                                "/ugv%d/%s" % (number, suffix)),
                                 kind, callback, callback_args=robot, queue_size=1)
        rospy.Subscriber("/five_ugv_formation/enable", Bool,
                         self.enable_cb, queue_size=1)
        rospy.Timer(rospy.Duration(1.0/self.rate), self.control_cb)

    def accept_stamp(self, stamp, previous):
        return (stamp.to_sec() > 0 and self.fresh(stamp, rospy.Time.now()) and
                (previous is None or stamp > previous))

    def pose_cb(self, msg, robot):
        with self.lock:
            if not self.accept_stamp(msg.header.stamp, robot.pose_stamp):
                return
            x, y = msg.pose.position.x, msg.pose.position.y
            if not isfinite(x) or not isfinite(y):
                robot.pose_stamp = None
                return
            robot.pose = (x, y, quaternion_to_yaw(msg.pose.orientation))
            robot.pose_stamp = msg.header.stamp

    def valid_cb(self, msg, robot):
        with self.lock:
            robot.valid, robot.valid_stamp = bool(msg.data), rospy.Time.now()
            if not robot.valid:
                robot.stopped, robot.stationary_since = False, None
                self.stop()

    def odom_cb(self, msg, robot):
        with self.lock:
            previous = robot.headings[-1][0] if robot.headings else None
            if not self.accept_stamp(msg.header.stamp, previous):
                return
            yaw = quaternion_to_yaw(msg.pose.pose.orientation)
            if yaw is None:
                robot.headings.clear()
                return
            robot.headings.append((msg.header.stamp, yaw))

    def velocity_cb(self, msg, robot):
        with self.lock:
            if not self.accept_stamp(msg.header.stamp, robot.velocity_stamp):
                return
            twist = msg.twist.twist
            velocity = (twist.linear.x, twist.linear.y, twist.angular.z)
            if not all(isfinite(value) for value in velocity):
                robot.velocity_stamp = None
                return
            robot.update_velocity(velocity, msg.header.stamp, self.velocity_filter_tau, self.data_timeout)

    def limit_cb(self, msg):
        value = float(msg.data)
        if not isfinite(value) or not 0.0 <= value <= 0.5:
            rospy.logwarn("Rejected invalid linear speed limit: %r", value)
            return
        with self.lock:
            changed = value != self.max_linear
            self.max_linear = value
            self.limit_pub.publish(Float64(data=self.max_linear))
        if changed:
            rospy.set_param("~max_linear", value)

    def enable_cb(self, msg):
        with self.lock:
            self.enabled = bool(msg.data)
            if not self.enabled:
                self.stop()
            self.publish_state("READY" if self.enabled else "DISABLED")
        rospy.logwarn("%s formation control %s", self.robot_name,
                      "enabled" if self.enabled else "disabled")

    def fresh(self, stamp, now):
        return stamp is not None and 0 <= (now - stamp).to_sec() <= self.data_timeout

    def data_ready(self, now):
        problem = None
        if not self.leader.valid or not self.follower.valid:
            problem = "UWB_INVALID"
        for robot in (self.leader, self.follower):
            required = (robot.pose_stamp, robot.velocity_stamp, robot.valid_stamp,
                        robot.headings[-1][0] if robot.headings else None)
            if not all(self.fresh(stamp, now) for stamp in required):
                problem = problem or "STALE_OR_MISSING_INPUT"
        if problem:
            for robot in (self.leader, self.follower):
                robot.stopped, robot.stationary_since = False, None
            return False, problem
        for robot in (self.leader, self.follower):
            robot.stationary(now, self.leader_linear_deadband, self.leader_angular_deadband,
                             self.stationary_duration)
            robot.align(self.prediction_limit)
        if self.leader.offset is None or self.follower.offset is None:
            return False, "WAIT_ALIGNMENT"
        return True, "FOLLOWING"

    def control_cb(self, _event):
        with self.lock:
            self.control_step()

    def control_step(self):
        now = rospy.Time.now()
        ready, state = self.data_ready(now)
        if not self.enabled:
            return
        if not ready:
            self.stop()
            self.publish_state(state)
            rospy.logwarn_throttle(1.0, "%s stopped: %s", self.robot_name, state)
            return

        leader_pose = self.leader.predict(now, self.prediction_limit)
        self_pose = self.follower.predict(now, self.prediction_limit)
        if leader_pose is None or self_pose is None:
            self.stop()
            self.publish_state("INPUT_TIME_SKEW")
            return
        leader_x, leader_y, leader_yaw = leader_pose
        self_x, self_y, self_yaw = self_pose
        dt = 1.0/self.rate if self.last_control is None else clamp(
            (now-self.last_control).to_sec(), 0.0, 2.0/self.rate)
        self.last_control = now

        leader_stopped = self.leader.stopped
        body_vx, body_vy, leader_omega = ((0.0, 0.0, 0.0) if leader_stopped
                                        else self.leader.filtered_velocity)
        leader_vx, leader_vy = rotate_offset(body_vx, body_vy, leader_yaw)
        target_x, target_y, feed_x, feed_y = target_and_velocity(
            leader_x, leader_y, leader_yaw, self.offset_x, self.offset_y,
            leader_vx, leader_vy, leader_omega)
        error_x = target_x - self_x
        error_y = target_y - self_y
        error_norm = math.hypot(error_x, error_y)
        velocity_x = feed_x + self.k_position*error_x
        velocity_y = feed_y + self.k_position*error_y
        speed = math.hypot(velocity_x, velocity_y)

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

        desired_yaw = wrap_angle(leader_yaw + self.offset_yaw)
        sync_error = wrap_angle(desired_yaw - self_yaw)
        # Continuous dead zone: zero correction within heading_tolerance.
        sync_correction = self.k_heading_sync * math.copysign(
            max(0.0, abs(sync_error) - self.heading_tolerance), sync_error)
        state = "APPROACH" if error_norm < self.approach_distance else "FOLLOWING"
        blend = 0.0
        angular_limit = self.max_angular
        reference_omega = 0.0
        if self.holding:
            state = "HOLD"
            blend = 1.0
            self.aligning = False
            target_yaw = desired_yaw
            heading_error = sync_error
            linear, angular = 0.0, sync_correction
            angular_limit = min(self.max_angular, self.hold_max_angular)
        else:
            reference_speed = math.hypot(feed_x, feed_y)
            motion_yaw = math.atan2(velocity_y, velocity_x) if speed > 1e-9 else self_yaw
            reference_yaw = math.atan2(feed_y, feed_x) if reference_speed > 1e-9 else motion_yaw
            # Tangent rotation: omega_L + d/dt atan2(vy + omega*dx, vx - omega*dy).
            ax, ay, alpha = (0.0, 0.0, 0.0) if leader_stopped else self.leader.acceleration
            qx, qy = body_vx-leader_omega*self.offset_y, body_vy+leader_omega*self.offset_x
            qdx, qdy = ax-alpha*self.offset_y, ay+alpha*self.offset_x
            reference_omega = clamp(leader_omega + (qx*qdy-qy*qdx)/max(
                reference_speed**2, self.velocity_deadband**2), -self.max_angular, self.max_angular)
            weight = smoothstep(reference_speed/self.velocity_deadband)
            ex, ey = rotate_offset(error_x, error_y, -self_yaw)
            reference_error = wrap_angle(reference_yaw-self_yaw)
            track_linear = reference_speed*math.cos(reference_error) + self.k_position*ex
            track_angular = (reference_omega + self.k_lateral*reference_speed*ey +
                             self.k_heading*math.sin(reference_error))
            # Smooth low-speed position capture; never pull a moving robot toward
            # the leader's final heading. At zero speed atan2 noise has zero gain.
            capture_error = wrap_angle(motion_yaw-self_yaw)
            capture_angular = self.k_heading*capture_error*smoothstep(speed/self.velocity_deadband)
            capture_linear = velocity_x*math.cos(self_yaw)+velocity_y*math.sin(self_yaw)
            linear = weight*track_linear+(1.0-weight)*capture_linear
            angular = weight*track_angular+(1.0-weight)*capture_angular
            target_yaw = wrap_angle(motion_yaw+weight*wrap_angle(reference_yaw-motion_yaw))
            heading_error = wrap_angle(target_yaw-self_yaw)
            if abs(heading_error) >= self.rotate_threshold:
                if not self.aligning:
                    self.turn_direction = math.copysign(1.0, heading_error)
                self.aligning = True
            elif abs(heading_error) <= self.rotate_exit:
                self.aligning = False
            if self.aligning:
                linear = 0.0
                turn_error = heading_error
                if abs(turn_error) > math.pi-0.2:
                    turn_error = math.copysign(abs(turn_error), self.turn_direction)
                angular = weight*reference_omega + self.k_heading*turn_error*smoothstep(
                    max(speed, reference_speed)/self.velocity_deadband)
            else:
                linear *= smoothstep((self.rotate_threshold-abs(heading_error)) /
                                     (self.rotate_threshold-self.rotate_exit))

        cmd = Twist()
        linear = clamp(linear, 0.0, self.max_linear)
        angular = clamp(angular, -angular_limit, angular_limit)
        cmd.linear.x = clamp(clamp(linear, self.last_cmd[0]-self.max_acceleration*dt,
                                  self.last_cmd[0]+self.max_acceleration*dt), 0.0, self.max_linear)
        cmd.angular.z = clamp(clamp(angular, self.last_cmd[1]-self.max_angular_acceleration*dt,
                                   self.last_cmd[1]+self.max_angular_acceleration*dt),
                              -angular_limit, angular_limit)
        # HOLD is a position lock; hard speed limits and invalid-input stops also
        # take precedence over normal-motion acceleration limits.
        if self.holding:
            cmd.linear.x = 0.0
        self.last_cmd = (cmd.linear.x, cmd.angular.z)
        self.cmd_pub.publish(cmd)
        self.reference_angular_pub.publish(Float64(data=reference_omega))
        self.publish_debug(now, target_x, target_y, error_x, error_y,
                           error_norm, target_yaw,
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
        with self.lock:
            self.holding = self.aligning = False
            self.hold_since = self.last_control = None
            self.last_cmd = (0.0, 0.0)
            if hasattr(self, "cmd_pub"):
                self.cmd_pub.publish(Twist())


def self_test():
    try:
        from unittest.mock import Mock, patch
    except ImportError:
        from mock import Mock, patch

    class Sink(object):
        def publish(self, value):
            self.last = value

    assert isfinite(0.0) and not isfinite(float("nan"))
    assert not isfinite(float("inf")) and not isfinite(float("-inf"))
    target = target_and_velocity(2.0, 3.0, math.pi/2, -1.0, 0.0, 0.2, 0.0, 0.5)
    assert all(abs(a-b) < 1e-9 for a, b in zip(target, (2.0, 2.0, 0.7, 0.0)))
    assert all(abs(a-b) < 1e-9 for a, b in zip(
        advance_pose(0, 0, 0, (1, 0, 1), math.pi/2), (1, 1, math.pi/2)))

    params = {"~auto_align_yaw": False, "~offset_x": 0.0, "~offset_y": 0.0}
    with patch.multiple(rospy, get_param=lambda name, default: params.get(name, default),
                        Subscriber=Mock(), Publisher=Mock(side_effect=lambda *a, **k: Sink()),
                        Timer=Mock(), on_shutdown=Mock(), loginfo=Mock(), set_param=Mock(),
                        logwarn=Mock(), loginfo_throttle=Mock(), logwarn_throttle=Mock()), \
            patch.object(rospy.Time, "now", return_value=rospy.Time(10)) as clock:
        def new_controller():
            c = DisplacementFollower()
            c.enabled = True
            return c

        def quaternion(q, angle):
            q.z, q.w = math.sin(angle/2), math.cos(angle/2)

        def feed(c, robot, pose, velocity, stamp=None, raw_offset=0.0):
            stamp = clock.return_value if stamp is None else stamp
            p = PoseStamped()
            p.header.stamp = stamp
            p.pose.position.x, p.pose.position.y = pose[:2]
            quaternion(p.pose.orientation, pose[2])
            c.pose_cb(p, robot)
            h = PoseWithCovarianceStamped()
            h.header.stamp = stamp
            quaternion(h.pose.pose.orientation, pose[2]+raw_offset)
            c.odom_cb(h, robot)
            v = Odometry()
            v.header.stamp = stamp
            v.twist.twist.linear.x, v.twist.twist.linear.y, v.twist.twist.angular.z = velocity
            c.velocity_cb(v, robot)
            c.valid_cb(Bool(data=True), robot)

        def step(c, t, leader=(0, 0, 0), follower=(0, 0, 0),
                 body=(0, 0, 0), own=(0, 0, 0), age=0.0):
            clock.return_value = rospy.Time.from_sec(t)
            stamp = rospy.Time.from_sec(t-age)
            feed(c, c.leader, leader, body, stamp)
            feed(c, c.follower, follower, own, stamp)
            c.control_cb(None)
            return c.cmd_pub.last

        # Subscribers/timers may invoke callbacks immediately during construction.
        for initial, enabled in ((False, True), (True, False)):
            params["~enabled"] = initial
            def subscribe(topic, kind, callback, **kwargs):
                if topic == "/five_ugv_formation/enable":
                    callback(Bool(data=enabled))
            with patch.object(rospy, "Subscriber", side_effect=subscribe):
                c = DisplacementFollower()
            assert c.enabled == enabled
            assert c.state_pub.last.data == ("READY" if enabled else "DISABLED")
        params.pop("~enabled")

        # Configuration rejects nonfinite gains and invalid hysteresis order.
        for name, value in (("~max_linear", .51), ("~k_position", float("nan")),
                            ("~rotate_exit_threshold", 1.1), ("~velocity_filter_tau", -1),
                            ("~data_timeout", 0), ("~offset_x", float("inf"))):
            params[name] = value
            try:
                DisplacementFollower()
                raise AssertionError(name)
            except ValueError:
                pass
            params.pop(name)
        params["~offset_x"] = 0.0

        # Runtime limits remain hard caps even while the normal command ramps.
        c = new_controller()
        for i in range(30):
            cmd = step(c, 20+i*.05, follower=(-1, 0, 0))
        assert cmd.linear.x == .15
        for i, value in enumerate((.07, 0, .3)):
            c.limit_cb(Float64(data=value))
            cmd = step(c, 22+i*.05, follower=(-1, 0, 0))
            assert 0 <= cmd.linear.x <= value
            assert c.limit_pub.last.data == value
        for value in (-.01, .51, float("nan"), float("inf")):
            c.limit_cb(Float64(data=value))
            assert c.max_linear == .3

        # Freshness uses source stamps: replay, future and missing inputs cannot drive.
        c = new_controller()
        step(c, 30, follower=(-1, 0, 0))
        step(c, 31, follower=(-1, 0, 0), age=1.0)
        assert c.state_pub.last.data == "STALE_OR_MISSING_INPUT"
        assert c.cmd_pub.last.linear.x == c.cmd_pub.last.angular.z == 0
        assert c.leader.stationary_since is None and not c.leader.stopped
        step(c, 32, follower=(-1, 0, 0), age=-.1)
        assert c.cmd_pub.last.linear.x == 0
        step(c, 33, follower=(-1, 0, 0))
        c.follower.velocity_stamp = None
        c.control_cb(None)
        assert c.state_pub.last.data == "STALE_OR_MISSING_INPUT"
        step(c, 34, follower=(-1, 0, 0))
        c.follower.valid_stamp = rospy.Time(32)
        c.control_cb(None)
        assert c.cmd_pub.last.linear.x == c.cmd_pub.last.angular.z == 0
        for i, velocity in enumerate(((float("nan"), 0, 0), (0, 0, float("inf")))):
            step(c, 35+i, follower=(-1, 0, 0))
            clock.return_value = rospy.Time.from_sec(35.1+i)
            feed(c, c.leader, (0, 0, 0), velocity)
            c.control_cb(None)
            assert c.cmd_pub.last.linear.x == c.cmd_pub.last.angular.z == 0
        c.valid_cb(Bool(data=False), c.leader)
        assert c.cmd_pub.last.linear.x == c.cmd_pub.last.angular.z == 0
        c.enable_cb(Bool(data=False))
        assert not c.holding and c.last_cmd == (0, 0)

        # Auto alignment captures map yaw, including nonzero launch headings.
        params["~auto_align_yaw"] = True
        c = new_controller()
        c.enabled = False
        for i in range(12):
            clock.return_value = rospy.Time.from_sec(40+i*.05)
            feed(c, c.leader, (0, 0, 1.1), (0, 0, 0), raw_offset=.7)
            feed(c, c.follower, (-1, 0, -.4), (0, 0, 0), raw_offset=-.8)
            c.control_cb(None)
            if i == 0:
                assert c.leader.offset is None
        assert abs(c.leader.offset+.7) < 1e-9 and abs(c.follower.offset-.8) < 1e-9
        # A moving robot cannot capture an arbitrary startup yaw as the map origin.
        moving = new_controller()
        for i in range(12):
            step(moving, 42+i*.05, body=(.1, 0, 0))
        assert moving.leader.offset is None
        assert moving.state_pub.last.data == "WAIT_ALIGNMENT"
        params["~auto_align_yaw"] = False

        # Source-time prediction reproduces a constant turn, up to its horizon.
        state = RobotState(0.0)
        state.pose, state.pose_stamp = (0, 0, 0), rospy.Time(50)
        state.velocity = (.1, 0, .2)
        state.headings.extend(((rospy.Time(50), 0), (rospy.Time.from_sec(50.1), .02)))
        predicted = state.predict(rospy.Time.from_sec(50.1), .2)
        expected = advance_pose(0, 0, 0, state.velocity, .1)
        assert all(abs(a-b) < 1e-9 for a, b in zip(predicted, expected))
        state.headings = deque(((rospy.Time.from_sec(50.3), .06),))
        assert state.predict(rospy.Time.from_sec(50.3), .2) is None

        # At exact moving formation, command and tangent feedforward are invariant.
        for omega in (-.05, .05):
            c = new_controller()
            c.offset_x, c.offset_y = -.8, .8
            vx, vy = .1-omega*.8, -omega*.8
            tangent = math.atan2(vy, vx)
            speed = math.hypot(vx, vy)
            for i in range(100):
                cmd = step(c, 60+i*.05, follower=(-.8, .8, tangent),
                           body=(.1, 0, omega), own=(speed, 0, omega))
            assert abs(cmd.linear.x-speed) < 1e-9
            assert abs(cmd.angular.z-omega) < 1e-9
            assert abs(c.reference_angular_pub.last.data-omega) < 1e-9
            assert c.heading_blend_pub.last.data == 0

        # Crossing the former 0.03 rad/s deadband is continuous.
        commands = []
        for omega in (.0299, .0301):
            c = new_controller()
            c.offset_x, c.offset_y = -.8, .8
            for i in range(50):
                cmd = step(c, 70+i*.05, follower=(-.8, .8, 0), body=(.06, 0, omega))
            commands.append(cmd.angular.z)
        assert abs(commands[1]-commands[0]) < .02
        # No linear-speed cliff at 0.7 rad; larger angles have hysteresis.
        commands = []
        for angle in (.699, .701):
            c = new_controller()
            for i in range(50):
                cmd = step(c, 75+i*.05, follower=(-.6, 0, -angle))
            commands.append(cmd.linear.x)
        assert min(commands) > 0 and abs(commands[1]-commands[0]) < .005
        c = new_controller()
        step(c, 80, follower=(-.6, 0, -1.0))
        assert c.aligning
        step(c, 80.05, follower=(-.6, 0, -.8))
        assert c.aligning
        step(c, 80.1, follower=(-.6, 0, -.5))
        assert not c.aligning

        # Preserve behind-target / opposing-final-heading regression.
        c = new_controller()
        for i, ey in enumerate((-.001, .001)):
            cmd = step(c, 85+i*.05, follower=(.3, -ey, math.pi))
            assert cmd.linear.x > 0 and abs(cmd.angular.z) < .05
        c = new_controller()
        signs = []
        for i in range(20):
            cmd = step(c, 87+i*.05, follower=(.3, .001*(-1)**i, 0))
            signs.append(math.copysign(1, cmd.angular.z))
        assert len(set(signs)) == 1  # Preserve chosen turn across the +/-pi boundary.

        # Static targets converge with jitter, then HOLD synchronizes final heading.
        scenarios = (((-1, -.4, 0), 0), ((.4, .2, 0), 0),
                     ((0, -.35, 0), 0), ((-.06, .29, -.87), 1.78))
        for start, leader_yaw in scenarios:
            c = new_controller()
            x, y, yaw = start
            own = (0, 0, 0)
            for i in range(1600):
                nx, ny = .012*math.sin(.7*i), .012*math.cos(.5*i)
                cmd = step(c, 100+.05*i, leader=(0, 0, leader_yaw),
                           follower=(x+nx, y+ny, yaw), own=own)
                if c.holding:
                    assert cmd.linear.x == 0
                x, y, yaw = advance_pose(x, y, yaw, (cmd.linear.x, 0, cmd.angular.z), .05)
                own = (cmd.linear.x, 0, cmd.angular.z)
            assert c.holding, (start, x, y, yaw)
            assert math.hypot(x, y) < c.hold_exit_tolerance
            assert abs(wrap_angle(yaw-leader_yaw)) < c.heading_tolerance+.01
            step(c, 180, follower=(.14, 0, leader_yaw))
            assert c.holding
            step(c, 180.05, follower=(.19, 0, leader_yaw))
            assert not c.holding
            c.valid_cb(Bool(data=False), c.leader)
            assert c.last_cmd == (0, 0) and c.hold_since is None

        # Moving turns: both sides of formation, curve reversal, source delay,
        # UWB jitter and first-order chassis lag. Metrics exclude initial capture.
        for profile in ("left", "right", "s_bend", "spin"):
            for oy in (-.8, .8):
                c = new_controller()
                c.offset_x, c.offset_y = -.8, oy
                leader = (0, 0, 0)
                follower = (-.95, oy+.12, 0)
                own = (0, 0, 0)
                history = deque(maxlen=4)
                errors, switches, zeros, last_sign = [], 0, 0, 0
                previous = (0, 0)
                for i in range(1800):
                    t = 200+i*.05
                    omega = (.06 if profile == "left" else -.06 if profile == "right"
                             else .07*math.sin(i*.05/10) if profile == "s_bend" else .07)
                    velocity = (0 if profile == "spin" else .06, 0, omega)
                    history.append((leader, follower, velocity, own))
                    delayed_leader, delayed_self, delayed_velocity, delayed_own = history[0]
                    age = .05*(len(history)-1)
                    nx, ny = .012*math.sin(.7*i), .012*math.cos(.5*i)
                    observed = (delayed_self[0]+nx, delayed_self[1]+ny, delayed_self[2])
                    cmd = step(c, t, delayed_leader, observed, delayed_velocity, delayed_own, age)
                    assert abs(cmd.linear.x-previous[0]) <= c.max_acceleration*.05+1e-8
                    assert abs(cmd.angular.z-previous[1]) <= c.max_angular_acceleration*.05+1e-8
                    previous = (cmd.linear.x, cmd.angular.z)
                    # 0.2-second actuation lag.
                    own = (own[0]+.2*(cmd.linear.x-own[0]), 0,
                           own[2]+.2*(cmd.angular.z-own[2]))
                    leader = advance_pose(*leader, velocity=velocity, dt=.05)
                    follower = advance_pose(*follower, velocity=own, dt=.05)
                    if i >= 600:
                        target = target_and_velocity(*leader, offset_x=-.8, offset_y=oy,
                                                     leader_vx=0, leader_vy=0, leader_omega=0)
                        errors.append(math.hypot(target[0]-follower[0], target[1]-follower[1]))
                        sign = 1 if cmd.angular.z > .02 else -1 if cmd.angular.z < -.02 else 0
                        if sign and last_sign and sign != last_sign:
                            switches += 1
                        if sign:
                            last_sign = sign
                        zeros += cmd.linear.x < .001
                rms = math.sqrt(sum(e*e for e in errors)/len(errors))
                p95 = sorted(errors)[int(.95*len(errors))]
                assert rms < .12 and p95 < .18, (profile, oy, rms, p95)
                assert zeros < .05*len(errors), (profile, oy, zeros)
                assert switches <= (4 if profile == "s_bend" else 2), (profile, oy, switches)
                print("turn %-6s offset_y=%+.1f RMS=%.3f P95=%.3f reversals=%d stopped=%d/%d" %
                      (profile, oy, rms, p95, switches, zeros, len(errors)))
                if profile == "s_bend":
                    for i in range(800):
                        cmd = step(c, 290+i*.05, leader, follower, (0, 0, 0), own)
                        own = (own[0]+.2*(cmd.linear.x-own[0]), 0,
                               own[2]+.2*(cmd.angular.z-own[2]))
                        follower = advance_pose(*follower, velocity=own, dt=.05)
                    assert c.holding and abs(wrap_angle(follower[2]-leader[2])) < .10
                    step(c, 330, leader, follower, (.06, 0, 0), own)
                    assert not c.holding
    print("displacement_follower self-test passed")


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        self_test()
    else:
        rospy.init_node("displacement_follower")
        DisplacementFollower()
        rospy.spin()
