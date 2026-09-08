#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import copy
import json
import math
import os
import threading
from collections import deque
try:
    from time import monotonic
except ImportError:
    # ROS Melodic/Python 2 on Linux: use the native monotonic clock.
    import ctypes

    class _Timespec(ctypes.Structure):
        _fields_ = [("tv_sec", ctypes.c_long), ("tv_nsec", ctypes.c_long)]

    _clock_gettime = ctypes.CDLL("librt.so.1", use_errno=True).clock_gettime
    _clock_gettime.argtypes = [ctypes.c_int, ctypes.POINTER(_Timespec)]
    _clock_gettime.restype = ctypes.c_int

    def monotonic():
        value = _Timespec()
        if _clock_gettime(1, ctypes.byref(value)) != 0:  # CLOCK_MONOTONIC
            raise OSError(ctypes.get_errno(), "clock_gettime(CLOCK_MONOTONIC) failed")
        return value.tv_sec + value.tv_nsec*1e-9
import numpy as np
import rospy
import yaml
from geometry_msgs.msg import PoseStamped, PointStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64, Int32, Int32MultiArray, String
from nlink_parser.msg import LinktrackNodeframe2
from robust_range_ekf import RangeEKF, FrameClock, predict_motion


class RobustUWBLocalizer(object):
    def __init__(self):
        self.config = rospy.get_param("~", {})
        self.config.pop("effective_config", None)  # A previous node may leave its snapshot on the master.
        calibration = os.path.expanduser(self.config.get("calibration_file", "~/.config/formation/uwb_calibration.yaml"))
        if os.path.isfile(calibration):
            with open(calibration) as stream:
                calibrated = yaml.safe_load(stream) or {}
            for key in ("range_biases", "range_stddevs", "tag_offset_xy", "gyro_bias_z"):
                if key in calibrated:
                    self.config[key] = calibrated[key]
            rospy.loginfo("Loaded fixed UWB calibration: %s", calibration)
        self.ekf = RangeEKF(self.config)
        self.world_frame = self.config.get("world_frame", "uwb_map")
        for name, default in (("solve_rate", 20.0), ("max_measurement_age", 0.15),
                              ("input_watchdog_timeout", 0.20), ("motion_timeout", 0.15)):
            value = float(self.config.get(name, default))
            if not np.isfinite(value) or value <= 0:
                raise ValueError("%s must be finite and positive" % name)
            setattr(self, name, value)
        self.watchdog_timeout = self.input_watchdog_timeout
        self.path_min_distance = float(self.config.get("path_min_distance", 0.03))
        self.path_publish_rate = float(self.config.get("path_publish_rate", 2.0))
        self.path_max_points = int(self.config.get("path_max_points", 1000))
        if not np.isfinite(self.path_publish_rate) or self.path_publish_rate < 0 or self.path_max_points < 1:
            raise ValueError("Invalid path settings")
        self.clock = FrameClock()
        self.input_lock, self.output_lock = threading.Lock(), threading.Lock()
        self.latest_frame = None
        self.dropped_frames = 0
        self.motion = {"odom": deque(maxlen=500), "imu": deque(maxlen=500)}
        self.last_input_time = self.last_valid_time = self.filter_time = self.last_update_time = None
        self.good_updates = 0
        self.recovery_frames = int(self.config.get("valid_recovery_frames", 3))
        if self.recovery_frames < 1:
            raise ValueError("valid_recovery_frames must be positive")
        self.last_path_xy = self.last_path_publish_time = None
        self.path_poses = deque(maxlen=self.path_max_points)
        self.path_dirty = False
        for name, topic, kind, latch in (
                ("pose", "pose", PoseStamped, False), ("point", "point", PointStamped, False),
                ("covariance", "pose_covariance", PoseWithCovarianceStamped, False),
                ("path", "path", Path, True), ("valid", "valid", Bool, True),
                ("status", "status", String, True), ("diagnostics", "diagnostics", String, False),
                ("rms", "residual_rms", Float64, False), ("full_rms", "full_residual_rms", Float64, False),
                ("count", "used_anchor_count", Int32, False),
                ("jump", "jump_rejected_ids", Int32MultiArray, False),
                ("loo", "loo_excluded_id", Int32, False),
                ("age", "measurement_age", Float64, False),
                ("processing", "processing_time", Float64, False),
                ("dropped", "dropped_frame_count", Int32, False)):
            setattr(self, name + "_pub", rospy.Publisher("uwb/" + topic, kind, queue_size=1, latch=latch))
        self.publish_valid(False)
        self.status_pub.publish(String(data="INITIALIZING"))
        # Save effective parameters (including per-car calibration) for the logger.
        rospy.set_param("~effective_config", self.config)
        rospy.Subscriber("odom", Odometry, self.motion_cb, "odom", queue_size=1)
        rospy.Subscriber("imu", Imu, self.motion_cb, "imu", queue_size=1)
        rospy.Subscriber(self.config["input_topic"], LinktrackNodeframe2, self.cb,
                         queue_size=1, buff_size=2**20)
        self.solve_timer = rospy.Timer(rospy.Duration(1.0/self.solve_rate), self.process_cb)
        self.watchdog_timer = rospy.Timer(rospy.Duration(0.05), self.watchdog_cb)
        rospy.loginfo("Robust range EKF started in %s; keep stationary facing UWB yaw %.3f during initialization",
                      rospy.get_namespace(), self.ekf.initial_yaw)

    def motion_cb(self, msg, kind):
        now, received = rospy.Time.now().to_sec(), monotonic()
        stamp = msg.header.stamp.to_sec()
        values = ((msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z)
                  if kind == "odom" else (msg.angular_velocity.z,))
        if (not np.all(np.isfinite(values)) or not 0 <= now-stamp <= self.motion_timeout or
                max(abs(v) for v in values) > 3.0):
            return
        with self.input_lock:
            history = self.motion[kind]
            sample_time = received - (now-stamp)
            if not history or sample_time > history[-1][0]:
                history.append((sample_time, values))

    def cb(self, msg):
        with self.input_lock:
            received, stamp = monotonic(), rospy.Time.now()
            timing = self.clock.accept(msg.local_time, received)
            if timing is None:
                self.dropped_frames += 1
                return
            measured, reset = timing
            if self.latest_frame is not None:
                self.dropped_frames += 1
            self.latest_frame = (msg, stamp, received, measured, reset)
            self.last_input_time = received

    def process_cb(self, _event):
        with self.input_lock:
            frame, self.latest_frame = self.latest_frame, None
            motion = {k: list(v) for k, v in self.motion.items()}
        if frame is None:
            return
        msg, received_stamp, received, measured, reset = frame
        started = monotonic()
        result = {"valid": False, "status": "STALE_INPUT", "accepted_ids": [], "rejected_ids": []}
        candidate = None
        try:
            if started-measured <= self.max_measurement_age:
                candidate = copy.deepcopy(self.ekf)
                result = {"valid": False, "status": "MOTION_STALE", "accepted_ids": [], "rejected_ids": []}
                # A lost motion interval cannot be reconstructed from a later velocity.
                start = measured if self.filter_time is None else self.filter_time
                fresh = predict_motion(candidate, start, measured, motion, self.motion_timeout)
                if reset or not fresh:
                    # Preserve the established world heading; require a node restart if
                    # a motion gap means the robot could have rotated while unobserved.
                    if self.ekf.x is not None:
                        result["status"] = "RESTART_REQUIRED"
                        candidate = None

                    else:
                        candidate.candidates.clear()
                elif getattr(self, "motion_fault", False):
                    result["status"] = "RESTART_REQUIRED"
                    candidate = None
                else:
                    # Bootstrap must be stationary. Initial yaw is the known launch heading.
                    velocity = next((v for t, v in reversed(motion["odom"]) if t <= measured), (9, 9, 9))
                    if candidate.x is None and max(abs(v) for v in velocity) > 0.02:
                        candidate.candidates.clear()
                        result["status"] = "WAIT_STATIONARY"
                    else:
                        measurements = {}
                        duplicate = False
                        for n in msg.nodes:
                            if int(n.id) in measurements:
                                duplicate = True
                            measurements[int(n.id)] = float(n.dis)
                        if duplicate:
                            result["status"] = "DUPLICATE_ANCHOR"
                        else:
                            result = candidate.update(measurements)
            with self.output_lock:
                finished = monotonic()
                if finished-measured > self.max_measurement_age:
                    candidate = None
                    result.update(valid=False, status="STALE_INPUT")
                if result["status"] == "RESTART_REQUIRED":
                    self.motion_fault = True
                self.last_update_time = measured
                if candidate is not None:
                    self.ekf, self.filter_time = candidate, measured
                if result["valid"]:
                    self.good_updates += 1
                    if self.good_updates < self.recovery_frames:
                        result.update(valid=False, status="RECOVERING")
                else:
                    self.good_updates = 0
                stamp = received_stamp - rospy.Duration(max(0.0, received-measured))
                result.update(stamp=stamp.to_sec(), receive_stamp=received_stamp.to_sec(),
                              local_time=int(msg.local_time), system_time=int(msg.system_time),
                              measurement_age=finished-measured, processing_time=finished-started,
                              dropped_frames=self.dropped_frames,
                              motion_ages={k: (measured-v[-1][0] if v else None) for k, v in motion.items()},
                              position_stddev=(self.ekf.position_stddev() if self.ekf.x is not None else None),
                              state=(self.ekf.x.tolist() if self.ekf.x is not None else None))
                self.publish_result(result)
                if result["valid"]:
                    self.last_valid_time = measured
                    self.publish_pose(stamp)
        except (ValueError, ArithmeticError, np.linalg.LinAlgError) as error:
            with self.output_lock:
                self.good_updates = 0
                self.publish_valid(False)
                self.status_pub.publish(String(data="FILTER_ERROR"))
            rospy.logerr_throttle(1.0, "UWB EKF: %s", error)

    def publish_result(self, result):
        self.publish_valid(result["valid"])
        self.status_pub.publish(String(data=result["status"]))
        self.diagnostics_pub.publish(String(data=json.dumps(result, sort_keys=True, allow_nan=False)))
        self.count_pub.publish(Int32(data=len(result.get("accepted_ids", []))))
        self.jump_pub.publish(Int32MultiArray(data=result.get("rejected_ids", [])))
        self.loo_pub.publish(Int32(data=-1))
        for name, key in (("rms", "residual_rms"), ("full_rms", "full_residual_rms"),
                          ("age", "measurement_age"), ("processing", "processing_time")):
            value = result.get(key)
            getattr(self, name + "_pub").publish(Float64(data=float("nan") if value is None else value))
        self.dropped_pub.publish(Int32(data=self.dropped_frames))

    def publish_pose(self, stamp):
        x, y, yaw = self.ekf.x
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp, self.world_frame
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.position.z = self.ekf.tag_height
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        self.pose_pub.publish(pose)
        point = PointStamped(); point.header = pose.header; point.point = pose.pose.position
        self.point_pub.publish(point)
        covariance = PoseWithCovarianceStamped()
        covariance.header, covariance.pose.pose = pose.header, pose.pose
        for i, row in enumerate((0, 1, 5)):
            for j, col in enumerate((0, 1, 5)):
                covariance.pose.covariance[6*row+col] = float(self.ekf.P[i, j])
        for i in (0, 7):
            covariance.pose.covariance[i] += self.ekf.systematic_stddev**2
        for i in (14, 21, 28):
            covariance.pose.covariance[i] = 1e6
        self.covariance_pub.publish(covariance)
        self.update_path(pose)

    def update_path(self, pose):
        if self.path_publish_rate == 0.0:
            return
        xy = (pose.pose.position.x, pose.pose.position.y)
        if self.last_path_xy is None or math.hypot(xy[0]-self.last_path_xy[0], xy[1]-self.last_path_xy[1]) >= self.path_min_distance:
            self.last_path_xy = xy
            self.path_poses.append(pose)
            self.path_dirty = True
        now = monotonic()
        if self.path_dirty and (self.last_path_publish_time is None or
                               now - self.last_path_publish_time >= 1.0/self.path_publish_rate):
            path = Path()
            path.header = self.path_poses[-1].header
            path.poses = list(self.path_poses)
            self.path_pub.publish(path)
            self.last_path_publish_time = monotonic()
            self.path_dirty = False

    def publish_valid(self, value):
        self.valid_pub.publish(Bool(data=bool(value)))

    def watchdog_cb(self, _event):
        with self.output_lock:
            now = monotonic()
            if (self.last_valid_time is None or now-self.last_valid_time > self.max_measurement_age or
                    self.last_input_time is None or now-self.last_input_time > self.watchdog_timeout):
                if self.last_update_time is None or now-self.last_update_time > self.max_measurement_age:
                    self.good_updates = 0
                self.publish_valid(False)
                self.status_pub.publish(String(data="RESTART_REQUIRED" if getattr(self, "motion_fault", False)
                                              else "WAIT_VALID_INPUT"))


if __name__ == "__main__":
    rospy.init_node("uwb_localizer")
    RobustUWBLocalizer()
    rospy.spin()
