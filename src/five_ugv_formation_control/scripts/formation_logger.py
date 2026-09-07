#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import csv
import math
import os
import sys
import threading
import time

import rospy
from geometry_msgs.msg import (PoseStamped, PoseWithCovarianceStamped, Twist,
                               Vector3Stamped)
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, Int32, Int32MultiArray, String


NAN = float("nan")


def isfinite(value):
    return not math.isnan(value) and not math.isinf(value)


def quaternion_to_yaw(q):
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if norm < 1e-9 or not isfinite(norm):
        return NAN
    x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    return math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))


def percentile(values, percent):
    values = sorted(v for v in values if isfinite(v))
    if not values:
        return NAN
    index = (len(values) - 1)*percent/100.0
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return values[lower]
    fraction = index - lower
    return values[lower]*(1.0-fraction) + values[upper]*fraction


class FormationLogger(object):
    ACTIVE_STATES = ("FOLLOWING", "APPROACH", "HOLD")
    COLUMNS = [
        "t", "ros_time",
        "leader_x", "leader_y", "leader_pose_stamp", "leader_pose_age",
        "leader_yaw", "leader_odom_stamp", "leader_odom_age",
        "follower_x", "follower_y", "follower_pose_stamp", "follower_pose_age",
        "follower_yaw", "follower_odom_stamp", "follower_odom_age",
        "target_x", "target_y", "target_yaw", "target_age",
        "error_x", "error_y", "error_norm", "heading_error", "error_age",
        "heading_sync_error", "heading_blend",
        "leader_vx", "leader_vy", "leader_omega",
        "leader_odom_vx", "leader_odom_vy", "leader_odom_omega",
        "leader_velocity_odom_stamp", "leader_velocity_odom_age",
        "desired_vx", "desired_vy", "desired_speed",
        "leader_cmd_linear_x", "leader_cmd_angular_z",
        "cmd_linear_x", "cmd_angular_z",
        "enabled", "state",
        "leader_valid", "leader_residual_rms", "leader_full_residual_rms",
        "leader_anchor_count", "leader_loo_id", "leader_jump_ids",
        "follower_valid", "follower_residual_rms", "follower_full_residual_rms",
        "follower_anchor_count", "follower_loo_id", "follower_jump_ids"
    ]

    def __init__(self):
        self.robot_name = rospy.get_param("~robot_name", "ugv1")
        self.leader_name = rospy.get_param("~leader_name", "ugv0")
        self.offset_x = float(rospy.get_param("~offset_x", -0.3))
        self.offset_y = float(rospy.get_param("~offset_y", 0.5))
        self.offset_yaw = float(rospy.get_param("~offset_yaw", 0.0))
        self.rate = max(1.0, float(rospy.get_param("~record_rate", 20.0)))
        self.flush_every = max(1, int(rospy.get_param("~flush_every", 20)))
        self.plot_stride = max(1, int(rospy.get_param("~plot_stride", 2)))
        self.output_root = os.path.expanduser(rospy.get_param(
            "~output_root", "~/formation_ws/logs/formation"))

        self.lock = threading.RLock()
        self.finished = False
        self.start_ros = rospy.Time.now().to_sec()
        self.values = dict((name, NAN) for name in self.COLUMNS)
        self.values.update({
            "enabled": int(bool(rospy.get_param("~initial_enabled", False))),
            "state": "NO_DATA",
            "leader_valid": 0, "follower_valid": 0,
            "leader_anchor_count": 0, "follower_anchor_count": 0,
            "leader_loo_id": -1, "follower_loo_id": -1,
            "leader_jump_ids": "", "follower_jump_ids": ""
        })
        self.received = {}
        self.run_dir = self.create_run_directory()
        self.write_parameters()
        self.csv_path = os.path.join(self.run_dir, "formation.csv")
        self.csv_file = (open(self.csv_path, "wb") if sys.version_info[0] < 3
                         else open(self.csv_path, "w", newline=""))
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(self.COLUMNS)
        self.row_count = 0

        self.create_subscribers()
        self.timer = rospy.Timer(rospy.Duration(1.0/self.rate), self.record)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Formation logger output: %s", self.run_dir)

    def create_run_directory(self):
        if not os.path.isdir(self.output_root):
            os.makedirs(self.output_root)
        stem = "%s_formation_%s" % (
            self.robot_name, time.strftime("%Y%m%d_%H%M%S"))
        path = os.path.join(self.output_root, stem)
        suffix = 1
        while os.path.exists(path):
            path = os.path.join(self.output_root, "%s_%02d" % (stem, suffix))
            suffix += 1
        os.makedirs(path)
        return path

    def write_parameters(self):
        sections = (
            ("formation_controller", rospy.get_param(
                "~controller_namespace", "/%s/formation_controller" % self.robot_name)),
            ("formation_logger", rospy.get_name())
        )
        with open(os.path.join(self.run_dir, "parameters.txt"), "w") as handle:
            for title, namespace in sections:
                handle.write("[%s]\n" % title)
                values = rospy.get_param(namespace, {})
                for key in sorted(values):
                    handle.write("%s: %r\n" % (key, values[key]))
                handle.write("\n")

    def topic(self, name, default):
        return rospy.get_param("~" + name, default)

    def create_subscribers(self):
        rospy.Subscriber(self.topic("leader_pose_topic", "/ugv0/uwb/pose"),
                         PoseStamped, self.pose_cb, callback_args="leader", queue_size=50)
        rospy.Subscriber(self.topic("follower_pose_topic", "/ugv1/uwb/pose"),
                         PoseStamped, self.pose_cb, callback_args="follower", queue_size=50)
        rospy.Subscriber(self.topic("leader_odom_topic", "/ugv0/odom_combined"),
                         PoseWithCovarianceStamped, self.odom_cb,
                         callback_args="leader", queue_size=50)
        rospy.Subscriber(self.topic("leader_velocity_odom_topic", "/ugv0/odom"),
                         Odometry, self.velocity_odom_cb, queue_size=50)
        rospy.Subscriber(self.topic("follower_odom_topic", "/ugv1/odom_combined"),
                         PoseWithCovarianceStamped, self.odom_cb,
                         callback_args="follower", queue_size=50)
        rospy.Subscriber(self.topic("leader_valid_topic", "/ugv0/uwb/valid"),
                         Bool, self.bool_cb, callback_args="leader_valid", queue_size=20)
        rospy.Subscriber(self.topic("follower_valid_topic", "/ugv1/uwb/valid"),
                         Bool, self.bool_cb, callback_args="follower_valid", queue_size=20)
        rospy.Subscriber(self.topic("enable_topic", "/five_ugv_formation/enable"),
                         Bool, self.bool_cb, callback_args="enabled", queue_size=5)
        rospy.Subscriber(self.topic("target_topic", "formation_controller/target_pose"),
                         PoseStamped, self.target_cb, queue_size=50)
        rospy.Subscriber(self.topic("error_topic", "formation_controller/tracking_error"),
                         Vector3Stamped, self.error_cb, queue_size=50)
        rospy.Subscriber(self.topic("state_topic", "formation_controller/state"),
                         String, self.state_cb, queue_size=10)
        rospy.Subscriber(self.topic("leader_velocity_topic",
                                    "formation_controller/leader_velocity"),
                         Vector3Stamped, self.vector_cb,
                         callback_args="leader", queue_size=50)
        rospy.Subscriber(self.topic("desired_velocity_topic",
                                    "formation_controller/desired_velocity"),
                         Vector3Stamped, self.vector_cb,
                         callback_args="desired", queue_size=50)
        rospy.Subscriber(self.topic("heading_error_topic",
                                    "formation_controller/heading_error"),
                         Float64, self.number_cb, callback_args="heading_error", queue_size=50)
        for name in ("heading_sync_error", "heading_blend"):
            rospy.Subscriber(self.topic(name + "_topic", "formation_controller/" + name),
                             Float64, self.number_cb, callback_args=name, queue_size=50)
        rospy.Subscriber(self.topic("leader_cmd_vel_topic", "/ugv0/cmd_vel"),
                         Twist, self.cmd_cb, callback_args="leader", queue_size=50)
        rospy.Subscriber(self.topic("cmd_vel_topic", "/ugv1/cmd_vel"),
                         Twist, self.cmd_cb, callback_args="follower", queue_size=50)

        for role, namespace in (("leader", "/" + self.leader_name),
                                ("follower", "/" + self.robot_name)):
            rospy.Subscriber(self.topic(role + "_residual_topic",
                                        namespace + "/uwb/residual_rms"),
                             Float64, self.number_cb,
                             callback_args=role + "_residual_rms", queue_size=50)
            rospy.Subscriber(self.topic(role + "_full_residual_topic",
                                        namespace + "/uwb/full_residual_rms"),
                             Float64, self.number_cb,
                             callback_args=role + "_full_residual_rms", queue_size=50)
            rospy.Subscriber(self.topic(role + "_anchor_count_topic",
                                        namespace + "/uwb/used_anchor_count"),
                             Int32, self.number_cb,
                             callback_args=role + "_anchor_count", queue_size=50)
            rospy.Subscriber(self.topic(role + "_loo_topic",
                                        namespace + "/uwb/loo_excluded_id"),
                             Int32, self.number_cb,
                             callback_args=role + "_loo_id", queue_size=50)
            rospy.Subscriber(self.topic(role + "_jump_topic",
                                        namespace + "/uwb/jump_rejected_ids"),
                             Int32MultiArray, self.jump_cb,
                             callback_args=role + "_jump_ids", queue_size=50)

    def update(self, values, received_key=None):
        with self.lock:
            self.values.update(values)
            if received_key is not None:
                self.received[received_key] = rospy.Time.now().to_sec()

    def pose_cb(self, msg, role):
        self.update({
            role + "_x": float(msg.pose.position.x),
            role + "_y": float(msg.pose.position.y),
            role + "_pose_stamp": msg.header.stamp.to_sec()
        }, role + "_pose")

    def odom_cb(self, msg, role):
        self.update({
            role + "_yaw": quaternion_to_yaw(msg.pose.pose.orientation),
            role + "_odom_stamp": msg.header.stamp.to_sec()
        }, role + "_odom")

    def target_cb(self, msg):
        self.update({
            "target_x": float(msg.pose.position.x),
            "target_y": float(msg.pose.position.y),
            "target_yaw": quaternion_to_yaw(msg.pose.orientation)
        }, "target")

    def velocity_odom_cb(self, msg):
        twist = msg.twist.twist
        self.update({
            "leader_odom_vx": twist.linear.x,
            "leader_odom_vy": twist.linear.y,
            "leader_odom_omega": twist.angular.z,
            "leader_velocity_odom_stamp": msg.header.stamp.to_sec()
        }, "leader_velocity_odom")

    def error_cb(self, msg):
        self.update({
            "error_x": float(msg.vector.x),
            "error_y": float(msg.vector.y),
            "error_norm": float(msg.vector.z)
        }, "error")

    def vector_cb(self, msg, kind):
        if kind == "leader":
            values = {"leader_vx": msg.vector.x,
                      "leader_vy": msg.vector.y,
                      "leader_omega": msg.vector.z}
        else:
            values = {"desired_vx": msg.vector.x,
                      "desired_vy": msg.vector.y,
                      "desired_speed": msg.vector.z}
        self.update(values)

    def cmd_cb(self, msg, role):
        prefix = "leader_" if role == "leader" else ""
        self.update({prefix + "cmd_linear_x": msg.linear.x,
                     prefix + "cmd_angular_z": msg.angular.z})

    def bool_cb(self, msg, key):
        self.update({key: int(bool(msg.data))})

    def number_cb(self, msg, key):
        self.update({key: msg.data})

    def jump_cb(self, msg, key):
        self.update({key: ";".join(str(int(value)) for value in msg.data)})

    def state_cb(self, msg):
        self.update({"state": msg.data})

    def record(self, _event):
        now = rospy.Time.now().to_sec()
        with self.lock:
            if self.finished:
                return
            row = dict(self.values)
            row["t"] = now - self.start_ros
            row["ros_time"] = now
            for key in ("leader_pose", "leader_odom", "follower_pose",
                        "follower_odom", "leader_velocity_odom", "target", "error"):
                row[key + "_age"] = (now - self.received[key]
                                      if key in self.received else NAN)
            self.writer.writerow([row[name] for name in self.COLUMNS])
            self.row_count += 1
            if self.row_count % self.flush_every == 0:
                self.csv_file.flush()

    @staticmethod
    def numbers(rows, name, scale=1.0):
        values = []
        for row in rows:
            try:
                values.append(float(row[name])*scale)
            except (KeyError, TypeError, ValueError):
                values.append(NAN)
        return values

    @staticmethod
    def finite_values(rows, name):
        return [value for value in FormationLogger.numbers(rows, name)
                if isfinite(value)]

    def save_plot(self, plt, filename):
        plt.tight_layout()
        plt.savefig(os.path.join(self.run_dir, filename), dpi=170)
        plt.close()

    def generate_plots(self, rows):
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as error:
            return "plots unavailable: %s" % error

        rows = rows[::self.plot_stride]
        t = self.numbers(rows, "t")

        plt.figure(figsize=(8, 7))
        for prefix, label in (("leader", "UGV0"),
                              ("follower", self.robot_name),
                              ("target", "target")):
            plt.plot(self.numbers(rows, prefix + "_x"),
                     self.numbers(rows, prefix + "_y"), label=label)
        plt.axis("equal"); plt.grid(True); plt.xlabel("x [m]"); plt.ylabel("y [m]")
        plt.title("Formation trajectories"); plt.legend()
        self.save_plot(plt, "01_trajectories.png")

        fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
        for prefix, label in (("leader", "UGV0"),
                              ("follower", self.robot_name),
                              ("target", "target")):
            axes[0].plot(t, self.numbers(rows, prefix + "_x"), label=label)
            axes[1].plot(t, self.numbers(rows, prefix + "_y"), label=label)
        axes[0].set_ylabel("x [m]"); axes[1].set_ylabel("y [m]")
        axes[1].set_xlabel("time [s]"); axes[0].set_title("Positions")
        for axis in axes: axis.grid(True); axis.legend()
        self.save_plot(plt, "02_positions.png")

        plt.figure(figsize=(11, 5))
        for name, label in (("error_x", "error x"), ("error_y", "error y"),
                            ("error_norm", "error norm")):
            plt.plot(t, self.numbers(rows, name), label=label)
        plt.grid(True); plt.xlabel("time [s]"); plt.ylabel("error [m]")
        plt.title("Formation tracking error"); plt.legend()
        self.save_plot(plt, "03_tracking_error.png")

        fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
        for name, label in (("leader_yaw", "UGV0 raw yaw"),
                            ("follower_yaw", self.robot_name + " raw yaw"),
                            ("target_yaw", "formation target yaw")):
            axes[0].plot(t, self.numbers(rows, name, 180.0/math.pi), label=label)
        axes[1].plot(t, self.numbers(rows, "heading_error", 180.0/math.pi),
                     label="controller heading error")
        axes[1].plot(t, self.numbers(rows, "heading_sync_error", 180.0/math.pi),
                     label="UGV0 heading synchronization error")
        axes[0].set_ylabel("yaw [deg]"); axes[1].set_ylabel("error [deg]")
        axes[1].set_xlabel("time [s]"); axes[0].set_title("Heading")
        for axis in axes: axis.grid(True); axis.legend()
        self.save_plot(plt, "04_heading.png")

        fig, axes = plt.subplots(3, 1, figsize=(11, 10), sharex=True)
        for name in ("leader_vx", "leader_vy", "desired_vx", "desired_vy"):
            axes[0].plot(t, self.numbers(rows, name), label=name)
        axes[1].plot(t, self.numbers(rows, "desired_speed"), label="desired speed")
        axes[1].plot(t, self.numbers(rows, "leader_cmd_linear_x"),
                     label="UGV0 cmd linear x")
        axes[1].plot(t, self.numbers(rows, "leader_odom_vx"),
                     label="UGV0 odom body vx")
        axes[1].plot(t, self.numbers(rows, "leader_odom_vy"),
                     label="UGV0 odom body vy")
        axes[1].plot(t, self.numbers(rows, "cmd_linear_x"),
                     label=self.robot_name + " cmd linear x")
        axes[2].plot(t, self.numbers(rows, "leader_omega"), label="leader omega")
        axes[2].plot(t, self.numbers(rows, "leader_odom_omega"),
                     label="UGV0 odom angular z")
        axes[2].plot(t, self.numbers(rows, "leader_cmd_angular_z"),
                     label="UGV0 cmd angular z")
        axes[2].plot(t, self.numbers(rows, "cmd_angular_z"),
                     label=self.robot_name + " cmd angular z")
        axes[0].set_ylabel("velocity [m/s]"); axes[1].set_ylabel("speed [m/s]")
        axes[2].set_ylabel("angular [rad/s]"); axes[2].set_xlabel("time [s]")
        axes[0].set_title("Controller velocities")
        for axis in axes: axis.grid(True); axis.legend(ncol=2)
        self.save_plot(plt, "05_velocities.png")

        fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
        axes[0].plot(t, self.numbers(rows, "leader_residual_rms"), label="UGV0")
        axes[0].plot(t, self.numbers(rows, "follower_residual_rms"),
                     label=self.robot_name)
        axes[1].plot(t, self.numbers(rows, "leader_anchor_count"), label="UGV0")
        axes[1].plot(t, self.numbers(rows, "follower_anchor_count"),
                     label=self.robot_name)
        axes[0].set_ylabel("residual RMS [m]"); axes[1].set_ylabel("anchors")
        axes[1].set_xlabel("time [s]"); axes[0].set_title("UWB quality")
        for axis in axes: axis.grid(True); axis.legend()
        self.save_plot(plt, "06_uwb_quality.png")

        fig, axes = plt.subplots(3, 1, figsize=(11, 10), sharex=True)
        for name, label in (("enabled", "enabled"), ("leader_valid", "UGV0 valid"),
                            ("follower_valid", self.robot_name + " valid")):
            axes[0].step(t, self.numbers(rows, name), where="post", label=label)
        axes[0].set_ylim(-0.1, 1.1); axes[0].set_yticks([0, 1])
        axes[0].set_title("Formation availability and control modes")
        axes[0].set_ylabel("valid / enabled"); axes[0].legend()
        states = list(dict.fromkeys(row.get("state", "NO_DATA") for row in rows))
        axes[1].step(t, [states.index(row.get("state", "NO_DATA")) for row in rows], where="post")
        axes[1].set_yticks(range(len(states))); axes[1].set_yticklabels(states)
        axes[1].set_ylabel("mode")
        axes[2].plot(t, self.numbers(rows, "heading_blend"))
        axes[2].set_ylim(-0.05, 1.05); axes[2].set_ylabel("heading blend")
        axes[2].set_xlabel("time [s]")
        for axis in axes: axis.grid(True)
        self.save_plot(plt, "07_availability.png")
        return "7 plots generated"

    def write_summary(self, rows, plot_status):
        active = [row for row in rows
                  if row.get("state") in self.ACTIVE_STATES and row.get("enabled") == "1"]
        errors = self.finite_values(active, "error_norm")
        headings = [abs(value)*180.0/math.pi
                    for value in self.finite_values(active, "heading_error")]
        sync_errors = [abs(value)*180.0/math.pi
                       for value in self.finite_values(active, "heading_sync_error")]
        duration = self.numbers(rows[-1:], "t")[0] if rows else 0.0
        with open(os.path.join(self.run_dir, "summary.txt"), "w") as handle:
            handle.write("robot: %s\n" % self.robot_name)
            handle.write("duration: %.3f s\n" % duration)
            handle.write("samples: %d\n" % len(rows))
            handle.write("active samples: %d\n" % len(active))
            for state in self.ACTIVE_STATES:
                handle.write("%s samples: %d\n" %
                             (state, sum(row.get("state") == state for row in active)))
            handle.write("desired offset: x=%.3f m, y=%.3f m, yaw=%.3f rad\n" %
                         (self.offset_x, self.offset_y, self.offset_yaw))
            if errors:
                handle.write("position error mean: %.4f m\n" %
                             (sum(errors)/len(errors)))
                handle.write("position error RMS: %.4f m\n" %
                             math.sqrt(sum(value*value for value in errors)/len(errors)))
                handle.write("position error p95: %.4f m\n" % percentile(errors, 95.0))
                handle.write("position error max: %.4f m\n" % max(errors))
            if headings:
                handle.write("absolute heading error mean: %.3f deg\n" %
                             (sum(headings)/len(headings)))
                handle.write("absolute heading error p95: %.3f deg\n" %
                             percentile(headings, 95.0))
                handle.write("absolute heading error max: %.3f deg\n" % max(headings))
            if sync_errors:
                handle.write("absolute heading synchronization error mean: %.3f deg\n" %
                             (sum(sync_errors)/len(sync_errors)))
                handle.write("absolute heading synchronization error p95: %.3f deg\n" %
                             percentile(sync_errors, 95.0))
            handle.write("plot status: %s\n" % plot_status)

    def shutdown(self):
        with self.lock:
            if self.finished:
                return
            self.finished = True
            self.csv_file.flush()
            self.csv_file.close()
        try:
            with open(self.csv_path, "r") as handle:
                rows = list(csv.DictReader(handle))
            plot_status = self.generate_plots(rows) if rows else "no data"
            self.write_summary(rows, plot_status)
            rospy.loginfo("Formation log completed: %s", self.run_dir)
        except Exception as error:
            rospy.logerr("Formation plot generation failed; CSV is safe: %s", error)


def self_test():
    assert isfinite(0.0) and not isfinite(float("nan"))
    assert not isfinite(float("inf")) and not isfinite(float("-inf"))
    class Quaternion(object):
        x = 0.0; y = 0.0
        z = math.sin(math.pi/4.0); w = math.cos(math.pi/4.0)
    assert abs(quaternion_to_yaw(Quaternion()) - math.pi/2.0) < 1e-9
    assert abs(percentile([1.0, 2.0, 3.0, 4.0], 50.0) - 2.5) < 1e-9
    print("formation_logger self-test passed")


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        self_test()
    else:
        rospy.init_node("formation_logger")
        FormationLogger()
        rospy.spin()
