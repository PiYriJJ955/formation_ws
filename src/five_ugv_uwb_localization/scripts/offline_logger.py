#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import csv
import math
import os
import time
import json
import threading
import tempfile

import rospy

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64, Int32, Int32MultiArray, String
from nlink_parser.msg import LinktrackNodeframe2


class LocalUGVOfflineLogger(object):
    """Per-car streaming CSV/JSONL recorder; shutdown generates plots."""

    def __init__(self):
        self.ugv_id = int(rospy.get_param("~ugv_id", 0))
        self.output_root = os.path.expanduser(
            rospy.get_param(
                "~output_root",
                "~/formation_ws/logs/uwb"
            )
        )
        self.duration = float(rospy.get_param("~duration", 0.0))

        anchors = rospy.get_param("~anchors", [])
        if not anchors:
            raise ValueError("Logger requires the localization anchor configuration")
        self.anchor_order = [int(a["id"]) for a in anchors]
        self.anchor_xy = {int(a["id"]): (a["x"], a["y"]) for a in anchors}
        if not os.path.isdir(self.output_root):
            os.makedirs(self.output_root)
        self.output_dir = tempfile.mkdtemp(prefix="ugv%d_test_%s_" %
                                          (self.ugv_id, time.strftime("%Y%m%d_%H%M%S")),
                                          dir=self.output_root)
        self.lock = threading.RLock()
        self._finished = False
        self.files, self.writers = {}, {}
        self.columns = {
            "localization": ["t", "x", "y", "valid", "residual_rms", "full_residual_rms",
                             "used_anchor_count", "jump_rejected_ids", "loo_excluded_id", "stamp", "status"],
            "raw_linktrack": (["t"] + ["range_id%d" % i for i in self.anchor_order] +
                              ["fp_rssi_id%d" % i for i in self.anchor_order] +
                              ["rx_rssi_id%d" % i for i in self.anchor_order] +
                              ["receive_stamp", "local_time", "system_time"]),
            "events": ["t", "event_type", "anchor_id"],
            "odom": ["t", "receive_stamp", "stamp", "vx", "vy", "omega"],
            "imu": ["t", "receive_stamp", "stamp", "gyro_z", "ax", "ay", "az"],
        }
        for name, columns in self.columns.items():
            self.files[name] = open(os.path.join(self.output_dir, name+".csv"), "w")
            self.writers[name] = csv.writer(self.files[name])
            self.writers[name].writerow(columns)
        self.files["diagnostics"] = open(os.path.join(self.output_dir, "diagnostics.jsonl"), "w")
        self.parameter_snapshot = False
        self.flush(None)
        self.start = rospy.Time.now()

        self.latest_valid = False
        self.latest_rms = float("nan")
        self.latest_full_rms = float("nan")
        self.latest_count = 0
        self.latest_jump_ids = []
        self.latest_loo_id = -1

        self.pose_rows = []
        self.raw_rows = []
        self.event_rows = []

        rospy.Subscriber("uwb/diagnostics", String, self.diagnostics_cb, queue_size=100)
        rospy.Subscriber("nlink_linktrack_nodeframe2", LinktrackNodeframe2, self.raw_cb, queue_size=100)
        rospy.Subscriber("odom", Odometry, self.motion_cb, "odom", queue_size=100)
        rospy.Subscriber("imu", Imu, self.motion_cb, "imu", queue_size=100)
        self.flush_timer = rospy.Timer(rospy.Duration(1.0), self.flush)

        rospy.on_shutdown(self.finish)

        if self.duration > 0.0:
            rospy.Timer(
                rospy.Duration(self.duration),
                self.auto_stop,
                oneshot=True
            )

        rospy.loginfo(
            "UGV%d offline logger started. Output: %s",
            self.ugv_id,
            self.output_dir
        )
        rospy.loginfo("CSV/JSONL recording active; flush every second. Ctrl+C generates plots.")

    def t(self):
        return (rospy.Time.now() - self.start).to_sec()

    def auto_stop(self, _event):
        rospy.signal_shutdown("offline logger duration reached")

    def finite(self, v):
        try:
            return not (math.isnan(v) or math.isinf(v))
        except Exception:
            return False

    def flush(self, _event):
        with self.lock:
            if self._finished:
                return
            for stream in self.files.values():
                stream.flush()
                os.fsync(stream.fileno())
            if not self.parameter_snapshot:
                params = rospy.get_param("uwb_localizer/effective_config", None)
                if params:
                    with open(os.path.join(self.output_dir, "parameters.json"), "w") as stream:
                        json.dump(params, stream, indent=2, sort_keys=True)
                    self.parameter_snapshot = True

    def diagnostics_cb(self, msg):
        # A single diagnostic frame keeps pose, validity, residual and IDs aligned.
        data = json.loads(msg.data)
        with self.lock:
            if self._finished:
                return
            self.files["diagnostics"].write(msg.data+"\n")
            state = data.get("state") or [float("nan"), float("nan")]
            row = {"t": self.t(), "x": state[0], "y": state[1], "valid": int(data["valid"]),
                   "rms": data.get("residual_rms"), "full_rms": data.get("full_residual_rms"),
                   "count": len(data.get("accepted_ids", [])),
                   "jump_ids": ",".join(str(i) for i in data.get("rejected_ids", [])), "loo_id": -1}
            # Numeric CSV/plots retain NaN for missing residuals.
            for key in ("rms", "full_rms"):
                if row[key] is None:
                    row[key] = float("nan")
            self.pose_rows.append(row)
            self.writers["localization"].writerow([row[k] for k in
                ("t", "x", "y", "valid", "rms", "full_rms", "count", "jump_ids", "loo_id")] +
                [data["stamp"], data["status"]])
            for aid in data.get("rejected_ids", []):
                self.event_rows.append({"t": row["t"], "type": "innovation", "anchor_id": aid})
                self.writers["events"].writerow([row["t"], "innovation", aid])

    def motion_cb(self, msg, kind):
        if kind == "odom":
            v = msg.twist.twist
            values = [v.linear.x, v.linear.y, v.angular.z]
        else:
            a = msg.linear_acceleration
            values = [msg.angular_velocity.z, a.x, a.y, a.z]
        with self.lock:
            if not self._finished:
                self.writers[kind].writerow([self.t(), rospy.Time.now().to_sec(), msg.header.stamp.to_sec()] + values)

    def raw_cb(self, msg):
        vals = {}

        for node in msg.nodes:
            aid = int(node.id)
            if aid not in self.anchor_order:
                continue

            vals[aid] = {
                "range": float(node.dis),
                "fp": float(node.fp_rssi),
                "rx": float(node.rx_rssi)
            }

        row = {"t": self.t()}

        for aid in self.anchor_order:
            if aid in vals:
                row["r%d" % aid] = vals[aid]["range"]
                row["fp%d" % aid] = vals[aid]["fp"]
                row["rx%d" % aid] = vals[aid]["rx"]
            else:
                row["r%d" % aid] = float("nan")
                row["fp%d" % aid] = float("nan")
                row["rx%d" % aid] = float("nan")

        with self.lock:
            if self._finished:
                return
            self.raw_rows.append(row)
            values = [row["t"]] + [row["%s%d" % (prefix, aid)]
                                   for prefix in ("r", "fp", "rx") for aid in self.anchor_order]
            self.writers["raw_linktrack"].writerow(values + [rospy.Time.now().to_sec(), msg.local_time, msg.system_time])

    def plot_trajectory(self):
        if len(self.pose_rows) < 2:
            return

        plt.figure(figsize=(8, 6))

        plt.plot(
            [r["x"] for r in self.pose_rows],
            [r["y"] for r in self.pose_rows],
            label="UGV%d" % self.ugv_id
        )

        for aid in self.anchor_order:
            ax, ay = self.anchor_xy[aid]
            plt.scatter([ax], [ay], s=45)
            plt.text(
                ax + 0.04,
                ay + 0.04,
                "ID%d" % aid
            )

        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.title(
            "UGV%d UWB Trajectory" % self.ugv_id
        )
        plt.grid(True)
        plt.axis("equal")
        plt.legend()
        plt.tight_layout()

        plt.savefig(
            os.path.join(
                self.output_dir,
                "01_trajectory.png"
            ),
            dpi=180
        )
        plt.close()

    def plot_xy(self):
        if len(self.pose_rows) < 2:
            return

        t = [r["t"] for r in self.pose_rows]

        plt.figure(figsize=(11, 4.5))
        plt.plot(
            t,
            [r["x"] for r in self.pose_rows],
            label="x"
        )
        plt.plot(
            t,
            [r["y"] for r in self.pose_rows],
            label="y"
        )

        plt.xlabel("Time [s]")
        plt.ylabel("Position [m]")
        plt.title(
            "UGV%d Estimated Position" % self.ugv_id
        )
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        plt.savefig(
            os.path.join(
                self.output_dir,
                "02_xy_vs_time.png"
            ),
            dpi=180
        )
        plt.close()

    def plot_residual(self):
        if len(self.pose_rows) < 2:
            return

        t = [r["t"] for r in self.pose_rows]

        plt.figure(figsize=(11, 4.5))
        plt.plot(
            t,
            [r["rms"] for r in self.pose_rows],
            label="used-set RMS"
        )
        plt.plot(
            t,
            [r["full_rms"] for r in self.pose_rows],
            label="full-set RMS"
        )

        plt.xlabel("Time [s]")
        plt.ylabel("Residual RMS [m]")
        plt.title(
            "UGV%d Localization Residual" % self.ugv_id
        )
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        plt.savefig(
            os.path.join(
                self.output_dir,
                "03_residual_rms.png"
            ),
            dpi=180
        )
        plt.close()

    def plot_valid(self):
        if len(self.pose_rows) < 2:
            return

        plt.figure(figsize=(11, 3.8))
        plt.plot(
            [r["t"] for r in self.pose_rows],
            [r["valid"] for r in self.pose_rows]
        )

        plt.xlabel("Time [s]")
        plt.ylabel("valid")
        plt.yticks([0, 1], ["False", "True"])
        plt.title(
            "UGV%d Localization Valid" % self.ugv_id
        )
        plt.grid(True)
        plt.tight_layout()

        plt.savefig(
            os.path.join(
                self.output_dir,
                "04_valid.png"
            ),
            dpi=180
        )
        plt.close()

    def plot_count(self):
        if len(self.pose_rows) < 2:
            return

        plt.figure(figsize=(11, 3.8))
        plt.plot(
            [r["t"] for r in self.pose_rows],
            [r["count"] for r in self.pose_rows]
        )

        plt.xlabel("Time [s]")
        plt.ylabel("Used anchors")
        plt.ylim(0, len(self.anchor_order)+0.5)
        plt.yticks(range(0, len(self.anchor_order)+1))
        plt.title(
            "UGV%d Used Anchor Count" % self.ugv_id
        )
        plt.grid(True)
        plt.tight_layout()

        plt.savefig(
            os.path.join(
                self.output_dir,
                "05_used_anchor_count.png"
            ),
            dpi=180
        )
        plt.close()

    def plot_events(self):
        if len(self.event_rows) == 0:
            return

        plt.figure(figsize=(11, 4.2))

        for r in self.event_rows:
            y = r["anchor_id"]
            marker = "o" if r["type"] == "jump" else "x"

            plt.scatter(
                [r["t"]],
                [y],
                marker=marker,
                s=28
            )

        plt.xlabel("Time [s]")
        plt.ylabel("Anchor ID")
        plt.yticks(self.anchor_order)
        plt.title(
            "UGV%d Range Rejection Events" % self.ugv_id
        )
        plt.grid(True)
        plt.tight_layout()

        plt.savefig(
            os.path.join(
                self.output_dir,
                "06_events.png"
            ),
            dpi=180
        )
        plt.close()

    def plot_raw(self, field, ylabel, filename, title):
        if len(self.raw_rows) < 2:
            return

        plt.figure(figsize=(11, 5.5))

        for aid in self.anchor_order:
            tt = []
            yy = []
            key = "%s%d" % (field, aid)

            for r in self.raw_rows:
                v = r[key]

                if self.finite(v):
                    tt.append(r["t"])
                    yy.append(v)

            if len(yy) > 0:
                plt.plot(
                    tt,
                    yy,
                    label="ID%d" % aid
                )

        plt.xlabel("Time [s]")
        plt.ylabel(ylabel)
        plt.title(
            "UGV%d %s" % (
                self.ugv_id,
                title
            )
        )
        plt.grid(True)
        plt.legend(ncol=3)
        plt.tight_layout()

        plt.savefig(
            os.path.join(
                self.output_dir,
                filename
            ),
            dpi=180
        )
        plt.close()

    def write_summary(self):
        path = os.path.join(
            self.output_dir,
            "summary.txt"
        )

        with open(path, "w") as f:
            f.write(
                "UGV%d localization summary\n"
                % self.ugv_id
            )
            f.write("========================\n\n")
            f.write(
                "pose samples: %d\n"
                % len(self.pose_rows)
            )
            f.write(
                "raw samples: %d\n"
                % len(self.raw_rows)
            )

            if len(self.pose_rows) > 0:
                valid_values = [
                    r["valid"]
                    for r in self.pose_rows
                ]
                valid_ratio = (
                    sum(valid_values) /
                    float(len(valid_values))
                )

                f.write(
                    "valid ratio: %.2f %%\n"
                    % (100.0 * valid_ratio)
                )

                rms_values = [
                    r["rms"]
                    for r in self.pose_rows
                    if self.finite(r["rms"])
                ]

                if len(rms_values) > 0:
                    f.write(
                        "mean residual RMS: %.4f m\n"
                        % (
                            sum(rms_values) /
                            float(len(rms_values))
                        )
                    )
                    f.write(
                        "max residual RMS: %.4f m\n"
                        % max(rms_values)
                    )

            jump_counts = dict(
                (aid, 0)
                for aid in self.anchor_order
            )
            loo_counts = dict(
                (aid, 0)
                for aid in self.anchor_order
            )

            for e in self.event_rows:
                if e["type"] in ("jump", "innovation"):
                    jump_counts[e["anchor_id"]] += 1
                elif e["type"] == "loo":
                    loo_counts[e["anchor_id"]] += 1

            f.write("\nRange rejection counts:\n")
            for aid in self.anchor_order:
                f.write(
                    "  ID%d: %d\n"
                    % (
                        aid,
                        jump_counts[aid]
                    )
                )

            f.write("\nLOO exclusion counts:\n")
            for aid in self.anchor_order:
                f.write(
                    "  ID%d: %d\n"
                    % (
                        aid,
                        loo_counts[aid]
                    )
                )

        return path

    def finish(self):
        with self.lock:
            if self._finished:
                return
            self.flush(None)
            self._finished = True
            for stream in self.files.values():
                stream.close()
        # Durable files are closed before plotting or printing to a terminal
        # that may already have disconnected.
        try:
            pose_path, raw_path, event_path = [os.path.join(self.output_dir, name+".csv")
                                               for name in ("localization", "raw_linktrack", "events")]
            self.plot_trajectory()
            self.plot_xy()
            self.plot_residual()
            self.plot_valid()
            self.plot_count()
            self.plot_events()

            self.plot_raw(
                "r",
                "Range [m]",
                "07_ranges.png",
                "Raw UWB Ranges"
            )
            self.plot_raw(
                "fp",
                "FP RSSI",
                "08_fp_rssi.png",
                "FP RSSI"
            )
            self.plot_raw(
                "rx",
                "RX RSSI",
                "09_rx_rssi.png",
                "RX RSSI"
            )

            summary_path = self.write_summary()

            print("========================================")
            print(
                "UGV%d offline logging completed"
                % self.ugv_id
            )
            print("Output directory:")
            print(self.output_dir)
            print("")
            print("CSV:")
            print(pose_path)
            print(raw_path)
            print(event_path)
            print("")
            print("Summary:")
            print(summary_path)
            print("")
            print("PNG:")
            print("01_trajectory.png")
            print("02_xy_vs_time.png")
            print("03_residual_rms.png")
            print("04_valid.png")
            print("05_used_anchor_count.png")
            print("06_events.png")
            print("07_ranges.png")
            print("08_fp_rssi.png")
            print("09_rx_rssi.png")
            print("========================================")
            print("")

        except Exception as e:
            print(
                "UGV%d offline logger failed:"
                % self.ugv_id
            )
            print(str(e))


if __name__ == "__main__":
    rospy.init_node("offline_logger")
    LocalUGVOfflineLogger()
    rospy.spin()
