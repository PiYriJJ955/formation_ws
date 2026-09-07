#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import csv
import math
import os
import time

import rospy

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64, Int32, Int32MultiArray
from nlink_parser.msg import LinktrackNodeframe2


class LocalUGVOfflineLogger(object):
    """
    Per-UGV offline logger.

    Run one copy under each UGV namespace:
      /ugv0/offline_logger
      /ugv1/offline_logger
      /ugv2/offline_logger

    It records only THIS vehicle:
      uwb/pose
      uwb/valid
      uwb/residual_rms
      uwb/full_residual_rms
      uwb/used_anchor_count
      uwb/jump_rejected_ids
      uwb/loo_excluded_id
      nlink_linktrack_nodeframe2 raw ranges/RSSI

    No real-time plotting.
    Ctrl+C -> save CSV + generate PNG.
    """

    def __init__(self):
        self.ugv_id = int(rospy.get_param("~ugv_id", 0))
        self.output_root = os.path.expanduser(
            rospy.get_param(
                "~output_root",
                "~/formation_ws/logs/uwb"
            )
        )
        self.duration = float(rospy.get_param("~duration", 0.0))

        self.anchor_order = [0, 4, 1, 2, 5, 3]
        self.anchor_xy = {
            0: (0.0, 0.0),
            4: (3.2, 0.0),
            1: (6.4, 0.0),
            2: (6.4, 4.4),
            5: (3.2, 4.4),
            3: (0.0, 4.4)
        }

        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.output_dir = os.path.join(
            self.output_root,
            "ugv%d_test_%s" % (self.ugv_id, stamp)
        )

        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

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

        # Relative names: namespace automatically resolves to /ugvX/...
        rospy.Subscriber(
            "uwb/pose",
            PoseStamped,
            self.pose_cb,
            queue_size=100
        )
        rospy.Subscriber(
            "uwb/valid",
            Bool,
            self.valid_cb,
            queue_size=100
        )
        rospy.Subscriber(
            "uwb/residual_rms",
            Float64,
            self.rms_cb,
            queue_size=100
        )
        rospy.Subscriber(
            "uwb/full_residual_rms",
            Float64,
            self.full_rms_cb,
            queue_size=100
        )
        rospy.Subscriber(
            "uwb/used_anchor_count",
            Int32,
            self.count_cb,
            queue_size=100
        )
        rospy.Subscriber(
            "uwb/jump_rejected_ids",
            Int32MultiArray,
            self.jump_cb,
            queue_size=100
        )
        rospy.Subscriber(
            "uwb/loo_excluded_id",
            Int32,
            self.loo_cb,
            queue_size=100
        )
        rospy.Subscriber(
            "nlink_linktrack_nodeframe2",
            LinktrackNodeframe2,
            self.raw_cb,
            queue_size=100
        )

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
        rospy.loginfo("No real-time plotting. Ctrl+C to save and plot.")

    def t(self):
        return (rospy.Time.now() - self.start).to_sec()

    def auto_stop(self, _event):
        rospy.signal_shutdown("offline logger duration reached")

    def finite(self, v):
        try:
            return not (math.isnan(v) or math.isinf(v))
        except Exception:
            return False

    def valid_cb(self, msg):
        self.latest_valid = bool(msg.data)

    def rms_cb(self, msg):
        self.latest_rms = float(msg.data)

    def full_rms_cb(self, msg):
        self.latest_full_rms = float(msg.data)

    def count_cb(self, msg):
        self.latest_count = int(msg.data)

    def jump_cb(self, msg):
        self.latest_jump_ids = [int(v) for v in msg.data]

        if len(self.latest_jump_ids) > 0:
            tt = self.t()
            for aid in self.latest_jump_ids:
                self.event_rows.append({
                    "t": tt,
                    "type": "jump",
                    "anchor_id": aid
                })

    def loo_cb(self, msg):
        self.latest_loo_id = int(msg.data)

        if self.latest_loo_id >= 0:
            self.event_rows.append({
                "t": self.t(),
                "type": "loo",
                "anchor_id": self.latest_loo_id
            })

    def pose_cb(self, msg):
        self.pose_rows.append({
            "t": self.t(),
            "x": float(msg.pose.position.x),
            "y": float(msg.pose.position.y),
            "valid": int(self.latest_valid),
            "rms": self.latest_rms,
            "full_rms": self.latest_full_rms,
            "count": self.latest_count,
            "jump_ids": ",".join(
                [str(v) for v in self.latest_jump_ids]
            ),
            "loo_id": self.latest_loo_id
        })

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

        self.raw_rows.append(row)

    def write_csvs(self):
        pose_path = os.path.join(
            self.output_dir,
            "localization.csv"
        )

        with open(pose_path, "w") as f:
            w = csv.writer(f)
            w.writerow([
                "t",
                "x",
                "y",
                "valid",
                "residual_rms",
                "full_residual_rms",
                "used_anchor_count",
                "jump_rejected_ids",
                "loo_excluded_id"
            ])

            for r in self.pose_rows:
                w.writerow([
                    r["t"],
                    r["x"],
                    r["y"],
                    r["valid"],
                    r["rms"],
                    r["full_rms"],
                    r["count"],
                    r["jump_ids"],
                    r["loo_id"]
                ])

        raw_path = os.path.join(
            self.output_dir,
            "raw_linktrack.csv"
        )

        with open(raw_path, "w") as f:
            w = csv.writer(f)

            header = ["t"]
            header += [
                "range_id%d" % aid
                for aid in self.anchor_order
            ]
            header += [
                "fp_rssi_id%d" % aid
                for aid in self.anchor_order
            ]
            header += [
                "rx_rssi_id%d" % aid
                for aid in self.anchor_order
            ]

            w.writerow(header)

            for r in self.raw_rows:
                out = [r["t"]]
                out += [
                    r["r%d" % aid]
                    for aid in self.anchor_order
                ]
                out += [
                    r["fp%d" % aid]
                    for aid in self.anchor_order
                ]
                out += [
                    r["rx%d" % aid]
                    for aid in self.anchor_order
                ]
                w.writerow(out)

        event_path = os.path.join(
            self.output_dir,
            "events.csv"
        )

        with open(event_path, "w") as f:
            w = csv.writer(f)
            w.writerow([
                "t",
                "event_type",
                "anchor_id"
            ])

            for r in self.event_rows:
                w.writerow([
                    r["t"],
                    r["type"],
                    r["anchor_id"]
                ])

        return pose_path, raw_path, event_path

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
        plt.ylim(0, 6.5)
        plt.yticks(range(0, 7))
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
            "UGV%d Jump / LOO Events" % self.ugv_id
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
                if e["type"] == "jump":
                    jump_counts[e["anchor_id"]] += 1
                elif e["type"] == "loo":
                    loo_counts[e["anchor_id"]] += 1

            f.write("\nJump rejection counts:\n")
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
        if getattr(self, "_finished", False):
            return

        self._finished = True

        print("")
        print(
            "UGV%d offline logger stopping..."
            % self.ugv_id
        )

        try:
            pose_path, raw_path, event_path = self.write_csvs()

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
