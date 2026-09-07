#!/usr/bin/env python3
"""Run after sourcing devel/setup.bash; no ROS master or hardware required."""
import ast
import importlib.util
import math
from pathlib import Path
import threading
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rospy
import yaml
from nlink_parser.msg import LinktrackNode2, LinktrackNodeframe2


PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "localizer", PACKAGE / "scripts/robust_uwb_localizer.py")
localizer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(localizer)


class RealtimeLocalizerTest(unittest.TestCase):
    def setUp(self):
        with (PACKAGE / "config/final_localization.yaml").open() as handle:
            self.params = yaml.safe_load(handle)
        self.params["input_topic"] = "/ugv1/nlink_linktrack_nodeframe2"
        self.now = 100.0
        self.publishers = Mock(side_effect=lambda *a, **kw: Mock())
        self.subscriber = Mock()
        self.timers = Mock()
        replacements = [
            patch.object(localizer, "monotonic", side_effect=lambda: self.now),
            patch.object(rospy.Time, "now", side_effect=lambda: rospy.Time.from_sec(1000.0 + self.now)),
            patch.object(rospy, "get_param", side_effect=lambda key, default=None: self.params.get(key[1:], default)),
            patch.object(rospy, "Publisher", self.publishers),
            patch.object(rospy, "Subscriber", self.subscriber),
            patch.object(rospy, "Timer", self.timers),
            patch.object(rospy, "loginfo"),
            patch.object(rospy, "loginfo_throttle"),
            patch.object(rospy, "logwarn_throttle"),
        ]
        for replacement in replacements:
            replacement.start()
            self.addCleanup(replacement.stop)
        self.node = localizer.RobustUWBLocalizer()

    def frame(self, x=2.0, y=2.0, ids=None):
        msg = LinktrackNodeframe2()
        for aid, anchor in self.node.anchors.items():
            if ids is not None and aid not in ids:
                continue
            node = LinktrackNode2()
            node.id = aid
            node.dis = math.sqrt((x-anchor[0])**2 + (y-anchor[1])**2 +
                                 (self.node.tag_z-anchor[2])**2)
            msg.nodes.append(node)
        return msg

    def tick(self, x=2.0, y=2.0):
        self.node.cb(self.frame(x, y))
        self.node.process_cb(None)

    def valid(self):
        return self.node.valid_pub.publish.call_args[0][0].data

    def test_native_linux_monotonic_clock(self):
        source = ast.parse((PACKAGE / "scripts/robust_uwb_localizer.py").read_text())
        clock_import = next(node for node in source.body if isinstance(node, ast.Try))
        fallback = ast.Module(body=clock_import.handlers[0].body, type_ignores=[])
        namespace = {}
        exec(compile(fallback, "<native monotonic clock>", "exec"), namespace)
        first = namespace["monotonic"]()
        self.assertGreater(first, 0.0)
        self.assertGreaterEqual(namespace["monotonic"](), first)
        namespace["_clock_gettime"] = lambda *args: -1
        with self.assertRaises(OSError):
            namespace["monotonic"]()

    def test_latest_frame_only_and_receive_timestamp(self):
        self.node.cb(self.frame(2.0))
        self.now += 0.02
        self.node.cb(self.frame(2.1))
        self.now += 0.02
        self.node.cb(self.frame(2.2))
        self.assertFalse(self.node.pose_pub.publish.called)
        stamp = rospy.Time.now()
        self.now += 0.03
        self.node.process_cb(None)
        pose = self.node.pose_pub.publish.call_args[0][0]
        self.assertAlmostEqual(pose.pose.position.x, 2.2, places=5)
        self.assertAlmostEqual(pose.pose.position.y, 2.0, places=5)
        self.assertEqual(pose.header.stamp, stamp)
        self.assertTrue(self.valid())
        self.assertEqual(self.node.dropped_frames, 2)
        self.assertEqual(self.node.dropped_pub.publish.call_args[0][0].data, 2)
        self.assertAlmostEqual(self.node.age_pub.publish.call_args[0][0].data, 0.03)
        self.node.process_cb(None)
        self.assertEqual(self.node.pose_pub.publish.call_count, 1)
        self.assertEqual(self.subscriber.call_args.kwargs["queue_size"], 1)
        self.assertEqual(self.subscriber.call_args.kwargs["buff_size"], 2**20)
        self.assertTrue(all(call.kwargs["queue_size"] == 1 for call in self.publishers.call_args_list))
        self.assertAlmostEqual(self.timers.call_args_list[0].args[0].to_sec(), 0.05)

    def test_stale_frame_is_discarded_before_solver(self):
        self.node.cb(self.frame())
        self.now += self.node.max_measurement_age + 0.01
        with patch.object(self.node, "wls", wraps=self.node.wls) as solve:
            self.node.process_cb(None)
            solve.assert_not_called()
        self.assertFalse(self.node.meas)
        self.assertIsNone(self.node.raw_solution)
        self.assertFalse(self.node.pose_pub.publish.called)
        self.assertFalse(self.valid())

    def test_expired_solution_does_not_commit_filter_state(self):
        self.tick()
        old_raw = self.node.raw_solution.copy()
        old_filtered = self.node.filtered.copy()
        old_valid_time = self.node.last_valid_time
        self.now += 0.05
        self.node.cb(self.frame(2.1))
        wls = self.node.wls

        def slow_solve(*args):
            result = wls(*args)
            self.now += self.node.max_measurement_age + 0.01
            return result

        with patch.object(self.node, "wls", side_effect=slow_solve):
            self.node.process_cb(None)
        np.testing.assert_array_equal(self.node.raw_solution, old_raw)
        np.testing.assert_array_equal(self.node.filtered, old_filtered)
        self.assertEqual(self.node.last_valid_time, old_valid_time)
        self.assertEqual(self.node.pose_pub.publish.call_count, 1)
        self.assertFalse(self.valid())
        self.assertGreater(self.node.processing_pub.publish.call_args[0][0].data, 0.15)

    def test_reception_and_watchdog_continue_during_slow_solver(self):
        self.tick()
        self.now += 0.05
        self.node.cb(self.frame(2.1))
        entered, release, received = threading.Event(), threading.Event(), threading.Event()
        errors = []
        wls = self.node.wls

        def blocked_solve(*args):
            entered.set()
            if not release.wait(2.0):
                raise AssertionError("solver was not released")
            return wls(*args)

        def run_solver():
            try:
                self.node.process_cb(None)
            except Exception as error:
                errors.append(error)

        def receive():
            self.node.cb(self.frame(2.2))
            self.node.cb(self.frame(2.25))
            received.set()

        with patch.object(self.node, "wls", side_effect=blocked_solve):
            worker = threading.Thread(target=run_solver)
            receiver = threading.Thread(target=receive)
            worker.start()
            try:
                self.assertTrue(entered.wait(1.0))
                self.now = 100.26
                receiver.start()
                self.assertTrue(received.wait(1.0), "reception blocked behind solver")
                self.node.watchdog_cb(None)
                self.assertFalse(self.valid(), "fresh input masked an expired output")
            finally:
                release.set()
                worker.join(2.0)
                if receiver.ident is not None:
                    receiver.join(2.0)
        self.assertFalse(worker.is_alive())
        self.assertFalse(receiver.is_alive())
        self.assertFalse(errors)
        self.assertEqual(self.node.pose_pub.publish.call_count, 1)
        self.assertFalse(self.valid(), "expired result overrode watchdog")
        self.assertEqual(self.node.dropped_frames, 1)
        self.node.process_cb(None)
        self.assertTrue(self.valid())
        self.assertAlmostEqual(self.node.raw_solution[0], 2.25, places=5)
        self.assertEqual(self.node.pose_pub.publish.call_args[0][0].header.stamp,
                         rospy.Time.from_sec(1100.26))

    def test_cached_anchor_ages_are_checked_after_solver(self):
        self.tick()
        self.now += 0.10
        self.node.cb(self.frame(ids=[0, 4]))
        wls = self.node.wls

        def delayed_solve(*args):
            result = wls(*args)
            self.now += 0.03
            return result

        with patch.object(self.node, "wls", side_effect=delayed_solve):
            self.node.process_cb(None)
        self.assertEqual(self.node.pose_pub.publish.call_count, 1)
        self.assertFalse(self.valid())

    def test_watchdog_after_input_stops_and_recovery(self):
        self.tick()
        self.now += 0.21
        self.node.watchdog_cb(None)
        self.assertFalse(self.valid())
        self.node.process_cb(None)
        self.assertEqual(self.node.pose_pub.publish.call_count, 1)
        self.tick()
        self.assertTrue(self.valid())

    def test_path_rate_limit_and_bounded_immutable_snapshots(self):
        self.params["path_max_points"] = 3
        self.node = localizer.RobustUWBLocalizer()
        self.node.alpha = 1.0
        times = []
        self.node.path_pub.publish.side_effect = lambda _: times.append(self.now)
        for i in range(16):
            self.now = 100.0 + i*0.1
            self.tick(2.0 + i*0.05)
        self.assertEqual(self.node.pose_pub.publish.call_count, 16)
        self.assertEqual(len(self.node.path_poses), 3)
        self.assertEqual(len(times), 4)
        self.assertTrue(all(b-a >= 0.5-1e-9 for a, b in zip(times, times[1:])))
        paths = [call.args[0] for call in self.node.path_pub.publish.call_args_list]
        self.assertEqual(len(paths[0].poses), 1)
        self.assertTrue(all(len(path.poses) <= 3 for path in paths))
        self.assertAlmostEqual(paths[-1].poses[-1].pose.position.x, 2.75, places=5)
        self.node.path_publish_rate = 0.0
        self.now += 0.5
        self.tick(2.8)
        self.assertEqual(len(times), 4)

    def test_filter_time_constant_and_invalid_range(self):
        self.assertAlmostEqual(self.node.filter_alpha(0.02), 0.35)
        self.assertAlmostEqual(self.node.filter_alpha(0.05), 1.0 - 0.65**2.5)
        self.assertAlmostEqual(1.0-self.node.filter_alpha(0.10),
                               (1.0-self.node.filter_alpha(0.05))**2)
        self.tick()
        self.now += 0.20
        msg = self.frame()
        for node in msg.nodes[:3]:
            node.dis = float("nan")
        self.node.cb(msg)
        self.node.process_cb(None)
        self.assertFalse(self.valid())
        self.assertEqual(self.node.pose_pub.publish.call_count, 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
