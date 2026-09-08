#!/usr/bin/env python3
"""Run after sourcing scripts/env.sh; no ROS master or hardware required."""
import ast
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch
import numpy as np
import rospy
import yaml
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from nlink_parser.msg import LinktrackNode2, LinktrackNodeframe2

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'scripts'))
import robust_uwb_localizer as localizer
import offline_logger


class RealtimeLocalizerTest(unittest.TestCase):
    def setUp(self):
        self.params = yaml.safe_load((PACKAGE / 'config/final_localization.yaml').read_text())
        self.params.update(input_topic='/ugv1/nlink_linktrack_nodeframe2', calibration_file='/nonexistent')
        self.now = 100.0
        replacements = [
            patch.object(localizer, 'monotonic', side_effect=lambda: self.now),
            patch.object(rospy.Time, 'now', side_effect=lambda: rospy.Time.from_sec(1000+self.now)),
            patch.object(rospy, 'get_param', side_effect=lambda key, default=None:
                         self.params if key == '~' else self.params.get(key.lstrip('~'), default)),
            patch.object(rospy, 'set_param'),
            patch.object(rospy, 'Publisher', side_effect=lambda *a, **kw: Mock()),
            patch.object(rospy, 'Subscriber'), patch.object(rospy, 'Timer'), patch.object(rospy, 'on_shutdown'),
            patch.object(rospy, 'loginfo'), patch.object(rospy, 'logerr_throttle'),
        ]
        for replacement in replacements:
            replacement.start(); self.addCleanup(replacement.stop)
        self.node = localizer.RobustUWBLocalizer()

    def frame(self, x=2, ids=None):
        msg = LinktrackNodeframe2(); msg.local_time = int(round(self.now*1000))
        for aid in self.node.ekf.anchors:
            if ids is None or aid in ids:
                node = LinktrackNode2(); node.id = aid
                node.dis = self.node.ekf.model(aid, np.array([x, 2, 0]))[0]
                msg.nodes.append(node)
        return msg

    def tick(self, x=2, ids=None, motion=True):
        if motion:
            for kind, msg in [('odom', Odometry()), ('imu', Imu())]:
                msg.header.stamp = rospy.Time.now()
                self.node.motion_cb(msg, kind)
        self.node.cb(self.frame(x, ids)); self.node.process_cb(None)

    def initialize(self):
        for _ in range(14):
            self.tick(); self.node.watchdog_cb(None); self.now += .05
        self.assertTrue(self.valid())

    def valid(self):
        return self.node.valid_pub.publish.call_args.args[0].data

    def status(self):
        return self.node.status_pub.publish.call_args.args[0].data

    def test_restart_parameter_snapshot_is_not_recursive(self):
        self.params["effective_config"] = {"previous": True}
        node = localizer.RobustUWBLocalizer()
        self.assertNotIn("effective_config", node.config)

    def test_native_monotonic(self):
        source = ast.parse((PACKAGE / 'scripts/robust_uwb_localizer.py').read_text())
        clock = next(n for n in source.body if isinstance(n, ast.Try))
        namespace = {}
        exec(compile(ast.Module(body=clock.handlers[0].body, type_ignores=[]), '<clock>', 'exec'), namespace)
        first = namespace['monotonic']()
        self.assertGreaterEqual(namespace['monotonic'](), first)
        namespace['_clock_gettime'] = lambda *args: -1
        with self.assertRaises(OSError): namespace['monotonic']()

    def test_latest_only_duplicate_and_missing_anchors(self):
        self.initialize()
        count = self.node.pose_pub.publish.call_count
        self.node.cb(self.frame()); self.node.cb(self.frame())
        self.now += .02; self.node.cb(self.frame())
        self.node.process_cb(None)
        self.assertEqual(self.node.pose_pub.publish.call_count, count+1)
        self.assertEqual(self.node.dropped_frames, 2)
        self.now += .05; self.tick(ids=[0, 4])
        self.assertFalse(self.valid())
        self.assertEqual(self.status(), 'INSUFFICIENT_ANCHORS')

    def test_stale_frame_and_transactional_expiry(self):
        self.initialize()
        before = self.node.ekf.x.copy(); P = self.node.ekf.P.copy()
        self.node.cb(self.frame()); self.now += .2
        self.node.process_cb(None)
        self.assertFalse(self.valid()); np.testing.assert_array_equal(self.node.ekf.x, before)
        self.tick()  # Motion gap latches a fault; construct a clean test node.
        self.node = localizer.RobustUWBLocalizer(); self.initialize()
        before = self.node.ekf.x.copy(); P = self.node.ekf.P.copy()
        old_time = self.node.filter_time
        update = localizer.RangeEKF.update
        def slow(obj, ranges):
            result = update(obj, ranges); self.now += .2; return result
        with patch.object(localizer.RangeEKF, 'update', slow): self.tick(2.02)
        np.testing.assert_array_equal(self.node.ekf.x, before)
        np.testing.assert_array_equal(self.node.ekf.P, P)
        self.assertEqual(old_time, self.node.filter_time)
        self.assertFalse(self.valid())

    def test_watchdog_and_receive_not_blocked_by_solver(self):
        self.initialize()
        count = self.node.pose_pub.publish.call_count
        entered, release = threading.Event(), threading.Event()
        update = localizer.RangeEKF.update
        def blocked(obj, ranges):
            entered.set(); self.assertTrue(release.wait(2)); return update(obj, ranges)
        with patch.object(localizer.RangeEKF, 'update', blocked):
            worker = threading.Thread(target=self.tick); worker.start()
            try:
                self.assertTrue(entered.wait(1))
                self.now += .21
                self.node.cb(self.frame())
                self.node.watchdog_cb(None)
                self.assertFalse(self.valid())
            finally:
                release.set(); worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertEqual(count, self.node.pose_pub.publish.call_count)
        self.assertFalse(self.valid())

    def test_motion_gap_requires_restart_and_never_resumes_bad_heading(self):
        self.initialize(); self.now += .3; self.tick(motion=False)
        self.assertFalse(self.valid()); self.assertEqual(self.status(), 'RESTART_REQUIRED')
        self.now += .05; self.tick()
        self.assertFalse(self.valid()); self.assertEqual(self.status(), 'RESTART_REQUIRED')

    def test_path_bounded_and_covariance(self):
        self.params.update(path_max_points=3, path_min_distance=0)
        self.node = localizer.RobustUWBLocalizer(); self.initialize()
        for _ in range(30): self.tick(); self.now += .05
        self.assertEqual(len(self.node.path_poses), 3)
        calls = self.node.path_pub.publish.call_args_list
        self.assertTrue(all(len(c.args[0].poses) <= 3 for c in calls))
        covariance = np.array(self.node.covariance_pub.publish.call_args.args[0].pose.covariance).reshape(6, 6)
        self.assertTrue(np.all(np.linalg.eigvalsh(covariance) >= 0))

    def test_logger_flushes_without_shutdown_and_uses_config(self):
        with tempfile.TemporaryDirectory() as root:
            self.params['output_root'] = root
            logger = offline_logger.LocalUGVOfflineLogger()
            logger.raw_cb(self.frame())
            self.initialize()
            logger.diagnostics_cb(self.node.diagnostics_pub.publish.call_args.args[0])
            logger.motion_cb(Odometry(), 'odom'); logger.motion_cb(Imu(), 'imu')
            logger.flush(None)
            output = Path(logger.output_dir)
            for name in ('raw_linktrack.csv', 'localization.csv', 'odom.csv', 'imu.csv'):
                self.assertEqual(len((output/name).read_text().splitlines()), 2)
            self.assertEqual(len((output/'diagnostics.jsonl').read_text().splitlines()), 1)
            # Exit output may fail; all handles must have closed before printing.
            with patch('builtins.print', side_effect=BrokenPipeError):
                try: logger.finish()
                except BrokenPipeError: pass
            self.assertTrue(all(s.closed for s in logger.files.values()))
            logger.raw_cb(self.frame())
            self.assertEqual(len((output/'raw_linktrack.csv').read_text().splitlines()), 2)


if __name__ == '__main__':
    unittest.main(verbosity=2)
