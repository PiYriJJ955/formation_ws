#!/usr/bin/env python3
"""Exercise the LinkTrack adapter with ROS message/clock doubles, without ROS."""
import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


class Stamp(float):
    def __sub__(self, other):
        return Stamp(float(self) - float(other))

    def to_sec(self):
        return float(self)


def pose_message():
    return SimpleNamespace(header=SimpleNamespace(), pose=SimpleNamespace(
        position=SimpleNamespace(), orientation=SimpleNamespace()))


class NLinkLocalizerChecks(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.ros = Mock()
        self.ros.get_param.side_effect = lambda _key, default: default
        self.ros.Time.now.side_effect = lambda: Stamp(self.now)
        self.ros.Publisher.side_effect = lambda *args, **kwargs: Mock()
        modules = {'rospy': self.ros,
                   'geometry_msgs.msg': SimpleNamespace(PoseStamped=pose_message),
                   'nlink_parser.msg': SimpleNamespace(LinktrackNodeframe2=SimpleNamespace),
                   'std_msgs.msg': SimpleNamespace(Bool=SimpleNamespace, String=SimpleNamespace)}
        path = Path(__file__).resolve().parents[1] / 'src/five_ugv_uwb_localization/scripts/nlink_localizer.py'
        spec = importlib.util.spec_from_file_location('adapter_under_test', str(path))
        self.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(self.module)
        self.node = self.module.NLinkLocalizer()

    def message(self, position=(1.2, 2.3, 0.25), quaternion=(2, 0, 0, 2)):
        return SimpleNamespace(pos_3d=position, quaternion=quaternion)

    def valid(self):
        return self.node.valid_pub.publish.call_args.args[0].data

    def test_position_and_normalized_wxyz_quaternion_reach_formation(self):
        self.assertFalse(self.valid())
        self.node.update(self.message())
        self.assertTrue(self.valid())
        pose = self.node.pose_pub.publish.call_args.args[0]
        self.assertEqual(vars(pose.pose.position), dict(x=1.2, y=2.3, z=0.25))
        self.assertEqual(pose.header.frame_id, 'linktrack_map')
        self.assertEqual(pose.header.stamp, Stamp(100))
        self.assertAlmostEqual(pose.pose.orientation.w, math.sqrt(0.5))
        self.assertAlmostEqual(pose.pose.orientation.z, math.sqrt(0.5))
        self.assertEqual(pose.pose.orientation.x, 0)
        self.assertEqual(pose.pose.orientation.y, 0)

    def test_invalid_frame_does_not_publish_pose_or_refresh_freshness(self):
        for message in (self.message(position=(float('nan'), 0, 0)),
                        self.message(quaternion=(1, float('inf'), 0, 0)),
                        self.message(quaternion=(0, 0, 0, 0))):
            with self.subTest(message=message):
                self.node.update(self.message())
                last = self.node.last
                count = self.node.pose_pub.publish.call_count
                self.now += 0.1
                self.node.update(message)
                self.node.watchdog(None)
                self.assertFalse(self.valid())
                self.assertEqual(self.node.last, last)
                self.assertEqual(self.node.pose_pub.publish.call_count, count)
        self.node.update(self.message())
        self.assertTrue(self.valid())

    def test_timeout_and_clock_rollback_invalidate_until_new_frame(self):
        for elapsed in (0.5, -1):
            with self.subTest(elapsed=elapsed):
                self.node.update(self.message())
                self.now += elapsed
                self.node.watchdog(None)
                self.assertFalse(self.valid())
                self.assertEqual(self.node.status_pub.publish.call_args.args[0].data, 'LINKTRACK_TIMEOUT')
                self.node.update(self.message())
                self.assertTrue(self.valid())

    def test_first_frame_during_subscription_is_not_overwritten_by_initial_state(self):
        self.ros.Subscriber.side_effect = lambda topic, kind, callback, **kwargs: callback(self.message())
        self.node = self.module.NLinkLocalizer()
        self.assertTrue(self.valid())

    def test_invalid_timeout_is_rejected(self):
        for timeout in (0, -1, float('nan'), float('inf')):
            self.ros.get_param.side_effect = lambda key, default: timeout if key == '~timeout' else default
            with self.assertRaises(ValueError):
                self.module.NLinkLocalizer()


if __name__ == '__main__':
    unittest.main(verbosity=2)
