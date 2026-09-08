#!/usr/bin/env python
"""Deterministic closed-loop tests. No ROS graph or vehicle connections."""
from __future__ import division
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'scripts'))
from leader_tracker import LeaderTracker, check_bounds, parse_points


class TrackingChecks(unittest.TestCase):
    def test_straight_turn_and_initial_alignment(self):
        for points, heading in [([(0, 0), (2, 0)], 0), ([(0, 0), (1, 0), (1, 1)], 0),
                                ([(0, 0), (1, 0)], math.pi),
                                ([(0, 0), (1, 0), (0.1, 0.1)], 0)]:
            tracker = LeaderTracker()
            pose, now = [0.0, 0.0, heading], 0.0
            tracker.start(points, 0.1, 0.4, pose, now)
            errors = []
            for _ in range(1800):
                now += 0.05
                previous_v, previous_w = tracker.v, tracker.w
                v, w = tracker.step(pose, now, 0.15)
                self.assertLessEqual(v, 0.15)
                self.assertLessEqual(abs(w), 0.5)
                if tracker.state == 'ALIGNING':
                    self.assertEqual(v, 0.0)
                if tracker.state == 'DONE':
                    break
                self.assertIn(tracker.state, tracker.ACTIVE)
                self.assertLessEqual(v-previous_v, 0.15*0.05+1e-8)
                self.assertLessEqual(abs(w-previous_w), 0.8*0.05+1e-8)
                pose[0] += v*math.cos(pose[2])*0.05
                pose[1] += v*math.sin(pose[2])*0.05
                pose[2] += w*0.05
                errors.append(tracker.cross_track)
            self.assertEqual(tracker.state, 'DONE', (points, pose, tracker.status()))
            self.assertLessEqual(math.hypot(pose[0]-points[-1][0], pose[1]-points[-1][1]), 0.101)
            self.assertLess(max(errors), 0.25)

    def test_limits_rotation_budget_and_explicit_resume(self):
        tracker = LeaderTracker()
        tracker.start([(0, 0), (2, 0)], 0.4, 0.4, (0, 0, 0), 0)
        for i in range(1, 30):
            v, w = tracker.step((0, 0, 0.3), i*0.05, 0.15, [(0.8, -0.8, 0.15)])
            self.assertLessEqual(math.hypot(v-w*(-0.8), w*0.8), 0.8*0.15+1e-8)
        self.assertEqual(tracker.step((0, 0, 0.3), 1.5, 0.03)[0], 0.03)
        self.assertEqual(tracker.step((0, 0, 0.3), 1.55, 0), (0, 0))
        self.assertEqual(tracker.reason, 'ZERO_LIMIT')
        self.assertEqual(tracker.step((0, 0, 0.3), 1.6, 0.15), (0, 0))
        tracker.resume(1.6)
        self.assertGreater(tracker.step((0, 0, 0.3), 1.65, 0.15)[0], 0)
        self.assertEqual(tracker.step((0.5, 0, 0.3), 1.7, 0.15), (0, 0))
        self.assertEqual(tracker.reason, 'POSE_JUMP')
        self.assertEqual(tracker.step((0.5, 0, 0.3), 1.75, 0.15), (0, 0))
        tracker.stop()
        with self.assertRaises(ValueError):
            tracker.resume(2)
        tracker.start([(0, 0), (0, 1)], 0.1, 0.4, (0, 0, 0), 3)
        for i in range(1, 40):
            v, w = tracker.step((0, 0, 0), 3+i*0.05, 0.15, [(0.8, 0.8, 0.15)])
            self.assertEqual(v, 0)
            self.assertLessEqual(abs(w)*math.hypot(0.8, 0.8), 0.8*0.15+1e-8)

    def test_path_validation_and_local_progress(self):
        self.assertEqual(parse_points('0, 0\n1 0\n1, 1'), [(0, 0), (1, 0), (1, 1)])
        for text in ('', '0, 0', '0, 0\nnan, 1', '0 0 0\n1 1', '0 0\n0.01 0'):
            with self.assertRaises(ValueError):
                parse_points(text)
        check_bounds([(1.5, 2), (3, 2)], (0, 0, 6.4, 4.4), [(0.8, 0.8)])
        with self.assertRaises(ValueError):
            check_bounds([(0.5, 2), (3, 2)], (0, 0, 6.4, 4.4), [(0.8, 0.8)])
        tracker = LeaderTracker()
        tracker.start([(0, 0), (2, 0), (2, 1), (0, 0.1)], 0.1, 0.4, (0, 0, 0), 0)
        tracker.step((0, 0.08, 0), 0.05, 0.15)
        self.assertLessEqual(tracker.progress, 0.4)
        self.assertNotEqual(tracker.state, 'DONE')


if __name__ == '__main__':
    unittest.main(verbosity=2)
