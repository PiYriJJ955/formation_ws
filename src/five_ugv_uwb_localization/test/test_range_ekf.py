#!/usr/bin/env python
from __future__ import division
import copy
import csv
import os
import sys
import tempfile
import unittest
import numpy as np
import yaml

PACKAGE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PACKAGE, 'scripts'))
from robust_range_ekf import RangeEKF, FrameClock, predict_motion
from calibrate_uwb import calibrate


class RangeEKFTest(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(PACKAGE, 'config/final_localization.yaml')) as stream:
            self.config = yaml.safe_load(stream)
        self.filter = RangeEKF(self.config)
        self.rng = np.random.RandomState(42)

    def ranges(self, xy=(2, 2), noise=0):
        state = np.r_[xy, 0.0]
        return {i: self.filter.model(i, state)[0]+noise*self.rng.randn() for i in self.filter.anchors}

    def initialize(self):
        for _ in range(12):
            result = self.filter.update(self.ranges())
        self.assertTrue(result['valid'])

    def test_straight_motion_with_alternating_and_persistent_outliers(self):
        self.initialize()
        errors, steps = [], []
        previous = self.filter.x.copy()
        for n in range(500):
            point = (2+0.005*(n+1), 2)
            self.filter.predict(0.05, 0.1, 0, 0)
            ranges = self.ranges(point, 0.06)
            if n < 250:
                ranges[0] += 1.5
                if n % 2 == 0:
                    ranges[4] += 1.5
            result = self.filter.update(ranges)
            self.assertTrue(result['valid'], result)
            errors.append(np.linalg.norm(self.filter.x[:2]-point))
            steps.append(np.linalg.norm(self.filter.x[:2]-previous[:2]))
            previous = self.filter.x.copy()
            self.assertGreaterEqual(min(np.linalg.eigvalsh(self.filter.P)), -1e-10)
        self.assertLess(np.percentile(errors, 95), 0.08)
        self.assertLess(max(steps), 0.06)
        self.assertEqual(result['quarantined_ids'], [])

    def test_stationary_missing_frames_and_link_recovery(self):
        self.initialize()
        rows = []
        for _ in range(100):
            self.filter.predict(0.05, 0, 0, 0)
            result = self.filter.update(self.ranges(noise=0.1))
            rows.append(self.filter.x[:2].copy())
        self.assertLess(np.max(np.ptp(rows, axis=0)), 0.08)
        bad = self.ranges(); bad[0] += 2
        for _ in range(3):
            self.filter.update(bad)
        for _ in range(14):
            result = self.filter.update(self.ranges())
            self.assertIn(0, result['quarantined_ids'])
        result = self.filter.update(self.ranges())
        self.assertNotIn(0, result['quarantined_ids'])
        self.assertLess(result['weights'][0], 0.1)
        result = self.filter.update({0: 1, 1: float('nan')})
        self.assertFalse(result['valid'])
        self.assertEqual(result['accepted_ids'], [])

    def test_bootstrap_and_inconsistent_position(self):
        for _ in range(7):
            self.assertFalse(self.filter.update(self.ranges())['valid'])
        self.assertIsNone(self.filter.x)
        self.initialize()
        before = self.filter.x.copy()
        result = self.filter.update(self.ranges((3, 3)))
        self.assertFalse(result['valid'])
        np.testing.assert_array_equal(before, self.filter.x)

    def test_jacobian_lever_arm_and_fixed_biases(self):
        self.config.update(tag_offset_xy=[0.2, -0.1], range_biases={0: 0.3})
        self.filter = RangeEKF(self.config)
        state = np.array([2., 2., .8])
        r, H = self.filter.model(0, state)
        for i in range(3):
            shifted = state.copy(); shifted[i] += 1e-6
            self.assertAlmostEqual((self.filter.model(0, shifted)[0]-r)/1e-6, H[i], places=5)
        self.assertAlmostEqual(self.filter.ranges({0: r+.3})[0], r)

    def test_clock_duplicate_reorder_wrap_restart_and_backlog(self):
        clock = FrameClock()
        self.assertEqual(clock.accept(2**32-10, 1), (1, False))
        self.assertIsNone(clock.accept(2**32-10, 1.001))
        measured, reset = clock.accept(40, 1.2)
        self.assertLess(abs(measured-1.05), 0.001)
        self.assertFalse(reset)
        self.assertIsNone(clock.accept(30, 1.21))
        self.assertTrue(clock.accept(0, 3)[1])

    def test_causal_motion_and_gap(self):
        self.initialize()
        histories = {'odom': [(1, (0.1, 0, 0)), (1.05, (0.2, 0, 0)), (1.15, (2, 0, 0))],
                     'imu': [(1, (0,)), (1.1, (0,))]}
        self.assertTrue(predict_motion(self.filter, 1, 1.1, histories, .15))
        self.assertAlmostEqual(self.filter.x[0], 2.015)
        self.assertFalse(predict_motion(self.filter, 1.1, 1.4, histories, .15))

    def test_calibration_from_known_tag_references(self):
        root = tempfile.mkdtemp()
        try:
            manifest = {'samples': []}
            for index, xy in enumerate(((1, 1), (2, 1), (3, 3))):
                name = os.path.join(root, '%d.csv' % index)
                with open(name, 'w') as stream:
                    writer = csv.DictWriter(stream, ['range_id%d' % i for i in self.filter.anchors])
                    writer.writeheader()
                    for _ in range(40):
                        writer.writerow({'range_id%d' % i: r+.15 for i, r in self.ranges(xy).items()})
                manifest['samples'].append({'csv': name, 'tag': list(xy)+[.25]})
            result = calibrate(self.config, manifest)
            for v in result['range_biases'].values():
                self.assertAlmostEqual(v, .15)
        finally:
            import shutil
            shutil.rmtree(root)


if __name__ == '__main__':
    unittest.main(verbosity=2)
