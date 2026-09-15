#!/usr/bin/env python3
"""Offline checks for the standalone IOT calibration workflow."""
import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from iot_calibration import (ANGLES, SessionStore, available_targets, discover_modules,
                             fit_code, linear_fit, prediction_band)


class CalibrationChecks(unittest.TestCase):
    def test_angle_groups_cover_negative_and_positive_sixty_degrees(self):
        self.assertEqual(ANGLES[0], -60)
        self.assertEqual(ANGLES[-1], 60)
        self.assertEqual(len(ANGLES), 25)
        self.assertTrue(all(second - first == 5 for first, second in zip(ANGLES, ANGLES[1:])))

    def test_module_choices_use_main_vehicle_rows_and_uid(self):
        modules = discover_modules({
            '192.168.0.110': {'robot_id': 'ugv1', 'iot_ports': {'uwb_iot': '/dev/u1'}},
            '192.168.0.109': {'robot_id': 'ugv3', 'iot_ports': {
                'uwb_iot': '/dev/right', 'uwb_iot_aux': '/dev/left'}},
            '192.168.0.108': {'robot_id': 'ugv2', 'iot_ports': {'uwb_iot': '/dev/u2'}},
        })
        self.assertEqual({module['uid'] for module in modules},
                         {0x14004E00, 0x2A003200, 0x30006E00, 0x15003B00})
        self.assertEqual({module['port'] for module in modules},
                         {'/dev/u1', '/dev/right', '/dev/left', '/dev/u2'})
        source = next(module for module in modules if module['uid'] == 0x14004E00)
        self.assertEqual({module['uid'] for module in available_targets(modules, source)},
                         {0x2A003200, 0x30006E00, 0x15003B00})
        target = next(module for module in modules if module['uid'] == 0x30006E00)
        self.assertEqual(fit_code(source, target), 'f13')

    def test_supplied_fit_and_prediction_band(self):
        points = [
            {'measured_mean_deg': 6.5563, 'actual_angle_deg': 0},
            {'measured_mean_deg': 14.8223, 'actual_angle_deg': 5},
            {'measured_mean_deg': 22.3621, 'actual_angle_deg': 10},
            {'measured_mean_deg': 35.9828, 'actual_angle_deg': 15},
            {'measured_mean_deg': 44.2294, 'actual_angle_deg': 20},
        ]
        fit = linear_fit(points)
        self.assertAlmostEqual(fit['a'], -2.7138, places=2)
        self.assertAlmostEqual(fit['b'], 0.5128, places=2)
        estimate, low, high = prediction_band(fit, fit['x_average'])
        self.assertLess(low, estimate)
        self.assertLess(estimate, high)
        with self.assertRaises(ValueError):
            linear_fit(points[:2])

    def test_session_store_keeps_empty_frames_and_replaces_angle(self):
        source = {'uid': 1, 'board': 's', 'robot': 'ugv1', 'ip': '1.1.1.1',
                  'sensor': 'uwb_iot', 'port': '/dev/s'}
        target = {'uid': 2, 'board': 't', 'robot': 'ugv2', 'ip': '1.1.1.2',
                  'sensor': 'uwb_iot', 'port': '/dev/t'}
        with tempfile.TemporaryDirectory() as directory:
            store = SessionStore(Path(directory) / 'cal', source, target, '/tmp/settings.json')
            store.write_observation({'capture_id': 'a', 'actual_angle_deg': 0,
                                     'source_uid': 1, 'target_uid': 2,
                                     'target_present': 0, 'valid_angle': 0})
            store.set_point({'actual_angle_deg': 0, 'measured_mean_deg': 1,
                             'measured_std_deg': 0, 'distance_mean_m': .5,
                             'distance_std_m': 0, 'source_frames': 1,
                             'target_present_frames': 0, 'valid_angle_samples': 0,
                             'missing_pct': 100, 'capture_id': 'a', 'captured_at': 'now'})
            store.set_point({'actual_angle_deg': 0, 'measured_mean_deg': 2,
                             'measured_std_deg': 0, 'distance_mean_m': .5,
                             'distance_std_m': 0, 'source_frames': 2,
                             'target_present_frames': 1, 'valid_angle_samples': 1,
                             'missing_pct': 50, 'capture_id': 'b', 'captured_at': 'later'})
            store.close('stopped')
            with (Path(directory) / 'cal' / 'observations.csv').open(newline='') as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]['target_present'], '0')
            data = json.loads((Path(directory) / 'cal' / 'session.json').read_text())
            with (Path(directory) / 'cal' / 'points.csv').open(newline='') as stream:
                points = list(csv.DictReader(stream))
            self.assertEqual(len(data['points']), 1)
            self.assertEqual(len(points), 1)
            self.assertEqual(data['points'][0]['measured_mean_deg'], 2)
            self.assertEqual(data['state'], 'stopped')
            self.assertEqual(data['angles_deg'], list(ANGLES))


if __name__ == '__main__':
    unittest.main()
