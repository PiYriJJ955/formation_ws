#!/usr/bin/env python3
"""Offline IOT discovery, missing-data and GUI configuration regressions."""
import json
from pathlib import Path
import struct
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from fleet_iot import IotHistory, IotWindow, sensors_for, validate_ports
from fleet_iot_remote import identify, monitor
from fleet_console import FleetConsole, load_settings


class IotChecks(unittest.TestCase):
    def test_checksum_protocol_identity_and_truncated_frames(self):
        frame = bytearray(struct.pack('<BBHIIBB', 0x6a, 0, 15, 0x2A003200, 100, 0, 0))
        frame.append(sum(frame) % 256)
        self.assertEqual(identify(b'noise' + frame), {'kind': 'uwb_iot', 'uid': 0x2A003200})
        self.assertEqual(identify(frame[:-1]), {'kind': 'unknown'})
        frame[-1] ^= 1
        self.assertEqual(identify(frame), {'kind': 'unknown'})
        link = bytearray(120)
        link[:4] = struct.pack('<BBH', 0x55, 4, 120)
        link[-1] = sum(link[:-1]) % 256
        self.assertEqual(identify(link), {'kind': 'uwb_linktrack'})
        link[-1] ^= 1
        self.assertEqual(identify(link), {'kind': 'unknown'})

    def test_separate_boards_ports_and_missing_values(self):
        sensors = sensors_for({'robot_id': 'ugv3'})
        self.assertEqual([s['uid'] for s in sensors], [0x30006E00, 0x2A003200])
        self.assertEqual(len(sensors_for({'robot_id': 'ugv2'})), 1)
        validate_ports('/dev/uwb_linktrack', sensors)
        for port in ('/dev/uwb_iot', 'ttyUSB1', '/dev/x\n'):
            with self.assertRaises(ValueError):
                validate_ports(port, sensors)
        sensors[1]['port'] = sensors[0]['port']
        with self.assertRaises(ValueError):
            validate_ports('/dev/uwb_linktrack', sensors)
        history = IotHistory()
        frame = dict(uid=0x2A003200, system_time=100, nodes=[dict(uid=0x15003B00, distance=.7, horizontal=51)])
        history.add(frame, 1.)
        history.add(dict(frame, nodes=[], system_time=200), 1.1)
        points = history.links[(0x2A003200, 0x15003B00)]
        self.assertEqual(list(points), [(1., .7, 51), (1.1, None, None)])
        self.assertEqual(history.counts[(0x2A003200, 0x15003B00)], 1)
        self.assertFalse(history.links[(0x15003B00, 0x2A003200)])

    def test_remote_rejects_wrong_vehicle_before_launch(self):
        # ROS imports are lazy; fake imports isolate this preflight from installed ROS.
        modules = {'rospy': SimpleNamespace(), 'rosbag': SimpleNamespace(),
                   'nlink_parser.msg': SimpleNamespace(IotFrame0=object)}
        with patch.dict(sys.modules, modules), patch.dict('os.environ', UGV_ID='2'), \
             patch('fleet_iot_remote.subprocess.Popen') as start:
            with self.assertRaisesRegex(RuntimeError, 'UGV_ID mismatch'):
                monitor(SimpleNamespace(robot='ugv3'))
            start.assert_not_called()
        sensors = sensors_for({'robot_id': 'ugv3'})
        args = SimpleNamespace(robot='ugv3', sensors=json.dumps(sensors))
        for result in ({'kind': 'uwb_linktrack'}, {'kind': 'uwb_iot', 'uid': 0x15003B00}):
            with patch.dict(sys.modules, modules), patch.dict('os.environ', UGV_ID='3'), \
                 patch('fleet_iot_remote.glob.glob', return_value=[]), \
                 patch('fleet_iot_remote.probe_port', return_value=result), \
                 patch('fleet_iot_remote.subprocess.Popen') as start:
                with self.assertRaises(RuntimeError):
                    monitor(args)
                start.assert_not_called()

    def test_gui_separate_config_and_live_chart(self):
        import tkinter as tk
        try:
            root = tk.Tk()
        except tk.TclError:
            self.skipTest('No desktop session')
        root.withdraw()
        with tempfile.TemporaryDirectory() as directory, \
             patch('fleet_console.connect_ssh', side_effect=AssertionError('Offline test')):
            app = FleetConsole(root, Path(directory) / 'settings.json')
            wb = app.workbench
            ip = '192.168.0.109'
            try:
                wb.vehicles.selection_set([ip])
                editor = wb.edit_vehicle()
                editor.iot_fields['UWB_PORT'].set('/dev/uwb_linktrack')
                editor.iot_fields['uwb_iot'].set('/dev/serial/by-id/right')
                editor.iot_fields['uwb_iot_aux'].set('/dev/serial/by-id/left')
                editor.iot_save()
                _, robots = load_settings(app.config_path)
                row = robots[ip]
                self.assertEqual(row['formation_config']['UWB_PORT'], '/dev/uwb_linktrack')
                self.assertNotIn('uwb_iot', row['formation_config'])
                self.assertEqual(row['iot_ports']['uwb_iot_aux'], '/dev/serial/by-id/left')
                session = SimpleNamespace(row=row, ip=ip, stop=threading.Event(), thread=SimpleNamespace(is_alive=lambda: False))
                with patch('fleet_iot.IotSession', return_value=session):
                    wb.open_iot()
                window = wb.iot_window
                first = window.dialog
                wb.open_iot()
                self.assertIs(wb.iot_window.dialog, first)
                now = time.monotonic()
                window.events.put((ip, dict(event='started', directory='/tmp/iot', sensors=sensors_for(row))))
                window.events.put((ip, dict(event='frame', uid=0x2A003200, system_time=100,
                    host_monotonic=now, nodes=[dict(uid=0x15003B00, distance=.7, horizontal=51)])))
                root.update()
                window.poll()
                key = '%d:%d' % (0x2A003200, 0x15003B00)
                self.assertEqual(window.link_menu.item(key)['text'], 'ugv3_left → ugv2')
                self.assertFalse(window.link_menu['columns'])
                self.assertEqual(str(window.link_menu['selectmode']), 'extended')
                self.assertLess(window.link_menu.winfo_rootx(), window.canvas.winfo_rootx())
                self.assertGreater(window.canvas.winfo_width(), window.link_menu.winfo_width())
                self.assertTrue(window.canvas.find_all())
                window.events.put((ip, dict(event='frame', uid=0x30006E00, system_time=100,
                    host_monotonic=now, nodes=[dict(uid=0x30003000, distance=.9, horizontal=15)])))
                window.poll()
                chart_colors = {window.canvas.itemcget(item, 'fill') for item in window.canvas.find_all()}
                self.assertTrue({'#1976d2', '#009688', '#d32f2f'} <= chart_colors)
                window.link_menu.selection_set([key])
                window.link_menu.event_generate('<<TreeviewSelect>>')
                chart_colors = {window.canvas.itemcget(item, 'fill') for item in window.canvas.find_all()}
                self.assertIn('#1976d2', chart_colors)
                self.assertNotIn('#009688', chart_colors)
                window.link_menu.selection_add(['%d:%d' % (0x30006E00, 0x30003000)])
                window.link_menu.event_generate('<<TreeviewSelect>>')
                chart_colors = {window.canvas.itemcget(item, 'fill') for item in window.canvas.find_all()}
                self.assertTrue({'#1976d2', '#009688'} <= chart_colors)
                window.events.put((ip, dict(event='frame', uid=0x2A003200, system_time=200, nodes=[])))
                window.poll()
                self.assertEqual(window.history.links[(0x2A003200, 0x15003B00)][-1][1:], (None, None))
                window.link_menu.selection_remove(window.link_menu.selection())
                window.link_menu.event_generate('<<TreeviewSelect>>')
                self.assertEqual([window.canvas.itemcget(item, 'text') for item in window.canvas.find_all()],
                                 ['请选择左侧有向链路'])
                window.close()
                self.assertTrue(session.stop.is_set())
                self.assertTrue(window.closed)
            finally:
                for timer in root.tk.splitlist(root.tk.call('after', 'info')):
                    root.after_cancel(timer)
                root.destroy()


if __name__ == '__main__':
    unittest.main()
