#!/usr/bin/env python3
"""Offline GUI / orchestration regressions. Never connects to a vehicle."""
import json
import os
from pathlib import Path
import queue
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from fleet_console import DEFAULTS, FleetConsole, RobotSession, load_settings
from fleet_deploy import parse_robot_config, replace_setting, robot_number, update_identity
from fleet_terminal import Terminal
from fleet_workbench import (FleetWorkbench, Monitor, launch_command, localization_config,
                             saved_data_timeout, saved_linear_limit, session_limits)

DEFAULTS = dict(DEFAULTS, password="test-secret")


class WorkbenchChecks(unittest.TestCase):
    def test_uniform_limit_reaches_all_launches_and_migrates_settings(self):
        options = dict(DEFAULTS, linear_limits_json=json.dumps({'192.0.2.1': 0.08, '192.0.2.2': 0.15}))
        self.assertEqual(saved_linear_limit(DEFAULTS), 0.15)
        self.assertEqual(session_limits(options, {'192.0.2.1': 4, '192.0.2.2': 2}), {'4': 0.08, '2': 0.08})
        self.assertIn('max_linear:=0.08', launch_command('follower', options, '192.0.2.1', 'ugv4', '/tmp/stage'))
        options['linear_limit'] = '0.12'
        self.assertEqual(session_limits(options, {'192.0.2.99': 7}), {'7': 0.12})  # Newly discovered car.
        self.assertIn('max_linear:=0.12', launch_command('follower', options, '192.0.2.2', 'ugv2', '/tmp/stage'))
        for value in (-1, 0.51, float('nan'), float('inf'), True, 'text'):
            with self.assertRaises(ValueError):
                saved_linear_limit(dict(options, linear_limit=value))

    def test_data_timeout_is_configurable_and_reaches_follower(self):
        self.assertEqual(saved_data_timeout(DEFAULTS), 0.6)
        options = dict(DEFAULTS, data_timeout='0.85')
        self.assertIn('data_timeout:=0.85',
                      launch_command('follower', options, '192.0.2.1', 'ugv4', '/tmp/stage'))
        for value in (0, -1, float('nan'), float('inf'), True, 'text'):
            with self.assertRaises((TypeError, ValueError)):
                saved_data_timeout(dict(options, data_timeout=value))

    def test_uwb_cannot_share_chassis_serial_through_alias(self):
        from fleet_ros import check_serial
        with tempfile.TemporaryDirectory() as directory:
            alias = Path(directory) / 'uwb'
            alias.symlink_to('/dev/wheeltec_controller')
            with patch.dict(os.environ, UWB_PORT=str(alias)):
                with self.assertRaisesRegex(RuntimeError, 'UWB_PORT points to the chassis'):
                    check_serial()

    def test_quick_start_orders_steps_and_aborts_on_unready_localization(self):
        workbench = FleetWorkbench.__new__(FleetWorkbench)
        workbench.app = SimpleNamespace(sessions={})
        workbench.events, workbench.cancel = queue.Queue(), threading.Event()
        workbench.monitor = None
        rows = {'192.168.0.106': {'robot_id':'ugv1'}, '192.168.0.108': {'robot_id':'ugv2'},
                '192.168.0.109': {'robot_id':'ugv3'}}
        clients = {ip: SimpleNamespace(close=lambda: None) for ip in rows}
        calls = []
        monitor = SimpleNamespace(ready=True)
        def run(client, command, *args, **kwargs):
            if 'master --timeout 0.5' in command:
                raise RuntimeError('start a local master')
        with patch.object(workbench, 'connect', side_effect=lambda ip, _: clients[ip]), \
             patch.object(workbench, 'stage', return_value='/tmp/stage'), \
             patch.object(workbench, 'launch', side_effect=lambda step, ip, *a: calls.append((step, ip))), \
             patch('fleet_workbench.read_robot_config', return_value={'robot_id':'ugv0'}), \
             patch('fleet_workbench.update_config', side_effect=lambda client, values: values), \
             patch('fleet_workbench.run_remote', side_effect=run), \
             patch('fleet_workbench.Monitor', return_value=monitor):
            workbench.run_start('all', DEFAULTS, rows, localization_config())
            self.assertEqual([step for step, _ in calls], ['master','chassis','chassis','chassis','follower','follower'])
            self.assertNotIn(('follower', '192.168.0.106'), calls)
            self.assertIs(workbench.monitor, monitor)
            calls.clear()
            workbench.monitor = None
            with patch.object(workbench, 'wait_ready', side_effect=RuntimeError('UWB invalid')):
                workbench.run_start('all', DEFAULTS, rows, localization_config())
            self.assertNotIn('follower', [step for step, _ in calls])
            self.assertIsNone(workbench.monitor)
        self.assertTrue(any('UWB invalid' in str(event) for event in list(workbench.events.queue)))
        failed_terminal = SimpleNamespace(status=lambda: {'state': '失败', 'error': 'serial occupied'})
        with self.assertRaisesRegex(RuntimeError, 'serial occupied'):
            workbench.wait_ready(clients['192.168.0.106'], DEFAULTS, '192.168.0.106', 'ugv0',
                                 '/tmp/stage', 'chassis', failed_terminal)

    def test_gui_heartbeat_failure_closes_shared_monitor(self):
        closed = []
        channel = SimpleNamespace(set_combine_stderr=lambda *_: None, settimeout=lambda *_: None,
                                  exec_command=lambda *_: None, sendall=lambda *_: None,
                                  shutdown_write=lambda: None, exit_status_ready=lambda: True,
                                  close=lambda: closed.append('channel'))
        client = SimpleNamespace(get_transport=lambda: SimpleNamespace(open_session=lambda **_: channel),
                                 close=lambda: closed.append('client'))
        events = queue.Queue()
        with patch('fleet_workbench.threading.Thread.start'):
            monitor = Monitor(client, 'unused', events)
        monitor.ui_heartbeat = time.monotonic() - 1
        monitor.run()
        self.assertIn('心跳超时', events.get_nowait()[1])
        self.assertEqual(closed, ['channel', 'client'])

    def test_individual_steps_check_configuration_only_when_requested(self):
        workbench = FleetWorkbench.__new__(FleetWorkbench)
        workbench.app = SimpleNamespace(sessions={})
        workbench.cancel = threading.Event()
        leader, follower, master = '192.0.2.1', '192.0.2.2', '192.0.2.3'
        options = dict(DEFAULTS, master_ip=master)
        rows = {leader: {'robot_id': 'ugv1'},
                follower: {'robot_id': 'ugv2', 'offset_x': '-1.2', 'offset_y': '0.4',
                           'formation_config': {'UWB_PORT': '/dev/ttyUSB7', 'UGV_OFFSET_X': '-2.0'}}}
        clients = {ip: SimpleNamespace(close=lambda: None) for ip in (leader, follower, master)}
        for step, hosts, launches in [
                ('config', [leader, follower, master], []),
                ('master', [master], [('master', master)]),
                ('chassis', [leader, follower, master], [('chassis', leader), ('chassis', follower)]),
                ('follower', [follower, master], [('follower', follower)]),
                ('monitor', [master], [])]:
            with self.subTest(step=step):
                workbench.events, workbench.monitor = queue.Queue(), None
                def run(client, command, *args, **kwargs):
                    if 'master --timeout 0.5' in command:
                        raise RuntimeError('Master not yet running')
                with patch.object(workbench, 'connect', side_effect=lambda ip, _: clients[ip]) as connect, \
                     patch.object(workbench, 'stage', return_value='/tmp/stage'), \
                     patch.object(workbench, 'launch') as launch, \
                     patch.object(workbench, 'wait_ready') as ready, \
                     patch('fleet_workbench.read_robot_config', return_value={'robot_id': 'ugv0'}) as read, \
                     patch('fleet_workbench.update_config', return_value={}) as update, \
                     patch('fleet_workbench.run_remote', side_effect=run) as remote, \
                     patch('fleet_workbench.Monitor', return_value=SimpleNamespace(ready=True)) as monitor:
                    workbench.run_start(step, options, rows, localization_config())
                    self.assertEqual([call[0][0] for call in connect.call_args_list], hosts)
                    self.assertEqual([call[0][:2] for call in launch.call_args_list], launches)
                    self.assertEqual(ready.call_count, len(launches) if step != 'master' else 0)
                    events = list(workbench.events.queue)
                    self.assertFalse(any('失败' in str(event) for event in events), events)
                    self.assertEqual(events[-1][0], 'done')
                    if step == 'config':
                        self.assertEqual(read.call_count, 2)
                        self.assertEqual(update.call_count, 2)
                        self.assertEqual(update.call_args_list[1][0][1]['UWB_PORT'], '/dev/ttyUSB7')
                        self.assertEqual(update.call_args_list[1][0][1]['UGV_OFFSET_X'], '-2.0')
                        remote.assert_not_called()
                        monitor.assert_not_called()
                        self.assertIn('配置检查完成', events[-1][1])
                    else:
                        read.assert_not_called()
                        update.assert_not_called()
                        self.assertFalse(any('SSH 配置检查中' in str(event) for event in events))
                        if step in ('chassis', 'monitor'):
                            monitor.assert_called_once()
                            self.assertIn('--offsets \'{"2": [-1.2, 0.4]}\'', monitor.call_args[0][1])
                            self.assertIn('--data-timeout 0.6', monitor.call_args[0][1])
                        else:
                            monitor.assert_not_called()

    def test_identity_write_readback_and_restart_record(self):
        text = 'export UGV_ID=2\nexport UWB_PORT=/dev/ttyUSB2\nexport UGV_OFFSET_Y=-0.8\nexport UGV_ID=3\n'
        updated = replace_setting(text, 'UGV_ID', 7)
        self.assertEqual(updated.count('export UGV_ID='), 1)
        self.assertEqual(parse_robot_config(updated)['robot_id'], 'ugv7')
        self.assertEqual(parse_robot_config(updated)['offset_y'], '-0.8')
        for name in ('ugv00', 'ugv-1', 'ugv1;touch /tmp/x', '1'):
            with self.assertRaises(ValueError):
                robot_number(name)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / '.config/formation/robot.env'
            path.parent.mkdir(parents=True)
            path.write_text(text)
            class SFTP:
                def __enter__(self): return self
                def __exit__(self, *_): pass
                def normalize(self, _): return directory
                def open(self, path, mode='r'): return open(path, mode + 'b')
                stat = staticmethod(os.stat)
                chmod = staticmethod(os.chmod)
                remove = staticmethod(os.remove)
                posix_rename = staticmethod(os.replace)
            result = update_identity(SimpleNamespace(open_sftp=SFTP), 'ugv7')
            self.assertEqual(result['robot_id'], 'ugv7')
            self.assertEqual(result['uwb_port'], '/dev/ttyUSB2')
            self.assertEqual(next(path.parent.glob('*.before-identity-*')).read_text(), text)
        client = SimpleNamespace(get_transport=lambda: SimpleNamespace(is_active=lambda: True))
        session = RobotSession('192.0.2.1', client, dict(DEFAULTS, desired_id='ugv7'), queue.Queue())
        calls = []
        with patch('fleet_console.sync_workspace', return_value={'robot_id':'ugv3'}), \
             patch('fleet_console.update_identity', side_effect=lambda *a: calls.append(a[1]) or {'robot_id':'ugv7'}):
            session.maintenance('prepare', False)
        self.assertEqual(calls, ['ugv7'])
        self.assertEqual(session.robot_id, 'ugv7')

    def test_anchor_geometry_and_solver_settings(self):
        base = localization_config()
        self.assertEqual(len(base['anchors']), 6)
        self.assertEqual(base['anchors'][2]['x'], 6.4)
        self.assertEqual(base['valid_max_residual_rms'], 0.6)
        changed = json.loads(json.dumps(dict(anchors=base['anchors'], tag_height=0.32,
                                             valid_max_residual_rms=0.55)))
        changed['anchors'][2].update(x=7.4, z=1.6)
        result = localization_config(json.dumps(changed))
        self.assertEqual(result['workspace_x_max'], 7.4)
        self.assertEqual(result['innovation_huber_sigma'], base['innovation_huber_sigma'])
        self.assertEqual(result['tag_height'], 0.32)
        self.assertEqual(result['valid_max_residual_rms'], 0.55)
        cases = [dict(changed, anchors=changed['anchors'][:3]),
                 dict(changed, tag_height=float('nan')),
                 dict(changed, valid_max_residual_rms=0),
                 dict(changed, valid_max_residual_rms=float('nan')),
                 dict(changed, anchors=[dict(a, id=0) for a in changed['anchors']]),
                 dict(changed, anchors=[dict(a, y=0) for a in changed['anchors']])]
        for value in cases:
            with self.assertRaises(ValueError):
                localization_config(json.dumps(value))
        self.assertEqual(localization_config()['anchors'][2]['x'], 6.4)

    def test_launch_sources_correct_environment_and_no_follower_on_leader(self):
        for step in ('master', 'chassis', 'follower'):
            command = launch_command(step, dict(DEFAULTS, workspace='~/workspace with space'), '192.0.2.1', 'ugv2', '/tmp/a b')
            subprocess.run(['bash', '-n', '-c', command], check=True)
            self.assertIn('ROS_IP=192.0.2.1', command)
            self.assertIn('CAR_MODE=mini_4wd', command)
            self.assertNotIn(DEFAULTS['password'], command)
            self.assertIn('/scripts/env.sh', command)
        follower = launch_command('follower', DEFAULTS, '192.0.2.1', 'ugv2', '/tmp/stage')
        self.assertIn('leader_id:=1 auto_enable:=false', follower)
        self.assertIn('check --step follower --ids 2', follower)
        with self.assertRaises(ValueError):
            launch_command('follower', DEFAULTS, '192.0.2.1', 'ugv1', '/tmp/stage')

    def test_terminal_credentials_are_not_in_arguments(self):
        with patch('fleet_terminal.shutil.which', return_value='/usr/bin/gnome-terminal'), \
             patch.object(Terminal, '_tabs', set()), \
             patch('fleet_terminal.subprocess.run', return_value=SimpleNamespace(
                 stdout='GNOME_TERMINAL_SERVICE=:1.42\nGNOME_TERMINAL_SCREEN=/screen/first\n')) as run:
            terminal = Terminal('192.0.2.1', DEFAULTS, '/tmp/test-known-hosts', 'printf test', 'test')
            try:
                args = run.call_args[0][0]
                self.assertIn('--tab', args)
                self.assertIn('--print-environment', args)
                self.assertNotIn(DEFAULTS['password'], ' '.join(args))
                request = terminal.directory / 'request.json'
                self.assertEqual(request.stat().st_mode & 0o777, 0o600)
                self.assertEqual(json.loads(request.read_text())['options']['password'], DEFAULTS['password'])
                terminal.stop()
                self.assertTrue((terminal.directory / 'stop').exists())
                second = Terminal('192.0.2.2', DEFAULTS, '/tmp/test-known-hosts', 'printf next', 'next')
                try:
                    environment = run.call_args[1]['env']
                    self.assertEqual(environment['GNOME_TERMINAL_SERVICE'], ':1.42')
                    self.assertEqual(environment['GNOME_TERMINAL_SCREEN'], '/screen/first')
                finally:
                    shutil.rmtree(second.directory)
            finally:
                shutil.rmtree(terminal.directory)

    def test_terminal_groups_reuse_only_matching_windows(self):
        terminals = []
        groups = ('chassis', 'follower', 'chassis', 'follower', 'shell', 'master')
        with patch('fleet_terminal.shutil.which', return_value='/usr/bin/gnome-terminal'), \
             patch.object(Terminal, '_tabs', set()), \
             patch('fleet_terminal.subprocess.run', side_effect=[SimpleNamespace(
                 stdout='GNOME_TERMINAL_SERVICE=:1.42\nGNOME_TERMINAL_SCREEN=/screen/%d\n' % number)
                 for number in range(len(groups))]) as run:
            try:
                for group in groups:
                    terminals.append(Terminal('192.0.2.1', DEFAULTS, '/tmp/test-known-hosts', group=group))
                for number, parent in enumerate((None, None, '/screen/0', '/screen/1', None, None)):
                    self.assertEqual(run.call_args_list[number][1]['env'].get('GNOME_TERMINAL_SCREEN'), parent)
            finally:
                for terminal in terminals:
                    shutil.rmtree(terminal.directory)
        workbench = FleetWorkbench.__new__(FleetWorkbench)
        workbench.app = SimpleNamespace(config_path=Path('/tmp/settings.json'))
        workbench.events, workbench.terminals = queue.Queue(), []
        with patch('fleet_workbench.Terminal') as terminal:
            for group in ('chassis', 'follower', 'master'):
                workbench.launch(group, '192.0.2.2', 'ugv2', DEFAULTS, '/tmp/stage')
                self.assertEqual(terminal.call_args[1]['group'], group)

    def test_terminal_skips_closed_tabs_and_reopens_closed_window(self):
        terminals = []
        with patch('fleet_terminal.shutil.which', return_value='/usr/bin/gnome-terminal'), \
             patch.object(Terminal, '_tabs', set()), \
             patch('fleet_terminal.subprocess.run', side_effect=[SimpleNamespace(
                 stdout='GNOME_TERMINAL_SERVICE=:1.42\nGNOME_TERMINAL_SCREEN=/screen/%d\n' % number)
                 for number in range(4)]) as run:
            try:
                for _ in range(2):
                    terminals.append(Terminal('192.0.2.1', DEFAULTS, '/tmp/test-known-hosts'))
                (terminals[1].directory / 'status.json').write_text('{"state": "已关闭"}')
                terminals.append(Terminal('192.0.2.1', DEFAULTS, '/tmp/test-known-hosts'))
                self.assertEqual(run.call_args[1]['env']['GNOME_TERMINAL_SCREEN'], '/screen/0')
                for terminal in terminals:
                    (terminal.directory / 'status.json').write_text('{"state": "已关闭"}')
                terminals.append(Terminal('192.0.2.1', DEFAULTS, '/tmp/test-known-hosts'))
                self.assertNotIn('GNOME_TERMINAL_SCREEN', run.call_args[1]['env'])
                self.assertNotIn('GNOME_TERMINAL_SERVICE', run.call_args[1]['env'])
            finally:
                for terminal in terminals:
                    shutil.rmtree(terminal.directory)

    def test_vehicle_context_menu_and_persistent_deletion(self):
        import tkinter as tk
        try:
            root = tk.Tk()
        except tk.TclError:
            self.skipTest('No desktop session')
        root.withdraw()
        with tempfile.TemporaryDirectory() as directory, \
             patch('fleet_console.connect_ssh', side_effect=AssertionError('Offline test')):
            app = FleetConsole(root, Path(directory) / 'settings.json')
            app.vars['export'].set(str(Path(directory) / 'fleet.csv'))
            wb = app.workbench
            first, second, third = list(app.robots)[:3]
            try:
                root.deiconify()
                for index, table in enumerate((app.table, wb.vehicles)):
                    app.notebook.select(index)
                    table.selection_set((first, second))
                    root.update()
                    with patch.object(app.vehicle_menu, 'tk_popup') as popup, \
                         patch.object(app.vehicle_menu, 'grab_release'), \
                         patch.object(wb, 'open_ssh') as open_ssh:
                        _, y, _, height = table.bbox(second)
                        table.event_generate('<Button-3>', x=10, y=y + height // 2)
                        self.assertEqual(table.selection(), (first, second))
                        popup.assert_called_once()
                        app.vehicle_menu.invoke(0)
                        open_ssh.assert_called_once_with([first, second])
                        _, y, _, height = table.bbox(third)
                        table.event_generate('<Button-3>', x=10, y=y + height // 2)
                        self.assertEqual(table.selection(), (third,))
                        popup.reset_mock()
                        table.event_generate('<Button-3>', x=10, y=0)
                        popup.assert_not_called()
                session = RobotSession(first, None, DEFAULTS, app.events)
                app.sessions[first] = session
                app.table.selection_set(first)
                app.selection_changed()
                wb.active_identities = {first: 1}
                old_scan = app.scan_cancel
                wb.vehicles.selection_set((first, second))
                with patch.object(wb, 'stop') as stop, patch('fleet_workbench.Terminal') as terminal:
                    wb.terminals = [('shell', first, terminal), ('shell', third, terminal.return_value)]
                    _, y, _, height = wb.vehicles.bbox(first)
                    with patch.object(app.vehicle_menu, 'tk_popup'), patch.object(app.vehicle_menu, 'grab_release'):
                        wb.vehicles.event_generate('<Button-3>', x=10, y=y + height // 2)
                    app.vehicle_menu.invoke(1)
                    stop.assert_called_once()
                    terminal.stop.assert_called_once()
                    terminal.return_value.stop.assert_not_called()
                    wb.terminals.clear()
                self.assertTrue(old_scan.is_set())
                self.assertTrue(session.closed.is_set())
                self.assertEqual(session.command[:2], (0, 0))
                self.assertNotIn(first, app.sessions)
                self.assertEqual(app.selected_ip, '')
                for ip in (first, second):
                    self.assertNotIn(ip, app.robots)
                    self.assertFalse(app.table.exists(ip))
                    self.assertFalse(wb.vehicles.exists(ip))
                restored, robots = load_settings(app.config_path)
                self.assertEqual(set(robots), set(app.robots))
                self.assertEqual(restored['formation_selected'], '')
                self.assertNotIn(first, Path(app.vars['export'].get()).read_text())
                late = RobotSession(first, None, DEFAULTS, app.events)
                app.events.put(('connected', late, {'cancelled': old_scan}))
                app.events.put(('auth_failed', first, {'error': 'late reply', 'cancelled': old_scan}))
                app.events.put(('status', session, {'robot_id': 'ugv1'}))
                wb.events.put(('config', first, {'robot_id': 'ugv1'}))
                app.poll()
                self.assertTrue(late.closed.is_set())
                self.assertNotIn(first, app.robots)
                with patch.object(app, 'robot_id_for') as identity:
                    app.selected_action('identity', addresses=[first])
                    identity.assert_not_called()
            finally:
                app.closing = True
                for timer in root.tk.splitlist(root.tk.call('after', 'info')):
                    root.after_cancel(timer)
                root.destroy()

    def test_leader_keyboard_events_and_stop_conditions(self):
        import tkinter as tk
        try:
            root = tk.Tk()
        except tk.TclError:
            self.skipTest('No desktop session')
        root.withdraw()
        with tempfile.TemporaryDirectory() as directory, \
             patch('fleet_console.connect_ssh', side_effect=AssertionError('Network forbidden in offline tests')):
            app = FleetConsole(root, Path(directory) / 'settings.json')
            wb = app.workbench
            try:
                app.notebook.select(2)
                root.deiconify()
                root.update()
                wb.open_leader_control()
                root.update()
                wb.keyboard.focus_force()
                root.update()
                wb.activate_keyboard()
                self.assertFalse(wb.armed.get())  # No monitor: no driving.
                app.vars['leader'].set('ugv3')
                wb.active_leader = 'ugv3'
                monitor = wb.monitor = SimpleNamespace(ready=True, last=time.monotonic(), subscribers=1,
                                                       drive=(0, 0, 0), enable=True, enable_sequence=0,
                                                       limits={'3': 0.15}, limit_status={})
                def activate():
                    monitor.last = time.monotonic()
                    wb.activate_keyboard()
                    self.assertTrue(wb.armed.get(), wb.keyboard_status.get())
                def press(key):
                    wb.keyboard.event_generate('<KeyPress>', keysym=key)
                def release(key):
                    wb.keyboard.event_generate('<KeyRelease>', keysym=key)
                    root.update_idletasks()
                activate()
                for key, expected in [('w', (0.1, 0)), ('Up', (0.1, 0)), ('s', (-0.1, 0)),
                                      ('Down', (-0.1, 0)), ('a', (0, 0.4)), ('Left', (0, 0.4)),
                                      ('d', (0, -0.4)), ('Right', (0, -0.4))]:
                    press(key)
                    self.assertEqual(monitor.drive[:2], expected, key)
                    release(key)
                    self.assertEqual(monitor.drive[:2], (0, 0))
                self.assertIn('ugv3', wb.keyboard_status.get())
                for travel, turn, expected in [('w', 'a', (0.1, 0.4)), ('w', 'd', (0.1, -0.4)),
                                               ('s', 'a', (-0.1, 0.4)), ('s', 'd', (-0.1, -0.4)),
                                               ('Up', 'Left', (0.1, 0.4)), ('Down', 'd', (-0.1, -0.4))]:
                    for order in ((travel, turn), (turn, travel)):
                        monitor.last = time.monotonic()
                        press(order[0])
                        press(order[1])
                        self.assertEqual(monitor.drive[:2], expected, order)
                        wb.keyboard.event_generate('<KeyRelease>', keysym=travel)
                        press(travel)  # Autorepeat preserves the other held axis.
                        root.update_idletasks()
                        self.assertEqual(monitor.drive[:2], expected)
                        release(travel)
                        self.assertEqual(monitor.drive[:2], (0, expected[1]))
                        press(travel)
                        release(turn)
                        self.assertEqual(monitor.drive[:2], (expected[0], 0))
                        release(travel)
                        self.assertEqual(monitor.drive[:2], (0, 0))
                press('w')
                press('Up')  # Equivalent keys do not double speed.
                press('s')  # Opposite travel cancels even with two forward keys.
                press('a')
                self.assertEqual(monitor.drive[:2], (0, 0.4))
                press('d')
                self.assertEqual(monitor.drive[:2], (0, 0))
                release('s')
                self.assertEqual(monitor.drive[:2], (0.1, 0))
                release('w')
                self.assertEqual(monitor.drive[:2], (0.1, 0))
                for key in ('Up', 'a', 'd'):
                    release(key)
                self.assertEqual(monitor.drive[:2], (0, 0))
                monitor.limits['3'] = 0.06
                press('w')
                self.assertEqual(monitor.drive[:2], (0.06, 0))
                press('a')
                self.assertEqual(monitor.drive[:2], (0.06, 0.4))
                release('a')
                release('w')
                press('s')
                self.assertEqual(monitor.drive[:2], (-0.06, 0))
                release('s')
                monitor.limits['3'] = 0.0
                press('w')
                self.assertEqual(monitor.drive[:2], (0, 0))
                press('d')
                self.assertEqual(monitor.drive[:2], (0, -0.4))
                release('d')
                release('w')
                monitor.limits['3'] = 0.15
                press('w')
                wb.keyboard.event_generate('<KeyRelease>', keysym='w')
                press('w')  # X11 repeat pair keeps the current motion.
                root.update_idletasks()
                self.assertEqual(monitor.drive[:2], (0.1, 0))
                press('a')
                self.assertEqual(monitor.drive[:2], (0.1, 0.4))
                press('space')
                release('space')
                self.assertFalse(wb.armed.get())
                self.assertFalse(monitor.enable)
                self.assertEqual(monitor.drive[:2], (0, 0))
                activate()
                press('w')  # Held key cannot undo the emergency stop.
                press('a')
                self.assertEqual(monitor.drive[:2], (0, 0))
                release('w')
                press('w')
                self.assertEqual(monitor.drive[:2], (0.1, 0))
                release('a')
                release('w')
                press('a')
                entry = next(w for w in wb.keyboard.master.winfo_children() if w.winfo_class() == 'TEntry')
                entry.focus_force()
                root.update()
                self.assertFalse(wb.armed.get())
                self.assertEqual(monitor.drive[:2], (0, 0))
                entry.event_generate('<KeyPress>', keysym='w')
                self.assertEqual(monitor.drive[:2], (0, 0))
                app.vars['linear'].set('0.1')
                activate()
                for failure in ('stale', 'disconnected', 'no_subscribers', 'leader_changed'):
                    press('Down')
                    self.assertEqual(monitor.drive[:2], (-0.1, 0), failure)
                    if failure == 'stale': monitor.last -= 1
                    elif failure == 'disconnected': monitor.ready = False
                    elif failure == 'no_subscribers': monitor.subscribers = 0
                    else: app.vars['leader'].set('ugv5')
                    wb.update_keyboard(time.monotonic())
                    self.assertFalse(wb.armed.get(), failure)
                    self.assertEqual(monitor.drive[:2], (0, 0), failure)
                    release('Down')
                    monitor.ready, monitor.subscribers = True, 1
                    app.vars['leader'].set('ugv3')
                    activate()
                for value in ('nan', 'inf', '-0.1', '0', '0.6', 'text'):
                    app.vars['linear'].set(value)
                    press('Up')
                    self.assertFalse(wb.armed.get(), value)
                    self.assertEqual(monitor.drive[:2], (0, 0))
                    release('Up')
                    app.vars['linear'].set('0.1')
                    activate()
                app.vars['angular'].set('2.0')
                press('Right')
                self.assertFalse(wb.armed.get())
                self.assertEqual(monitor.drive[:2], (0, 0))
                release('Right')
                app.vars['angular'].set('0.4')
                activate()
                press('s')
                wb.keyboard.master.pack_forget()
                wb.update_keyboard(time.monotonic())
                self.assertFalse(wb.armed.get())
                self.assertEqual(monitor.drive[:2], (0, 0))
                wb.keyboard.master.pack(fill='both')
                root.update()
                activate()
                press('d')
                app.notebook.select(1)
                root.update()
                self.assertFalse(wb.armed.get())
                self.assertEqual(monitor.drive[:2], (0, 0))
                app.vars['linear'].set('0.12')
                app.save()
                saved = json.loads(app.config_path.read_text())
                self.assertEqual(saved['options']['linear'], '0.12')
                self.assertEqual(saved['options']['leader_control_mode'], 'keyboard')
            finally:
                app.closing = True
                for timer in root.tk.splitlist(root.tk.call('after', 'info')):
                    root.after_cancel(timer)
                root.destroy()

    def test_gui_tabs_map_pending_identity_and_aggregate_sync(self):
        import tkinter as tk
        from tkinter import ttk
        try:
            root = tk.Tk()
        except tk.TclError:
            self.skipTest('No desktop session')
        root.withdraw()
        with tempfile.TemporaryDirectory() as directory, \
             patch('fleet_console.connect_ssh', side_effect=AssertionError('Network forbidden in offline tests')), \
             patch.dict(ttk.__dict__):
            ttk.__dict__.pop('Spinbox', None)  # Python 3.6 has tk.Spinbox only.
            app = FleetConsole(root, Path(directory) / 'settings.json')
            try:
                wb = app.workbench
                self.assertEqual(float(wb.limit_input.get()), 0.15)
                ip = '192.168.0.106'
                wb.limit_input.set('0.12')
                wb.apply_limits()
                restored, _ = load_settings(app.config_path)
                self.assertEqual(saved_linear_limit(restored), 0.12)
                wb.active_identities = {host: n for n, host in enumerate(app.robots, 1)}
                wb.current_ids = list(wb.active_identities.values())
                monitor = wb.monitor = SimpleNamespace(ready=True, last=time.monotonic(),
                                                      limits={'1': 0.15}, limit_status={},
                                                      enable=False, enable_sequence=0)
                wb.apply_limits()
                self.assertEqual(monitor.limits, {str(n): 0.12 for n in range(1, 6)})
                with patch.object(wb, 'fresh', return_value=True):
                    wb.set_enabled(True)
                    self.assertFalse(monitor.enable)  # No readback, including older controllers.
                    monitor.limit_status = {str(n): {'ready': True, 'applied': 0.12, 'requested': 0.12}
                                            for n in range(1, 6)}
                    wb.set_enabled(True)
                    self.assertTrue(monitor.enable)
                    wb.refresh_limits()
                    self.assertEqual(wb.limits_status.get(), '已生效 5/5')
                    wb.limit_input.set('0.09')
                    wb.refresh_limits()
                    self.assertEqual(wb.limit_input.get(), '0.09')  # Refresh retains unsaved edits.
                    wb.apply_limits()
                    self.assertFalse(wb.limit_ready(1))  # Old ACK cannot confirm a changed limit.
                    self.assertIn('已生效 0/5', wb.limits_status.get())
                wb.limit_input.set('nan')
                with patch.object(app.messagebox, 'showerror') as error:
                    wb.apply_limits()
                    error.assert_called_once()
                self.assertEqual(saved_linear_limit(app.current_options()), 0.09)
                wb.limit_input.set('0')
                wb.apply_limits()
                self.assertEqual(monitor.limits, {str(n): 0.0 for n in range(1, 6)})
                wb.monitor = None
                self.assertEqual([app.notebook.tab(tab, 'text') for tab in app.notebook.tabs()],
                                 ['扫描与连接', '编队算法', '小车定位图'])
                page = root.nametowidget(app.notebook.tabs()[1])
                steps = next(widget for widget in page.winfo_children() if isinstance(widget, ttk.Notebook))
                self.assertEqual([steps.tab(tab, 'text') for tab in steps.tabs()],
                                 ['1 · 配置检查', '2 · Master', '3 · 底盘与定位', '4 · 跟随算法', '5 · 监视与使能'])
                with patch.object(wb, 'start') as start:
                    for tab, action in zip(steps.tabs(), ('config', 'master', 'chassis', 'follower', 'monitor')):
                        frame = root.nametowidget(tab)
                        buttons = next(widget for widget in frame.winfo_children() if isinstance(widget, ttk.Frame))
                        buttons.winfo_children()[0].invoke()
                        start.assert_called_with(action)
                steps.select(4)
                root.update()
                self.assertEqual(app.vars['formation_step'].get(), '4')
                with patch.object(app, 'selected_action') as action:
                    wb.vehicles.selection_set('192.168.0.106')
                    wb.serial_permissions()
                    action.assert_called_once_with('serial', addresses=['192.168.0.106', '192.168.0.108',
                                                    '192.168.0.109', '192.168.0.110', '192.168.0.114'], launch=False)
                ip = '192.168.0.106'
                app.table.selection_set(ip)
                app.selection_changed()
                with patch.object(app, 'selected_action') as action:
                    app.name_var.set('ugv7')
                    app.apply_identity()
                    self.assertEqual(app.robots[ip]['pending_id'], 'ugv7')
                    action.assert_called_once_with('identity', addresses=[ip], popup=False)
                wb.last_sample = time.monotonic()
                wb.samples = {'0': {'pose': {'value': [1.2, 2], 'age': 0},
                                    'valid': {'value': True, 'age': 0},
                                    'target': {'value': [1.3, 2], 'age': 0},
                                    'error': {'value': 0.1, 'age': 0}}}
                self.assertTrue(wb.fresh('0', wb.last_sample + 0.1))
                self.assertFalse(wb.fresh('0', wb.last_sample + 1))
                root.deiconify()
                app.notebook.select(2)
                root.update()
                wb.paint_map(wb.last_sample + 0.1)
                self.assertGreater(len(wb.canvas.find_all()), 20)
                wb.edit_anchors()
                dialog = next(w for w in root.winfo_children() if isinstance(w, tk.Toplevel))
                self.assertIn('基站', dialog.title())
                dialog.destroy()
                app.batches['sync'] = {ip:None, '192.168.0.108':None}
                app.events.put(('task_done', SimpleNamespace(ip=ip), {'action':'sync','error':''}))
                app.events.put(('task_done', SimpleNamespace(ip='192.168.0.108'), {'action':'sync','error':'dirty'}))
                app.robots[ip].pop('pending_id')
                with patch.object(wb, 'result_popup') as popup:
                    app.poll()
                    popup.assert_called_once()
                    self.assertEqual(popup.call_args[0][1][ip], '成功')
                    self.assertEqual(popup.call_args[0][1]['192.168.0.108'], 'dirty')
                wb.vehicles.selection_set([ip])
                with patch.object(wb, 'connect', side_effect=RuntimeError('test connection failed')):
                    wb.start('master')
                    wb.worker.join(2)
                with patch.object(wb, 'result_popup') as popup:
                    wb.poll()
                    self.assertIn('test connection failed', str(popup.call_args))
                app.save()
                saved = json.loads(app.config_path.read_text())
                self.assertEqual(saved['options']['leader'], 'ugv1')
                self.assertIn('anchors_json', saved['options'])
            finally:
                app.closing = True
                for timer in root.tk.splitlist(root.tk.call('after', 'info')):
                    root.after_cancel(timer)
                root.destroy()

    def test_leader_popup_current_start_preview_and_pause_on_close(self):
        import tkinter as tk
        try:
            root = tk.Tk()
        except tk.TclError:
            self.skipTest('No desktop session')
        with tempfile.TemporaryDirectory() as directory, \
             patch('fleet_console.connect_ssh', side_effect=AssertionError('Offline test')):
            app = FleetConsole(root, Path(directory)/'settings.json')
            wb = app.workbench
            try:
                app.vars['leader'].set('ugv3')
                wb.active_leader, wb.active_identities = 'ugv3', {'192.168.0.109': 3}
                wb.monitor = SimpleNamespace(ready=True, last=time.monotonic(), subscribers=1, enable=False,
                                             enable_sequence=0, limits={'3': 0.15}, limit_status={},
                                             drive=(0, 0, 0), tracking={'state': 'STOPPED', 'mode': 'keyboard'},
                                             control_request={'sequence': 0, 'action': 'keyboard'})
                wb.last_sample = time.monotonic()
                wb.samples = {'3': {'pose': {'value': [2.1, 2.2], 'age': 0}, 'valid': {'value': True, 'age': 0}}}
                app.notebook.select(2)
                wb.open_leader_control()
                root.update()
                radios = [w for frame in wb.control_dialog.winfo_children() for w in frame.winfo_children()
                          if w.winfo_class() == 'TRadiobutton']
                radios[1].invoke()
                self.assertEqual(wb.monitor.control_request['action'], 'path')
                wb.begin_path_pick()
                root.update()
                self.assertEqual(wb.control_dialog.state(), 'withdrawn')
                wb.edit_path('clear')
                wb.use_current_start()
                self.assertEqual(len(wb.canvas.find_withtag('path_point')), 1)
                ox, oy, scale = wb.map_transform
                wb.canvas.event_generate('<Button-1>', x=round(ox + 3 * scale), y=round(oy - 2.2 * scale))
                wb.open_leader_control()
                points = wb.preview_path()
                self.assertEqual(points[0], (2.1, 2.2))
                self.assertAlmostEqual(points[1][0], 3, delta=1 / scale)
                self.assertAlmostEqual(points[1][1], 2.2, delta=1 / scale)
                self.assertTrue(wb.canvas.find_withtag('reference_path'))
                wb.trajectory_action('start')
                self.assertEqual(wb.monitor.control_request['action'], 'start')
                self.assertEqual(wb.monitor.control_request['points'], points)
                self.assertEqual(wb.monitor.drive[:2], (0, 0))
                wb.close_control_dialog()
                self.assertEqual(wb.monitor.control_request['action'], 'pause')
                self.assertIsNone(wb.keyboard)
                wb.open_leader_control()
                self.assertEqual(wb.monitor.control_request['action'], 'pause')  # Opening never starts motion.
                app.save()
                options, _ = load_settings(app.config_path)
                self.assertEqual(options['leader_control_mode'], 'path')
                self.assertIn('2.1000, 2.2000', options['leader_path'])
                app.vars['leader'].set('ugv2')
                wb.poll()
                self.assertEqual(wb.monitor.control_request['action'], 'stop')
            finally:
                app.closing = True
                for timer in root.tk.splitlist(root.tk.call('after', 'info')):
                    root.after_cancel(timer)
                root.destroy()

    def test_map_curve_drag_restore_save_and_control_request(self):
        import tkinter as tk
        from leader_tracker import LeaderTracker, sample_path
        try:
            root = tk.Tk()
        except tk.TclError:
            self.skipTest('No desktop session')
        with tempfile.TemporaryDirectory() as directory, \
             patch('fleet_console.connect_ssh', side_effect=AssertionError('Offline test')):
            app = FleetConsole(root, Path(directory) / 'settings.json')
            wb = app.workbench
            try:
                app.vars['leader_control_mode'].set('path')
                app.vars['leader_path'].set('1, 2\n3, 2\n4, 2')
                wb.open_leader_control()
                wb.begin_path_pick()
                root.update()
                points = list(wb.path_preview)

                def mouse(kind, x, y):
                    ox, oy, scale = wb.map_transform
                    wb.canvas.event_generate(kind, x=round(ox+x*scale), y=round(oy-y*scale))

                def drag(start, end):
                    mouse('<Button-1>', *start)
                    self.assertIsNotNone(wb.path_drag)
                    mouse('<B1-Motion>', *end)
                    mouse('<ButtonRelease-1>', *end)

                self.assertEqual(len(wb.canvas.find_withtag('curve_handle')), 2)
                drag((2, 2), (2, 3))
                self.assertAlmostEqual(wb.path_bends[0], 1, delta=0.03)
                self.assertEqual(wb.path_bends[1], 0)
                self.assertEqual(wb.path_preview, points)
                self.assertEqual(len(wb.canvas.find_withtag('path_point')), 3)
                line = wb.canvas.find_withtag('reference_path')[0]
                self.assertGreater(len(wb.canvas.coords(line)), 4)
                saved = list(wb.path_bends)
                drag((2, 2+saved[0]), (2, 5))
                self.assertIn('未保存', wb.path_pick_status.get())
                self.assertEqual(wb.path_bends, saved)

                root.geometry('1050x700')
                root.update()
                # Grab the curved line away from the midpoint after resizing.
                drag((1.5, 2+0.75*saved[0]), (1.5, 2+0.75*saved[0]-0.5))
                self.assertAlmostEqual(wb.path_bends[0], 0.5, delta=0.04)
                mouse('<Button-3>', 2, 2+wb.path_bends[0])
                self.assertEqual(wb.path_bends, [0, 0])
                self.assertEqual(len(wb.canvas.coords(wb.canvas.find_withtag('reference_path')[0])), 4)
                drag((3.5, 2), (3.5, 1.5))
                self.assertAlmostEqual(wb.path_bends[1], -0.5, delta=0.03)
                wb.open_leader_control()
                root.update()
                saved = list(wb.path_bends)
                wb.active_leader = app.vars['leader'].get()
                wb.monitor = SimpleNamespace(ready=True, last=time.monotonic(), subscribers=1, enable=False,
                                             drive=(0, 0, 0), tracking={'state': 'STOPPED'},
                                             control_request={'sequence': 0, 'action': 'path'})
                with patch.object(app.messagebox, 'showerror') as error:
                    wb.trajectory_action('start')
                    self.assertIn('重新连接', error.call_args[0][1])
                self.assertEqual(wb.monitor.control_request['action'], 'path')
                wb.monitor.tracking['curves_supported'] = True
                wb.trajectory_action('start')
                request = wb.monitor.control_request
                self.assertEqual(request['action'], 'start')
                self.assertEqual(request['points'], points)
                self.assertEqual(request['bends'], saved)
                tracker = LeaderTracker()
                tracker.start(request['points'], request['speed'], request['lookahead'], (1, 2, 0), 0,
                              bends=request['bends'])
                self.assertEqual(tracker.points, sample_path(points, saved))
                self.assertEqual(tracker.status()['waypoints'], points)
                wb.monitor = None

                app.save()
                options, _ = load_settings(app.config_path)
                self.assertEqual(json.loads(options['leader_path_bends']), saved)
                wb.close_control_dialog()
                wb.open_leader_control()
                self.assertEqual(wb.path_bends, saved)
                wb.edit_path('append', (5, 2))
                self.assertEqual(wb.path_bends, saved+[0])
                wb.edit_path('undo')
                self.assertEqual(wb.path_bends, saved)
                root.update()
                wb.path_editor.insert('end', '\n5, 2')
                root.update()
                self.assertEqual(json.loads(app.vars['leader_path_bends'].get()), [])
                self.assertEqual(wb.read_path()[1], [0, 0, 0])
                wb.edit_path('clear')
                self.assertEqual(wb.path_bends, [])
                wb.edit_path('append', (2, 2))
                wb.edit_path('append', (2.2, 2))
                wb.begin_path_pick()
                root.update()
                drag((2.1, 2), (2.1, 2.2))  # Handle overlaps endpoints at this zoom.
                self.assertAlmostEqual(wb.path_bends[0], 0.2, delta=0.03)
            finally:
                app.closing = True
                for timer in root.tk.splitlist(root.tk.call('after', 'info')):
                    root.after_cancel(timer)
                root.destroy()

    def test_map_path_editing_coordinates_validation_and_resize(self):
        import tkinter as tk
        from leader_tracker import parse_points
        try:
            root = tk.Tk()
        except tk.TclError:
            self.skipTest('No desktop session')
        with tempfile.TemporaryDirectory() as directory, \
             patch('fleet_console.connect_ssh', side_effect=AssertionError('Offline test')):
            app = FleetConsole(root, Path(directory) / 'settings.json')
            wb = app.workbench
            try:
                config = localization_config()
                anchors = [dict(a, x=a['x']-4, y=a['y']-3) for a in config['anchors']]
                app.vars['anchors_json'].set(json.dumps(dict(anchors=anchors, tag_height=config['tag_height'])))
                app.vars['leader_control_mode'].set('path')
                app.vars['leader_path'].set('')
                # Last telemetry from a disconnected run must not hide the draft.
                wb.tracking = {'state': 'TRACKING', 'points': [(0, 0), (1, 0)]}
                wb.open_leader_control()
                wb.begin_path_pick()
                root.update()

                def click(x, y):
                    ox, oy, scale = wb.map_transform
                    wb.canvas.event_generate('<Button-1>', x=round(ox+x*scale), y=round(oy-y*scale))
                    root.update()
                    return scale

                scale = click(-2, -1)
                self.assertEqual(len(wb.path_preview), 1)
                self.assertAlmostEqual(wb.path_preview[0][0], -2, delta=1/scale)
                self.assertAlmostEqual(wb.path_preview[0][1], -1, delta=1/scale)
                self.assertEqual(len(wb.canvas.find_withtag('path_point')), 1)
                self.assertFalse(wb.canvas.find_withtag('reference_path'))
                click(-2, -1)
                self.assertIn('0.15 m', wb.path_pick_status.get())
                self.assertEqual(len(wb.path_preview), 1)
                click(-4, -1)
                self.assertIn('范围', wb.path_pick_status.get())
                self.assertEqual(len(wb.path_preview), 1)
                wb.use_current_start()
                self.assertIn('定位无效或过期', wb.path_pick_status.get())
                self.assertEqual(len(wb.path_preview), 1)

                previous_transform = wb.map_transform
                root.geometry('1050x700')
                root.update()
                self.assertNotEqual(wb.map_transform, previous_transform)
                scale = click(1, 0.5)
                self.assertEqual(len(wb.path_preview), 2)
                self.assertAlmostEqual(wb.path_preview[1][0], 1, delta=1/scale)
                self.assertAlmostEqual(wb.path_preview[1][1], 0.5, delta=1/scale)
                self.assertTrue(wb.canvas.find_withtag('reference_path'))
                self.assertEqual(parse_points(app.vars['leader_path'].get()), wb.path_preview)
                wb.canvas.event_generate('<Button-3>')
                self.assertEqual(len(wb.path_preview), 1)
                wb.edit_path('clear')
                self.assertEqual(wb.path_preview, [])
                self.assertEqual(app.vars['leader_path'].get(), '')
                self.assertFalse(wb.canvas.find_withtag('path_point'))
                self.assertFalse(wb.canvas.find_withtag('reference_path'))
                click(-2, -1)
                click(1, 0.5)
                wb.open_leader_control()
                root.update()
                self.assertFalse(wb.path_picking)
                self.assertEqual(wb.control_dialog.state(), 'normal')
                self.assertFalse(wb.path_tools.winfo_ismapped())
                before = list(wb.path_preview)
                click(0, 0)
                self.assertEqual(wb.path_preview, before)
                app.save()
                options, _ = load_settings(app.config_path)
                self.assertEqual(parse_points(options['leader_path']), before)
                wb.close_control_dialog()
                wb.open_leader_control()
                self.assertEqual(parse_points(wb.path_editor.get('1.0', 'end-1c')), before)

                wb.monitor = SimpleNamespace(ready=True, tracking={'state': 'TRACKING'})
                with patch.object(app.messagebox, 'showerror') as error:
                    wb.begin_path_pick()
                    error.assert_called_once()
                    self.assertIn('先暂停', error.call_args[0][1])
                self.assertFalse(wb.path_picking)
                self.assertEqual(wb.path_preview, before)
                wb.monitor = None
            finally:
                app.closing = True
                for timer in root.tk.splitlist(root.tk.call('after', 'info')):
                    root.after_cancel(timer)
                root.destroy()


if __name__ == '__main__':
    unittest.main(verbosity=2)
