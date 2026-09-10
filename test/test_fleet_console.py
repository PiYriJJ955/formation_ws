#!/usr/bin/env python3
"""Run with python3 test/test_fleet_console.py; no robot motion or network access."""
import csv
import json
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import queue
import threading
import socket
import select
import shlex
import subprocess

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from fleet_console import (DEFAULTS, FleetConsole, expand_addresses, export_csv,
                           load_settings, save_settings, targets_from, validate, RobotSession, connect_ssh,
                           watch_git_server)
from fleet_bridge import velocity, COMMAND_TIMEOUT, check_master_port
from fleet_deploy import replace_master, update_master, read_robot_config, grant_serial_permissions


class ConsoleChecks(unittest.TestCase):
    def test_new_computer_uses_its_own_server_and_remembers_selected_repository(self):
        with tempfile.TemporaryDirectory() as directory, \
                patch('fleet_console.local_git_urls', return_value=['http://192.168.8.25:8000/formation.git']):
            path = Path(directory) / 'settings.json'
            options, robots = load_settings(path)
            self.assertEqual(options['repository'], 'http://192.168.8.25:8000/formation.git')
            options['repository'] = 'https://git.example.com/formation.git'
            save_settings(path, options, robots)
            self.assertEqual(load_settings(path)[0]['repository'], options['repository'])

    def test_git_server_starts_once_and_tracks_service_changes(self):
        events, stopped = queue.Queue(), Mock()
        stopped.is_set.side_effect = [False, False, False, False, True]
        states = [(0, ''), (0, 'active'), (3, 'inactive'), (3, 'failed'), (0, 'active')]
        with patch('fleet_console.subprocess.run', side_effect=[
                subprocess.CompletedProcess([], code, state + '\n', '') for code, state in states]) as run, \
                patch('fleet_console.server_info', return_value={'authentication': 'password'}), \
                patch('fleet_console.local_git_urls', side_effect=[['http://10.1.2.3:8000/formation.git'], [], [],
                                                                 ['http://10.1.2.4:8000/formation.git']]):
            watch_git_server(events, stopped)
        self.assertEqual([call[0][0] for call in run.call_args_list],
                         [['systemctl', '--user', 'start', 'formation-git-http.service']] +
                         [['systemctl', '--user', 'is-active', 'formation-git-http.service']] * 4)
        for expected, running in [('运行中', True), ('未运行', False), ('运行失败', False), ('运行中', True)]:
            event, _, data = events.get_nowait()
            self.assertEqual(event, 'git_server')
            self.assertIn(expected, data['text'])
            self.assertEqual(data['running'], running)
            if running:
                self.assertIn(data['urls'][0], data['text'])
        self.assertEqual(stopped.wait.call_count, 4)

    def test_git_server_installs_missing_user_unit(self):
        events, stopped = queue.Queue(), Mock()
        stopped.is_set.side_effect = [False, True]
        missing = subprocess.CompletedProcess([], 1, '',
                                              'Failed to start formation-git-http.service: '
                                              'Unit formation-git-http.service not found.')
        active = subprocess.CompletedProcess([], 0, 'active\n', '')
        with patch('fleet_console.subprocess.run', side_effect=[missing,
                subprocess.CompletedProcess([], 0, '', ''), active, active]) as run, \
                patch('fleet_console.server_info', return_value={'authentication': 'password'}), \
                patch('fleet_console.local_git_urls', return_value=[]):
            watch_git_server(events, stopped)
        self.assertEqual(run.call_args_list[1][0][0][-1], '--install-service')
        self.assertEqual(run.call_args_list[2][0][0][-2:], ['start', 'formation-git-http.service'])
        self.assertTrue(events.get_nowait()[2]['running'])

    def test_git_server_reports_start_and_status_errors_then_recovers(self):
        for failure in (subprocess.CompletedProcess([], 1, '', 'Unit not found'),
                        FileNotFoundError('systemctl not found'),
                        subprocess.TimeoutExpired('systemctl', 15)):
            with self.subTest(failure=failure):
                events, stopped = queue.Queue(), Mock()
                stopped.is_set.side_effect = [False, False, False, False, True]
                with patch('fleet_console.subprocess.run', side_effect=[failure,
                        subprocess.CompletedProcess([], 3, 'inactive\n', ''),
                        subprocess.TimeoutExpired('systemctl', 3),
                        subprocess.CompletedProcess([], 0, 'active\n', ''),
                        subprocess.CompletedProcess([], 3, 'inactive\n', '')]), \
                        patch('fleet_console.server_info', return_value={'authentication': 'password'}), \
                        patch('fleet_console.local_git_urls', return_value=[]):
                    watch_git_server(events, stopped)
                data = events.get_nowait()[2]
                self.assertFalse(data['running'])
                self.assertIn('Unit not found' if isinstance(failure, subprocess.CompletedProcess) else str(failure),
                              data['text'])
                self.assertIn('状态检查失败', events.get_nowait()[2]['text'])
                self.assertTrue(events.get_nowait()[2]['running'])
                self.assertEqual(events.get_nowait()[2]['text'], 'Git server：未运行')

    def test_git_server_requires_http_push_health_not_just_an_active_process(self):
        events, stopped = queue.Queue(), Mock()
        stopped.is_set.side_effect = [False, True]
        with patch('fleet_console.subprocess.run', return_value=subprocess.CompletedProcess([], 0, 'active\n', '')), \
                patch('fleet_console.server_info', side_effect=ValueError('请安装并重启 Git HTTP 服务')), \
                patch('fleet_console.local_git_urls', return_value=[]):
            watch_git_server(events, stopped)
        data = events.get_nowait()[2]
        self.assertFalse(data['running'])
        self.assertIn('请安装并重启', data['text'])

    def test_serial_permissions_stdin_sudo_and_device_failures(self):
        # Run the real shell command against temporary files and a fake sudo.
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            devices = [folder / ('ttyCH343USB%d' % n) for n in range(2)]
            for device in devices:
                device.touch(mode=0o600)
            password = 'test $(`no execution`) password'
            sudo = folder / 'sudo'
            sudo.write_text('#!' + sys.executable + '\nimport os,sys\n'
                            'if sys.stdin.readline().rstrip("\\n") != ' + repr(password) + ':\n'
                            ' print("sudo authentication failed"); sys.exit(1)\n'
                            'command=sys.argv[sys.argv.index("--")+1:]\n'
                            'os.execvp(command[0],command)\n')
            sudo.chmod(0o700)
            commands = []
            class Channel:
                process = None
                def get_pty(self): raise AssertionError('sudo password must not be sent to an echoing PTY')
                def set_combine_stderr(self, _): pass
                def settimeout(self, _): pass
                def exec_command(self, command):
                    commands.append(command)
                    argv = shlex.split(command)
                    argv[-1] = argv[-1].replace('/dev/ttyCH343USB*', shlex.quote(str(folder / 'ttyCH343USB')) + '*')
                    self.process = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                                    stderr=subprocess.STDOUT,
                                                    env=dict(os.environ, PATH=directory + ':' + os.environ['PATH']))
                def sendall(self, value): self.process.stdin.write(value); self.process.stdin.flush()
                def shutdown_write(self): self.process.stdin.close()
                def recv_ready(self): return bool(select.select([self.process.stdout], [], [], 0)[0])
                def recv(self, count): return os.read(self.process.stdout.fileno(), count)
                def exit_status_ready(self): return self.process.poll() is not None
                def recv_exit_status(self): return self.process.wait()
                def close(self):
                    if self.process.poll() is None:
                        self.process.terminate()
                        self.process.wait()
                    self.process.stdout.close()
            client = SimpleNamespace(get_transport=lambda: SimpleNamespace(open_session=lambda **_: Channel()))
            # Pipes remain readable at EOF, unlike Paramiko's buffered recv_ready.
            original_recv = Channel.recv
            original_ready = Channel.recv_ready
            def recv(self, count):
                value = original_recv(self, count)
                if not value:
                    self.eof = True
                return value
            Channel.recv = recv
            Channel.recv_ready = lambda self: not getattr(self, 'eof', False) and original_ready(self)
            result = grant_serial_permissions(client, password, threading.Event())
            self.assertEqual([device.stat().st_mode & 0o777 for device in devices], [0o777, 0o777])
            self.assertIn('ttyCH343USB1: 777', result)
            self.assertNotIn(password, commands[0])
            with self.assertRaisesRegex(RuntimeError, 'sudo authentication failed'):
                grant_serial_permissions(client, 'wrong password', threading.Event())
            for device in devices:
                device.unlink()
            with self.assertRaisesRegex(RuntimeError, '未找到'):
                grant_serial_permissions(client, password, threading.Event())

    def test_vm_is_blocked_in_scan_history_export_and_direct_ssh(self):
        options = dict(DEFAULTS, targets='192.168.0.135-137', exclude='',
                       selected='192.168.0.136', formation_selected='192.168.0.136,192.168.0.109')
        self.assertEqual(targets_from(options), ['192.168.0.135', '192.168.0.137'])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'settings.json'
            rows = {'192.168.0.136': {'robot_id': 'ugv7', 'name': 'VM'}}
            save_settings(path, options, rows)
            loaded, robots = load_settings(path)
            self.assertNotIn('192.168.0.136', robots)
            self.assertEqual(loaded['selected'], '')
            self.assertEqual(loaded['formation_selected'], '192.168.0.109')
            export_csv(Path(directory) / 'fleet.csv', rows)
            self.assertNotIn('192.168.0.136', (Path(directory) / 'fleet.csv').read_text())
            with patch('fleet_console.paramiko.SSHClient') as client:
                with self.assertRaisesRegex(ValueError, '虚拟机'):
                    connect_ssh('192.168.0.136', options, Path(directory) / 'known_hosts')
                client.assert_not_called()

    def test_master_port_can_restart_but_rejects_live_listener(self):
        server = socket.socket()
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('127.0.0.1', 0))
        server.listen(1)
        port = server.getsockname()[1]
        with self.assertRaises(OSError):
            check_master_port(port)
        client = socket.create_connection(('127.0.0.1', port))
        accepted, _ = server.accept()
        accepted.close()
        self.assertEqual(client.recv(1), b'')
        client.close()
        server.close()
        check_master_port(port)

    def test_scan_ranges_exclusions_and_bounds(self):
        options = dict(DEFAULTS, targets='192.168.0.109-112,192.168.0.111', exclude='192.168.0.110')
        self.assertEqual(targets_from(options), ['192.168.0.109', '192.168.0.111', '192.168.0.112'])
        self.assertEqual(expand_addresses('192.168.0.0/30'), {'192.168.0.1', '192.168.0.2'})
        self.assertEqual(expand_addresses('192.168.0.1-192.168.0.2，192.168.1.1/32'),
                         {'192.168.0.1', '192.168.0.2', '192.168.1.1'})
        for spec in ('192.168.0.300', '192.168.0.3-1', '10.0.0.0/8', '::1', '$(id)'):
            with self.assertRaises(ValueError):
                expand_addresses(spec)
        for key, value in [('linear', 'nan'), ('angular', 'inf'), ('workers', '0'), ('port', '65536')]:
            with self.assertRaises(ValueError):
                validate(dict(DEFAULTS, **{key: value}))

    def test_history_permissions_and_export(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'settings.json'
            options = dict(DEFAULTS, loop=True, exclude='192.168.0.5', selected='192.168.0.111')
            robots = {'192.168.0.111': {'name': 'ugv0', 'remark': '左侧小车', 'ssh': '已连接'},
                      '192.168.0.112': {'name': '=1+1', 'remark': ' @SUM(A1)'}}
            save_settings(path, options, robots)
            self.assertEqual(os.stat(path).st_mode & 0o777, 0o600)
            loaded_options, loaded_robots = load_settings(path)
            self.assertEqual(loaded_options, options)
            self.assertEqual(loaded_robots['192.168.0.111']['remark'], '左侧小车')
            output = Path(directory) / 'fleet.csv'
            export_csv(output, robots)
            with output.open(encoding='utf-8-sig') as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows[0]['remark'], '左侧小车')
            self.assertEqual(rows[1]['name'], "'=1+1")
            self.assertNotIn('password', rows[0])

    def test_remote_deadman_rejects_delayed_or_invalid_motion(self):
        command = {'tick': 100.0, 'linear': 0.1, 'angular': 0.2}
        self.assertEqual(velocity(command, 100.1), (0.1, 0.2))
        self.assertEqual(velocity(command, 100.0 + COMMAND_TIMEOUT + 0.001), (0, 0))
        self.assertEqual(velocity(command, 99.0), (0, 0))
        for change in ({'linear': float('nan')}, {'angular': float('inf')},
                       {'tick': float('nan')}, {'linear': 0.51}, {'angular': -1.51},
                       {'linear': None}):
            self.assertEqual(velocity(dict(command, **change), 100.1), (0, 0))
        self.assertEqual(velocity({}, 100.1), (0, 0))

    def test_master_update_preserves_settings_and_backup(self):
        original = ('# robot settings\nexport UGV_ID=2\nexport ROS_IP=192.168.0.109\n'
                    'export ROS_MASTER_URI="http://192.168.0.112:11311"\n'
                    'export UWB_PORT=/dev/ttyCH343USB1\nexport CAR_MODE=mini_4wd\n')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / '.config/formation/robot.env'
            path.parent.mkdir(parents=True)
            path.write_text(original)
            path.chmod(0o640)

            class SFTP:
                def __enter__(self): return self
                def __exit__(self, *_): pass
                def normalize(self, _): return directory
                def open(self, path, mode='r'): return open(path, mode + 'b')
                stat = staticmethod(os.stat)
                chmod = staticmethod(os.chmod)
                remove = staticmethod(os.remove)
                posix_rename = staticmethod(os.replace)

            client = SimpleNamespace(open_sftp=SFTP)
            result = update_master(client, '192.168.0.117')
            self.assertEqual(result['ros_master'], 'http://192.168.0.117:11311')
            self.assertEqual(result['ros_ip'], '192.168.0.109')
            self.assertEqual(result['robot_id'], 'ugv2')
            self.assertIn('export UWB_PORT=/dev/ttyCH343USB1', path.read_text())
            self.assertEqual(path.stat().st_mode & 0o777, 0o640)
            backups = list(path.parent.glob('*.before-master-*'))
            self.assertEqual(len(backups), 1)
            self.assertEqual(backups[0].read_text(), original)
            update_master(client, '192.168.0.117')
            self.assertEqual(len(list(path.parent.glob('*.before-master-*'))), 1)
            self.assertEqual(read_robot_config(client), result)
        for value in ('192.168.0.999', '$(id)', 'host:11311'):
            with self.assertRaises(ValueError):
                replace_master(original, value)

    def test_auto_sync_must_succeed_before_chassis_start(self):
        events, calls = queue.Queue(), []
        client = SimpleNamespace(get_transport=lambda: SimpleNamespace(is_active=lambda: True))
        session = RobotSession('192.168.0.111', client, DEFAULTS, events)
        with patch('fleet_console.sync_workspace', side_effect=lambda *a, **kw: calls.append('sync') or {}), \
             patch.object(session, 'run', side_effect=lambda: calls.append('chassis')):
            session.start()
            session.task_thread.join(2)
            session.thread.join(2)
        self.assertEqual(calls, ['sync', 'chassis'])
        session = RobotSession('192.168.0.111', client, DEFAULTS, events)
        with patch('fleet_console.sync_workspace', side_effect=RuntimeError('dirty files')):
            session.start()
            session.task_thread.join(2)
        self.assertIsNone(session.thread)
        self.assertFalse(session.ready)

    def test_gui_selection_stop_and_keyboard_repeat(self):
        import tkinter as tk
        try:
            root = tk.Tk()
        except tk.TclError:
            self.skipTest('Desktop display is unavailable')
        root.withdraw()
        with tempfile.TemporaryDirectory() as directory:
            app = FleetConsole(root, Path(directory) / 'settings.json')
            for running in (True, False):
                app.events.put(('git_server', None, {'text': 'Git server：测试状态', 'running': running}))
                app.poll()
                self.assertEqual(app.git_status.get(), 'Git server：测试状态')
                self.assertEqual(str(app.git_status_label.cget('foreground')), '#267346' if running else '#b52b33')
            self.assertIs(app.git_status_label.master, root)
            commands = {}
            actions = []

            class Session:
                ready = True

                def __init__(self, ip):
                    self.ip = ip
                    self.closed = threading.Event()

                def drive(self, linear=0, angular=0):
                    commands[self.ip] = (linear, angular)

                def alive(self):
                    return True

                def maintain(self, action, options, launch=False):
                    actions.append((self.ip, action, options['master_ip']))
                    return True

            for ip in ('192.168.0.106', '192.168.0.108'):
                app.sessions[ip] = Session(ip)
            app.table.selection_set('192.168.0.106')
            app.selection_changed()
            app.begin_motion('forward')
            self.assertEqual(commands['192.168.0.106'], (0.1, 0))
            self.assertEqual(commands['192.168.0.108'], (0, 0))
            app.table.selection_set('192.168.0.108')
            app.selection_changed()
            self.assertTrue(all(value == (0, 0) for value in commands.values()))
            app.key_press(SimpleNamespace(keysym='w'))
            self.assertEqual(commands['192.168.0.108'], (0.1, 0))
            app.stop_motion()
            app.key_press(SimpleNamespace(keysym='w'))  # Key repeat cannot undo stop.
            self.assertEqual(commands['192.168.0.108'], (0, 0))
            app.key_release(SimpleNamespace(keysym='w'))
            root.update_idletasks()
            app.key_press(SimpleNamespace(keysym='w'))
            self.assertEqual(commands['192.168.0.108'], (0.1, 0))
            # Exercise real Tk focus/mouse ordering when switching from keyboard control.
            def descendants(widget):
                for child in widget.winfo_children():
                    yield child
                    yield from descendants(child)
            buttons = [w for w in descendants(root) if w.winfo_class() == 'TButton']
            keyboard = next(w for w in buttons if w.cget('text').startswith('点击这里'))
            forward = next(w for w in buttons if w.cget('text') == '▲ 前进')
            root.deiconify()
            root.update()
            keyboard.focus_force()
            root.update()
            forward.event_generate('<ButtonPress-1>', x=10, y=10)
            root.update()
            self.assertEqual(commands['192.168.0.108'], (0.1, 0))
            forward.event_generate('<ButtonRelease-1>', x=10, y=10)
            root.update()
            self.assertTrue(all(value == (0, 0) for value in commands.values()))
            app.table.selection_set(('192.168.0.106', '192.168.0.108'))
            app.selection_changed()
            app.begin_motion('forward')
            self.assertIsNone(app.motion)
            app.vars['master_ip'].set('192.168.0.117')
            app.selected_action('master')
            self.assertEqual(actions, [('192.168.0.106', 'master', '192.168.0.117'),
                                       ('192.168.0.108', 'master', '192.168.0.117')])
            app.table.selection_set('192.168.0.108')
            app.selection_changed()
            app.vars['linear'].set('nan')
            app.begin_motion('forward')
            self.assertTrue(all(value == (0, 0) for value in commands.values()))
            app.remark_var.set('第二辆')
            app.save()
            self.assertEqual(json.loads(app.config_path.read_text())['robots']['192.168.0.108']['remark'], '第二辆')
            root.destroy()


if __name__ == '__main__':
    unittest.main(verbosity=2)
