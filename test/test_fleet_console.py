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
from unittest.mock import patch
import queue
import threading
import socket
import select
import shlex
import subprocess

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from fleet_console import (DEFAULTS, FleetConsole, expand_addresses, export_csv,
                           load_settings, save_settings, targets_from, validate, RobotSession, connect_ssh)
from fleet_bridge import velocity, COMMAND_TIMEOUT, check_master_port
from fleet_deploy import replace_master, update_master, read_robot_config, grant_serial_permissions


class ConsoleChecks(unittest.TestCase):
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
