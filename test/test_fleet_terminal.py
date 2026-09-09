#!/usr/bin/env python3
"""Open a real GNOME terminal against an in-process loopback SSH server; no vehicles."""
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time

import paramiko

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from fleet_console import DEFAULTS
from fleet_terminal import Terminal


def main():
    stop, finish_command = threading.Event(), threading.Event()
    executed, shells = [], []
    server = socket.socket()
    server.bind(('127.0.0.1', 0))
    server.listen(1)
    server.settimeout(1)
    key = paramiko.RSAKey.generate(2048)
    transports = []
    class SSHServer(paramiko.ServerInterface):
        def check_auth_password(self, username, password):
            return paramiko.AUTH_SUCCESSFUL if (username, password) == ('offline-test', 'test-token') else paramiko.AUTH_FAILED
        def get_allowed_auths(self, username): return 'password'
        def check_channel_request(self, kind, chanid):
            return paramiko.OPEN_SUCCEEDED if kind == 'session' else paramiko.OPEN_FAILED_ADMINISTRATIVELY_PROHIBITED
        def check_channel_pty_request(self, *args): return True
        def check_channel_window_change_request(self, *args): return True
        def check_channel_exec_request(self, channel, command):
            executed.append(command.decode())
            return True
        def check_channel_shell_request(self, channel):
            shells.append(channel)
            return True
    def serve():
        ssh = None
        try:
            while not stop.is_set():
                try:
                    connection, _ = server.accept()
                    break
                except socket.timeout:
                    continue
            else:
                return
            ssh = paramiko.Transport(connection)
            transports.append(ssh)
            ssh.add_server_key(key)
            ssh.start_server(server=SSHServer())
            channel = ssh.accept(10)
            if channel is None:
                return
            while not executed and not stop.wait(0.05):
                pass
            channel.sendall(b'Offline loopback SSH command is running.\r\n')
            finish_command.wait(10)
            channel.send_exit_status(0)
            channel.close()
            shell = ssh.accept(10)
            if shell:
                shell.settimeout(0.5)
                shell.sendall(b'Offline interactive shell ready.\r\n$ ')
                while not stop.is_set():
                    try:
                        if not shell.recv(1024):
                            break
                    except socket.timeout:
                        pass
                shell.close()
        except EOFError:
            pass  # The terminal can close its transport before the server sends EOF.
        finally:
            if ssh:
                ssh.close()
    groups = ('chassis', 'follower', 'chassis', 'follower')
    threads = [threading.Thread(target=serve, daemon=True) for _ in groups]
    for thread in threads:
        thread.start()
    terminals = []
    with tempfile.TemporaryDirectory() as directory:
        try:
            options = dict(DEFAULTS, username='offline-test', password='test-token', port=str(server.getsockname()[1]))
            for number, group in enumerate(groups):
                terminals.append(Terminal('127.0.0.1', options, Path(directory) / ('known_hosts_%d' % number),
                                          'printf offline-test', 'Formation offline terminal check %s %d' % (group, number),
                                          group=group))
            deadline = time.monotonic() + 12
            while any(terminal.status()['state'] != '运行中' for terminal in terminals):
                if time.monotonic() > deadline:
                    raise AssertionError([terminal.status() for terminal in terminals])
                time.sleep(0.05)
            assert len(executed) == len(groups) and all('printf offline-test' in command for command in executed), executed
            for terminal in terminals:
                assert not (terminal.directory / 'request.json').exists(), 'Credentials must be consumed'
            if os.environ.get('DISPLAY') and shutil.which('xwininfo'):
                windows = subprocess.check_output(['xwininfo', '-root', '-tree'], universal_newlines=True)
                windows = [line for line in windows.splitlines() if 'Formation offline terminal check' in line]
                assert len(windows) == 2, windows
                for group in set(groups):
                    assert sum(group in line for line in windows) == 1, windows
            finish_command.set()
            deadline = time.monotonic() + 5
            while len(shells) < len(groups) and time.monotonic() < deadline:
                time.sleep(0.05)
            assert len(shells) == len(groups), 'Finished commands must leave interactive SSH shells'
            for terminal in terminals:
                terminal.stop()
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                if all(terminal.status()['state'] == '已关闭' for terminal in terminals):
                    break
                time.sleep(0.1)
            assert all(terminal.status()['state'] == '已关闭' for terminal in terminals), [t.status() for t in terminals]
            print('Native GNOME terminal passed: chassis and follower windows with two tabs each, visible exec, interactive shells, stop and credential-file removal.')
        finally:
            finish_command.set()
            for terminal in terminals:
                terminal.stop()
            stop.set()
            for transport in transports:
                transport.close()
            server.close()
            for thread in threads:
                thread.join(2)
            for terminal in terminals:
                shutil.rmtree(terminal.directory)


if __name__ == '__main__':
    main()
