#!/usr/bin/env python3
"""Open a real GNOME terminal against an in-process loopback SSH server; no vehicles."""
import os
from pathlib import Path
import shutil
import socket
import sys
import tempfile
import threading
import time

import paramiko

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from fleet_console import DEFAULTS
from fleet_terminal import Terminal


def main():
    stop, finish_command, shell_open = threading.Event(), threading.Event(), threading.Event()
    executed = []
    server = socket.socket()
    server.bind(('127.0.0.1', 0))
    server.listen(1)
    server.settimeout(1)
    key = paramiko.RSAKey.generate(2048)
    transport = [None]
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
            shell_open.set()
            return True
    def serve():
        try:
            while not stop.is_set():
                try:
                    connection, _ = server.accept()
                    break
                except socket.timeout:
                    continue
            else:
                return
            ssh = transport[0] = paramiko.Transport(connection)
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
            if transport[0]:
                transport[0].close()
    thread = threading.Thread(target=serve, daemon=True)
    thread.start()
    terminal = None
    with tempfile.TemporaryDirectory() as directory:
        try:
            options = dict(DEFAULTS, username='offline-test', password='test-token', port=str(server.getsockname()[1]))
            terminal = Terminal('127.0.0.1', options, Path(directory) / 'known_hosts',
                                'printf offline-test', 'Formation offline terminal check')
            deadline = time.monotonic() + 12
            while terminal.status()['state'] != '运行中':
                if time.monotonic() > deadline:
                    raise AssertionError(terminal.status())
                time.sleep(0.05)
            assert executed and 'printf offline-test' in executed[0], executed
            assert not (terminal.directory / 'request.json').exists(), 'Credentials must be consumed'
            finish_command.set()
            assert shell_open.wait(5), 'A finished command must leave an interactive SSH shell'
            terminal.stop()
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                status = terminal.status()
                if status['state'] == '已关闭':
                    break
                time.sleep(0.1)
            assert terminal.status()['state'] == '已关闭', terminal.status()
            print('Native GNOME terminal passed: loopback SSH login, visible exec, interactive shell, stop and credential-file removal.')
        finally:
            finish_command.set()
            if terminal:
                terminal.stop()
            stop.set()
            if transport[0]:
                transport[0].close()
            server.close()
            thread.join(2)
            if terminal:
                shutil.rmtree(terminal.directory)


if __name__ == '__main__':
    main()
