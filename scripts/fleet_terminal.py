#!/usr/bin/env python3
"""Visible native terminal with an SSH PTY. Credentials travel in a 0600 file."""
import argparse
import json
import os
from pathlib import Path
import select
import shutil
import signal
import subprocess
import sys
import tempfile
import termios
import threading
import time
import tty
import weakref


def write_json(path, value):
    temporary = str(path) + '.tmp'
    fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, 'w') as stream:
        json.dump(value, stream, ensure_ascii=False)
    os.replace(temporary, path)


class Terminal:
    _tabs = weakref.WeakSet()
    _lock = threading.Lock()

    def __init__(self, ip, options, host_keys, command='', title='SSH'):
        executable = shutil.which('gnome-terminal')
        if not executable:
            raise RuntimeError('需要 gnome-terminal：sudo apt install gnome-terminal')
        self.directory = Path(tempfile.mkdtemp(prefix='formation-terminal-'))
        self.ip, self.title, self.started = ip, title, time.monotonic()
        request = self.directory / 'request.json'
        write_json(request, dict(ip=ip, options=options, host_keys=str(host_keys), command=command))
        try:
            # GNOME needs the previous screen ID to attach separate invocations as tabs.
            with self._lock:
                environment = dict(os.environ)
                for key in ('GNOME_TERMINAL_SERVICE', 'GNOME_TERMINAL_SCREEN'):
                    environment.pop(key, None)
                for terminal in list(self._tabs):
                    status = terminal.status()
                    if 'pid' in status:
                        try:
                            os.kill(status['pid'], 0)
                        except ProcessLookupError:
                            continue
                    elif status['state'] != '打开终端中':
                        continue
                    environment.update(terminal.environment)
                    break
                result = subprocess.run([executable, '--tab', '--print-environment', '--title=' + title, '--',
                                         sys.executable, str(Path(__file__).resolve()), str(request)],
                                        env=environment, stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE, universal_newlines=True, check=True, timeout=10)
                self.environment = dict(line.split('=', 1) for line in result.stdout.splitlines()
                                        if line.startswith(('GNOME_TERMINAL_SERVICE=', 'GNOME_TERMINAL_SCREEN=')))
                if 'GNOME_TERMINAL_SCREEN' not in self.environment:
                    raise RuntimeError('终端未启动：' + result.stderr.strip())
                self._tabs.add(self)
        except Exception:
            shutil.rmtree(str(self.directory))
            raise

    def status(self):
        try:
            result = json.loads((self.directory / 'status.json').read_text())
            if result['state'] in ('正在连接', '运行中', 'SSH 已连接', '已结束'):
                try:
                    os.kill(result['pid'], 0)
                except ProcessLookupError:
                    return {'state': '已关闭', 'error': '终端进程已退出'}
            return result
        except (OSError, ValueError):
            if time.monotonic() - self.started > 15:
                try:
                    (self.directory / 'request.json').unlink()
                except FileNotFoundError:
                    pass
                return {'state': '失败', 'error': '终端未启动，请检查桌面会话'}
            return {'state': '打开终端中'}

    def stop(self):
        (self.directory / 'stop').touch()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('request', type=Path)
    args = parser.parse_args()
    directory = args.request.parent
    request = json.loads(args.request.read_text())
    args.request.unlink()  # Remove password as soon as the terminal consumes it.
    status_path = directory / 'status.json'
    client = channel = None
    state = {'state': '正在连接', 'pid': os.getpid()}

    def status(**data):
        state.update(data)
        write_json(status_path, state)

    def resize(*_):
        if channel:
            size = shutil.get_terminal_size()
            channel.resize_pty(width=size.columns, height=size.lines)

    def relay(ch):
        previous = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while True:
                if (directory / 'stop').exists():
                    ch.sendall(b'\x03')
                    return False
                readable, _, _ = select.select([ch, sys.stdin], [], [], 0.1)
                if ch in readable:
                    data = ch.recv(32768)
                    if not data:
                        return True
                    sys.stdout.buffer.write(data)
                    sys.stdout.buffer.flush()
                if sys.stdin in readable:
                    data = os.read(sys.stdin.fileno(), 4096)
                    if not data:
                        return False
                    ch.sendall(data)
                if ch.exit_status_ready() and not ch.recv_ready():
                    return True
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, previous)

    try:
        from fleet_console import connect_ssh
        status()
        if (directory / 'stop').exists():
            status(state='已停止')
            return
        print('SSH %s@%s — Ctrl+C 停止命令，命令结束后可继续输入。' %
              (request['options']['username'], request['ip']))
        client = connect_ssh(request['ip'], request['options'], request['host_keys'])
        channel = client.get_transport().open_session(timeout=5)
        channel.get_pty(term=os.environ.get('TERM', 'xterm-256color'))
        resize()
        signal.signal(signal.SIGWINCH, resize)
        if request['command']:
            import shlex
            print('\n$ ' + request['command'] + '\n', flush=True)
            channel.exec_command('bash -c ' + shlex.quote(request['command']))
            status(state='运行中')
            completed = relay(channel)
            if not completed:
                # Give roslaunch time to stop its children before hanging up the PTY.
                deadline = time.monotonic() + 12
                while not channel.exit_status_ready() and time.monotonic() < deadline:
                    if channel.recv_ready():
                        sys.stdout.buffer.write(channel.recv(32768))
                        sys.stdout.buffer.flush()
                    time.sleep(0.1)
                status(state='已停止')
                return
            code = channel.recv_exit_status()
            status(state='已结束' if code == 0 else '失败', error='退出码 %s' % code if code else '', code=code)
            print('\n命令退出码：%s；现在可以继续输入 SSH 命令。' % code, flush=True)
            channel.close()
            channel = client.get_transport().open_session(timeout=5)
            channel.get_pty(term=os.environ.get('TERM', 'xterm-256color'))
        else:
            status(state='SSH 已连接')
        channel.invoke_shell()
        resize()
        relay(channel)
    except Exception as error:
        status(state='失败', error=str(error))
        print('\nSSH 错误：' + str(error), flush=True)
        # Keep errors visible, while allowing the GUI's stop action to close this window.
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline and not (directory / 'stop').exists():
            if select.select([sys.stdin], [], [], 0.2)[0]:
                break
    finally:
        if channel:
            channel.close()
        if client:
            client.close()
        if state['state'] not in ('失败', '已结束', '已停止'):
            status(state='已关闭')


if __name__ == '__main__':
    main()
