#!/usr/bin/env python3
"""Exercise real Git HTTP clients against an isolated loopback repository."""
import base64
from concurrent.futures import ThreadPoolExecutor
import gzip
import http.client
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from git_http_server import (GitHTTPServer, GitHandler, push_password, install_service,
                             local_git_urls, WORKSPACE, MAX_BODY)


class GitHTTPChecks(unittest.TestCase):
    def test_addresses_follow_local_interfaces_and_work_without_ip_command(self):
        for addresses, expected in [
                (['192.168.8.25', '10.0.0.12', '192.168.8.25', '127.0.0.1', '169.254.1.2', '0.0.0.0'],
                 ['http://192.168.8.25:8123/formation.git', 'http://10.0.0.12:8123/formation.git']),
                (['192.168.8.26'], ['http://192.168.8.26:8123/formation.git']),
                ([], [])]:
            interfaces = [{'addr_info': [{'family': 'inet', 'local': address} for address in addresses]}]
            with patch('git_http_server.subprocess.run', return_value=subprocess.CompletedProcess(
                    [], 0, json.dumps(interfaces), '')):
                self.assertEqual(local_git_urls(8123), expected)
        probe = Mock()
        probe.getsockname.return_value = ('192.168.4.8', 12345)
        with patch('git_http_server.subprocess.run', side_effect=FileNotFoundError()), \
                patch('git_http_server.socket.socket') as socket_type, \
                patch('git_http_server.socket.gethostbyname_ex', return_value=('host', [], ['127.0.1.1'])):
            socket_type.return_value.__enter__.return_value = probe
            self.assertEqual(local_git_urls(), ['http://192.168.4.8:8000/formation.git'])
            probe.connect.side_effect = OSError('network unavailable')
            self.assertEqual(local_git_urls(), [])

    def test_clients_clone_push_pull_and_authentication(self):
        with tempfile.TemporaryDirectory(prefix='git server 中文 ') as directory:
            root = Path(directory)
            env = dict(os.environ, GIT_CONFIG_NOSYSTEM='1', GIT_CONFIG_GLOBAL=os.devnull,
                       GIT_TERMINAL_PROMPT='0', GIT_AUTHOR_NAME='Test', GIT_AUTHOR_EMAIL='test@example.com',
                       GIT_COMMITTER_NAME='Test', GIT_COMMITTER_EMAIL='test@example.com')
            def git(folder, *args, **kwargs):
                return subprocess.run(['git', '-C', str(folder)] + list(args), env=env,
                                      stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                      timeout=30, **kwargs)
            bare = root / 'renamed.git'
            git(root, 'init', '--bare', str(bare), check=True)
            git(root, 'init', 'seed', check=True)
            seed = root / 'seed'
            git(seed, 'checkout', '-b', 'master', check=True)
            (seed / 'hello.txt').write_text('first\n')
            git(seed, 'add', '.', check=True)
            git(seed, 'commit', '-m', 'initial', check=True)
            git(seed, 'push', str(bare), 'master', check=True)
            git(bare, 'symbolic-ref', 'HEAD', 'refs/heads/master', check=True)
            password_file = root / 'password'
            password = push_password(password_file)
            self.assertEqual(push_password(password_file), password)
            self.assertEqual(password_file.stat().st_mode & 0o777, 0o600)
            auth = 'Authorization: Basic ' + base64.b64encode(('formation:' + password).encode()).decode()
            with GitHTTPServer(('127.0.0.1', 0), bare, password) as server, \
                    patch.object(GitHandler, 'log_message'):
                thread = threading.Thread(target=server.serve_forever, daemon=True)
                thread.start()
                url = 'http://127.0.0.1:%d/formation.git' % server.server_port
                try:
                    # Independent clients negotiate old and current Git protocol versions.
                    def clone(version):
                        git(root, '-c', 'protocol.version=' + str(version), 'clone', url,
                            'client%d' % version, check=True)
                    with ThreadPoolExecutor(max_workers=2) as pool:
                        list(pool.map(clone, (0, 2)))
                    first, second = root / 'client0', root / 'client2'
                    (first / 'large.bin').write_bytes(os.urandom(2 * 1024 * 1024))
                    git(first, 'add', '.', check=True)
                    git(first, 'commit', '-m', 'large change', check=True)
                    self.assertNotEqual(git(first, 'push').returncode, 0)
                    self.assertNotEqual(git(first, '-c', 'http.extraHeader=Authorization: Basic !!!', 'push').returncode, 0)
                    # A small postBuffer forces Git's chunked upload path.
                    git(first, '-c', 'http.postBuffer=1024', '-c', 'http.extraHeader=' + auth,
                        'push', check=True)
                    git(second, '-c', 'protocol.version=2', 'pull', '--ff-only', check=True)
                    self.assertEqual((second / 'large.bin').read_bytes(), (first / 'large.bin').read_bytes())
                    (second / 'hello.txt').write_text('second computer\n')
                    git(second, 'commit', '-am', 'second change', check=True)
                    git(second, '-c', 'http.extraHeader=' + auth, 'push', check=True)
                    (first / 'hello.txt').write_text('concurrent change\n')
                    git(first, 'commit', '-am', 'divergent change', check=True)
                    self.assertNotEqual(git(first, '-c', 'http.extraHeader=' + auth, 'push').returncode, 0)
                    self.assertEqual(git(bare, 'rev-parse', 'master', check=True).stdout,
                                     git(second, 'rev-parse', 'HEAD', check=True).stdout)
                    connection = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=5)
                    for path in ('/formation.git/git-receive-pack', '/formation.git/info/refs?service=git-receive-pack'):
                        connection.request('POST' if path.endswith('/git-receive-pack') else 'GET', path)
                        response = connection.getresponse()
                        self.assertEqual(response.status, 401)
                        self.assertIn('Basic', response.getheader('WWW-Authenticate'))
                        response.read()
                    for path in ('/formation.git/config', '/formation.git/%2e%2e/password',
                                 '/formation.git/..%5cpassword', '/password', '/other.git/HEAD'):
                        connection.request('GET', path)
                        response = connection.getresponse()
                        self.assertEqual(response.status, 404, path)
                        response.read()
                    connection.request('GET', '/health')
                    health = json.loads(connection.getresponse().read())
                    self.assertEqual(health['authentication'], 'password')
                    # Git can gzip negotiation requests when many refs/commits are involved.
                    def pkt(value):
                        return ('%04x' % (len(value) + 4)).encode() + value
                    request = pkt(b'command=ls-refs\n') + b'0001' + pkt(b'symrefs\n') + b'0000'
                    headers = {'Content-Type': 'application/x-git-upload-pack-request', 'Git-Protocol': 'version=2'}
                    connection.request('POST', '/formation.git/git-upload-pack', gzip.compress(request),
                                       dict(headers, **{'Content-Encoding': 'gzip'}))
                    response = connection.getresponse()
                    self.assertEqual(response.status, 200)
                    self.assertIn(b'refs/heads/master', response.read())
                    connection.putrequest('POST', '/formation.git/git-upload-pack')
                    for key, value in dict(headers, **{'Transfer-Encoding': 'chunked'}).items():
                        connection.putheader(key, value)
                    connection.endheaders()
                    connection.send(('%x\r\n' % len(request)).encode() + request + b'\r\n0\r\nX-Test: trailer\r\n\r\n')
                    response = connection.getresponse()
                    self.assertEqual(response.status, 200)
                    self.assertIn(b'refs/heads/master', response.read())
                    for framing, status in [({'Content-Length': '-1'}, 400),
                                            ({'Content-Length': '1', 'Transfer-Encoding': 'chunked'}, 400),
                                            ({'Content-Length': str(MAX_BODY + 1)}, 413)]:
                        connection.request('POST', '/formation.git/git-upload-pack', b'', framing)
                        response = connection.getresponse()
                        self.assertEqual(response.status, status)
                        response.read()
                    # The anonymous LAN mode uses the same native Git protocol.
                    server.password = None
                    git(second, 'commit', '--allow-empty', '-m', 'LAN write', check=True)
                    git(second, 'push', check=True)
                    connection.close()
                finally:
                    server.shutdown()
                    thread.join(5)

    def test_service_paths_follow_checkout_and_python(self):
        with tempfile.TemporaryDirectory(prefix='service space 中文 %$ ') as directory:
            workspace = Path(directory) / 'workspace'
            repository = workspace / '.local/http/formation.git'
            (workspace / 'deploy').mkdir(parents=True)
            (workspace / 'scripts').mkdir()
            for name in ('formation-git-http.service', 'post-update'):
                shutil.copyfile(str(WORKSPACE / 'deploy' / name), str(workspace / 'deploy' / name))
            subprocess.run(['git', 'init', str(workspace)], check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            original_run = subprocess.run
            def run_command(command, **kwargs):
                if command[0] == 'systemctl':
                    return subprocess.CompletedProcess(command, 0)
                return original_run(command, **kwargs)
            with patch.dict(os.environ, XDG_CONFIG_HOME=directory), \
                    patch('git_http_server.WORKSPACE', workspace), patch('git_http_server.REPOSITORY', repository), \
                    patch('git_http_server.subprocess.run', side_effect=run_command) as run:
                install_service()
            unit_path = Path(directory) / 'systemd/user/formation-git-http.service'
            unit = unit_path.read_text()
            self.assertIn('scripts/git_http_server.py"', unit)
            self.assertIn('"' + sys.executable + '"', unit)
            self.assertNotIn('@WORKSPACE@', unit)
            self.assertNotIn('[Install]', unit)
            self.assertEqual(run.call_args_list[-1][0][0], ['systemctl', '--user', 'daemon-reload'])
            self.assertEqual((repository / 'hooks/post-update').read_text(), (workspace / 'deploy/post-update').read_text())
            if shutil.which('systemd-analyze'):
                subprocess.run(['systemd-analyze', '--user', 'verify', str(unit_path)], check=True,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE)


if __name__ == '__main__':
    unittest.main(verbosity=2)
