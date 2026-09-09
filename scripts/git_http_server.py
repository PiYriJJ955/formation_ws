#!/usr/bin/env python3
"""Serve formation.git with Git's Smart HTTP backend (Python 3.6+ and Git)."""
import argparse
import base64
import binascii
from email.parser import BytesHeaderParser
import hmac
from http.client import HTTPConnection
from http.server import BaseHTTPRequestHandler, HTTPServer
import ipaddress
import json
import os
from pathlib import Path
import secrets
import shutil
import socket
from socketserver import ThreadingMixIn
import subprocess
import sys
import tempfile
from urllib.parse import unquote, urlsplit, parse_qs


WORKSPACE = Path(__file__).resolve().parents[1]
REPOSITORY = WORKSPACE / '.local/http/formation.git'
PASSWORD_FILE = WORKSPACE / '.local/git-http-password'
MAX_BODY = 512 * 1024 * 1024


def local_git_urls(port=8000):
    try:
        result = subprocess.run(['ip', '-j', '-4', 'addr', 'show', 'up', 'scope', 'global'],
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                universal_newlines=True, timeout=2, check=True)
        candidates = [address['local'] for interface in json.loads(result.stdout)
                      for address in interface.get('addr_info', []) if address.get('family') == 'inet']
    except (OSError, subprocess.SubprocessError, ValueError, KeyError):
        candidates = []
        # A UDP connect selects the local route without sending a packet.
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                probe.connect(('192.0.2.1', 9))
                candidates.append(probe.getsockname()[0])
        except OSError:
            pass
        try:
            candidates.extend(socket.gethostbyname_ex(socket.gethostname())[2])
        except OSError:
            pass
    urls = []
    for candidate in candidates:
        address = ipaddress.IPv4Address(candidate)
        if address.is_loopback or address.is_unspecified or address.is_link_local or address.is_multicast:
            continue
        url = 'http://%s:%d/formation.git' % (address, port)
        if url not in urls:
            urls.append(url)
    return urls


def server_info():
    connection = HTTPConnection('127.0.0.1', 8000, timeout=2)
    try:
        connection.request('GET', '/health')
        response = connection.getresponse()
        if response.status != 200:
            raise ValueError('请安装并重启 Git HTTP 服务')
        info = json.loads(response.read(8192))
        if not isinstance(info, dict) or info.get('service') != 'formation-git' or info.get('push') is not True:
            raise ValueError('端口 8000 未提供 Git 推送服务')
        return info
    finally:
        connection.close()


def push_password(path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        fd = os.open(str(path), os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
    except FileExistsError:
        pass
    else:
        with os.fdopen(fd, 'w') as stream:
            stream.write(secrets.token_urlsafe(24) + '\n')
    path.chmod(0o600)
    password = path.read_text().strip()
    if not password or '\n' in password or '\r' in password:
        raise ValueError('推送密码文件必须包含一行非空密码：%s' % path)
    return password


class GitHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True

    def __init__(self, address, repository, password):
        self.repository = Path(repository).resolve()
        self.password = password
        bare = subprocess.check_output(['git', '--git-dir=' + str(self.repository),
                                        'rev-parse', '--is-bare-repository']).strip()
        if bare != b'true':
            raise ValueError('Git 服务需要 bare 仓库：%s' % self.repository)
        super().__init__(address, GitHandler)


class GitHandler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def setup(self):
        super().setup()
        self.connection.settimeout(60)

    def authorized(self):
        if self.server.password is None:
            return True
        scheme, _, value = self.headers.get('Authorization', '').partition(' ')
        try:
            supplied = base64.b64decode(value, validate=True) if scheme.lower() == 'basic' else b''
        except (ValueError, binascii.Error):
            supplied = b''
        expected = ('formation:' + self.server.password).encode('utf-8')
        if hmac.compare_digest(supplied, expected):
            return True
        self.send_response(401)
        self.send_header('WWW-Authenticate', 'Basic realm="Formation Git", charset="UTF-8"')
        self.send_header('Content-Length', '0')
        self.send_header('Connection', 'close')
        self.end_headers()
        self.close_connection = True
        return False

    def read_body(self, output):
        encoding = self.headers.get('Transfer-Encoding', '').lower()
        lengths = self.headers.get_all('Content-Length', [])
        if encoding not in ('', 'chunked') or len(lengths) > 1 or (encoding and lengths):
            raise ValueError('Invalid HTTP body framing')
        total = 0
        if encoding != 'chunked':
            remaining = int(lengths[0]) if lengths else 0
            if remaining < 0:
                raise ValueError('Invalid Content-Length')
        while True:
            if encoding == 'chunked':
                line = self.rfile.readline(8193)
                if not line.endswith(b'\r\n') or len(line) > 8192:
                    raise ValueError('Invalid chunk header')
                remaining = int(line.split(b';', 1)[0], 16)
                if remaining < 0:
                    raise ValueError('Invalid chunk size')
                if not remaining:
                    trailer_size = 0
                    while True:
                        trailer = self.rfile.readline(8193)
                        trailer_size += len(trailer)
                        if not trailer.endswith(b'\r\n') or trailer_size > 65536:
                            raise ValueError('Invalid chunk trailers')
                        if trailer == b'\r\n':
                            break
                    break
            total += remaining
            if total > MAX_BODY:
                raise OverflowError('Git request exceeds 512 MiB')
            while remaining:
                data = self.rfile.read(min(remaining, 65536))
                if not data:
                    raise ValueError('Incomplete HTTP body')
                output.write(data)
                remaining -= len(data)
            if encoding != 'chunked':
                break
            if self.rfile.read(2) != b'\r\n':
                raise ValueError('Invalid chunk terminator')
        output.seek(0)
        return total

    def serve_git(self):
        url = urlsplit(self.path)
        path = unquote(url.path)
        if path == '/health' and self.command in ('GET', 'HEAD'):
            data = json.dumps({'service': 'formation-git', 'push': True,
                               'authentication': 'none' if self.server.password is None else 'password'}).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            if self.command != 'HEAD':
                self.wfile.write(data)
            return
        if (not path.startswith('/formation.git/') or '\\' in path or
                any(part in ('.', '..') for part in path.split('/')) or
                any(ord(char) < 32 for char in path)):
            self.send_error(404)
            return
        push = path.endswith('/git-receive-pack') or 'git-receive-pack' in parse_qs(url.query).get('service', [])
        if push and not self.authorized():
            return
        env = dict(os.environ, GIT_PROJECT_ROOT=str(self.server.repository.parent),
                   GIT_HTTP_EXPORT_ALL='1', PATH_INFO=path,
                   PATH_TRANSLATED=str(self.server.repository) + path[len('/formation.git'):],
                   REQUEST_METHOD=self.command, QUERY_STRING=url.query,
                   CONTENT_TYPE=self.headers.get('Content-Type', ''),
                   HTTP_CONTENT_ENCODING=self.headers.get('Content-Encoding', ''),
                   REMOTE_ADDR=self.client_address[0], GIT_PROTOCOL=self.headers.get('Git-Protocol', ''))
        # Authentication is checked above for both discovery and the write RPC.
        env['REMOTE_USER'] = 'formation' if push else ''
        if self.server.repository.name != 'formation.git':
            env.pop('GIT_PROJECT_ROOT')  # PATH_TRANSLATED supports a differently named bare directory.
        with tempfile.TemporaryFile() as body, tempfile.TemporaryFile() as response:
            try:
                env['CONTENT_LENGTH'] = str(self.read_body(body))
            except OverflowError as error:
                self.send_error(413, str(error))
                return
            except (ValueError, OSError) as error:
                self.send_error(400, str(error))
                return
            try:
                # Spool packs to disk so large pushes/clones do not fill RAM.
                result = subprocess.run(['git', 'http-backend'], stdin=body, stdout=response,
                                        stderr=subprocess.PIPE, env=env, timeout=300)
            except (OSError, subprocess.TimeoutExpired):
                self.send_error(502, 'Git backend unavailable')
                return
            if result.stderr:
                self.log_error('%s', result.stderr.decode('utf-8', 'replace').strip())
            size = response.tell()
            response.seek(0)
            headers = bytearray()
            while len(headers) < 65536:
                line = response.readline(8192)
                if line in (b'\r\n', b'\n', b''):
                    break
                headers.extend(line)
            else:
                self.send_error(502, 'Invalid Git backend headers')
                return
            if not headers:
                self.send_error(502, 'Empty Git backend response')
                return
            parsed = BytesHeaderParser().parsebytes(bytes(headers))
            self.send_response(int(parsed.get('Status', '200 OK').split()[0]))
            for key, value in parsed.items():
                if key.lower() not in ('status', 'content-length'):
                    self.send_header(key, value)
            self.send_header('Content-Length', str(size - response.tell()))
            self.end_headers()
            if self.command != 'HEAD':
                shutil.copyfileobj(response, self.wfile)

    do_GET = do_HEAD = do_POST = serve_git


def install_service():
    def quoted(path):
        return '"' + str(path).replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%').replace('$', '$$') + '"'
    if '\n' in str(WORKSPACE) or '\r' in str(WORKSPACE):
        raise ValueError('服务仓库路径不能包含换行符')
    unit = (WORKSPACE / 'deploy/formation-git-http.service').read_text()
    unit = unit.replace('@WORKSPACE@', str(WORKSPACE).replace('%', '%%')).replace('@PYTHON@', quoted(sys.executable))
    unit = unit.replace('@SERVER@', quoted(Path(__file__).resolve()))
    if not REPOSITORY.exists():
        REPOSITORY.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(['git', 'clone', '--bare', str(WORKSPACE), str(REPOSITORY)], check=True)
    hook = REPOSITORY / 'hooks/post-update'
    if not hook.exists():
        shutil.copyfile(str(WORKSPACE / 'deploy/post-update'), str(hook))
        hook.chmod(0o755)
    subprocess.run(['git', '--git-dir=' + str(REPOSITORY), 'update-server-info'], check=True)
    folder = Path(os.environ.get('XDG_CONFIG_HOME', Path.home() / '.config')) / 'systemd/user'
    folder.mkdir(parents=True, exist_ok=True)
    subprocess.run(['systemctl', '--user', 'disable', 'formation-git-http.service'],
                   stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    (folder / 'formation-git-http.service').write_text(unit)
    subprocess.run(['systemctl', '--user', 'daemon-reload'], check=True)
    print('Git service installed; open fleet_console.py to start it.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bind', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8000)
    parser.add_argument('--repository', type=Path, default=REPOSITORY)
    parser.add_argument('--password-file', type=Path, default=PASSWORD_FILE)
    parser.add_argument('--anonymous-push', action='store_true', help='Allow LAN clients to push without a password')
    parser.add_argument('--install-service', action='store_true', help='Install the Linux user service for this checkout')
    args = parser.parse_args()
    if args.install_service:
        install_service()
        return
    password = None if args.anonymous_push else push_password(args.password_file)
    with GitHTTPServer((args.bind, args.port), args.repository, password) as server:
        print('Git HTTP: %s:%d/formation.git' % server.server_address, flush=True)
        if args.bind == '0.0.0.0':
            for url in local_git_urls(server.server_port):
                print('Repository: ' + url, flush=True)
        server.serve_forever()


if __name__ == '__main__':
    main()
