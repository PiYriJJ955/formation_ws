#!/usr/bin/env python3
"""LAN discovery, persistent SSH connections and hold-to-drive mini_4wd GUI."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import csv
from contextlib import contextmanager
from datetime import datetime
import fcntl
import ipaddress
import json
import math
import os
from pathlib import Path
import queue
import re
import shlex
import socket
import tempfile
import threading
import time
import uuid
from urllib.parse import urlsplit

import paramiko
from fleet_deploy import (read_robot_config, sync_workspace, update_master, update_identity,
                          robot_number, grant_serial_permissions)
from fleet_workbench import DEFAULTS as WORKBENCH_DEFAULTS


CONFIG_DIR = Path(os.environ.get('XDG_CONFIG_HOME', Path.home() / '.config')) / 'formation-console'
CONFIG_PATH = CONFIG_DIR / 'settings.json'
DEFAULTS = {
    'targets': '192.168.0.1-254', 'exclude': '192.168.0.111,192.168.0.112,192.168.0.117,192.168.0.136',
    'username': 'wheeltec', 'password': os.environ.get('FORMATION_SSH_PASSWORD', ''), 'port': '22',
    'timeout': '2', 'workers': '32', 'interval': '30',
    'workspace': '~/formation_ws', 'loop': False, 'auto_start': True,
    'repository': 'http://192.168.0.117:8000/formation.git', 'auto_sync': True,
    'master_ip': '192.168.0.106',
    'linear': '0.10', 'angular': '0.40', 'geometry': '1180x780',
    'selected': '', 'export': str(Path.home() / '.local/share/formation-console/fleet.csv'),
}
KNOWN = {'192.168.0.106': 'ugv1', '192.168.0.108': 'ugv2', '192.168.0.109': 'ugv3',
         '192.168.0.110': 'ugv4', '192.168.0.114': 'ugv5'}
BLOCKED_IPS = {'192.168.0.136'}  # Other VM, never enroll or connect as a robot.
DEFAULTS.update(WORKBENCH_DEFAULTS)
CSV_FIELDS = ['ip', 'name', 'robot_id', 'remark', 'ssh', 'repository', 'ros_master',
              'chassis', 'car_mode', 'voltage', 'last_seen', 'error']
HOST_KEY_LOCK = threading.Lock()


@contextmanager
def host_key_lock(path):
    # Native SSH terminal windows run in separate processes.
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with HOST_KEY_LOCK:
        fd = os.open(str(path) + '.lock', os.O_CREAT | os.O_RDWR, 0o600)
        with os.fdopen(fd, 'w') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            yield


def expand_addresses(spec):
    result = set()
    for token in re.split(r'[,;，；\s]+', spec.strip()):
        if not token:
            continue
        if '/' in token:
            network = ipaddress.IPv4Network(token, strict=False)
            if network.num_addresses > 4096:
                raise ValueError('单段范围最多 4096 个地址')
            addresses = [network.network_address] if network.prefixlen == 32 else network.hosts()
        elif '-' in token:
            first, last = token.split('-', 1)
            if '.' not in last:
                last = first.rsplit('.', 1)[0] + '.' + last
            start, end = int(ipaddress.IPv4Address(first)), int(ipaddress.IPv4Address(last))
            if not 0 <= end - start < 4096:
                raise ValueError('IP 范围必须递增，且最多 4096 个地址')
            addresses = (ipaddress.IPv4Address(i) for i in range(start, end + 1))
        else:
            addresses = [ipaddress.IPv4Address(token)]
        result.update(str(ip) for ip in addresses)
        if len(result) > 4096:
            raise ValueError('每轮最多扫描 4096 个地址')
    return result


def targets_from(options):
    addresses = expand_addresses(options['targets']) - expand_addresses(options['exclude']) - BLOCKED_IPS
    if not addresses:
        raise ValueError('排除后没有可扫描的 IP')
    return sorted(addresses, key=ipaddress.IPv4Address)


def validate(options):
    targets_from(options)
    for name, low, high in [('port', 1, 65535), ('workers', 1, 64)]:
        if not low <= int(options[name]) <= high:
            raise ValueError('%s 必须在 %s–%s 之间' % (name, low, high))
    for name, low, high in [('timeout', 0.2, 30), ('interval', 1, 3600),
                            ('linear', 0.01, 0.5), ('angular', 0.01, 1.5)]:
        value = float(options[name])
        if not math.isfinite(value) or not low <= value <= high:
            raise ValueError('%s 必须在 %s–%s 之间' % (name, low, high))
    if not options['username'].strip() or not options['workspace'].strip():
        raise ValueError('用户名和工作空间不能为空')
    ipaddress.IPv4Address(options['master_ip'])
    url = urlsplit(options['repository'])
    if url.scheme not in ('http', 'https') or not url.hostname:
        raise ValueError('仓库地址必须是完整的 HTTP / HTTPS URL')
    if not options['workspace'].startswith(('~/', '/')) or '\n' in options['workspace']:
        raise ValueError('工作空间应使用 ~/ 开头或绝对路径')


def save_settings(path, options, robots):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix='.settings-', dir=str(path.parent))
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            json.dump({'options': options, 'robots': robots}, stream, ensure_ascii=False, indent=2)
        os.replace(temporary, path)  # mkstemp creates mode 0600, including remembered password.
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def load_settings(path):
    options = dict(DEFAULTS)
    robots = {ip: {'name': name, 'remark': ''} for ip, name in KNOWN.items()}
    if Path(path).exists():
        with open(path, encoding='utf-8') as stream:
            data = json.load(stream)
        options.update(data['options'])
        robots.update(data['robots'])
    for ip in BLOCKED_IPS:
        robots.pop(ip, None)
    for key in ('selected', 'formation_selected'):
        options[key] = ','.join(ip for ip in options.get(key, '').split(',') if ip not in BLOCKED_IPS)
    return options, robots


def export_csv(path, robots):
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8-sig', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS, extrasaction='ignore')
        writer.writeheader()
        for ip in sorted(set(robots) - BLOCKED_IPS, key=ipaddress.IPv4Address):
            row = dict(robots[ip], ip=ip, car_mode='mini_4wd')
            # Remarks and hostnames can be opened in spreadsheet software.
            for key, value in row.items():
                if isinstance(value, str) and value.lstrip().startswith(('=', '+', '-', '@')):
                    row[key] = "'" + value
            writer.writerow(row)


class RememberHostKey(paramiko.MissingHostKeyPolicy):
    def __init__(self, path):
        self.path = Path(path)

    def missing_host_key(self, client, hostname, key):
        with host_key_lock(self.path):
            keys = paramiko.HostKeys()
            if self.path.exists():
                keys.load(str(self.path))
            existing = keys.lookup(hostname)
            if existing and key.get_name() in existing and existing[key.get_name()] != key:
                raise paramiko.BadHostKeyException(hostname, key, existing[key.get_name()])
            keys.add(hostname, key.get_name(), key)
            self.path.parent.mkdir(parents=True, exist_ok=True)
            keys.save(str(self.path))
            os.chmod(self.path, 0o600)
            client.get_host_keys().add(hostname, key.get_name(), key)


def connect_ssh(ip, options, host_keys):
    if ip in BLOCKED_IPS:
        raise ValueError('非小车 IP 已屏蔽：%s（虚拟机）' % ip)
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    with host_key_lock(host_keys):
        if Path(host_keys).exists():
            client.load_host_keys(str(host_keys))
    client.set_missing_host_key_policy(RememberHostKey(host_keys))
    try:
        timeout = float(options['timeout'])
        client.connect(ip, port=int(options['port']), username=options['username'],
                       password=options['password'], timeout=timeout,
                       banner_timeout=max(3, timeout), auth_timeout=max(3, timeout),
                       look_for_keys=False, allow_agent=False)
        client.get_transport().set_keepalive(2)
        return client
    except Exception:
        client.close()
        raise


def workspace_setup(workspace):
    workspace = workspace.rstrip('/')
    if workspace.startswith('~/'):
        return '"$HOME"/' + shlex.quote(workspace[2:] + '/devel/setup.bash')
    return shlex.quote(workspace + '/devel/setup.bash')


class RobotSession:
    def __init__(self, ip, client, options, events):
        self.ip, self.client, self.options, self.events = ip, client, dict(options), events
        self.ready = False
        self.starting = False
        self.closed = threading.Event()
        self.command = (0.0, 0.0, 0.0)
        self.thread = None
        self.task_thread = None
        self.bridge_stop = threading.Event()
        self.synced = False
        self.robot_id = KNOWN.get(ip, '')
        self.host_keys = CONFIG_PATH.with_name('known_hosts')

    def emit(self, event, **data):
        self.events.put((event, self, data))

    def alive(self):
        transport = self.client.get_transport() if self.client is not None else None
        return not self.closed.is_set() and transport is not None and transport.is_active()

    def drive(self, linear=0.0, angular=0.0):
        self.command = (linear, angular, time.monotonic())

    def busy(self):
        return self.task_thread is not None and self.task_thread.is_alive()

    def start(self, launch=True):
        if not (self.thread and self.thread.is_alive()):
            self.maintain('prepare', launch=launch)

    def maintain(self, action, options=None, launch=False):
        if self.closed.is_set() or self.busy():
            return False
        if options:
            self.options.update(options)
        self.task_thread = threading.Thread(target=self.maintenance, args=(action, launch), daemon=True)
        self.task_thread.start()
        return True

    def maintenance(self, action, launch):
        failure = ''
        result = ''
        try:
            if not self.alive():
                if self.client is not None:
                    self.client.close()
                self.emit('status', ssh='正在连接', error='')
                self.client = connect_ssh(self.ip, self.options, self.host_keys)
                self.emit('status', ssh='已连接', last_seen=datetime.now().isoformat(timespec='seconds'))
            if self.closed.is_set():
                return
            if action == 'sync' or (action == 'prepare' and self.options['auto_sync'] and not self.synced):
                self.ready = False
                self.drive()
                self.bridge_stop.set()
                if self.thread and self.thread.is_alive():
                    self.thread.join(15)
                    if self.thread.is_alive():
                        raise RuntimeError('底盘尚未关闭，请稍后重试同步')
                self.emit('status', repository='正在同步/编译', chassis='未启动', error='')
                config = sync_workspace(self.client, self.options, self.ip, self.robot_id, self.closed,
                                        lambda text: self.emit('status', progress=text))
                self.synced = True
                self.emit('status', repository='已同步', progress='', **config)
            elif action == 'serial':
                self.emit('status', error='', progress='正在设置 /dev/ttyCH343USB* 权限…')
                result = grant_serial_permissions(self.client, self.options['password'], self.closed)
                config = {}
                self.emit('status', error='', progress='串口权限已设置为 777：' + result.replace('\n', '；'))
            elif action == 'identity':
                config = update_identity(self.client, self.options['desired_id'])
                self.emit('status', **config, error='', progress='UGV_ID 已同步；新启动的 ROS 进程生效')
            elif action == 'master':
                config = update_master(self.client, self.options['master_ip'])
                self.emit('status', **config, error='', progress='ROS Master 已保存；新启动的 ROS 进程生效')
            else:
                config = read_robot_config(self.client)
                self.emit('status', **config, error='', progress='已读取车端配置', master_read=action == 'read')
            if action in ('prepare', 'sync') and self.options.get('desired_id'):
                config = update_identity(self.client, self.options['desired_id'])
                self.emit('status', **config, error='', progress='UGV_ID 已同步')
            if config.get('robot_id'):
                self.robot_id = config['robot_id']
            if launch and not self.closed.is_set():
                self.bridge_stop.clear()
                self.starting = True
                self.thread = threading.Thread(target=self.run, daemon=True)
                self.thread.start()
        except Exception as error:
            failure = str(error)
            if not self.closed.is_set():
                state = {'error': str(error), 'progress': '操作失败'}
                if action in ('sync', 'prepare'):
                    state['repository'] = '等待更新' if getattr(error, 'returncode', None) == 75 else '同步失败'
                self.emit('status', **state)
        finally:
            self.emit('task_done', action=action, error=failure, result=result)
            if self.closed.is_set() and self.client is not None:
                self.client.close()

    def close(self):
        if self.closed.is_set():
            return
        self.drive()
        self.closed.set()
        if not self.busy() and (not self.thread or not self.thread.is_alive()):
            if self.client is not None:
                self.client.close()
            self.emit('status', ssh='已断开', chassis='已停止')

    def run(self):
        remote = '/tmp/formation-console-%s.py' % uuid.uuid4().hex
        channel = None
        try:
            self.emit('status', chassis='正在启动', error='')
            with self.client.open_sftp() as sftp:
                sftp.put(str(Path(__file__).with_name('fleet_bridge.py')), remote)
                sftp.chmod(remote, 0o600)
            command = ('set -e; source %s; trap %s EXIT; python -u %s' % (
                workspace_setup(self.options['workspace']),
                shlex.quote('rm -f ' + shlex.quote(remote)), shlex.quote(remote)))
            channel = self.client.get_transport().open_session(timeout=5)
            channel.set_combine_stderr(True)
            channel.exec_command('bash -c ' + shlex.quote(command))
            channel.settimeout(0.1)
            buffer, recent_log = b'', ''
            tick, sent = 0.0, 0.0
            received = time.monotonic()
            deadline = received + 50
            while not self.closed.is_set() and not self.bridge_stop.is_set():
                now = time.monotonic()
                if channel.recv_ready():
                    chunk = channel.recv(32768)
                    if not chunk:
                        break
                    buffer += chunk
                    if len(buffer) > 65536:
                        raise RuntimeError('底盘输出过长')
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1)
                        text = line.decode('utf-8', errors='replace')
                        try:
                            data = json.loads(text)
                            event = data.get('event')
                        except (ValueError, AttributeError):
                            recent_log = text[-500:]
                            continue
                        if event == 'ready':
                            self.ready, self.starting = True, False
                            self.emit('status', chassis='就绪', error='')
                            received = now
                        elif event == 'state':
                            tick = data['tick']
                            received = now
                            data.pop('event', None)
                            self.emit('telemetry', **data)
                        elif event == 'error':
                            raise RuntimeError(data['message'])
                if channel.exit_status_ready() and not channel.recv_ready():
                    raise RuntimeError(recent_log or '底盘进程已退出，请检查车端 chassis.log')
                if self.ready:
                    if now - received > 2:
                        raise RuntimeError('底盘心跳超时，已停止控制')
                    if now - sent >= 0.1:
                        linear, angular, changed = self.command
                        if now - changed > 0.3:
                            linear = angular = 0.0
                        payload = {'tick': tick, 'linear': linear, 'angular': angular}
                        channel.sendall((json.dumps(payload) + '\n').encode())
                        sent = now
                elif now > deadline:
                    raise RuntimeError('底盘启动超时，请检查车端 chassis.log')
                self.closed.wait(0.02)
        except Exception as error:
            if not self.closed.is_set():
                self.emit('status', chassis='启动/连接失败', error=str(error))
        finally:
            self.ready = self.starting = False
            if channel is not None:
                try:
                    channel.sendall(b'{"quit": true}\n')
                    channel.shutdown_write()
                    deadline = time.monotonic() + 12
                    while not channel.exit_status_ready() and time.monotonic() < deadline:
                        if channel.recv_ready():
                            channel.recv(32768)
                        time.sleep(0.05)
                except Exception:
                    pass
                channel.close()
            try:
                with self.client.open_sftp() as sftp:
                    sftp.remove(remote)
            except Exception:
                pass
            if self.closed.is_set():
                self.client.close()
                self.emit('status', ssh='已断开', chassis='已停止')
            elif self.bridge_stop.is_set():
                self.emit('status', chassis='已停止')


class FleetConsole:
    def __init__(self, root, config_path=CONFIG_PATH):
        import tkinter as tk
        from tkinter import ttk, messagebox
        self.tk, self.ttk, self.messagebox = tk, ttk, messagebox
        self.root, self.config_path = root, Path(config_path)
        self.options, self.robots = load_settings(self.config_path)
        self.events = queue.Queue()
        self.sessions = {}
        self.scanning = False
        self.scan_cancel = threading.Event()
        self.closing = False
        self.loop_timer = self.save_timer = None
        self.motion = None
        self.motion_source = None
        self.key_releases = {}
        self.keys_down = set()
        self.selected_ip = ''
        self.selected_rows = ()
        self.vars = {key: (tk.BooleanVar(value=value) if isinstance(value, bool)
                           else tk.StringVar(value=value)) for key, value in self.options.items()}
        root.title('小车编队控制台 · mini_4wd')
        root.geometry(self.options['geometry'])
        root.minsize(1120, 780)
        style = ttk.Style(root)
        style.theme_use('clam')
        style.configure('Treeview', rowheight=30)
        style.configure('TButton', padding=6)
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill='both', expand=True)
        outer = ttk.Frame(self.notebook, padding=8)
        self.notebook.add(outer, text='扫描与连接')
        settings = ttk.LabelFrame(outer, text='扫描与连接（自动记住上次设置）', padding=10)
        settings.pack(fill='x')
        self.entry(settings, '扫描范围', 'targets', 0, 0, width=65, span=5)
        self.entry(settings, '排除列表', 'exclude', 1, 0, width=65, span=5)
        ttk.Label(settings, text='支持 192.168.0.1-254、192.168.0.0/24、单个 IP；多个用逗号或空格分隔').grid(
            row=2, column=0, columnspan=6, sticky='w', pady=(0, 5))
        self.entry(settings, 'SSH 用户', 'username', 3, 0)
        self.entry(settings, 'SSH 密码', 'password', 3, 2, show='•')
        self.entry(settings, 'SSH 端口', 'port', 3, 4, width=8)
        self.entry(settings, '超时（秒）', 'timeout', 4, 0)
        self.entry(settings, '并发数', 'workers', 4, 2)
        self.entry(settings, '循环间隔（秒）', 'interval', 4, 4, width=8)
        self.entry(settings, '车端工作空间', 'workspace', 5, 0, width=40, span=3)
        toggles = ttk.Frame(settings)
        toggles.grid(row=5, column=4, columnspan=2, sticky='w')
        ttk.Checkbutton(toggles, text='循环检测', variable=self.vars['loop'], command=self.loop_changed).pack(side='left')
        ttk.Checkbutton(toggles, text='连接后启动底盘', variable=self.vars['auto_start']).pack(side='left')
        self.entry(settings, 'Git 仓库地址', 'repository', 6, 0, width=60, span=3)
        ttk.Checkbutton(settings, text='SSH 连接后自动克隆 / 同步', variable=self.vars['auto_sync']).grid(
            row=6, column=4, columnspan=2, sticky='w')
        for column in (1, 3, 5):
            settings.columnconfigure(column, weight=1)
        toolbar = ttk.Frame(outer)
        toolbar.pack(fill='x', pady=8)
        self.scan_button = ttk.Button(toolbar, text='扫描并连接', command=self.scan)
        self.scan_button.pack(side='left')
        ttk.Button(toolbar, text='停止扫描', command=self.cancel_scan).pack(side='left', padx=5)
        ttk.Button(toolbar, text='启动已连接底盘', command=self.start_chassis).pack(side='left')
        ttk.Button(toolbar, text='断开全部 / 关闭底盘', command=self.disconnect_all).pack(side='left', padx=5)
        ttk.Button(toolbar, text='导出列表 CSV', command=self.choose_export).pack(side='right')
        management = ttk.Frame(outer)
        management.pack(fill='x', pady=(0, 6))
        ttk.Button(management, text='同步选中车辆', command=lambda: self.selected_action('sync')).pack(side='left')
        ttk.Label(management, text='编队 ROS Master IP').pack(side='left', padx=(12, 5))
        ttk.Entry(management, textvariable=self.vars['master_ip'], width=16).pack(side='left')
        ttk.Button(management, text='读取选中配置', command=lambda: self.selected_action('read')).pack(side='left', padx=5)
        ttk.Button(management, text='更新选中 ROS Master', command=lambda: self.selected_action('master')).pack(side='left')
        ttk.Label(management, text='Ctrl / Shift 多选 IP').pack(side='left', padx=10)
        self.status = tk.StringVar(value='就绪：扫描后自动连接；选择车辆，按住方向按钮行驶。')
        ttk.Label(outer, textvariable=self.status).pack(fill='x')
        table_frame = ttk.Frame(outer)
        table_frame.pack(fill='both', expand=True, pady=8)
        columns = ('ip', 'name', 'remark', 'ssh', 'repository', 'chassis', 'ros_master', 'voltage')
        self.table = ttk.Treeview(table_frame, columns=columns, show='headings', selectmode='extended', height=4)
        for col, title, width in zip(columns, ['IP', '名称', '备注', 'SSH', '仓库', '底盘', '车端 ROS Master', '电压 V'],
                                     [130, 70, 125, 90, 115, 115, 220, 70]):
            self.table.heading(col, text=title)
            self.table.column(col, width=width, minwidth=60, stretch=col == 'remark')
        scrollbar = ttk.Scrollbar(table_frame, orient='vertical', command=self.table.yview)
        self.table.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side='right', fill='y')
        self.table.pack(side='left', fill='both', expand=True)
        self.table.bind('<<TreeviewSelect>>', self.selection_changed)
        for ip, row in self.robots.items():
            row.update(ssh='未连接', chassis='未启动', voltage='', error='')
            row.setdefault('robot_id', KNOWN.get(ip, ''))
            self.refresh_row(ip)
        details = ttk.Frame(outer)
        details.pack(fill='x')
        self.name_var, self.remark_var = tk.StringVar(), tk.StringVar()
        for label, variable, width in [('UGV 名称', self.name_var, 14), ('备注', self.remark_var, 45)]:
            ttk.Label(details, text=label).pack(side='left', padx=(0, 4))
            entry = ttk.Entry(details, textvariable=variable, width=width)
            entry.pack(side='left', padx=(0, 12))
            if variable is self.name_var:
                self.name_entry = entry
                entry.bind('<Return>', self.apply_identity)
                entry.bind('<FocusOut>', self.apply_identity)
            variable.trace_add('write', self.edit_robot)
        ttk.Label(details, text='名称回车或离开输入框后自动 SSH 应用').pack(side='left')
        self.detail = tk.StringVar(value='尚未选择车辆')
        ttk.Label(outer, textvariable=self.detail, wraplength=1080).pack(fill='x', pady=5)
        controls = ttk.LabelFrame(outer, text='按住行驶 · 松手停车 · 切换车辆先停车', padding=8)
        controls.pack(fill='x')
        speed = ttk.Frame(controls)
        speed.pack(side='left', fill='y')
        self.entry(speed, '线速度 m/s', 'linear', 0, 0, width=8)
        self.entry(speed, '角速度 rad/s', 'angular', 1, 0, width=8)
        keyboard = ttk.Button(speed, text='点击这里启用 WASD / 方向键', takefocus=True)
        keyboard.configure(command=keyboard.focus_set)
        keyboard.grid(row=2, column=0, columnspan=2, pady=5)
        keyboard.bind('<KeyPress>', self.key_press)
        keyboard.bind('<KeyRelease>', self.key_release)
        keyboard.bind('<FocusOut>', lambda _: self.stop_motion() if self.motion_source == 'keyboard' else None)
        pad = ttk.Frame(controls)
        pad.pack(side='left', padx=30)
        for label, direction, row, col in [('▲ 前进', 'forward', 0, 1), ('◀ 左转', 'left', 1, 0),
                                           ('后退 ▼', 'backward', 1, 1), ('右转 ▶', 'right', 1, 2)]:
            button = ttk.Button(pad, text=label, width=10)
            button.grid(row=row, column=col, padx=3, pady=3)
            button.bind('<ButtonPress-1>', lambda event, d=direction: self.mouse_press(event, d))
            button.bind('<Leave>', lambda _: self.stop_motion())
        tk.Button(controls, text='全部停车\n空格 / Esc', bg='#b52b33', fg='white',
                  activebackground='#8c2027', activeforeground='white', font=('', 14, 'bold'),
                  command=lambda: self.workbench.emergency(), padx=18, pady=10).pack(side='right', padx=10)
        root.bind_all('<ButtonRelease-1>', lambda _: self.stop_motion(), add='+')
        root.bind('<Escape>', lambda _: self.stop_motion())
        root.bind('<space>', lambda _: self.stop_motion())
        root.bind('<FocusOut>', lambda _: root.after_idle(self.check_focus))
        root.protocol('WM_DELETE_WINDOW', self.close)
        from fleet_workbench import FleetWorkbench
        self.workbench = FleetWorkbench(self)
        self.notebook.select(max(0, min(2, int(self.vars['active_tab'].get()))))
        def tab_changed(_):
            self.stop_motion()
            self.workbench.armed.set(False)
            self.vars['active_tab'].set(str(self.notebook.index(self.notebook.select())))
        self.notebook.bind('<<NotebookTabChanged>>', tab_changed)
        self.batches = {}
        for variable in self.vars.values():
            variable.trace_add('write', self.schedule_save)
        selected = [ip for ip in self.options.get('selected', '').split(',') if ip in self.robots]
        if selected:
            self.table.selection_set(selected)
        self.root.after(50, self.poll)

    def entry(self, parent, label, key, row, col, width=16, span=1, **kwargs):
        self.ttk.Label(parent, text=label).grid(row=row, column=col, sticky='w', padx=(0, 6), pady=3)
        self.ttk.Entry(parent, textvariable=self.vars[key], width=width, **kwargs).grid(
            row=row, column=col + 1, columnspan=span, sticky='ew', padx=(0, 14), pady=3)

    def current_options(self):
        return {key: value.get() for key, value in self.vars.items()}

    def schedule_save(self, *_):
        if self.save_timer:
            self.root.after_cancel(self.save_timer)
        self.save_timer = self.root.after(400, self.save)

    def save(self):
        self.save_timer = None
        options = self.current_options()
        options.update(geometry=self.root.geometry(), selected=','.join(self.table.selection()))
        try:
            save_settings(self.config_path, options, self.robots)
        except OSError as error:
            self.status.set('保存设置失败：' + str(error))

    def refresh_row(self, ip):
        row = self.robots[ip]
        values = [ip] + [row.get(key, '') for key in ('name', 'remark', 'ssh', 'repository', 'chassis', 'ros_master', 'voltage')]
        if self.table.exists(ip):
            self.table.item(ip, values=values)
        else:
            self.table.insert('', 'end', iid=ip, values=values)

    def selection_changed(self, _=None):
        selection = self.table.selection()
        ip = selection[0] if len(selection) == 1 else ''
        if selection == self.selected_rows:
            return
        self.selected_rows = selection
        self.stop_motion()
        self.selected_ip = ''
        row = self.robots.get(ip, {})
        self.name_var.set(row.get('name', ''))
        self.remark_var.set(row.get('remark', ''))
        self.selected_ip = ip
        self.update_detail()
        self.schedule_save()

    def edit_robot(self, *_):
        if self.selected_ip:
            row = self.robots[self.selected_ip]
            row.update(name=self.name_var.get(), remark=self.remark_var.get())
            self.refresh_row(self.selected_ip)
            self.schedule_save()

    def apply_identity(self, _=None):
        ip = self.selected_ip
        if not ip:
            return
        name = self.name_var.get().strip()
        try:
            robot_number(name)
            if any(other != ip and name in (r.get('pending_id'), r.get('robot_id'))
                   for other, r in self.robots.items()):
                raise ValueError('编号 %s 已被其他 IP 使用' % name)
        except ValueError as error:
            self.status.set(str(error))
            return
        row = self.robots[ip]
        if name != row.get('robot_id'):
            row['pending_id'] = name
            self.selected_action('identity', addresses=[ip], popup=False)
        self.save()

    def update_detail(self):
        ip = self.selected_ip
        if ip:
            row = self.robots[ip]
            self.detail.set('已选：%s %s | mini_4wd | %s %s' % (
                row.get('name', ''), ip, row.get('robot_id', ''),
                (row.get('error') or row.get('progress') or row.get('chassis', ''))[-450:]))
        else:
            self.detail.set('已选 %d 台，可批量同步或更新 ROS Master；行驶控制需只选一辆车。' % len(self.table.selection()))

    def begin_motion(self, direction, source='mouse'):
        if self.workbench.active():
            self.status.set('编队模式运行中，请使用“小车定位图”页的领航车键盘控制')
            return
        session = self.sessions.get(self.selected_ip)
        if not session or not session.ready:
            self.status.set('请先选择底盘已就绪的车辆')
            return
        try:
            options = self.current_options()
            for name, maximum in [('linear', 0.5), ('angular', 1.5)]:
                value = float(options[name])
                if not math.isfinite(value) or not 0 < value <= maximum:
                    raise ValueError('速度超出范围')
        except ValueError:
            self.stop_motion()
            self.status.set('线速度需在 0–0.5 m/s，角速度需在 0–1.5 rad/s，且不能为零')
            return
        self.stop_motion()
        linear, angular = float(options['linear']), float(options['angular'])
        velocity = {'forward': (linear, 0), 'backward': (-linear, 0),
                    'left': (0, angular), 'right': (0, -angular)}[direction]
        self.motion = (session, velocity)
        self.motion_source = source
        session.drive(*velocity)
        self.status.set('控制中：%s · 松手停车' % self.selected_ip)

    def mouse_press(self, event, direction):
        event.widget.focus_set()
        self.begin_motion(direction)
        return 'break'

    def stop_motion(self):
        if self.motion:
            self.status.set('已发送停车指令')
        self.motion = None
        self.motion_source = None
        for session in self.sessions.values():
            session.drive()

    def check_focus(self):
        if not self.closing and self.root.focus_displayof() is None:
            self.stop_motion()

    def key_press(self, event):
        key = event.keysym.lower()
        directions = {'w': 'forward', 'up': 'forward', 's': 'backward', 'down': 'backward',
                      'a': 'left', 'left': 'left', 'd': 'right', 'right': 'right'}
        if key in directions:
            pending = self.key_releases.pop(key, None)
            if pending:
                self.root.after_cancel(pending)
            elif key not in self.keys_down:
                self.keys_down.add(key)
                self.begin_motion(directions[key], source='keyboard')
            return 'break'

    def key_release(self, event):
        key = event.keysym.lower()
        if key in ('w', 'a', 's', 'd', 'up', 'down', 'left', 'right'):
            def release():
                self.key_releases.pop(key, None)
                self.keys_down.discard(key)
                self.stop_motion()
            self.key_releases[key] = self.root.after_idle(release)
            return 'break'

    def loop_changed(self):
        if self.loop_timer:
            self.root.after_cancel(self.loop_timer)
            self.loop_timer = None
        if self.vars['loop'].get() and not self.scanning:
            self.scan()

    def scan(self):
        if self.scanning or self.closing:
            return
        options = self.current_options()
        try:
            validate(options)
            addresses = targets_from(options)
        except ValueError as error:
            self.messagebox.showerror('设置有误', str(error))
            return
        if self.loop_timer:
            self.root.after_cancel(self.loop_timer)
            self.loop_timer = None
        for ip, session in list(self.sessions.items()):
            connection_keys = ('username', 'password', 'port', 'workspace', 'repository')
            if not session.closed.is_set() and (ip not in addresses or any(
                    options[key] != session.options[key] for key in connection_keys)):
                session.close()
                self.robots[ip].update(ssh='已断开', chassis='正在停止')
                self.refresh_row(ip)
        existing = {ip for ip, session in self.sessions.items() if session.alive() or session.busy()}
        for ip in existing:
            if not self.sessions[ip].busy():
                self.sessions[ip].options.update(options)
        if options['auto_start'] and not self.workbench.active():
            for ip in existing:
                self.sessions[ip].start()
        self.scan_cancel = threading.Event()
        self.scanning = True
        self.scan_button.state(['disabled'])
        self.status.set('正在扫描 %d 个 IP…' % len(addresses))
        self.save()
        threading.Thread(target=self.scan_worker, args=(addresses, existing, options, self.scan_cancel), daemon=True).start()

    def scan_worker(self, addresses, existing, options, cancelled):
        def probe(ip):
            if cancelled.is_set() or ip in existing:
                return
            try:
                with socket.create_connection((ip, int(options['port'])), float(options['timeout'])):
                    pass
            except OSError as error:
                self.events.put(('unreachable', ip, {'error': str(error)}))
                return
            if cancelled.is_set():
                return
            try:
                client = connect_ssh(ip, options, self.config_path.with_name('known_hosts'))
                session = RobotSession(ip, client, options, self.events)
                if cancelled.is_set():
                    session.close()
                else:
                    self.events.put(('connected', session, {'cancelled': cancelled}))
            except Exception as error:
                self.events.put(('auth_failed', ip, {'error': str(error)}))
        with ThreadPoolExecutor(max_workers=int(options['workers'])) as pool:
            futures = [pool.submit(probe, ip) for ip in addresses]
            for count, future in enumerate(as_completed(futures), 1):
                try:
                    future.result()
                except Exception as error:
                    self.events.put(('message', '', {'text': str(error)}))
                if count % 10 == 0:
                    self.events.put(('progress', '', {'count': count, 'total': len(addresses)}))
        self.events.put(('scan_done', '', {'cancelled': cancelled.is_set(), 'total': len(addresses)}))

    def cancel_scan(self):
        self.scan_cancel.set()
        if self.loop_timer:
            self.root.after_cancel(self.loop_timer)
            self.loop_timer = None
        self.vars['loop'].set(False)
        self.status.set('扫描已取消，已连接车辆保持连接')

    def start_chassis(self):
        if self.workbench.active():
            self.status.set('请先在编队页停止编队进程，再启动独立底盘')
            return
        options = self.current_options()
        for session in self.sessions.values():
            if not session.busy():
                session.options.update(options)
            session.start()

    def robot_id_for(self, ip):
        row = self.robots[ip]
        if not row.get('robot_id'):
            used = {r.get('robot_id') for r in self.robots.values()} | set(KNOWN.values())
            number = 0
            while 'ugv%d' % number in used:
                number += 1
            row['robot_id'] = 'ugv%d' % number
        return row['robot_id']

    def selected_action(self, action, addresses=None, popup=True, launch=None):
        addresses = list(self.table.selection() if addresses is None else addresses)
        if not addresses:
            self.status.set('请先选择要更新的 IP；Ctrl / Shift 可多选')
            return
        options = self.current_options()
        try:
            if action == 'sync':
                validate(options)
            elif action == 'master':
                ipaddress.IPv4Address(options['master_ip'])
        except ValueError as error:
            self.messagebox.showerror('设置有误', str(error))
            return
        self.stop_motion()
        if action == 'sync' and self.workbench.active():
            self.messagebox.showerror('编队正在运行', '请先点击编队页的“停止本次启动”，释放 ROS 进程后再同步仓库。')
            return
        if action in self.batches:
            self.status.set('同类批量任务正在执行，请等待结果')
            return
        batch = {}
        started = 0
        for ip in addresses:
            session = self.sessions.get(ip)
            if session is None or session.closed.is_set():
                session = RobotSession(ip, None, options, self.events)
                session.host_keys = self.config_path.with_name('known_hosts')
                self.sessions[ip] = session
            session.robot_id = self.robot_id_for(ip)
            per_options = dict(options, desired_id=self.robots[ip].get('pending_id', ''))
            should_launch = action == 'sync' and options['auto_start'] if launch is None else launch
            if session.maintain(action, per_options, launch=should_launch):
                started += 1
                batch[ip] = None
            else:
                batch[ip] = '忙碌，请稍后重试'
        if popup and started:
            self.batches[action] = batch
        self.status.set(('已提交 %d 台车辆的%s任务' % (started, {'sync': '同步', 'read': '配置读取', 'master': 'ROS Master 更新', 'identity': '编号更新', 'serial': '串口权限设置'}[action]))
                        if started else '选中车辆正在执行其他任务，请稍后重试')
        self.save()

    def disconnect_all(self):
        self.cancel_scan()
        self.stop_motion()
        for session in self.sessions.values():
            session.close()
        self.workbench.stop()

    def choose_export(self):
        from tkinter import filedialog
        current = Path(self.vars['export'].get()).expanduser()
        path = filedialog.asksaveasfilename(title='导出小车列表', defaultextension='.csv',
                                          initialdir=str(current.parent), initialfile=current.name,
                                          filetypes=[('CSV 表格', '*.csv')])
        if path:
            self.vars['export'].set(path)
            self.write_export()
            self.status.set('列表已导出：' + path)

    def write_export(self):
        try:
            export_csv(self.vars['export'].get(), self.robots)
        except OSError as error:
            self.status.set('导出失败：' + str(error))

    def poll(self):
        for _ in range(300):
            try:
                event, source, data = self.events.get_nowait()
            except queue.Empty:
                break
            if event == 'connected':
                if self.closing or data['cancelled'].is_set():
                    source.close()
                    continue
                ip = source.ip
                previous = self.sessions.get(ip)
                if previous:
                    previous.close()
                self.sessions[ip] = source
                self.robots.setdefault(ip, {'name': KNOWN.get(ip, ''), 'remark': ''})
                source.robot_id = self.robot_id_for(ip)
                source.options['desired_id'] = self.robots[ip].get('pending_id', '')
                source.host_keys = self.config_path.with_name('known_hosts')
                self.robots[ip].update(ssh='已连接', chassis='未启动', error='',
                                       last_seen=datetime.now().isoformat(timespec='seconds'))
                self.refresh_row(ip)
                source.start(launch=source.options['auto_start'] and not self.workbench.active())
            elif event in ('unreachable', 'auth_failed'):
                ip = source
                if event == 'auth_failed':
                    self.robots.setdefault(ip, {'name': '', 'remark': ''})
                if ip in self.robots:
                    self.robots[ip].update(ssh='认证失败' if event == 'auth_failed' else '不可达',
                                           chassis='未连接', error=data['error'], voltage='')
                    self.refresh_row(ip)
            elif event == 'task_done':
                batch = self.batches.get(data['action'])
                if batch is not None and source.ip in batch:
                    batch[source.ip] = data['error'] or data.get('result') or '成功'
                    if all(value is not None for value in batch.values()):
                        self.batches.pop(data['action'])
                        self.workbench.result_popup('串口权限设置结果' if data['action'] == 'serial' else '车辆任务结果', batch)
                if data['action'] != 'identity' and self.robots.get(source.ip, {}).get('pending_id') and not self.closing:
                    self.root.after(150, lambda ip=source.ip: self.selected_action('identity', addresses=[ip], popup=False)
                                    if not self.closing else None)
            elif event in ('status', 'telemetry'):
                ip = source.ip
                if self.sessions.get(ip) is not source:
                    continue
                if event == 'status':
                    if data.pop('master_read', False) and self.selected_ip == ip and data.get('ros_master'):
                        self.vars['master_ip'].set(urlsplit(data['ros_master']).hostname)
                    if not data.get('robot_id'):
                        data.pop('robot_id', None)
                    self.robots[ip].update(data)
                    row = self.robots[ip]
                    if data.get('robot_id'):
                        if row.get('pending_id') == data['robot_id']:
                            row.pop('pending_id', None)
                            source.options['desired_id'] = ''
                        if not row.get('pending_id'):
                            row['name'] = data['robot_id']
                            if self.selected_ip == ip and self.root.focus_get() is not getattr(self, 'name_entry', None):
                                self.selected_ip = ''
                                self.name_var.set(data['robot_id'])
                                self.selected_ip = ip
                    self.schedule_save()
                    self.write_export()
                else:
                    voltage = data.get('voltage')
                    self.robots[ip]['voltage'] = '%.2f' % voltage if voltage is not None else ''
                self.refresh_row(ip)
                self.update_detail()
            elif event == 'progress' and not self.scan_cancel.is_set():
                self.status.set('扫描进度 %s/%s；SSH 已连接 %s 台' % (
                    data['count'], data['total'], sum(s.alive() for s in self.sessions.values())))
            elif event == 'scan_done':
                self.scanning = False
                self.scan_button.state(['!disabled'])
                self.status.set('%s：扫描 %d 个 IP，SSH 已连接 %d 台；列表：%s' % (
                    '已停止' if data['cancelled'] else '本轮完成', data['total'],
                    sum(s.alive() for s in self.sessions.values()), self.vars['export'].get()))
                self.save()
                self.write_export()
                if self.vars['loop'].get() and not self.closing:
                    try:
                        delay = max(1, float(self.vars['interval'].get()))
                        self.loop_timer = self.root.after(int(delay * 1000), self.scan)
                    except (ValueError, OverflowError):
                        self.vars['loop'].set(False)
            elif event == 'message':
                self.status.set(data['text'])
        if self.motion:
            session, velocity = self.motion
            if session.ready and session.alive() and session is self.sessions.get(self.selected_ip):
                session.drive(*velocity)
            else:
                self.stop_motion()
        for ip, session in self.sessions.items():
            if not session.alive() and not session.busy() and self.robots[ip].get('ssh') == '已连接':
                session.close()
                self.robots[ip].update(ssh='已断开', chassis='连接中断', voltage='')
                self.refresh_row(ip)
        self.workbench.poll()
        if not self.closing:
            self.root.after(50, self.poll)

    def close(self):
        if self.closing:
            return
        self.closing = True
        self.workbench.close()
        remember_loop = self.vars['loop'].get()
        self.disconnect_all()
        self.vars['loop'].set(remember_loop)
        self.status.set('正在停车并关闭底盘…')
        self.save()
        deadline = time.monotonic() + 15

        def finish():
            self.poll()
            if (self.scanning or self.workbench.active() or any(s.busy() or (s.thread and s.thread.is_alive()) for s in self.sessions.values())) and time.monotonic() < deadline:
                self.root.after(100, finish)
                return
            for ip, session in self.sessions.items():
                if session.client is not None:
                    session.client.close()
                self.robots[ip].update(ssh='已断开', chassis='已停止')
            self.save()
            self.write_export()
            self.root.destroy()
        self.root.after(100, finish)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=CONFIG_PATH, help='独立的历史配置文件')
    parser.add_argument('--check-gui', action='store_true', help='检测 GUI 后退出')
    args = parser.parse_args()
    try:
        import tkinter as tk
        root = tk.Tk()
    except Exception as error:
        parser.exit(1, 'GUI 不可用：%s；需要桌面会话与 python3-tk\n' % error)
    if args.check_gui:
        root.withdraw()
        root.update()
        print('GUI 可用：Tk %s，屏幕 %sx%s' % (tk.TkVersion, root.winfo_screenwidth(), root.winfo_screenheight()))
        root.destroy()
        return
    try:
        FleetConsole(root, args.config)
    except (OSError, ValueError, KeyError, TypeError) as error:
        from tkinter import messagebox
        messagebox.showerror('配置读取失败', '%s\n配置：%s' % (error, args.config))
        root.destroy()
        return
    root.mainloop()


if __name__ == '__main__':
    main()
