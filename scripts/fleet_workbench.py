"""Formation startup, SSH terminals, leader keyboard control and UWB map."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor, as_completed
from functools import lru_cache
import ipaddress
import json
import math
from pathlib import Path
import queue
import shlex
import threading
import time
import uuid
from xml.sax.saxutils import escape

import yaml

from fleet_deploy import (read_robot_config, robot_number, run_remote, shell_path,
                          update_config)
from fleet_terminal import Terminal
from leader_tracker import parse_points, check_bounds, curve_point, sample_path

ROOT = Path(__file__).resolve().parents[1]
LOCALIZATION = ROOT / 'src/five_ugv_uwb_localization'
PROFILE_DIR = LOCALIZATION / 'config/uwb'
ANCHOR_PROFILES = {'outdoor': '外场', 'indoor': '内场'}
LOCALIZATION_MODES = ('five_ugv', 'linktrack')
DEFAULTS = {
    'leader': 'ugv1', 'formation_selected': '192.168.0.106,192.168.0.108,192.168.0.109,192.168.0.110,192.168.0.114',
    'localization_mode': 'five_ugv', 'anchor_profile': 'outdoor',
    'anchors_json': '',
    'anchor_overrides_json': '{}',
    'linear_limit': '',
    'leader_control_mode': 'keyboard', 'leader_path': '2.0, 2.2\n3.0, 2.2\n3.0, 2.8',
    'leader_path_bends': '[]',
    'path_speed': '0.10', 'path_lookahead': '0.40',
    'data_timeout': '0.6',
    'active_tab': '0', 'formation_step': '0', 'fold_logs': True,
}
COLORS = ['#1976d2', '#e76622', '#009688', '#9b51b6', '#c0395a', '#8a7600']
KEY_DIRECTIONS = {'w': (1, 0), 'up': (1, 0), 's': (-1, 0), 'down': (-1, 0),
                  'a': (0, 1), 'left': (0, 1), 'd': (0, -1), 'right': (0, -1)}
TRACKING_TEXT = {
    'IDLE': '待命', 'STOPPED': '已停止', 'TRACKING': '路径跟踪中', 'ALIGNING': '正在对齐方向',
    'PAUSED': '已暂停', 'DONE': '已到达终点', 'USER_PAUSE': '点击继续可恢复',
    'UWB_INVALID': '领航定位无效', 'STALE_INPUT': '定位或里程计数据过期',
    'WAIT_ALIGNMENT': '请保持领航车静止，等待航向对齐', 'FOLLOWER_INVALID': '跟随车定位无效或过期',
    'FORMATION_OFFSETS': '缺少跟随车编队偏移，请重新连接监视', 'LIMIT_PENDING': '等待限速回读确认',
    'NO_CHASSIS': '领航底盘没有接收速度指令', 'CONTROL_TIMEOUT': '控制连接超时，请手动继续',
    'INVALID_POSE': '位姿无效', 'POSE_JUMP': '检测到位置跳变，请检查定位后继续',
    'ZERO_LIMIT': '线速度上限为零', 'PATH_ERROR': '偏离当前路径超过 0.6 m，请检查定位或手动移回',
    'PATH_POINTS': '请填写 2–50 个坐标点，每行两个有限数值 X, Y',
    'PATH_SEGMENT_SHORT': '相邻路径点距离至少 0.15 m',
    'PATH_CURVES': '曲线设置无效，请重新编辑路径',
    'PATH_CURVE_LONG': '曲线路径过长，请减少弯曲幅度或路径点',
    'PATH_OUTSIDE': '路径超出基站范围或没有足够的车体 / 编队转弯余量',
    'PATH_SETTINGS': '巡航速度需在 0–0.5 m/s（不含 0），预瞄距离需在 0.3–0.5 m',
    'START_TOO_FAR': '路径起点距领航车超过 0.5 m，请使用当前位置或先手动移至起点',
    'NO_PAUSED_PATH': '没有可继续的路径，请点击开始',
}


def path_bounds(config):
    return [config['workspace_x_min'], config['workspace_y_min'],
            config['workspace_x_max'], config['workspace_y_max']]


def saved_linear_limit(options):
    value = options.get('linear_limit', '')
    if value == '':
        # Migrate saved per-IP limits without increasing any vehicle's bound.
        previous = json.loads(options.get('linear_limits_json', '{}'))
        value = min(previous.values()) if previous else 0.15
    if isinstance(value, bool):
        raise ValueError('线速度上限必须在 0–0.5 m/s 之间')
    value = float(value)
    if not 0 <= value <= 0.5:
        raise ValueError('线速度上限必须在 0–0.5 m/s 之间')
    return value


def session_limits(options, identities):
    value = saved_linear_limit(options)
    return {str(number): value for number in identities.values()}


def saved_data_timeout(options):
    value = options.get('data_timeout', 0.6)
    if isinstance(value, bool):
        raise ValueError('数据过期阈值必须是大于 0 的有限秒数')
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError('数据过期阈值必须是大于 0 的有限秒数')
    return value


@lru_cache(maxsize=32)
def localization_config(encoded='', profile='outdoor'):
    if profile not in ANCHOR_PROFILES:
        profile = 'outdoor'
    base = yaml.safe_load((LOCALIZATION / 'config/final_localization.yaml').read_text())
    path = PROFILE_DIR / (profile + '.yaml')
    if path.exists():
        base.update(yaml.safe_load(path.read_text()) or {})
    if encoded:
        custom = json.loads(encoded)
        base.update(anchors=custom['anchors'], tag_height=custom['tag_height'])
        if 'valid_max_residual_rms' in custom:
            base['valid_max_residual_rms'] = custom['valid_max_residual_rms']
    anchors = base['anchors']
    if len(anchors) < base['min_anchors']:
        raise ValueError('定位至少需要 %d 个基站' % base['min_anchors'])
    ids = [a['id'] for a in anchors]
    if any(type(value) is not int or not 0 <= value <= 255 for value in ids) or len(set(ids)) != len(ids):
        raise ValueError('基站 ID 必须是 0–255 的不重复整数')
    for value in [base['tag_height']] + [a[key] for a in anchors for key in ('x', 'y', 'z')]:
        if not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('坐标和高度必须是有限数值，单位米')
    if base['tag_height'] < 0 or any(a['z'] < 0 for a in anchors):
        raise ValueError('高度不能小于零')
    residual_limit = base['valid_max_residual_rms']
    if (isinstance(residual_limit, bool) or not isinstance(residual_limit, (int, float)) or
            not math.isfinite(residual_limit) or residual_limit <= 0):
        raise ValueError('有效残差阈值必须是大于 0 的有限数值（米）')
    a = anchors[0]
    if not any(abs((b['x'] - a['x']) * (c['y'] - a['y']) -
                   (b['y'] - a['y']) * (c['x'] - a['x'])) > 1e-5 for b in anchors for c in anchors):
        raise ValueError('基站在 XY 平面不能全部共线')
    base.update(workspace_x_min=min(a['x'] for a in anchors), workspace_x_max=max(a['x'] for a in anchors),
                workspace_y_min=min(a['y'] for a in anchors), workspace_y_max=max(a['y'] for a in anchors))
    return base


def ros_command(options, ip, command):
    master = str(ipaddress.IPv4Address(options['master_ip']))
    ipaddress.IPv4Address(ip)
    return ('set -e; source %s; export ROS_MASTER_URI=%s ROS_IP=%s CAR_MODE=mini_4wd; '
            'unset ROS_HOSTNAME ROS_NAMESPACE; %s' %
            (shell_path(options['workspace'].rstrip('/') + '/scripts/env.sh'),
             shlex.quote('http://%s:11311' % master), shlex.quote(ip), command))


def launch_command(step, options, ip, name, remote, row=None):
    number = robot_number(name)
    helper = 'python -u %s/fleet_ros.py' % shlex.quote(remote)
    if step == 'master':
        command = 'exec roscore'
    elif step == 'chassis':
        command = ('%s check --step chassis --localization-mode %s --ids %d; '
                   'exec flock -n "$HOME/.cache/formation-console/chassis.lock" '
                   'roslaunch %s/ugv.launch ugv_id:=%d car_mode:=mini_4wd localization_mode:=%s '
                   'localization_config:=%s/localization.yaml' %
                   (helper, shlex.quote(options.get('localization_mode', 'five_ugv')), number,
                    shlex.quote(remote), number, shlex.quote(options.get('localization_mode', 'five_ugv')),
                    shlex.quote(remote)))
    elif step == 'follower':
        leader = robot_number(options['leader'])
        if number == leader:
            raise ValueError('领航车不启动跟随控制器')
        command = ('%s check --step follower --ids %d; exec flock -n "$HOME/.cache/formation-console/follower.lock" '
                   'roslaunch five_ugv_formation_control follower.launch ugv_id:=%d leader_id:=%d auto_enable:=false '
                   'max_linear:=%s data_timeout:=%s' %
                   (helper, number, number, leader, saved_linear_limit(options), saved_data_timeout(options)))
    else:
        raise ValueError('未知启动步骤')
    return ros_command(options, ip, command)


class Monitor:
    def __init__(self, client, command, events, limits=None):
        self.client, self.command, self.events = client, command, events
        self.stop = threading.Event()
        self.drive = (0.0, 0.0, 0.0)
        self.enable_sequence, self.enable = 0, False
        self.ready, self.last, self.tick = False, 0.0, 0.0
        self.ui_heartbeat = time.monotonic()
        self.subscribers = 0
        self.limits = limits or {}
        self.limit_status = {}
        self.control_request = {'sequence': 0, 'action': 'keyboard'}
        self.tracking = {}
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        channel = None
        try:
            channel = self.client.get_transport().open_session(timeout=5)
            channel.set_combine_stderr(True)
            channel.settimeout(0.3)
            channel.exec_command('bash -c ' + shlex.quote(self.command))
            buffer, recent, sent = b'', '', 0.0
            self.last = time.monotonic()
            while not self.stop.is_set():
                now = time.monotonic()
                if now - self.ui_heartbeat > 0.6:
                    raise RuntimeError('界面心跳超时，已停止监视和编队控制')
                if channel.recv_ready():
                    chunk = channel.recv(65536)
                    if not chunk:
                        raise RuntimeError('监视 SSH 已断开')
                    buffer += chunk
                    if len(buffer) > 262144:
                        raise RuntimeError('监视数据过长')
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1)
                        try:
                            data = json.loads(line.decode())
                        except ValueError:
                            recent = line.decode(errors='replace')[-400:]
                            continue
                        if data.get('event') == 'error':
                            raise RuntimeError(data['message'])
                        if data.get('event') == 'sample':
                            self.ready, self.last, self.tick = True, now, data['tick']
                            self.subscribers = data['leader_subscribers']
                            self.limit_status = data.get('linear_limits', {})
                            self.tracking = data.get('tracking', {})
                            self.events.put(('sample', data))
                if channel.exit_status_ready() and not channel.recv_ready():
                    raise RuntimeError('监视进程退出：' + recent)
                if now - self.last > (2 if self.ready else 20):
                    raise RuntimeError('监视数据超时')
                if now - sent >= 0.1:
                    linear, angular, changed = self.drive
                    if now - changed > 0.25 or not self.ready:
                        linear = angular = 0.0
                    payload = dict(tick=self.tick, linear=linear, angular=angular,
                                   enable_sequence=self.enable_sequence, enable=self.enable,
                                   linear_limits=self.limits, control=self.control_request)
                    channel.sendall((json.dumps(payload) + '\n').encode())
                    sent = now
                self.stop.wait(0.02)
        except Exception as error:
            if not self.stop.is_set():
                self.events.put(('monitor_error', str(error)))
        finally:
            self.ready = False
            if channel:
                try:
                    channel.sendall(b'{"quit":true}\n')
                    channel.shutdown_write()
                    deadline = time.monotonic() + 2
                    while not channel.exit_status_ready() and time.monotonic() < deadline:
                        if channel.recv_ready():
                            channel.recv(65536)
                        time.sleep(0.05)
                except Exception:
                    pass
                channel.close()
            self.client.close()


class FleetWorkbench:
    def __init__(self, app):
        self.app, self.root, self.tk, self.ttk = app, app.root, app.tk, app.ttk
        self.selected_anchor_profile = app.vars['anchor_profile'].get()
        self.events = queue.Queue()
        self.cancel = threading.Event()
        self.worker = None
        self.stopper = None
        self.signature = None
        self.terminals = []
        self.monitor = None
        self.iot_window = None
        self.iot_sessions = []
        self.iot_probes = []
        self.active_identities = {}
        self.active_offsets = {}
        self.control_dialog = self.keyboard = None
        self.path_preview, self.tracking = [], {}
        self.path_picking, self.map_transform = False, None
        self.path_bends, self.path_drag = [], None
        self.keys_down, self.key_releases = set(), {}
        self.keyboard_keys = set()
        self.run_id = uuid.uuid4().hex
        self.samples, self.trails, self.last_pose, self.yaw_origin = {}, {}, {}, {}
        self.target_trails, self.error_history = {}, {}
        self.last_sample = self.last_paint = self.last_status = 0.0
        self.robot_states, self.current_ids = {}, []
        self.terminal_states = {}
        self.armed = self.tk.BooleanVar(value=False)
        self.status = self.tk.StringVar(value='选择车辆，然后总启动或按步骤启动；双击车辆打开 SSH 终端。')
        self.keyboard_status = self.tk.StringVar(value='连接实时监视后，点击键盘控制区启用。')
        self.leader_status = self.tk.StringVar(value='点击“领航控制”选择键盘或参考路径。')
        page = self.ttk.Frame(app.notebook, padding=10)
        app.notebook.add(page, text='编队算法')
        toolbar = self.ttk.Frame(page)
        toolbar.pack(fill='x')
        for title, action in [('快捷总启动', lambda: self.start('all')), ('停止本次启动', self.stop),
                              ('立即同步 Git 仓库', self.sync), ('所有小车串口权限（777）', self.serial_permissions),
                              ('打开选中 SSH 终端', self.open_ssh)]:
            self.ttk.Button(toolbar, text=title, command=action).pack(side='left', padx=(0, 7))
        self.ttk.Label(toolbar, text='领航车').pack(side='left', padx=(8, 4))
        self.ttk.Entry(toolbar, textvariable=app.vars['leader'], width=8).pack(side='left')
        self.ttk.Label(page, textvariable=self.status, wraplength=1080).pack(fill='x', pady=7)
        self.ttk.Label(page, text='Ctrl / Shift 选择参与车辆；总启动自动退出独立底盘模式。车辆朝向 UWB +X 后启动。').pack(anchor='w')
        columns = ('name', 'ip', 'id', 'ssh', 'master', 'step')
        vehicle_frame = self.ttk.Frame(page)
        vehicle_frame.pack(fill='x')
        self.vehicles = self.ttk.Treeview(vehicle_frame, columns=columns, show='headings', height=5, selectmode='extended')
        for key, title, width in zip(columns, ['UGV', 'IP（双击 SSH）', 'UGV_ID', 'SSH 连接', 'ROS Master', '启动状态'],
                                     [80, 145, 70, 115, 240, 290]):
            self.vehicles.heading(key, text=title)
            self.vehicles.column(key, width=width, minwidth=60, stretch=key == 'step')
        scroll = self.ttk.Scrollbar(vehicle_frame, command=self.vehicles.yview)
        self.vehicles.configure(yscrollcommand=scroll.set)
        scroll.pack(side='right', fill='y')
        self.vehicles.pack(fill='x')
        self.vehicles.bind('<Double-1>', self.open_clicked_ssh)
        self.vehicles.bind('<<TreeviewSelect>>', self.remember_selection)
        self.refresh_vehicles()
        self.vehicles.selection_set([ip for ip in app.vars['formation_selected'].get().split(',') if ip in app.robots])
        self.ttk.Button(page, text='编辑选中车 UWB 串口 / 编队偏移', command=self.edit_vehicle).pack(anchor='w', pady=5)
        steps = self.ttk.Notebook(page)
        steps.pack(fill='x', pady=4)
        for title, description, actions in [
            ('1 · 配置检查', '通过 SSH 检查并同步所选车辆的 UGV_ID、mini_4wd、ROS_IP、Master、UWB 串口和编队偏移。首次使用或修改车端配置后执行。',
             [('检查并同步配置', lambda: self.start('config'))]),
            ('2 · Master', '在 Master IP 主机检查或启动 roscore。各启动步骤直接使用已有车端配置。',
             [('启动 Master', lambda: self.start('master'))]),
            ('3 · 底盘与定位', '为每辆车打开 roslaunch SSH 标签页，等待 odom、UWB pose 和有效定位。基站设置随启动上传。',
             [('启动底盘 + UWB 定位', lambda: self.start('chassis'))]),
            ('4 · 跟随算法', '为领航车以外的所选车辆启动 follower.launch，使用各车偏移；等待控制器状态。',
             [('启动跟随控制器', lambda: self.start('follower'))]),
            ('5 · 监视与使能', '接收共享 Master 上的实时状态；使能后跟随车开始跟踪目标。',
             [('连接实时监视', lambda: self.start('monitor')), ('使能编队跟随', lambda: self.set_enabled(True)),
              ('立即停车 / 禁用跟随', self.emergency)])]:
            frame = self.ttk.Frame(steps, padding=8)
            steps.add(frame, text=title)
            self.ttk.Label(frame, text=description, wraplength=1060).pack(anchor='w')
            row = self.ttk.Frame(frame)
            row.pack(anchor='w', pady=(6, 0))
            for label, command in actions:
                self.ttk.Button(row, text=label, command=command).pack(side='left', padx=(0, 8))
        steps.select(max(0, min(4, int(app.vars['formation_step'].get()))))
        steps.bind('<<NotebookTabChanged>>', lambda _: app.vars['formation_step'].set(str(steps.index(steps.select()))))
        logs = self.fold(page, '终端与启动日志', 'fold_logs', expand=True)
        self.log = self.tk.Text(logs, height=7, state='disabled', wrap='word', font=('Monospace', 10))
        scroll = self.ttk.Scrollbar(logs, command=self.log.yview)
        self.log.configure(yscrollcommand=scroll.set)
        scroll.pack(side='right', fill='y')
        self.log.pack(fill='both', expand=True)
        self.make_map()
        self.root.bind('<Escape>', lambda _: self.emergency(), add='+')
        self.root.bind('<space>', self.space_stop, add='+')
        self.root.bind('<FocusOut>', lambda _: self.root.after_idle(self.focus_stop), add='+')
        self.armed.trace_add('write', self.arm_changed)

    def fold(self, parent, title, key, expand=False):
        opened = self.app.vars[key].get()
        shell = self.ttk.Frame(parent)
        shell.pack(fill='both' if expand else 'x', expand=expand and opened, pady=3)
        content = self.ttk.Frame(shell, padding=4)
        visible = [opened]
        def toggle():
            visible[0] = not visible[0]
            self.app.vars[key].set(visible[0])
            if visible[0]:
                content.pack(fill='both', expand=expand)
            else:
                content.pack_forget()
            shell.pack_configure(expand=expand and visible[0])
            button.configure(text=('▾ ' if visible[0] else '▸ ') + title)
        button = self.ttk.Button(shell, text=('▾ ' if opened else '▸ ') + title, command=toggle)
        button.pack(fill='x')
        if opened:
            content.pack(fill='both', expand=expand)
        return content

    def remember_selection(self, _=None):
        self.app.vars['formation_selected'].set(','.join(self.vehicles.selection()))

    def refresh_vehicles(self):
        for ip in self.vehicles.get_children():
            if ip not in self.app.robots:
                self.vehicles.delete(ip)
        for ip, row in self.app.robots.items():
            name = row.get('pending_id') or row.get('robot_id') or row.get('name', '')
            terminal_states = [terminal.status()['state'] for _, host, terminal in self.terminals if host == ip]
            ssh = row.get('ssh', '未连接')
            if any(state in ('运行中', 'SSH 已连接') for state in terminal_states):
                ssh = '已连接（终端）'
            elif '正在连接' in terminal_states:
                ssh = '正在连接'
            elif ssh == '已连接（启动会话）':
                ssh = '配置连接已结束'
            values = (name, ip, name[3:] if name.startswith('ugv') else '', ssh,
                      row.get('ros_master', ''), self.robot_states.get(ip, row.get('progress', '')))
            if self.vehicles.exists(ip):
                self.vehicles.item(ip, values=values)
            else:
                self.vehicles.insert('', 'end', iid=ip, values=values)
        if hasattr(self, 'limit_input'):
            self.refresh_limits()

    def addresses(self):
        addresses = list(self.vehicles.selection())
        if not addresses:
            raise ValueError('请选择参与操作的车辆（Ctrl / Shift 可多选）')
        return addresses

    def append_log(self, text):
        self.log.configure(state='normal')
        self.log.insert('end', time.strftime('%H:%M:%S ') + text + '\n')
        if int(self.log.index('end-1c').split('.')[0]) > 500:
            self.log.delete('1.0', '100.0')
        self.log.see('end')
        self.log.configure(state='disabled')

    def result_popup(self, title, results):
        dialog = self.tk.Toplevel(self.root)
        dialog.title(title)
        dialog.geometry('820x370')
        dialog.transient(self.root)
        text = self.tk.Text(dialog, wrap='word', padx=12, pady=12)
        text.pack(fill='both', expand=True)
        text.insert('end', '\n\n'.join('%s  %s\n%s' % (self.app.robots.get(ip, {}).get('name', ''), ip, value)
                                    for ip, value in results.items()))
        text.configure(state='disabled')
        self.ttk.Button(dialog, text='关闭', command=dialog.destroy).pack(pady=8)

    def sync(self):
        try:
            addresses = self.addresses()
        except ValueError as error:
            self.app.messagebox.showerror('同步仓库', str(error))
            return
        self.status.set('正在自动 SSH 连接并同步；各车进度见列表，完成后弹窗显示结果。')
        for ip in addresses:
            self.robot_states.pop(ip, None)
        self.app.selected_action('sync', addresses=addresses, launch=False)

    def serial_permissions(self):
        addresses = sorted((ip for ip, row in self.app.robots.items() if row.get('robot_id')),
                           key=ipaddress.IPv4Address)
        for ip in addresses:
            self.robot_states.pop(ip, None)
        self.status.set('正在为全部 %d 辆车设置串口权限；完成后逐车显示设备和权限。' % len(addresses))
        self.app.selected_action('serial', addresses=addresses, launch=False)
        self.status.set(self.app.status.get())

    def open_clicked_ssh(self, event):
        ip = self.vehicles.identify_row(event.y)
        if ip:
            self.open_ssh([ip])

    def open_ssh(self, addresses=None):
        try:
            for ip in addresses or self.addresses():
                name = self.app.robots[ip].get('robot_id') or self.app.robots[ip].get('name', '')
                terminal = Terminal(ip, self.app.current_options(), self.app.config_path.with_name('known_hosts'),
                                    title='%s · %s · SSH' % (name, ip))
                self.terminals.append(('shell', ip, terminal))
        except Exception as error:
            self.app.messagebox.showerror('SSH 终端', str(error))

    def active(self):
        if self.stopper and self.stopper.is_alive():
            return True
        if self.worker and self.worker.is_alive():
            return True
        if self.monitor and self.monitor.thread.is_alive():
            return True
        return any(step != 'shell' and terminal.status()['state'] in ('打开终端中', '正在连接', '运行中')
                   for step, _, terminal in self.terminals)

    def start(self, step):
        if (self.worker and self.worker.is_alive()) or (self.stopper and self.stopper.is_alive()):
            self.status.set('正在执行启动任务，请等待或停止本次启动')
            return
        try:
            options = self.app.current_options()
            saved_linear_limit(options)
            saved_data_timeout(options)
            if options.get('localization_mode', 'five_ugv') not in LOCALIZATION_MODES:
                raise ValueError('定位方式必须是 five_ugv 或 linktrack')
            if options.get('anchor_profile', 'outdoor') not in ANCHOR_PROFILES:
                raise ValueError('UWB 基站配置不存在')
            addresses = self.addresses()
            ipaddress.IPv4Address(options['master_ip'])
            robot_number(options['leader'])
            config = localization_config(options['anchors_json'], options.get('anchor_profile', 'outdoor'))
            rows = {ip: dict(self.app.robots[ip], robot_id=self.app.robots[ip].get('pending_id') or
                            self.app.robot_id_for(ip)) for ip in addresses}
            names = [row['robot_id'] for row in rows.values()]
            if len(set(names)) != len(names):
                raise ValueError('所选车辆 UGV_ID 重复，请先修改编号')
            if step in ('all', 'follower', 'monitor') and options['leader'] not in names:
                raise ValueError('请同时选择领航车 ' + options['leader'])
            for name in names:
                robot_number(name)
            signature = (options['master_ip'], options['leader'], options['workspace'],
                         options.get('localization_mode', 'five_ugv'), options.get('anchor_profile', 'outdoor'),
                         options['anchors_json'],
                         tuple(sorted((ip, row['robot_id'], json.dumps(row.get('formation_config', {}), sort_keys=True))
                                      for ip, row in rows.items())))
            if self.active() and self.signature and self.signature != signature:
                raise ValueError('参与车辆或启动配置已改变，请先停止本次启动，再使用新设置')
            if any(session.busy() for session in self.app.sessions.values()):
                raise ValueError('车辆维护任务尚未结束，请等同步完成再启动编队')
        except Exception as error:
            self.app.messagebox.showerror('启动设置', str(error))
            return
        self.app.cancel_scan()
        self.app.vars['auto_start'].set(False)
        self.app.stop_motion()
        self.armed.set(False)
        for session in self.app.sessions.values():
            session.drive()
            session.bridge_stop.set()
        self.cancel = threading.Event()
        self.signature = signature
        self.status.set('正在启动：' + step)
        self.app.save()
        self.worker = threading.Thread(target=self.run_start, args=(step, options, rows, config), daemon=True)
        self.worker.start()

    def connect(self, ip, options):
        from fleet_console import connect_ssh
        return connect_ssh(ip, options, self.app.config_path.with_name('known_hosts'))

    def stage(self, client, config):
        with client.open_sftp() as sftp:
            home = sftp.normalize('.')
            remote = home + '/.cache/formation-console/' + self.run_id
            # mkdir through SSH handles missing parent cache directories.
        run_remote(client, 'mkdir -p ' + shlex.quote(remote), self.cancel, timeout=10)
        with client.open_sftp() as sftp:
            for name in ('fleet_ros.py', 'fleet_bridge.py', 'leader_tracker.py'):
                sftp.put(str(Path(__file__).with_name(name)), remote + '/' + name)
            for name in ('ugv.launch', 'ugv_deploy.launch'):
                source = (LOCALIZATION / 'launch' / name).read_text()
                source = source.replace('$(find five_ugv_uwb_localization)/launch/ugv_deploy.launch',
                                        escape(remote + '/ugv_deploy.launch', {'"': '&quot;'}))
                with sftp.open(remote + '/' + name, 'w') as stream:
                    stream.write(source.encode())
            with sftp.open(remote + '/localization.yaml', 'w') as stream:
                stream.write(yaml.safe_dump(config).encode())
        return remote

    def launch(self, step, ip, name, options, remote):
        for previous, host, terminal in self.terminals:
            if previous == step and host == ip and terminal.status()['state'] in ('打开终端中', '正在连接', '运行中'):
                self.events.put(('log', '%s %s：终端已存在，检查就绪状态' % (name, step)))
                return terminal
        terminal = Terminal(ip, options, self.app.config_path.with_name('known_hosts'),
                            launch_command(step, options, ip, name, remote),
                            '%s · %s · %s' % ({'master': 'ROS Master', 'chassis': '底盘与定位',
                                              'follower': '跟随控制器'}[step], name, ip), group=step)
        self.terminals.append((step, ip, terminal))
        self.events.put(('log', '%s %s：已打开 SSH 启动终端' % (name, step)))
        return terminal

    def run_start(self, step, options, rows, config):
        clients, remotes, results, launched = {}, {}, {}, {}
        configs = dict(rows)
        current = options['master_ip']
        try:
            for session in list(self.app.sessions.values()):
                if session.thread and session.thread.is_alive():
                    session.thread.join(15)
                    if session.thread.is_alive():
                        raise RuntimeError('独立底盘尚未停止：' + session.ip)
            hosts = list(rows)
            if step in ('master', 'monitor'):
                hosts = []
            elif step == 'follower':
                hosts = [ip for ip in rows if rows[ip]['robot_id'] != options['leader']]
            for ip in dict.fromkeys(hosts + [options['master_ip']]):
                current = ip
                if self.cancel.is_set():
                    raise RuntimeError('启动已取消')
                self.events.put(('state', ip, 'SSH 配置检查中' if step in ('all', 'config') else 'SSH 连接中'))
                client = clients[ip] = self.connect(ip, options)
                if step in ('all', 'config') and ip in rows:
                    existing = read_robot_config(client)
                    if not existing['robot_id']:
                        raise RuntimeError('工作空间尚未安装，请先点击“立即同步 Git 仓库”')
                    values = dict(UGV_ID=str(robot_number(rows[ip]['robot_id'])),
                                  ROS_MASTER_URI='http://%s:11311' % options['master_ip'], ROS_IP=ip, CAR_MODE='mini_4wd')
                    values.update(rows[ip].get('formation_config', {}))
                    existing = configs[ip] = update_config(client, values)
                    self.events.put(('config', ip, existing))
                remote = remotes[ip] = self.stage(client, config)
                results[ip] = '配置已检查，启动文件已上传' if step in ('all', 'config') else '启动文件已上传'
                self.events.put(('state', ip, results[ip]))
            if step == 'config':
                self.events.put(('done', '配置检查完成；可继续启动 Master。', results))
                return
            master = options['master_ip']
            probe = 'python -u %s/fleet_ros.py master --timeout ' % shlex.quote(remotes[master])
            current = master
            if step in ('all', 'master'):
                try:
                    run_remote(clients[master], ros_command(options, master, probe + '0.5'), self.cancel, timeout=6)
                    self.events.put(('log', master + ' ROS Master 已在线，接入现有 Master'))
                except RuntimeError:
                    if self.cancel.is_set():
                        raise
                    self.launch('master', master, rows.get(master, {}).get('robot_id', options['leader']), options, remotes[master])
            run_remote(clients[master], ros_command(options, master, probe + '25'), self.cancel, timeout=35)
            results[master] = 'ROS Master 已就绪'
            self.events.put(('state', master, results[master]))
            if step in ('all', 'chassis', 'follower'):
                jobs = []
                stage = 'chassis' if step in ('all', 'chassis') else 'follower'
                for ip, row in rows.items():
                    current = ip
                    name = row['robot_id']
                    if stage == 'follower' and name == options['leader']:
                        continue
                    if self.cancel.is_set():
                        raise RuntimeError('启动已取消')
                    launched[ip] = self.launch(stage, ip, name, options, remotes[ip])
                    jobs.append((ip, row, stage))
                if stage == 'follower':
                    self.wait_ready_parallel(jobs, clients, options, remotes, launched, results)
                else:
                    for ip, row, stage in jobs:
                        current = ip
                        self.wait_ready(clients[ip], options, ip, row['robot_id'], remotes[ip], stage, launched[ip])
                        results[ip] = stage + ' 已就绪'
                        self.events.put(('state', ip, results[ip]))
            if step == 'all':
                jobs = []
                for ip, row in rows.items():
                    current = ip
                    if row['robot_id'] == options['leader']:
                        continue
                    if self.cancel.is_set():
                        raise RuntimeError('启动已取消')
                    terminal = self.launch('follower', ip, row['robot_id'], options, remotes[ip])
                    launched[ip] = terminal
                    jobs.append((ip, row, 'follower'))
                self.wait_ready_parallel(jobs, clients, options, remotes, launched, results,
                                         ready_text='底盘 / 定位 / 跟随已就绪（未使能）')
            if step in ('all', 'monitor', 'chassis'):
                current = master
                if self.cancel.is_set():
                    raise RuntimeError('启动已取消')
                if self.monitor:
                    self.monitor.stop.set()
                    self.monitor.thread.join(4)
                    if self.monitor.thread.is_alive():
                        raise RuntimeError('上次实时监视尚未退出')
                ids = [robot_number(row['robot_id']) for row in rows.values()]
                if robot_number(options['leader']) not in ids:
                    self.events.put(('log', '当前未选领航车，实时监视请在“监视与使能”启动'))
                else:
                    # Use the last read-back offsets; edits take effect in the configuration step.
                    offsets = {str(robot_number(row['robot_id'])):
                               [float(configs[ip].get('offset_x', -0.8)), float(configs[ip].get('offset_y', 0.8))]
                               for ip, row in rows.items() if row['robot_id'] != options['leader']}
                    identities = {ip: robot_number(row['robot_id']) for ip, row in rows.items()}
                    limits = session_limits(options, identities)
                    command = ros_command(options, master, 'python -u %s/fleet_ros.py monitor --ids %s --leader %d --linear-limits %s --offsets %s --bounds=%s --data-timeout %s' %
                                          (shlex.quote(remotes[master]), ','.join(map(str, ids)), robot_number(options['leader']),
                                           shlex.quote(json.dumps(limits)), shlex.quote(json.dumps(offsets)),
                                           ','.join(str(v) for v in path_bounds(config)), saved_data_timeout(options)))
                    self.monitor = Monitor(clients.pop(master), command, self.events, limits)
                    self.events.put(('monitor_started', ids, options['leader'], identities, offsets))
                    deadline = time.monotonic() + 20
                    while not self.monitor.ready:
                        if self.cancel.wait(0.1) or not self.monitor.thread.is_alive() or time.monotonic() > deadline:
                            raise RuntimeError('实时监视未就绪，检查 SSH / ROS Master')
            self.events.put(('done', '启动任务完成；在“监视与使能”检查并使能跟随。', results))
        except Exception as error:
            current = getattr(error, 'formation_ip', current)
            results[current] = '失败：' + str(error)
            self.events.put(('state', current, results[current]))
            self.events.put(('done', '启动未完成；已启动步骤保留供检查，可点击“停止本次启动”。', results))
        finally:
            for client in clients.values():
                client.close()

    def wait_ready(self, client, options, ip, name, remote, step, terminal=None):
        def check_terminal(_=None):
            if terminal:
                status = terminal.status()
                if status['state'] != '运行中':
                    raise RuntimeError('启动终端 %s：%s' % (status['state'], status.get('error', '请查看终端输出')))
        if terminal:
            deadline = time.monotonic() + 15
            while terminal.status()['state'] in ('打开终端中', '正在连接') and time.monotonic() < deadline:
                if self.cancel.wait(0.1):
                    raise RuntimeError('启动已取消')
            check_terminal()
        command = ros_command(options, ip, 'python -u %s/fleet_ros.py wait --step %s --ids %d --timeout 45' %
                              (shlex.quote(remote), step, robot_number(name)))
        self.events.put(('state', ip, '等待 ' + step + ' 数据（最长 45 秒）'))
        run_remote(client, command, self.cancel, timeout=55, progress=check_terminal)
        check_terminal()

    def wait_ready_parallel(self, jobs, clients, options, remotes, launched, results,
                            ready_text=None):
        if not jobs:
            return
        with ThreadPoolExecutor(max_workers=len(jobs)) as pool:
            futures = {pool.submit(self.wait_ready, clients[ip], options, ip, row['robot_id'],
                                    remotes[ip], stage, launched[ip]): (ip, stage)
                       for ip, row, stage in jobs}
            for future in as_completed(futures):
                ip, stage = futures[future]
                try:
                    future.result()
                except Exception as error:
                    error.formation_ip = ip
                    raise
                results[ip] = ready_text if ready_text else stage + ' 已就绪'
                self.events.put(('state', ip, results[ip]))

    def set_enabled(self, enabled):
        if not self.monitor or not self.monitor.ready:
            self.status.set('请先连接实时监视，等待 ROS 数据')
            return
        if enabled:
            now = time.monotonic()
            for number in self.current_ids:
                if not self.limit_ready(number):
                    self.status.set('ugv%d 线速度上限尚未确认，请在第三页检查；旧控制器需同步仓库并重新启动' % number)
                    return
                if not self.fresh(str(number), now):
                    self.status.set('ugv%d 定位无效或过期，不能使能跟随' % number)
                    return
            self.monitor.enable = True
        else:
            self.monitor.enable = False
        self.monitor.enable_sequence += 1
        self.status.set('正在发送编队使能' if enabled else '正在发送禁用跟随 / 停车')

    def emergency(self):
        self.armed.set(False)
        self.send_control('stop')
        if self.monitor:
            self.monitor.drive = (0, 0, time.monotonic())
            self.monitor.enable = False
            self.monitor.enable_sequence += 1
        self.app.stop_motion()

    def stop(self):
        self.cancel.set()
        self.emergency()
        if not self.stopper or not self.stopper.is_alive():
            def shutdown():
                if self.monitor:
                    self.monitor.stop.set()
                    self.monitor.thread.join(3)
                # The startup worker observes cancel before creating more terminals.
                for step, _, terminal in reversed(list(self.terminals)):
                    if step != 'shell':
                        terminal.stop()
                if self.worker and self.worker.is_alive():
                    self.worker.join(8)
                for step, _, terminal in reversed(list(self.terminals)):
                    if step != 'shell':
                        terminal.stop()
            self.stopper = threading.Thread(target=shutdown, daemon=True)
            self.stopper.start()
        self.status.set('正在停止本次启动的 ROS 终端和监视进程')

    def space_stop(self, _):
        if self.armed.get() or str(self.root.tk.call('focus')) in (str(self.canvas), str(self.keyboard)):
            self.emergency()

    def focus_stop(self):
        # Native Tk message boxes have no Python widget object to resolve.
        if self.root.winfo_exists() and not self.root.tk.call('focus', '-displayof', self.root._w):
            self.armed.set(False)

    def arm_changed(self, *_):
        self.stop_keyboard()
        if not self.armed.get():
            self.keyboard_status.set('键盘已停用；点击控制区重新启用，再按方向键。')

    def stop_keyboard(self):
        # Keep keys_down until release so a held key cannot re-arm after a stop.
        self.keyboard_keys.clear()
        if self.monitor:
            self.monitor.drive = (0, 0, time.monotonic())

    def keyboard_connection_error(self, now):
        if not self.monitor or not self.monitor.ready or now - self.monitor.last > 0.6:
            return '请先点击本页“连接实时监视”，等待连接恢复后重新启用键盘。'
        if self.app.vars['leader'].get().strip() != getattr(self, 'active_leader', ''):
            return '领航车设置已改变，请按新设置重新启动，再启用键盘。'
        if self.monitor.subscribers < 1:
            return '领航车速度话题没有接收节点，请先启动领航车底盘。'
        return ''

    def activate_keyboard(self):
        self.armed.set(False)
        error = self.keyboard_connection_error(time.monotonic())
        if error:
            self.keyboard_status.set(error)
            return
        if self.app.vars['leader_control_mode'].get() != 'keyboard':
            return
        self.send_control('keyboard')
        self.keyboard.focus_set()
        self.armed.set(True)
        self.update_keyboard(time.monotonic())

    def key_press(self, event):
        key = event.keysym.lower()
        if key in ('space', 'escape'):
            self.emergency()
            return 'break'
        if key in KEY_DIRECTIONS:
            pending = self.key_releases.pop(key, None)
            if pending:
                self.root.after_cancel(pending)  # X11 autorepeat sends a release/press pair.
            elif key not in self.keys_down:
                self.keys_down.add(key)
                if self.armed.get():
                    self.keyboard_keys.add(key)
                    self.update_keyboard(time.monotonic())
            return 'break'

    def key_release(self, event):
        key = event.keysym.lower()
        if key in ('space', 'escape'):
            return 'break'  # Space must not invoke the focused button and re-enable driving.
        if key in KEY_DIRECTIONS:
            pending = self.key_releases.pop(key, None)
            if pending:
                self.root.after_cancel(pending)
            def release():
                self.key_releases.pop(key, None)
                self.keys_down.discard(key)
                self.keyboard_keys.discard(key)
                self.update_keyboard(time.monotonic())
            self.key_releases[key] = self.root.after_idle(release)
            return 'break'

    def update_keyboard(self, now):
        if not self.armed.get():
            return
        if str(self.root.tk.call('focus')) != str(self.keyboard) or not self.keyboard.winfo_viewable():
            self.armed.set(False)
            return
        error = self.keyboard_connection_error(now)
        if error:
            self.armed.set(False)
            self.keyboard_status.set(error)
            return
        try:
            linear, angular = (float(self.app.vars[key].get()) for key in ('linear', 'angular'))
            if not (math.isfinite(linear) and math.isfinite(angular) and
                    0 < linear <= 0.5 and 0 < angular <= 1.5):
                raise ValueError('speed')
        except (ValueError, self.tk.TclError):
            self.armed.set(False)
            self.keyboard_status.set('线速度需大于 0 且不超过 0.5 m/s，角速度需大于 0 且不超过 1.5 rad/s。')
            return
        directions = {KEY_DIRECTIONS[key] for key in self.keyboard_keys}
        x = int((1, 0) in directions) - int((-1, 0) in directions)
        z = int((0, 1) in directions) - int((0, -1) in directions)
        linear = min(linear, self.monitor.limits[str(robot_number(self.active_leader))])
        self.monitor.drive = (x * linear, z * angular, now)
        self.keyboard_status.set('键盘已启用 → %s · 输出 %.2f m/s，%.2f rad/s · 全部松开停车' %
                                 (self.active_leader, x * linear, z * angular))

    def edit_vehicle(self):
        from fleet_iot import edit_ports
        return edit_ports(self)

    def open_iot(self):
        from fleet_iot import IotWindow, sensors_for
        if self.iot_window and not self.iot_window.closed:
            self.iot_window.dialog.lift()
            return
        if any(session.thread.is_alive() for session in self.iot_sessions):
            self.status.set('IOT 正在停止并保存数据，请稍后重试')
            return
        try:
            rows = {ip: dict(self.app.robots[ip]) for ip in self.addresses()}
            for row in rows.values():
                if row.get('pending_id'):
                    raise ValueError('请先应用车辆编号，再启动 IOT')
                sensors_for(row)
            self.iot_window = IotWindow(self, rows)
            self.iot_sessions = self.iot_window.sessions
        except ValueError as error:
            self.app.messagebox.showerror('IOT 监视', str(error))

    def limit_ready(self, number):
        monitor = self.monitor
        if not monitor or not monitor.ready or time.monotonic() - monitor.last > 0.6:
            return False
        row = monitor.limit_status.get(str(number), {})
        expected = monitor.limits.get(str(number))
        return row.get('ready', False) and expected is not None and row.get('applied') == row.get('requested') == expected

    def refresh_limits(self):
        value = saved_linear_limit(self.app.current_options())
        try:
            editing = float(self.limit_input.get()) != value
        except ValueError:
            editing = True
        if editing:
            message = '尚未应用'
        elif not self.active_identities or not self.monitor or not self.monitor.ready:
            message = '已保存 %g m/s · 待连接' % value
        else:
            pending = [number for number in self.active_identities.values() if not self.limit_ready(number)]
            total = len(self.active_identities)
            message = '已生效 %d/%d' % (total - len(pending), total)
            if pending:
                message += ' · 待确认 ' + '/'.join('ugv%d' % n for n in sorted(pending))
            other = len(set(self.app.robots) - set(self.active_identities))
            if other:
                message += ' · 待启动 %d' % other
        self.limits_status.set(message)

    def apply_limits(self):
        if self.worker and self.worker.is_alive():
            self.limits_status.set('启动中，请完成后再应用')
            return
        try:
            value = saved_linear_limit({'linear_limit': float(self.limit_input.get())})
            self.app.vars['linear_limit'].set(str(value))
            self.app.save()
            if self.monitor:
                self.monitor.limits = session_limits(self.app.current_options(), self.active_identities)
            self.update_keyboard(time.monotonic())
            self.refresh_limits()
        except ValueError:
            self.app.messagebox.showerror('线速度约束', '请输入 0–0.5 m/s 的线速度上限')

    def send_control(self, action, **values):
        if self.monitor:
            previous = getattr(self.monitor, 'control_request', {'sequence': 0})
            self.monitor.drive = (0, 0, time.monotonic())
            self.monitor.control_request = dict(values, sequence=previous['sequence']+1, action=action)

    def close_control_dialog(self):
        self.set_path_picking(False)
        self.armed.set(False)
        self.send_control('pause' if self.app.vars['leader_control_mode'].get() == 'path' else 'stop')
        if self.control_dialog:
            self.control_dialog.destroy()
            self.control_dialog = self.keyboard = None

    def open_leader_control(self):
        self.set_path_picking(False)
        if self.control_dialog and self.control_dialog.winfo_exists():
            self.control_dialog.deiconify()
            self.control_dialog.lift()
            return
        dialog = self.control_dialog = self.tk.Toplevel(self.root)
        dialog.title('领航控制')
        dialog.transient(self.root)
        dialog.geometry('720x490')
        dialog.protocol('WM_DELETE_WINDOW', self.close_control_dialog)
        dialog.bind('<Escape>', lambda _: self.emergency())
        top = self.ttk.Frame(dialog, padding=10)
        top.pack(fill='x')
        self.ttk.Label(top, text='领航车（第二页设置）：').pack(side='left')
        self.ttk.Label(top, textvariable=self.app.vars['leader']).pack(side='left', padx=6)
        keyboard = self.ttk.Frame(dialog, padding=12)
        path = self.ttk.Frame(dialog, padding=12)
        def switch(send=True):
            self.armed.set(False)
            mode = self.app.vars['leader_control_mode'].get()
            keyboard.pack_forget()
            path.pack_forget()
            (path if mode == 'path' else keyboard).pack(fill='both', expand=True)
            if send:
                self.send_control(mode)
        for text, value in [('键盘控制', 'keyboard'), ('参考路径', 'path')]:
            self.ttk.Radiobutton(top, text=text, variable=self.app.vars['leader_control_mode'],
                                 value=value, command=switch).pack(side='left', padx=10)
        pad = self.ttk.Frame(keyboard)
        pad.pack(fill='x', pady=8)
        for label, key in [('线速度 m/s', 'linear'), ('角速度 rad/s', 'angular')]:
            self.ttk.Label(pad, text=label).pack(side='left', padx=(4, 2))
            self.ttk.Entry(pad, textvariable=self.app.vars[key], width=7).pack(side='left')
        self.keyboard = self.ttk.Button(pad, text='点击启用 · WASD / 方向键', command=self.activate_keyboard, takefocus=True)
        self.keyboard.pack(side='left', padx=8)
        self.keyboard.bind('<KeyPress>', self.key_press)
        self.keyboard.bind('<KeyRelease>', self.key_release)
        self.keyboard.bind('<FocusOut>', lambda _: self.armed.set(False))
        self.ttk.Label(keyboard, text='W / ↑ 前进；S / ↓ 后退；A / ← 左转；D / → 右转。\n可组合按键，例如 W+A 边前进边左转；松开一个键保留另一方向。\n同轴反向键抵消，全部松开停车；失焦后需重新启用。空格 / Esc 停车。').pack(anchor='w', pady=12)
        self.ttk.Label(keyboard, textvariable=self.keyboard_status, wraplength=650).pack(fill='x')
        self.ttk.Label(path, text='可在地图上选点，或每行填写 X, Y（UWB 坐标，单位米）；至少两点。').pack(anchor='w')
        self.path_editor = self.tk.Text(path, height=5, width=40)
        self.path_editor.pack(fill='x', pady=5)
        self.path_editor.insert('1.0', self.app.vars['leader_path'].get())
        try:
            self.path_preview, self.path_bends = self.read_path(min_points=0)
        except ValueError:
            self.path_preview, self.path_bends = [], []
        def remember(_=None):
            if self.path_editor.edit_modified():
                text = self.path_editor.get('1.0', 'end-1c')
                if text != self.app.vars['leader_path'].get():
                    self.path_preview, self.path_bends = [], []
                    self.app.vars['leader_path_bends'].set('[]')
                    self.app.vars['leader_path'].set(text)
                self.path_editor.edit_modified(False)
        self.path_editor.bind('<<Modified>>', remember)
        edit = self.ttk.Frame(path)
        edit.pack(fill='x', pady=4)
        for label, command in [('地图选点', self.begin_path_pick),
                               ('撤销末点', lambda: self.edit_path('undo')),
                               ('清空路径', lambda: self.edit_path('clear'))]:
            self.ttk.Button(edit, text=label, command=command).pack(side='left', padx=(0, 8))
        fields = self.ttk.Frame(path)
        fields.pack(fill='x', pady=5)
        self.ttk.Button(fields, text='使用当前位置为起点', command=self.use_current_start).pack(side='left', padx=(0, 10))
        for label, key in [('巡航 m/s', 'path_speed'), ('预瞄 m', 'path_lookahead')]:
            self.ttk.Label(fields, text=label).pack(side='left', padx=(5, 3))
            self.ttk.Entry(fields, textvariable=self.app.vars[key], width=7).pack(side='left')
        self.ttk.Label(path, text='地图中可拖动线段设置曲线；手动修改坐标会恢复直线。\n先预览再开始；预瞄 0.3–0.5 m，建议巡航 0.10 m/s。关闭弹窗会暂停。').pack(anchor='w', pady=4)
        actions = self.ttk.Frame(path)
        actions.pack(fill='x', pady=4)
        self.ttk.Button(actions, text='预览路径', command=self.preview_path).pack(side='left', padx=(0, 8))
        for label, action in [('开始', 'start'), ('暂停', 'pause'), ('继续', 'resume'), ('停止', 'stop')]:
            self.ttk.Button(actions, text=label, command=lambda action=action: self.trajectory_action(action)).pack(side='left', padx=(0, 8))
        self.ttk.Label(path, textvariable=self.leader_status, wraplength=650).pack(fill='x', pady=5)
        self.ttk.Button(dialog, text='立即停车 / 禁用跟随', command=self.emergency).pack(side='bottom', pady=8)
        switch(False)

    def use_current_start(self):
        self.edit_path('current')

    def check_path_editable(self):
        if self.monitor and self.monitor.ready:
            tracking = getattr(self.monitor, 'tracking', {})
            request = getattr(self.monitor, 'control_request', {})
            if (self.tracking.get('state') in ('TRACKING', 'ALIGNING') or
                    tracking.get('state') in ('TRACKING', 'ALIGNING') or
                    (request.get('action') in ('start', 'resume') and
                     request.get('sequence', 0) > tracking.get('ack', -1))):
                raise ValueError('请先暂停，再编辑、预览或开始新路径')

    def set_path_picking(self, enabled):
        self.path_drag = None
        self.path_picking = enabled
        self.canvas.configure(cursor='crosshair' if enabled else '')
        if enabled:
            self.path_tools.pack(fill='x', before=self.canvas, pady=5)
        else:
            self.path_tools.pack_forget()

    def begin_path_pick(self):
        try:
            self.check_path_editable()
        except ValueError as error:
            self.app.messagebox.showerror('参考路径', str(error), parent=self.control_dialog)
            return
        self.armed.set(False)
        self.path_pick_status.set('左键空白处添加点；拖动线段 / 中点圆环设置曲线；右键线段恢复直线，右键空白处撤销末点。')
        try:
            self.path_preview, self.path_bends = self.read_path(min_points=0)
        except ValueError as error:
            self.path_preview = []
            self.path_pick_status.set(TRACKING_TEXT.get(str(error), str(error)) + '；可清空路径后重新选点。')
        self.app.notebook.select(self.canvas.master)
        self.set_path_picking(True)
        self.control_dialog.withdraw()
        self.paint_map(time.monotonic())

    def read_path(self, min_points=2):
        points = parse_points(self.path_editor.get('1.0', 'end-1c'), min_points=min_points)
        bends = json.loads(self.app.vars['leader_path_bends'].get())
        sample_path(points, bends, min_points=min_points)
        return points, bends or [0.0] * max(0, len(points)-1)

    def check_curve_bounds(self, points, bends):
        sampled = sample_path(points, bends, min_points=0)
        config = localization_config(self.app.vars['anchors_json'].get(),
                                     self.app.vars['anchor_profile'].get())
        offsets = list(self.active_offsets.values()) if self.monitor and self.monitor.enable else []
        check_bounds(sampled, path_bounds(config), offsets)

    def save_path_bends(self, bends):
        self.path_bends = bends
        self.app.vars['leader_path_bends'].set(json.dumps(bends))

    def edit_path(self, action, point=None):
        try:
            self.check_path_editable()
            self.path_drag = None
            lines = [line for line in self.path_editor.get('1.0', 'end-1c').splitlines() if line.strip()]
            if action == 'clear':
                lines = []
            elif action == 'undo':
                lines = lines[:-1]
            elif action == 'append':
                lines.append('%.4f, %.4f' % point)
            elif action == 'current':
                number = str(robot_number(self.app.vars['leader'].get()))
                if not self.fresh(number, time.monotonic()):
                    raise ValueError('领航车定位无效或过期，请连接实时监视')
                lines = ['%.4f, %.4f' % tuple(self.samples[number]['pose']['value'])] + lines[1:]
            points = parse_points('\n'.join(lines), min_points=0)
            bends = self.path_bends[:max(0, len(points)-1)]
            bends += [0.0] * max(0, len(points)-1-len(bends))
            if action == 'current' and bends:
                bends[0] = 0.0
            if action in ('append', 'current'):
                self.check_curve_bounds(points, bends)
            text = '\n'.join('%.4f, %.4f' % p for p in points)
            self.path_editor.delete('1.0', 'end')
            self.path_editor.insert('1.0', text)
            self.app.vars['leader_path'].set(text)
            self.save_path_bends(bends)
            self.path_preview = points
            self.path_pick_status.set('已选 %d 个点；拖动线段设置曲线，右键线段恢复直线，右键空白处撤销末点。%s' %
                                      (len(points), '至少需要两点。' if len(points) < 2 else '选好后点击“完成选点”。'))
            self.paint_map(time.monotonic())
        except ValueError as error:
            message = TRACKING_TEXT.get(str(error), str(error))
            if self.path_picking:
                self.path_pick_status.set(message)
            else:
                self.app.messagebox.showerror('参考路径', message, parent=self.control_dialog)

    def map_path_click(self, event):
        self.canvas.focus_set()
        if not self.path_picking:
            return
        self.path_drag = None
        x, y = self.canvas.canvasx(event.x), self.canvas.canvasy(event.y)
        items = self.canvas.find_overlapping(x-6, y-6, x+6, y+6)
        tags = [tag for item in reversed(items) for tag in self.canvas.gettags(item)]
        segment = next((int(tag.split(':')[1]) for tag in tags if tag.startswith('path_segment:')), None)
        if segment is not None:
            try:
                self.check_path_editable()
                if event.num == 3:
                    bends = list(self.path_bends)
                    bends[segment] = 0.0
                    self.check_curve_bounds(self.path_preview, bends)
                    self.save_path_bends(bends)
                    self.path_pick_status.set('该段已恢复直线；可再次拖动调整。')
                    self.paint_map(time.monotonic())
                else:
                    self.path_drag = dict(index=segment, start=(x, y), transform=self.map_transform,
                                          bends=list(self.path_bends), valid=False)
                    self.path_pick_status.set('拖动调整弯曲幅度，松开鼠标保存；两端路径点保持不变。')
            except ValueError as error:
                self.path_pick_status.set(TRACKING_TEXT.get(str(error), str(error)))
            return
        if event.num == 3:
            self.edit_path('undo')
        elif self.map_transform:
            ox, oy, scale = self.map_transform
            self.edit_path('append', ((self.canvas.canvasx(event.x) - ox) / scale,
                                      (oy - self.canvas.canvasy(event.y)) / scale))

    def drag_path_curve(self, event):
        drag = self.path_drag
        if not drag:
            return
        drag['valid'] = False
        try:
            self.check_path_editable()
            index = drag['index']
            a, b = self.path_preview[index:index+2]
            dx, dy = b[0]-a[0], b[1]-a[1]
            scale = drag['transform'][2]
            mx = (self.canvas.canvasx(event.x)-drag['start'][0])/scale
            my = (drag['start'][1]-self.canvas.canvasy(event.y))/scale
            bends = list(self.path_bends)
            bends[index] = round(bends[index]+(-dy*mx+dx*my)/math.hypot(dx, dy), 4)
            self.check_curve_bounds(self.path_preview, bends)
            drag.update(bends=bends, valid=True)
            self.path_pick_status.set('弯曲幅度 %.2f m；松开鼠标保存。' % abs(bends[index]))
        except ValueError as error:
            self.path_pick_status.set(TRACKING_TEXT.get(str(error), str(error)) + '；本次拖动未保存。')
        self.paint_map(time.monotonic())

    def finish_path_curve(self, event):
        if not self.path_drag:
            return
        self.drag_path_curve(event)
        drag, self.path_drag = self.path_drag, None
        if drag['valid']:
            self.save_path_bends(drag['bends'])
            self.path_pick_status.set('曲线已保存；右键该线段可恢复直线，选好后点击“完成选点”。')
        self.paint_map(time.monotonic())

    def preview_path(self):
        try:
            self.check_path_editable()
            points, bends = self.read_path()
            self.check_curve_bounds(points, bends)
            self.path_bends = bends
            self.path_preview = points
            self.app.vars['leader_path'].set(self.path_editor.get('1.0', 'end-1c'))
            self.app.save()
            self.paint_map(time.monotonic())
            return points
        except ValueError as error:
            self.app.messagebox.showerror('参考路径', TRACKING_TEXT.get(str(error), str(error)), parent=self.control_dialog)

    def trajectory_action(self, action):
        self.armed.set(False)
        try:
            error = self.keyboard_connection_error(time.monotonic())
            if error:
                raise ValueError(error)
            if not getattr(self.monitor, 'tracking', {}):
                raise ValueError('请重新连接实时监视以加载路径控制功能')
            values = {}
            if action == 'start':
                points = self.preview_path()
                if points is None:
                    return
                if any(self.path_bends) and not self.monitor.tracking.get('curves_supported'):
                    raise ValueError('请重新连接实时监视以加载曲线路径功能')
                speed = float(self.app.vars['path_speed'].get())
                lookahead = float(self.app.vars['path_lookahead'].get())
                if not 0 < speed <= 0.5 or not 0.3 <= lookahead <= 0.5:
                    raise ValueError('PATH_SETTINGS')
                values = dict(points=points, bends=self.path_bends, speed=speed, lookahead=lookahead)
            self.send_control(action, **values)
            self.leader_status.set('正在发送领航控制指令…')
        except ValueError as error:
            self.app.messagebox.showerror('领航控制', TRACKING_TEXT.get(str(error), str(error)), parent=self.control_dialog)

    def make_map(self):
        page = self.ttk.Frame(self.app.notebook, padding=10)
        self.app.notebook.add(page, text='小车定位图')
        toolbar = self.ttk.Frame(page)
        toolbar.pack(fill='x')
        self.ttk.Button(toolbar, text='连接实时监视', command=lambda: self.start('monitor')).pack(side='left')
        self.ttk.Button(toolbar, text='启动 IOT 实时监视', command=self.open_iot).pack(side='left', padx=6)
        self.ttk.Button(toolbar, text='编辑基站坐标 / 定位参数', command=self.edit_anchors).pack(side='left', padx=8)
        self.ttk.Button(toolbar, text='清空轨迹', command=self.clear_trails).pack(side='left')
        self.ttk.Button(toolbar, text='立即停车 / 禁用跟随', command=self.emergency).pack(side='right')
        self.ttk.Button(toolbar, text='领航控制…', command=self.open_leader_control).pack(side='left', padx=8)
        profile = self.ttk.Frame(page)
        profile.pack(fill='x', pady=(4, 0))
        self.ttk.Label(profile, text='UWB 基站配置').pack(side='left')
        self.anchor_profile_box = self.ttk.Combobox(
            profile, textvariable=self.app.vars['anchor_profile'], state='readonly', width=12,
            values=list(ANCHOR_PROFILES))
        self.anchor_profile_box.pack(side='left', padx=6)
        self.anchor_profile_box.bind('<<ComboboxSelected>>', self.select_anchor_profile)
        self.ttk.Label(profile, text='可在“编辑基站”中修改、增加、删除基站；切换配置后重新启动定位。').pack(side='left')
        self.ttk.Label(profile, text='定位方式').pack(side='left', padx=(18, 4))
        self.localization_mode_box = self.ttk.Combobox(
            profile, textvariable=self.app.vars['localization_mode'], state='readonly', width=16,
            values=LOCALIZATION_MODES)
        self.localization_mode_box.pack(side='left')
        self.ttk.Label(profile, text='five_ugv 算法 / LinkTrack 输出').pack(side='left', padx=6)
        self.ttk.Label(page, textvariable=self.leader_status, wraplength=1080).pack(fill='x', pady=4)
        limits = self.ttk.Frame(page)
        limits.pack(fill='x', pady=5)
        self.ttk.Label(limits, text='全车线速度上限 m/s').pack(side='left', padx=(4, 5))
        self.limit_input = self.tk.StringVar(value=str(saved_linear_limit(self.app.current_options())))
        self.tk.Spinbox(limits, from_=0, to=0.5, increment=0.01, width=7,
                        textvariable=self.limit_input).pack(side='left')
        self.ttk.Button(limits, text='应用到全部', command=self.apply_limits).pack(side='left', padx=8)
        self.ttk.Label(limits, text='0–0.5；0 时路径暂停').pack(side='left')
        self.ttk.Label(limits, text='数据过期秒数').pack(side='left', padx=(14, 5))
        self.tk.Spinbox(limits, from_=0.05, to=5.0, increment=0.05, width=7,
                        textvariable=self.app.vars['data_timeout']).pack(side='left')
        self.ttk.Label(limits, text='重连监视 / 重启跟随后生效').pack(side='left', padx=(5, 0))
        self.limits_status = self.tk.StringVar()
        self.ttk.Label(limits, textvariable=self.limits_status, wraplength=430).pack(side='left', padx=10)
        self.refresh_limits()
        self.map_status = self.tk.StringVar(value='尚未连接。基站坐标读取自所选 UWB 配置；单位：米。')
        self.ttk.Label(page, textvariable=self.map_status, wraplength=1080).pack(fill='x', pady=7)
        self.ttk.Label(page, text='三角形：基站　圆点：车辆　实线：实际轨迹　虚线：目标 / 误差　紫色虚线：参考路径　灰色：定位过期或无效').pack(anchor='w')
        self.canvas = self.tk.Canvas(page, background='#f7fafc', highlightthickness=0, takefocus=True)
        self.canvas.pack(fill='both', expand=True)
        self.canvas.bind('<Button-1>', self.map_path_click)
        self.canvas.bind('<Button-3>', self.map_path_click)
        self.canvas.bind('<B1-Motion>', self.drag_path_curve)
        self.canvas.bind('<ButtonRelease-1>', self.finish_path_curve)
        self.canvas.bind('<Configure>', lambda _: self.paint_map(time.monotonic()))
        self.path_tools = self.ttk.Frame(page)
        buttons = self.ttk.Frame(self.path_tools)
        buttons.pack(fill='x')
        for label, command in [('完成选点', self.open_leader_control),
                               ('使用当前位置为起点', self.use_current_start),
                               ('撤销末点', lambda: self.edit_path('undo')),
                               ('清空路径', lambda: self.edit_path('clear'))]:
            self.ttk.Button(buttons, text=label, command=command).pack(side='left', padx=(0, 8))
        self.path_pick_status = self.tk.StringVar()
        self.ttk.Label(self.path_tools, textvariable=self.path_pick_status, wraplength=1080).pack(anchor='w', pady=4)
        self.metrics = self.tk.StringVar(value='三角形：基站　圆点：车辆　十字：跟踪目标　虚线：位置误差　灰色：定位无效 / 数据过期')
        self.ttk.Label(page, textvariable=self.metrics, wraplength=1080).pack(fill='x', pady=6)

    def clear_trails(self):
        self.trails.clear()
        self.last_pose.clear()
        self.target_trails.clear()
        self.error_history.clear()

    def edit_anchors(self):
        dialog = self.tk.Toplevel(self.root)
        dialog.title('基站坐标与定位参数')
        dialog.geometry('610x530')
        config = localization_config(self.app.vars['anchors_json'].get(),
                                     self.app.vars['anchor_profile'].get())
        table = self.ttk.Treeview(dialog, columns=('id', 'x', 'y', 'z'), show='headings', height=9)
        for key in ('id', 'x', 'y', 'z'):
            table.heading(key, text='ID' if key == 'id' else key.upper() + (' 高度' if key == 'z' else ''))
            table.column(key, width=130)
        table.pack(fill='both', expand=True, padx=10, pady=8)
        for anchor in config['anchors']:
            table.insert('', 'end', values=[anchor[key] for key in ('id', 'x', 'y', 'z')])
        fields = [self.tk.StringVar(value='0') for _ in range(4)]
        editor = self.ttk.Frame(dialog)
        editor.pack(fill='x', padx=10)
        for key, variable in zip(('ID', 'X', 'Y', 'Z'), fields):
            self.ttk.Label(editor, text=key).pack(side='left')
            self.ttk.Entry(editor, textvariable=variable, width=10).pack(side='left', padx=(3, 8))
        def select(_):
            if table.selection():
                for variable, value in zip(fields, table.item(table.selection()[0], 'values')):
                    variable.set(value)
        table.bind('<<TreeviewSelect>>', select)
        def put(new=False):
            try:
                values = [int(fields[0].get())] + [float(v.get()) for v in fields[1:]]
                if any(not math.isfinite(v) for v in values):
                    raise ValueError('坐标必须是有限数值')
                if new or not table.selection():
                    table.insert('', 'end', values=values)
                else:
                    table.item(table.selection()[0], values=values)
            except ValueError as error:
                self.app.messagebox.showerror('坐标有误', str(error), parent=dialog)
        buttons = self.ttk.Frame(dialog)
        buttons.pack(pady=7)
        for title, command in [('新增一行', lambda: put(True)), ('修改选中行', put),
                               ('删除选中行', lambda: table.delete(*table.selection()))]:
            self.ttk.Button(buttons, text=title, command=command).pack(side='left', padx=5)
        height = self.tk.StringVar(value=str(config['tag_height']))
        height_row = self.ttk.Frame(dialog)
        height_row.pack()
        self.ttk.Label(height_row, text='车载标签高度 Z（米）').pack(side='left')
        self.ttk.Entry(height_row, textvariable=height, width=10).pack(side='left')
        residual_limit = self.tk.StringVar(value=str(config['valid_max_residual_rms']))
        self.ttk.Label(height_row, text='有效残差阈值（米）').pack(side='left', padx=(18, 0))
        self.ttk.Entry(height_row, textvariable=residual_limit, width=10).pack(side='left')
        self.ttk.Label(dialog, text='保存后地图立即更新；下次“底盘与定位”启动时上传完整定位配置。\n已运行的定位节点需先停止再启动，修改行后请点击“修改选中行”。',
                       wraplength=560).pack(pady=7)
        def save(reset=False):
            try:
                anchors = []
                for item in table.get_children():
                    value = table.item(item, 'values')
                    anchors.append(dict(id=int(value[0]), x=float(value[1]), y=float(value[2]), z=float(value[3])))
                encoded = '' if reset else json.dumps(dict(
                    anchors=anchors, tag_height=float(height.get()),
                    valid_max_residual_rms=float(residual_limit.get())))
                localization_config(encoded, self.app.vars['anchor_profile'].get())
                self.app.vars['anchors_json'].set(encoded)
                self.app.save()
                self.clear_trails()
                dialog.destroy()
            except ValueError as error:
                self.app.messagebox.showerror('基站配置有误', str(error), parent=dialog)
        bottom = self.ttk.Frame(dialog)
        bottom.pack(pady=6)
        self.ttk.Button(bottom, text='保存基站设置', command=save).pack(side='left', padx=8)
        self.ttk.Button(bottom, text='恢复所选档案默认值', command=lambda: save(True)).pack(side='left')

    def select_anchor_profile(self, _=None):
        overrides = json.loads(self.app.vars['anchor_overrides_json'].get())
        overrides[self.selected_anchor_profile] = self.app.vars['anchors_json'].get()
        self.selected_anchor_profile = self.app.vars['anchor_profile'].get()
        self.app.vars['anchors_json'].set(overrides.get(self.selected_anchor_profile, ''))
        self.app.vars['anchor_overrides_json'].set(json.dumps(overrides))
        self.clear_trails()
        self.map_status.set('已切换到%s；下次启动定位时生效。' %
                            ANCHOR_PROFILES.get(self.app.vars['anchor_profile'].get(), '选定'))
        self.app.save()

    def fresh(self, number, now):
        row = self.samples.get(number, {})
        delta = now - self.last_sample
        try:
            timeout = saved_data_timeout(self.app.current_options())
        except (TypeError, ValueError):
            timeout = 0.6
        return (row.get('valid', {}).get('value') is True and
                row.get('valid', {}).get('age', 999) + delta < timeout and
                row.get('pose', {}).get('age', 999) + delta < timeout)

    def paint_map(self, now):
        canvas = self.canvas
        width, height = canvas.winfo_width(), canvas.winfo_height()
        if width < 100 or height < 100:
            return
        config = localization_config(self.app.vars['anchors_json'].get(),
                                     self.app.vars['anchor_profile'].get())
        try:
            data_timeout = saved_data_timeout(self.app.current_options())
        except (TypeError, ValueError):
            data_timeout = 0.6
        anchors = config['anchors']
        points = [(a['x'], a['y']) for a in anchors]
        waypoints = self.path_preview
        bends = self.path_drag['bends'] if self.path_drag and self.path_drag['valid'] else self.path_bends
        active = (not self.path_picking and self.monitor and self.monitor.ready and
                  self.tracking.get('state') in ('TRACKING', 'ALIGNING'))
        if active:
            path = self.tracking.get('points', [])
            waypoints = self.tracking.get('waypoints', path)
            sections = [path] if len(path) > 1 else []
        else:
            sections = [sample_path([a, b], [bend]) for a, b, bend in zip(waypoints, waypoints[1:], bends)]
            path = [p for section in sections for p in section] or waypoints
        points.extend(path)
        for row in self.samples.values():
            for key in ('pose', 'target'):
                if key in row:
                    points.append(row[key]['value'])
        xmin, xmax = min(p[0] for p in points) - 1, max(p[0] for p in points) + 1
        ymin, ymax = min(p[1] for p in points) - 1, max(p[1] for p in points) + 1
        scale = min((width - 90) / max(1, xmax - xmin), (height - 65) / max(1, ymax - ymin))
        ox, oy = width / 2 - (xmin + xmax) / 2 * scale, height / 2 + (ymin + ymax) / 2 * scale
        if self.path_drag:
            ox, oy, scale = self.path_drag['transform']
        self.map_transform = (ox, oy, scale)
        def xy(point):
            return ox + point[0] * scale, oy - point[1] * scale
        canvas.delete('all')
        grid_step = max(1, math.ceil(max(xmax - xmin, ymax - ymin) / 25))
        for x in range(math.floor(xmin), math.ceil(xmax) + 1, grid_step):
            a, b = xy((x, ymin)), xy((x, ymax))
            canvas.create_line(*a, *b, fill='#e2e8f0')
            canvas.create_text(a[0], a[1] + 12, text=str(x), fill='#64748b')
        for y in range(math.floor(ymin), math.ceil(ymax) + 1, grid_step):
            a, b = xy((xmin, y)), xy((xmax, y))
            canvas.create_line(*a, *b, fill='#e2e8f0')
            canvas.create_text(a[0] - 18, a[1], text=str(y), fill='#64748b')
        canvas.create_text(width - 30, height - 15, text='X / m')
        canvas.create_text(30, 15, text='Y / m')
        for a in anchors:
            x, y = xy((a['x'], a['y']))
            canvas.create_polygon(x, y - 8, x - 7, y + 6, x + 7, y + 6, fill='#334155')
            canvas.create_text(x, y - 20, text='A%s · z=%.2f' % (a['id'], a['z']), fill='#334155')
        for index, section in enumerate(sections):
            tags = ('reference_path', 'path_segment:%d' % index)
            canvas.create_line(*[v for point in section for v in xy(point)], fill='#7c3aed', width=2, dash=(8, 4), tags=tags)
            if self.path_picking:
                x, y = xy(curve_point(waypoints[index], waypoints[index+1], bends[index], 0.5))
                canvas.create_oval(x-6, y-6, x+6, y+6, fill='white', outline='#7c3aed', width=2,
                                   tags=('curve_handle', tags[1]))
        for index, point in enumerate(waypoints):
            x, y = xy(point)
            canvas.create_oval(x-4, y-4, x+4, y+4, fill='#7c3aed', outline='white', tags='path_point')
            canvas.create_text(x+5, y-10, text='P%d' % index, fill='#7c3aed', anchor='w')
        lookahead = self.tracking.get('target')
        if lookahead and now-self.last_sample < data_timeout and self.tracking.get('mode') == 'path':
            x, y = xy(lookahead)
            canvas.create_oval(x-5, y-5, x+5, y+5, outline='#7c3aed', width=2)
        metrics = []
        for index, number in enumerate(sorted(self.samples, key=int)):
            row = self.samples[number]
            if 'pose' not in row:
                status = row.get('uwb_status', {})
                text = status.get('value', '等待定位') if status.get('age', 999) + now-self.last_sample < data_timeout else '等待定位数据'
                metrics.append('ugv%s · UWB %s' % (number, text))
                continue
            valid = self.fresh(number, now)
            color = COLORS[index % len(COLORS)] if valid else '#94a3b8'
            pose = row['pose']['value']
            x, y = xy(pose)
            trail = self.trails.get(number, [])
            if len(trail) > 1:
                canvas.create_line(*[coordinate for point in trail for coordinate in xy(point)], fill=color, width=2)
            target_trail = self.target_trails.get(number, [])
            if len(target_trail) > 1:
                canvas.create_line(*[coordinate for point in target_trail for coordinate in xy(point)], fill=color, dash=(3, 4))
            target = row.get('target')
            if target and target['age'] + now - self.last_sample < data_timeout:
                tx, ty = xy(target['value'])
                canvas.create_line(tx - 6, ty, tx + 6, ty, fill=color, width=2)
                canvas.create_line(tx, ty - 6, tx, ty + 6, fill=color, width=2)
                canvas.create_line(x, y, tx, ty, fill=color, dash=(4, 3))
            canvas.create_oval(x - 7, y - 7, x + 7, y + 7, fill=color, outline='white', width=2)
            heading = row.get('yaw')
            if heading and heading['age'] + now - self.last_sample < data_timeout:
                angle = heading['value'] - self.yaw_origin.setdefault(number, heading['value'])
                if 'ugv'+number == getattr(self, 'active_leader', '') and self.tracking.get('yaw') is not None:
                    angle = self.tracking['yaw']
                canvas.create_line(x, y, x + 23 * math.cos(angle), y - 23 * math.sin(angle), fill=color, width=2, arrow='last')
            canvas.create_text(x + 10, y + 17, text='ugv' + number + ('' if valid else ' 过期/无效'), fill=color, anchor='w')
            error = row.get('error', {})
            state = row.get('state', {})
            error_text = '%.3f m' % error['value'] if error.get('age', 999) + now - self.last_sample < data_timeout else '—'
            state_text = state.get('value', '—') if state.get('age', 999) + now - self.last_sample < data_timeout else '数据过期'
            errors = self.error_history.get(number, [])
            rms = '%.3f m' % math.sqrt(sum(v*v for v in errors) / len(errors)) if errors else '—'
            metrics.append('ugv%s  (%.2f, %.2f)  误差 %s  最近 1000 样本 RMS %s  %s' %
                           (number, pose[0], pose[1], error_text, rms, state_text + ' · UWB ' +
                            (row.get('uwb_status', {}).get('value', '—') if row.get('uwb_status', {}).get('age', 999) + now-self.last_sample < data_timeout else '数据过期')))
        online = self.monitor and self.monitor.ready and now - self.last_sample < 2
        profile_name = ANCHOR_PROFILES.get(self.app.vars['anchor_profile'].get(), '未知配置')
        if self.app.vars['anchors_json'].get():
            profile_name += '（已自定义）'
        self.map_status.set('%s · 基站 %d 个 · 标签高度 %.2f m · 有效残差阈值 %.2f m · %s' %
                            ('实时监视已连接' if online else '实时监视未连接 / 数据过期', len(anchors), config['tag_height'],
                             config['valid_max_residual_rms'],
                             profile_name + '（新启动定位时生效）'))
        self.metrics.set('\n'.join(metrics) if metrics else '暂无实时误差数据。领航航向对齐到 UWB 地图；跟随车航向以连接时的朝向为 +X 参考。')

    def poll(self):
        if self.monitor:
            self.monitor.ui_heartbeat = time.monotonic()
        for _ in range(200):
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                break
            kind = event[0]
            if kind == 'sample':
                self.samples = event[1]['robots']
                self.tracking = event[1].get('tracking', {})
                if 'offsets' in self.tracking:
                    self.active_offsets = self.tracking['offsets']
                self.last_sample = time.monotonic()
                for number, row in self.samples.items():
                    if self.fresh(number, self.last_sample):
                        point = row['pose']['value']
                        previous = self.last_pose.get(number)
                        if previous is None or math.hypot(point[0] - previous[0], point[1] - previous[1]) > 0.01:
                            self.trails.setdefault(number, deque(maxlen=1000)).append(point)
                            self.last_pose[number] = point
                        target = row.get('target', {})
                        if target.get('age', 999) < 0.6:
                            trail = self.target_trails.setdefault(number, deque(maxlen=1000))
                            if not trail or trail[-1] != target['value']:
                                trail.append(target['value'])
                        error = row.get('error', {})
                        if error.get('age', 999) < 0.6:
                            self.error_history.setdefault(number, deque(maxlen=1000)).append(error['value'])
                if self.monitor and event[1].get('enable_sequence') == self.monitor.enable_sequence:
                    sent = event[1].get('enable_sent', False)
                    marker = (self.monitor.enable_sequence, sent)
                    if marker != getattr(self, 'enable_ack', None):
                        self.enable_ack = marker
                        message = 'ROS 已发送：' + ('使能编队跟随' if sent else '禁用跟随')
                        if self.monitor.enable and not sent:
                            message = '跟随未使能：请检查连接和限速状态后重新使能'
                            self.status.set(message)
                        self.append_log(message)
            elif kind == 'monitor_started':
                self.current_ids = event[1]
                self.active_leader = event[2]
                self.active_identities = event[3]
                self.active_offsets = event[4]
                self.tracking = {}
                self.samples.clear()
                self.clear_trails()
                self.yaw_origin.clear()
                self.armed.set(False)
            elif kind == 'monitor_error':
                self.emergency()
                self.status.set(event[1])
                self.append_log(event[1])
            elif kind == 'state':
                self.robot_states[event[1]] = event[2]
                self.append_log(event[1] + ' ' + event[2])
            elif kind == 'config':
                ip, config = event[1:]
                if ip not in self.app.robots:
                    continue
                self.app.robots[ip].update(config, name=config['robot_id'], ssh='已连接（启动会话）')
                if self.app.robots[ip].get('pending_id') == config['robot_id']:
                    self.app.robots[ip].pop('pending_id', None)
                self.app.refresh_row(ip)
                self.app.schedule_save()
            elif kind == 'done':
                self.status.set(event[1])
                if not self.app.closing:
                    self.result_popup('启动结果', event[2])
            elif kind == 'log':
                self.append_log(event[1])
        now = time.monotonic()
        if self.monitor and self.app.vars['leader'].get().strip() != getattr(self, 'active_leader', ''):
            if getattr(self.monitor, 'control_request', {}).get('action') != 'stop':
                self.emergency()
        if self.tracking.get('mode') == 'path' or (self.tracking.get('mode') == 'stopped' and self.tracking.get('points')):
            state = self.tracking.get('state', 'IDLE')
            reason = self.tracking.get('error') or self.tracking.get('reason') or self.tracking.get('input_problem', '')
            detail = TRACKING_TEXT.get(reason, reason)
            message = '%s · %s · %.2f / %.2f m · 横向误差 %.3f m%s' % (
                getattr(self, 'active_leader', ''), TRACKING_TEXT.get(state, state),
                self.tracking.get('progress', 0), self.tracking.get('total', 0),
                self.tracking.get('cross_track', 0), ' · '+detail if detail else '')
            if now-self.last_sample > 0.6:
                message = '领航遥测已过期；检查连接，恢复后需点击继续'
            self.leader_status.set(message)
            marker = (state, reason, self.tracking.get('ack'))
            if marker != getattr(self, 'tracking_marker', None):
                self.tracking_marker = marker
                self.append_log(message)
        elif self.app.vars['leader_control_mode'].get() == 'path':
            self.leader_status.set('参考路径待命；连接实时监视后预览并开始。')
        else:
            self.leader_status.set(self.keyboard_status.get())
        self.update_keyboard(now)
        if now - self.last_status > 1:
            for step, ip, terminal in list(self.terminals):
                status = terminal.status()
                key = str(terminal.directory)
                value = status['state'] + ('：' + status['error'] if status.get('error') else '')
                if self.terminal_states.get(key) != value:
                    self.terminal_states[key] = value
                    self.append_log('%s %s %s' % (ip, step, value))
                    if step != 'shell':
                        self.robot_states[ip] = step + ' ' + value
            self.refresh_vehicles()
            self.last_status = now
        if now - self.last_paint > 0.2:
            self.paint_map(now)
            self.last_paint = now

    def close(self):
        if self.iot_window and not self.iot_window.closed:
            self.iot_window.close()
        for session in self.iot_sessions + self.iot_probes:
            session.stop.set()
        self.stop()
        for pending in self.key_releases.values():
            self.root.after_cancel(pending)
        self.key_releases.clear()
