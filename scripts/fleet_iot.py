"""UWB port editor and independent IOT charts for the Tk fleet console."""
from collections import deque
from datetime import datetime
import json
import math
import ipaddress
from pathlib import Path
import queue
import shlex
import shutil
import stat
import threading
import time
import uuid

import yaml

from fleet_deploy import robot_number, run_remote, shell_path

UIDS = {'ugv2': (0x15003B00,), 'ugv3': (0x30006E00, 0x2A003200), 'ugv4': (0x30003000,)}
NAMES = {0x15003B00: 'ugv2', 0x30006E00: 'ugv3_right',
         0x2A003200: 'ugv3_left', 0x30003000: 'ugv4'}
LINKS = [(a, b) for pair in [(0x2A003200, 0x15003B00), (0x30006E00, 0x30003000),
                           (0x15003B00, 0x30003000)] for a, b in (pair, pair[::-1])]
COLORS = ['#1976d2', '#e76622', '#009688', '#9b51b6', '#c0395a', '#8a7600',
          '#455a64', '#683b20', '#4169a1', '#ac4a05', '#087e71', '#69519c']


def uid_name(uid):
    return NAMES.get(uid, '0x%08X' % uid)


def sensors_for(row):
    name = row.get('robot_id') or row.get('name', '')
    robot_number(name)
    ports = row.get('iot_ports', {})
    return [dict(name=key, port=ports.get(key, '/dev/' + key),
                 uid=UIDS.get(name, (0,))[index])
            for index, key in enumerate(['uwb_iot', 'uwb_iot_aux'] if name == 'ugv3' else ['uwb_iot'])]


def validate_ports(linktrack, sensors):
    ports = [linktrack] + [s['port'] for s in sensors]
    if any(not p.startswith('/dev/') or any(c in p for c in '\n\r\x00') for p in ports):
        raise ValueError('各设备串口必须填写 /dev/ 开头的设备路径')
    if len(set(ports)) != len(ports):
        raise ValueError('LinkTrack 和每块 IOT 需要使用不同串口')
    if linktrack in ('/dev/uwb_iot', '/dev/uwb_iot_aux'):
        raise ValueError('编队定位串口应选择 uwb_linktrack')


class IotSession:
    """One SSH worker per vehicle. UI events and recording never use the control bridge."""
    def __init__(self, workbench, ip, row, options, events, directory=None, ports=()):
        self.workbench, self.ip, self.row, self.options = workbench, ip, row, options
        self.events, self.directory, self.ports = events, directory, ports
        self.stop = threading.Event()
        self.ui_heartbeat = time.monotonic()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        client = channel = recording = None
        buffer, recent, last = b'', '', time.monotonic()
        failed = False

        def receive(chunk):
            nonlocal buffer, recent, last
            buffer += chunk
            if len(buffer) > 1048576:
                raise RuntimeError('IOT 数据过长')
            while b'\n' in buffer:
                line, buffer = buffer.split(b'\n', 1)
                try:
                    data = json.loads(line.decode())
                except ValueError:
                    recent = line.decode(errors='replace')[-500:]
                    continue
                data['host_received'] = time.time()
                data['host_monotonic'] = time.monotonic()
                if recording:
                    recording.write(json.dumps(data, ensure_ascii=False) + '\n')
                    recording.flush()
                if data.get('event') == 'error':
                    raise RuntimeError(data['message'])
                self.events.put((self.ip, data))
                last = time.monotonic()

        try:
            client = self.workbench.connect(self.ip, self.options)
            if self.stop.is_set():
                return
            with client.open_sftp() as sftp:
                home = sftp.normalize('.')
            stage = home + '/.cache/formation-console/iot-' + uuid.uuid4().hex
            run_remote(client, 'mkdir -p ' + shlex.quote(stage), self.stop, timeout=10)
            with client.open_sftp() as sftp:
                for name in ('fleet_iot_remote.py', 'fleet_bridge.py'):
                    sftp.put(str(Path(__file__).with_name(name)), stage + '/' + name)
                sftp.put(str(Path(__file__).resolve().parents[1] / 'src/nlink_parser/launch/iot_monitor.launch'),
                         stage + '/iot_monitor.launch')
            command = 'set -e; source %s; exec python -u %s ' % (
                shell_path(self.options['workspace'].rstrip('/') + '/scripts/env.sh'),
                shlex.quote(stage + '/fleet_iot_remote.py'))
            if self.directory is None:
                command += 'probe --ports ' + shlex.quote(json.dumps(list(self.ports)))
            else:
                robot = self.row['robot_id']
                sensors = sensors_for(self.row)
                validate_ports(self.row.get('formation_config', {}).get('UWB_PORT', '/dev/uwb_linktrack'), sensors)
                workspace = self.options['workspace'].rstrip('/')
                if workspace.startswith('~/'):
                    workspace = home + workspace[1:]
                destination = workspace + '/logs/uwb_iot/' + self.directory.name
                command += 'monitor --robot %s --sensors %s --directory %s --launch %s' % (
                    shlex.quote(robot), shlex.quote(json.dumps(sensors)), shlex.quote(destination),
                    shlex.quote(stage + '/iot_monitor.launch'))
                self.directory.mkdir(parents=True, exist_ok=True)
                recording = (self.directory / (robot + '-' + self.ip + '.jsonl')).open('w')
            if self.stop.is_set():
                return
            channel = client.get_transport().open_session(timeout=5)
            channel.set_combine_stderr(True)
            channel.settimeout(0.3)
            channel.exec_command('bash -c ' + shlex.quote(command))
            sent = 0.0
            last = time.monotonic()
            while not self.stop.is_set():
                now = time.monotonic()
                if now - self.ui_heartbeat > 5:
                    raise RuntimeError('界面心跳超时')
                if channel.recv_ready():
                    receive(channel.recv(65536))
                if channel.exit_status_ready() and not channel.recv_ready():
                    if channel.recv_exit_status():
                        raise RuntimeError('IOT 进程退出：' + recent)
                    break
                if now - last > 40:
                    raise RuntimeError('IOT 数据接收超时')
                if self.directory is not None and now - sent >= 0.5:
                    channel.sendall(b'{}\n')
                    sent = now
                self.stop.wait(0.02)
        except Exception as error:
            failed = True
            if not self.stop.is_set():
                self.events.put((self.ip, dict(event='error', message=str(error))))
        finally:
            if channel:
                try:
                    if not channel.exit_status_ready():
                        channel.sendall(b'{"quit":true}\n')
                        channel.shutdown_write()
                    deadline = time.monotonic() + 12
                    while time.monotonic() < deadline:
                        if channel.recv_ready():
                            receive(channel.recv(65536))
                        elif channel.exit_status_ready():
                            if channel.recv_exit_status():
                                raise RuntimeError('IOT 进程退出异常：' + recent)
                            break
                        time.sleep(0.05)
                    else:
                        raise RuntimeError('IOT 停止确认超时，请检查车端 bag 和日志')
                except Exception as error:
                    if not failed:
                        self.events.put((self.ip, dict(event='error', message=str(error))))
                channel.close()
            if recording:
                recording.close()
            if client:
                client.close()
            self.events.put((self.ip, dict(event='closed')))


def edit_ports(workbench):
    selected = workbench.vehicles.selection()
    if len(selected) != 1:
        workbench.status.set('请选择一辆车编辑串口和偏移')
        return
    tk, ttk, app = workbench.tk, workbench.ttk, workbench.app
    ip = selected[0]
    row = app.robots[ip]
    sensors = sensors_for(row)
    dialog = tk.Toplevel(workbench.root)
    dialog.title(ip + ' · UWB 设备与编队偏移')
    dialog.geometry('950x580')
    dialog.columnconfigure(1, weight=1)
    fields, controls = {}, {}
    definitions = [('UWB_PORT', 'uwb_linktrack · 编队定位',
                    row.get('formation_config', {}).get('UWB_PORT', row.get('uwb_port') or '/dev/uwb_linktrack'))]
    definitions += [(s['name'], 'uwb_iot · ' + (uid_name(s['uid']) if s['uid'] else row['robot_id']) +
                     (' · 0x%08X' % s['uid'] if s['uid'] else ''), s['port']) for s in sensors]
    definitions += [(key, label, row.get('formation_config', {}).get(key, fallback)) for key, label, fallback in
                    [('UGV_OFFSET_X', '跟随 X 偏移（米）', row.get('offset_x', '-0.8')),
                     ('UGV_OFFSET_Y', '跟随 Y 偏移（米）', row.get('offset_y', '0.8'))]]
    for index, (key, label, value) in enumerate(definitions):
        ttk.Label(dialog, text=label).grid(row=index, column=0, sticky='w', padx=12, pady=7)
        fields[key] = tk.StringVar(value=value)
        controls[key] = ttk.Combobox(dialog, textvariable=fields[key], width=42) if 'OFFSET' not in key else ttk.Entry(
            dialog, textvariable=fields[key], width=42)
        controls[key].grid(row=index, column=1, sticky='ew', padx=12)
    bar = ttk.Frame(dialog)
    bar.grid(row=5, column=0, columnspan=2, sticky='ew', padx=12, pady=8)
    status = tk.StringVar(value='点击识别，读取空闲串口协议并填写匹配路径；保存前可手动调整。')
    ttk.Label(dialog, textvariable=status, wraplength=900).grid(row=6, column=0, columnspan=2, sticky='w', padx=12)
    table = ttk.Treeview(dialog, columns=('kind', 'port', 'uid', 'aliases'), show='headings', height=6)
    for key, label, width in [('kind', '识别结果', 140), ('port', '当前串口', 180),
                              ('uid', 'IOT UID', 110), ('aliases', '固定路径', 450)]:
        table.heading(key, text=label)
        table.column(key, width=width)
    table.grid(row=7, column=0, columnspan=2, sticky='nsew', padx=12, pady=8)
    dialog.rowconfigure(7, weight=1)
    events, sessions = queue.Queue(), []
    timer = None
    workbench.iot_probes = getattr(workbench, 'iot_probes', [])

    def scan():
        if any(s.thread.is_alive() for s in sessions):
            return
        status.set('正在通过 SSH 识别 UWB 设备…')
        session = IotSession(workbench, ip, row, app.current_options(), events,
                             ports=[fields[key].get() for key in fields if 'OFFSET' not in key])
        sessions.append(session)
        workbench.iot_probes.append(session)

    def poll():
        nonlocal timer
        if not dialog.winfo_exists():
            return
        for session in sessions:
            session.ui_heartbeat = time.monotonic()
        while not events.empty():
            _, data = events.get_nowait()
            if data['event'] == 'error':
                status.set('识别失败：' + data['message'])
            elif data['event'] == 'devices':
                table.delete(*table.get_children())
                candidates = [d['port'] for d in data['devices']]
                candidates += [p for d in data['devices'] for p in d['aliases']]
                for key in fields:
                    if 'OFFSET' not in key:
                        controls[key].configure(values=candidates)
                for device in data['devices']:
                    kind, uid = device['kind'], device.get('uid', 0)
                    label = {'busy': '占用中（未读协议）', 'unknown': '未识别', 'unavailable': '不可读取'}.get(kind, kind)
                    if kind == 'busy':
                        aliases = [p[5:] for p in device['aliases'] if p.startswith('/dev/uwb_')]
                        if aliases:
                            label = '/'.join(aliases) + ' · 占用（路径识别）'
                    table.insert('', 'end', values=(label, device['port'], '0x%08X' % uid if uid else '',
                                                   ', '.join(device['aliases']) or device.get('error', '')))
                    if kind == 'uwb_linktrack' or (kind == 'busy' and '/dev/uwb_linktrack' in device['aliases']):
                        fields['UWB_PORT'].set('/dev/uwb_linktrack' if '/dev/uwb_linktrack' in device['aliases'] else device['port'])
                    for sensor in sensors:
                        if kind == 'uwb_iot' and (uid == sensor['uid'] or (not sensor['uid'] and
                                sum(d['kind'] == 'uwb_iot' for d in data['devices']) == 1)):
                            alias = '/dev/' + sensor['name']
                            fields[sensor['name']].set(alias if alias in device['aliases'] else device['port'])
                status.set('识别完成。请核对设备、UID 和端口后保存；占用中的设备不会打开。')
        timer = dialog.after(100, poll)

    def save():
        try:
            for sensor in sensors:
                sensor['port'] = fields[sensor['name']].get().strip()
            validate_ports(fields['UWB_PORT'].get().strip(), sensors)
            updated = {key: fields[key].get().strip() for key in ('UWB_PORT', 'UGV_OFFSET_X', 'UGV_OFFSET_Y')}
            if not all(math.isfinite(float(updated[key])) for key in ('UGV_OFFSET_X', 'UGV_OFFSET_Y')):
                raise ValueError('偏移必须是有限数值')
            row['formation_config'] = dict(row.get('formation_config', {}), **updated)
            row['iot_ports'] = {s['name']: s['port'] for s in sensors}
            app.save()
            workbench.status.set(ip + ' 已保存：IOT 下次监视启动时使用；LinkTrack / 偏移在配置检查时应用。')
            close()
        except ValueError as error:
            app.messagebox.showerror('配置有误', str(error), parent=dialog)

    def close():
        for session in sessions:
            session.stop.set()
        if timer:
            dialog.after_cancel(timer)
        dialog.destroy()

    ttk.Button(bar, text='识别 UWB 设备', command=scan).pack(side='left')
    ttk.Button(bar, text='保存配置', command=save).pack(side='right')
    dialog.protocol('WM_DELETE_WINDOW', close)
    timer = dialog.after(100, poll)
    # Expose the same controls to GUI regression checks and support diagnostics.
    dialog.iot_fields, dialog.iot_scan, dialog.iot_save = fields, scan, save
    return dialog


class IotHistory:
    def __init__(self):
        self.links = {key: deque(maxlen=1200) for key in LINKS}
        self.latest, self.sources, self.counts = {}, {}, {}

    def add(self, frame, now):
        uid = frame['uid']
        self.sources[uid] = (now, frame['system_time'])
        nodes = {node['uid']: node for node in frame['nodes']}
        for target in nodes:
            self.links.setdefault((uid, target), deque(maxlen=1200))
        for key, points in self.links.items():
            if key[0] != uid:
                continue
            node = nodes.get(key[1])
            values = (node['distance'], node['horizontal']) if node else (None, None)
            points.append((now,) + values)
            if node:
                self.latest[key] = (now,) + values
                self.counts[key] = self.counts.get(key, 0) + 1


class IotWindow:
    def __init__(self, workbench, rows):
        self.workbench, self.app = workbench, workbench.app
        tk, ttk = workbench.tk, workbench.ttk
        self.dialog = tk.Toplevel(workbench.root)
        self.dialog.title('UWB IOT · 距离与水平角实时监视')
        self.dialog.geometry('%dx%d+40+40' % (min(1280, self.dialog.winfo_screenwidth() - 80),
                                            min(860, self.dialog.winfo_screenheight() - 100)))
        self.dialog.minsize(980, 650)
        self.history, self.events, self.sessions = IotHistory(), queue.Queue(), []
        self.states, self.sensor_counts = {}, {}
        self.devices, self.names = {}, dict(NAMES)
        self.closed = False
        self.stopped_at = None
        self.directory = Path(__file__).resolve().parents[1] / 'logs/uwb_iot' / (
            datetime.now().strftime('live_%Y%m%d_%H%M%S_') + uuid.uuid4().hex[:6])
        toolbar = ttk.Frame(self.dialog, padding=8)
        toolbar.pack(fill='x')
        ttk.Button(toolbar, text='停止采集', command=self.stop).pack(side='left')
        ttk.Label(toolbar, text='距离 m / 水平角 ° · 来源板局部坐标 · 最近 60 秒 · 红点表示超出 ±50°').pack(side='left', padx=12)
        self.status = tk.StringVar(value='正在启动所选车辆的 IOT…')
        ttk.Label(self.dialog, textvariable=self.status, wraplength=1230).pack(fill='x', padx=10)
        panes = ttk.Panedwindow(self.dialog, orient='horizontal')
        panes.pack(fill='both', expand=True, padx=10, pady=8)
        sidebar, chart = ttk.Frame(panes, padding=(0, 0, 8, 0)), ttk.Frame(panes)
        panes.add(sidebar, weight=0)
        panes.add(chart, weight=1)
        heading = ttk.Frame(sidebar)
        heading.pack(fill='x', pady=(0, 6))
        ttk.Label(heading, text='有向链路').pack(side='left')
        ttk.Button(heading, text='清空', width=5,
                   command=lambda: self.link_menu.selection_remove(self.link_menu.selection())).pack(side='right')
        ttk.Button(heading, text='全选', width=5,
                   command=lambda: self.link_menu.selection_set(self.link_menu.get_children())).pack(side='right', padx=4)
        ttk.Label(sidebar, text='单击单选 · Ctrl / Shift 多选').pack(anchor='w', pady=(0, 8))
        menu_frame = ttk.Frame(sidebar)
        menu_frame.pack(fill='both', expand=True)
        self.link_menu = ttk.Treeview(menu_frame, show='tree', selectmode='extended')
        self.link_menu.column('#0', width=285, minwidth=220, stretch=True)
        scroll = ttk.Scrollbar(menu_frame, command=self.link_menu.yview)
        self.link_menu.configure(yscrollcommand=scroll.set)
        scroll.pack(side='right', fill='y')
        self.link_menu.pack(fill='both', expand=True)
        self.swatches = []
        for color in COLORS:
            swatch = tk.PhotoImage(master=self.dialog, width=10, height=10)
            swatch.put(color, to=(1, 1, 9, 9))
            self.swatches.append(swatch)
        self.canvas = tk.Canvas(chart, background='#f7fafc', highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)
        self.link_menu.bind('<<TreeviewSelect>>', lambda _: self.paint(self.stopped_at or time.monotonic()))
        ttk.Label(self.dialog, text='本机帧记录：' + str(self.directory), wraplength=1230).pack(anchor='w', padx=10, pady=5)
        self.dialog.protocol('WM_DELETE_WINDOW', self.close)
        options = self.app.current_options()
        for ip, row in rows.items():
            self.states[ip] = '正在连接'
            self.sessions.append(IotSession(workbench, ip, row, options, self.events, self.directory))
        self.timer = self.dialog.after(100, self.poll)

    def stop(self):
        for session in self.sessions:
            session.stop.set()
        self.status.set('正在停止采集并保存 bag…')

    def close(self):
        self.stop()
        self.closed = True
        self.dialog.after_cancel(self.timer)
        self.dialog.destroy()

    def poll(self):
        if self.closed:
            return
        now = time.monotonic()
        for session in self.sessions:
            session.ui_heartbeat = now
        for unused in range(500):
            try:
                ip, data = self.events.get_nowait()
            except queue.Empty:
                break
            event = data['event']
            if event == 'frame':
                self.history.add(data, data.get('host_monotonic', now))
                uid = data['uid']
                self.sensor_counts[uid] = self.sensor_counts.get(uid, 0) + 1
            elif event == 'started':
                self.states[ip] = '采集中'
                self.devices[ip] = data['sensors']
                robot = next(s.row['robot_id'] for s in self.sessions if s.ip == ip)
                for sensor in data['sensors']:
                    self.names.setdefault(sensor['uid'], robot)
            elif event == 'error':
                self.states[ip] = '失败：' + data['message']
            elif event == 'stopped':
                self.states[ip] = '已保存'
            elif event == 'closed' and not self.states[ip].startswith(('失败', '已保存')):
                self.states[ip] = '已停止'
        self.status.set('  |  '.join(ip + ' ' + state for ip, state in self.states.items()))
        first_menu = not self.link_menu.get_children()
        for index, key in enumerate(self.history.links):
            iid = '%d:%d' % key
            label = self.names.get(key[0], uid_name(key[0])) + ' → ' + self.names.get(key[1], uid_name(key[1]))
            if self.link_menu.exists(iid):
                self.link_menu.item(iid, text=label)
            else:
                self.link_menu.insert('', 'end', iid=iid, text=label, image=self.swatches[index % len(COLORS)])
        if first_menu:
            self.link_menu.selection_set(['%d:%d' % key for key in LINKS])
        if self.stopped_at is None and not any(s.thread.is_alive() for s in self.sessions):
            self.stopped_at = now
        self.paint(self.stopped_at or now)
        self.timer = self.dialog.after(200, self.poll)

    def paint(self, now):
        canvas = self.canvas
        canvas.delete('all')
        width, height = max(canvas.winfo_width(), 300), max(canvas.winfo_height(), 200)
        selected = set(self.link_menu.selection())
        if not selected:
            canvas.create_text(width / 2, height / 2, text='请选择左侧有向链路', fill='#526574')
            return
        curves = [(key, points, COLORS[i % len(COLORS)]) for i, (key, points) in enumerate(self.history.links.items())
                  if '%d:%d' % key in selected]
        for component, title in [(1, '距离（m）'), (2, '水平角（°）')]:
            left, right = 65, width - 20
            top, bottom = (component - 1) * height / 2 + 28, component * height / 2 - 28
            values = [p[component] for _, points, _ in curves for p in points
                      if now - p[0] <= 60 and p[component] is not None and math.isfinite(p[component])]
            low, high = ((min([0] + values), max([1] + values) * 1.1) if component == 1 else
                         (min([-55] + values) - 3, max([55] + values) + 3))
            def y(value):
                return bottom - (value - low) / (high - low) * (bottom - top)
            canvas.create_text(left, top - 15, text=title, anchor='w', fill='#243746')
            for tick in range(5):
                value = low + (high - low) * tick / 4
                canvas.create_line(left, y(value), right, y(value), fill='#dce4ea')
                canvas.create_text(left - 7, y(value), text='%.1f' % value, anchor='e', fill='#526574')
            for seconds in range(0, 61, 15):
                x = right - seconds / 60 * (right - left)
                canvas.create_text(x, bottom + 13, text='-%ds' % seconds if seconds else '现在', fill='#526574')
            if component == 2:
                for limit in (-50, 50):
                    canvas.create_line(left, y(limit), right, y(limit), fill='#c94040', dash=(4, 4))
            for _, points, color in curves:
                previous = None
                for point in points:
                    stamp, value = point[0], point[component]
                    if now - stamp > 60 or value is None or not math.isfinite(value):
                        previous = None
                        continue
                    x, yy = right - (now - stamp) / 60 * (right - left), y(value)
                    if previous and stamp - previous[0] < 0.5:
                        canvas.create_line(previous[1], previous[2], x, yy, fill=color, width=1.5)
                    dot = '#d32f2f' if component == 2 and abs(value) > 50 else color
                    canvas.create_oval(x-1.5, yy-1.5, x+1.5, yy+1.5, fill=dot, outline='')
                    previous = stamp, x, yy


def download_iot_logs(client, workspace, destination):
    """Download the vehicle's recorded IOT directory and return local files."""
    destination = Path(destination)
    with client.open_sftp() as sftp:
        home = sftp.normalize('.')
        root = home + workspace[1:] if workspace.startswith('~/') else workspace
        root = root.rstrip('/') + '/logs/uwb_iot'
        try:
            sftp.stat(root)
        except IOError as error:
            raise RuntimeError('车端没有 IOT 数据目录：%s' % root) from error
        files = []

        def walk(remote, relative):
            for entry in sftp.listdir_attr(remote):
                name = entry.filename
                if name in ('.', '..') or '/' in name:
                    continue
                child = remote.rstrip('/') + '/' + name
                rel = relative / name
                if stat.S_ISDIR(entry.st_mode):
                    walk(child, rel)
                elif stat.S_ISREG(entry.st_mode):
                    local = destination / rel
                    local.parent.mkdir(parents=True, exist_ok=True)
                    sftp.get(child, str(local))
                    files.append(local)

        walk(root, Path())
    return files


class IotExtractPage:
    """Vehicle list for downloading recorded IOT data from each workspace."""
    def __init__(self, workbench, page):
        self.workbench, self.app, self.tk, self.ttk = workbench, workbench.app, workbench.tk, workbench.ttk
        self.page = page
        self.events, self.files, self.states = queue.Queue(), {}, {}
        self.destination = Path(__file__).resolve().parents[1] / 'logs/uwb_iot' / (
            'extracted_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
        self.ttk.Label(page, text='从车端工作空间 logs/uwb_iot 提取已保存的 IOT 数据；可按车查看并下载到本机。',
                       wraplength=1080).pack(anchor='w')
        toolbar = self.ttk.Frame(page)
        toolbar.pack(fill='x', pady=6)
        self.ttk.Button(toolbar, text='一键提取全部车辆', command=self.extract_all).pack(side='left')
        self.ttk.Button(toolbar, text='提取选中车辆', command=self.extract_selected).pack(side='left', padx=6)
        self.ttk.Button(toolbar, text='查看选中车辆', command=self.show_selected).pack(side='left')
        self.ttk.Button(toolbar, text='下载选中文件', command=self.download_selected).pack(side='left', padx=6)
        self.status = self.tk.StringVar(value='请选择车辆，或点击“一键提取全部车辆”。')
        self.ttk.Label(page, textvariable=self.status, wraplength=1080).pack(fill='x')
        self.ttk.Label(page, text='本机保存目录：' + str(self.destination), wraplength=1080).pack(anchor='w')
        panes = self.ttk.Panedwindow(page, orient='horizontal')
        panes.pack(fill='both', expand=True, pady=8)
        left, right = self.ttk.Frame(panes), self.ttk.Frame(panes)
        panes.add(left, weight=1)
        panes.add(right, weight=2)
        self.vehicles = self.ttk.Treeview(left, columns=('ip', 'name', 'state'), show='headings',
                                          selectmode='extended')
        for key, title, width in [('ip', 'IP', 145), ('name', '车辆', 80), ('state', '状态', 150)]:
            self.vehicles.heading(key, text=title)
            self.vehicles.column(key, width=width, minwidth=60, stretch=key == 'state')
        self.vehicles.pack(fill='both', expand=True)
        self.vehicles.bind('<<TreeviewSelect>>', lambda _: self.show_selected())
        self.files_view = self.ttk.Treeview(right, columns=('path', 'size'), show='headings', selectmode='browse')
        self.files_view.heading('path', text='本地文件')
        self.files_view.heading('size', text='大小')
        self.files_view.column('path', width=520, stretch=True)
        self.files_view.column('size', width=100, stretch=False)
        self.files_view.pack(fill='both', expand=True)
        self.files_view.bind('<Double-1>', lambda _: self.view_selected_file())
        self._refresh_vehicles()
        self.timer = None

    def close(self):
        self.timer = None

    def _refresh_vehicles(self):
        known = {ip for ip, row in self.app.robots.items() if row.get('robot_id')}
        for ip in self.vehicles.get_children():
            if ip not in known:
                self.vehicles.delete(ip)
        for ip, row in self.app.robots.items():
            if not row.get('robot_id'):
                continue
            self.states.setdefault(ip, '未提取')
            values = (ip, row.get('robot_id', ''), self.states[ip])
            if self.vehicles.exists(ip):
                self.vehicles.item(ip, values=values)
            else:
                self.vehicles.insert('', 'end', iid=ip, values=values)

    def _selected(self):
        return [ip for ip in self.vehicles.selection() if ip in self.app.robots]

    def extract_all(self):
        self.extract(list(self.vehicles.get_children()))

    def extract_selected(self):
        selected = self._selected()
        if not selected:
            self.status.set('请先选择车辆')
            return
        self.extract(selected)

    def extract(self, addresses):
        options = self.app.current_options()
        self.destination.mkdir(parents=True, exist_ok=True)
        for ip in addresses:
            if self.states.get(ip) == '提取中':
                continue
            self.states[ip] = '提取中'
            threading.Thread(target=self._extract_one, args=(ip, options), daemon=True).start()
        self._refresh_vehicles()
        self.status.set('正在提取 %d 辆车的数据；完成后可在右侧查看文件。' % len(addresses))

    def _extract_one(self, ip, options):
        client = None
        try:
            client = self.workbench.connect(ip, options)
            robot = self.app.robots[ip].get('robot_id') or ip
            files = download_iot_logs(client, options['workspace'], self.destination / robot)
            self.events.put((ip, 'done', files))
        except Exception as error:
            self.events.put((ip, 'error', str(error)))
        finally:
            if client:
                client.close()

    def show_selected(self):
        selected = self._selected()
        if len(selected) != 1:
            return
        self.files_view.delete(*self.files_view.get_children())
        for index, path in enumerate(self.files.get(selected[0], ())):
            path = Path(path)
            try:
                size = '%d KB' % max(1, path.stat().st_size // 1024)
            except OSError:
                size = '—'
            self.files_view.insert('', 'end', iid=str(index), values=(str(path), size))

    def selected_file(self):
        selected = self.files_view.selection()
        if not selected:
            return None
        value = self.files_view.item(selected[0], 'values')
        return Path(value[0]) if value else None

    def view_selected_file(self):
        path = self.selected_file()
        if not path:
            return
        try:
            text = path.read_text(encoding='utf-8', errors='replace')[:500000]
        except OSError as error:
            self.status.set('读取文件失败：' + str(error))
            return
        dialog = self.tk.Toplevel(self.workbench.root)
        dialog.title('IOT 数据 · ' + path.name)
        editor = self.tk.Text(dialog, wrap='none', padx=10, pady=10)
        editor.pack(fill='both', expand=True)
        editor.insert('1.0', text)
        editor.configure(state='disabled')
        self.ttk.Button(dialog, text='关闭', command=dialog.destroy).pack(pady=6)

    def download_selected(self):
        path = self.selected_file()
        if not path:
            self.status.set('请先在右侧选择文件')
            return
        from tkinter import filedialog
        target = filedialog.asksaveasfilename(parent=self.workbench.root, initialfile=path.name,
                                              title='下载 IOT 文件')
        if target:
            try:
                shutil.copy2(path, target)
                self.status.set('已下载：' + target)
            except OSError as error:
                self.status.set('下载失败：' + str(error))

    def poll(self):
        self._refresh_vehicles()
        changed = False
        while True:
            try:
                ip, event, value = self.events.get_nowait()
            except queue.Empty:
                break
            if event == 'done':
                self.files[ip] = value
                self.states[ip] = '已提取 %d 个文件' % len(value)
            else:
                self.states[ip] = '失败：' + value
            changed = True
            self._refresh_vehicles()
            self.show_selected()
        if changed and self.states and not any(value == '提取中' for value in self.states.values()):
            self.status.set('提取完成；请选择车辆查看文件，双击文件可查看内容。')


# ROS topic labels are intentionally suffix based: every vehicle publishes the
# same topic set under /ugvN, while /tf and /rosout stay global.
ROS_TOPIC_LABELS = {
    '/rosout': 'ROS 日志', '/rosout_agg': 'ROS 聚合日志', '/tf': '坐标变换',
    '/tf_static': '静态坐标变换', 'PowerVoltage': '电源电压', 'cmd_vel': '速度指令',
    'imu': 'IMU 惯性数据', 'joint_states': '关节状态',
    'nlink_linktrack_data_transmission': 'LinkTrack 数据传输',
    'nlink_linktrack_nodeframe2': 'LinkTrack 节点帧', 'odom': '里程计',
    'odom_combined': '融合里程计', 'uwb/diagnostics': 'UWB 诊断',
    'uwb/dropped_frame_count': 'UWB 丢帧数', 'uwb/full_residual_rms': 'UWB 总残差 RMS',
    'uwb/jump_rejected_ids': 'UWB 跳变剔除 ID', 'uwb/loo_excluded_id': 'UWB 留一剔除 ID',
    'uwb/measurement_age': 'UWB 测量年龄', 'uwb/path': 'UWB 轨迹',
    'uwb/point': 'UWB 点位', 'uwb/pose': 'UWB 位姿',
    'uwb/pose_covariance': 'UWB 位姿协方差', 'uwb/processing_time': 'UWB 处理耗时',
    'uwb/residual_rms': 'UWB 残差 RMS', 'uwb/status': 'UWB 定位状态',
    'uwb/used_anchor_count': 'UWB 使用基站数', 'uwb/valid': 'UWB 有效标志',
}
ROS_FIELD_LABELS = {
    'header': '消息头', 'stamp': '时间戳', 'frame_id': '坐标系', 'seq': '序号',
    'position': '位置', 'orientation': '方向', 'linear': '线速度', 'angular': '角速度',
    'velocity': '速度', 'twist': '速度变化', 'pose': '位姿', 'covariance': '协方差',
    'x': 'X', 'y': 'Y', 'z': 'Z', 'w': 'W', 'roll': '横滚角', 'pitch': '俯仰角',
    'yaw': '偏航角', 'name': '关节名称', 'position': '位置', 'effort': '力矩 / 努力',
    'status': '状态', 'valid': '有效', 'value': '数值', 'data': '数据',
}


def topic_label(topic):
    """Return a compact Chinese label while keeping the full topic visible."""
    matches = [(key, label) for key, label in ROS_TOPIC_LABELS.items()
               if topic == key or topic.endswith('/' + key.lstrip('/'))]
    if matches:
        return max(matches, key=lambda item: len(item[0]))[1]
    return topic.rsplit('/', 1)[-1]


def parse_topic_listing(text):
    """Parse ``topic<TAB>type`` output, tolerating plain ``rostopic list``."""
    result, seen = [], set()
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith('WARNING:') or line.startswith('__'):
            continue
        if '\t' in line:
            topic, msg_type = line.split('\t', 1)
        else:
            topic, msg_type = line, ''
        topic, msg_type = topic.strip(), msg_type.strip()
        if not topic.startswith('/') or topic in seen:
            continue
        seen.add(topic)
        result.append({'topic': topic, 'type': msg_type, 'label': topic_label(topic)})
    return sorted(result, key=lambda item: item['topic'])


def parse_topic_query(text):
    """Extract type and one YAML message from the marked remote command output."""
    msg_type, payload = '', []
    for line in text.splitlines():
        if line.startswith('__ROS_TYPE__'):
            msg_type = line[len('__ROS_TYPE__'):].strip()
        elif line.startswith('__ROS_BEGIN__'):
            continue
        else:
            payload.append(line)
    raw = '\n'.join(payload).strip()
    if not raw:
        return msg_type, None, ''
    try:
        value = yaml.safe_load(raw)
    except yaml.YAMLError:
        return msg_type, None, raw
    return msg_type, value, raw


def _friendly_scalar(value):
    if isinstance(value, bool):
        return '是' if value else '否'
    if isinstance(value, float):
        if not math.isfinite(value):
            return str(value)
        return '%.6g' % value
    if value is None:
        return '—'
    return str(value)


def flatten_ros_message(value, prefix='', limit=120):
    """Flatten a ROS YAML message into (friendly path, value) rows."""
    rows = []

    def walk(node, path):
        if len(rows) >= limit:
            return
        if isinstance(node, dict):
            for key, child in node.items():
                label = ROS_FIELD_LABELS.get(str(key), str(key))
                walk(child, (path + ' · ' if path else '') + label)
        elif isinstance(node, (list, tuple)):
            for index, child in enumerate(node[:40]):
                walk(child, '%s [%d]' % (path or '数据', index))
            if len(node) > 40:
                rows.append((path or '数据', '…（其余 %d 项已省略）' % (len(node) - 40)))
        else:
            rows.append((path or '数据', _friendly_scalar(node)))

    walk(value, prefix)
    return rows


def _ros_topic_command(options, ip, command):
    master = str(ipaddress.IPv4Address(options.get('master_ip', ip)))
    address = str(ipaddress.IPv4Address(ip))
    workspace = options.get('workspace', '~/formation_ws').rstrip('/')
    return ('set -e; source %s; export ROS_MASTER_URI=%s ROS_IP=%s; '
            'unset ROS_HOSTNAME ROS_NAMESPACE; %s' %
            (shell_path(workspace + '/scripts/env.sh'), shlex.quote('http://%s:11311' % master),
             shlex.quote(address), command))


class RosTopicPage:
    """Inspect ROS topics on the selected vehicles through their existing SSH setup."""
    def __init__(self, workbench, page):
        self.workbench, self.app, self.tk, self.ttk = workbench, workbench.app, workbench.tk, workbench.ttk
        self.page = page
        self.events, self.busy, self.requests = queue.Queue(), set(), []
        self.topic_cache, self.states = {}, {}
        self.current_ip = None
        self.closed = False

        self.ttk.Label(page, text='通过 SSH 连接车辆的 ROS Master，查看话题列表、消息类型和最新一条数据；全局话题（/tf、/rosout）也会列出。',
                       wraplength=1080).pack(anchor='w')
        toolbar = self.ttk.Frame(page)
        toolbar.pack(fill='x', pady=6)
        self.ttk.Button(toolbar, text='刷新全部状态', command=self.refresh_all).pack(side='left')
        self.ttk.Button(toolbar, text='刷新选中车辆话题', command=self.refresh_selected).pack(side='left', padx=6)
        self.ttk.Button(toolbar, text='查询选中话题', command=self.query_selected).pack(side='left')
        self.ttk.Button(toolbar, text='清空消息', command=self.clear_message).pack(side='left', padx=6)
        self.ttk.Label(toolbar, text='筛选').pack(side='left', padx=(18, 4))
        self.filter_var = self.tk.StringVar()
        self.filter_var.trace_add('write', lambda *_: self._show_topics())
        self.ttk.Entry(toolbar, textvariable=self.filter_var, width=28).pack(side='left')
        self.status = self.tk.StringVar(value='请选择车辆并刷新话题。')
        self.ttk.Label(page, textvariable=self.status, wraplength=1080).pack(fill='x')

        panes = self.ttk.Panedwindow(page, orient='horizontal')
        panes.pack(fill='both', expand=True, pady=8)
        left, right = self.ttk.Frame(panes, padding=(0, 0, 8, 0)), self.ttk.Frame(panes)
        panes.add(left, weight=1)
        panes.add(right, weight=2)
        self.ttk.Label(left, text='车辆 ROS 状态').pack(anchor='w', pady=(0, 5))
        self.vehicles = self.ttk.Treeview(left, columns=('ip', 'name', 'state', 'count'), show='headings', selectmode='browse')
        for key, title, width in [('ip', 'IP', 130), ('name', '车辆', 68), ('state', 'ROS 状态', 150), ('count', '话题数', 65)]:
            self.vehicles.heading(key, text=title)
            self.vehicles.column(key, width=width, minwidth=55, stretch=key == 'state')
        self.vehicles.pack(fill='both', expand=True)
        self.vehicles.bind('<<TreeviewSelect>>', self._vehicle_selected)
        self.ttk.Label(right, text='话题（双击或点击“查询选中话题”读取最新消息）').pack(anchor='w', pady=(0, 5))
        self.topics = self.ttk.Treeview(right, columns=('topic', 'label', 'type', 'state'), show='headings', selectmode='browse')
        for key, title, width in [('topic', '话题', 300), ('label', '友好名称', 150), ('type', '消息类型', 180), ('state', '数据状态', 95)]:
            self.topics.heading(key, text=title)
            self.topics.column(key, width=width, minwidth=70, stretch=key in ('topic', 'label'))
        topic_scroll = self.ttk.Scrollbar(right, orient='vertical', command=self.topics.yview)
        self.topics.configure(yscrollcommand=topic_scroll.set)
        topic_scroll.pack(side='right', fill='y')
        self.topics.pack(fill='both', expand=True)
        self.topics.bind('<Double-1>', lambda _: self.query_selected())
        self.ttk.Label(right, text='最新消息（字段已翻译；列表过长时自动折叠数组）').pack(anchor='w', pady=(8, 3))
        message_frame = self.ttk.Frame(right)
        message_frame.pack(fill='both', expand=True)
        self.message = self.tk.Text(message_frame, height=10, wrap='none', state='disabled', font=('Monospace', 10))
        message_scroll = self.ttk.Scrollbar(message_frame, orient='vertical', command=self.message.yview)
        self.message.configure(yscrollcommand=message_scroll.set)
        message_scroll.pack(side='right', fill='y')
        self.message.pack(fill='both', expand=True)
        self._refresh_vehicles()

    def _refresh_vehicles(self):
        known = {ip for ip, row in self.app.robots.items() if row.get('robot_id')}
        for ip in self.vehicles.get_children():
            if ip not in known:
                self.vehicles.delete(ip)
        for ip, row in self.app.robots.items():
            if not row.get('robot_id'):
                continue
            self.states.setdefault(ip, '未查询')
            values = (ip, row.get('robot_id', ''), self.states[ip], len(self.topic_cache.get(ip, ())))
            if self.vehicles.exists(ip):
                self.vehicles.item(ip, values=values)
            else:
                self.vehicles.insert('', 'end', iid=ip, values=values)
        if self.current_ip not in known:
            self.current_ip = next(iter(sorted(known, key=ipaddress.IPv4Address)), None)
            if self.current_ip and self.vehicles.exists(self.current_ip):
                self.vehicles.selection_set(self.current_ip)

    def _vehicle_selected(self, _=None):
        selected = self.vehicles.selection()
        if not selected:
            return
        self.current_ip = selected[0]
        self._show_topics()

    def _selected_ip(self):
        selected = self.vehicles.selection()
        return selected[0] if selected else self.current_ip

    def _show_topics(self):
        self.topics.delete(*self.topics.get_children())
        topics = self.topic_cache.get(self.current_ip, ())
        needle = self.filter_var.get().strip().lower()
        for item in topics:
            if needle and needle not in (item['topic'] + ' ' + item['label'] + ' ' + item.get('type', '')).lower():
                continue
            state = item.get('state', '可查询')
            self.topics.insert('', 'end', iid=item['topic'], values=(item['topic'], item['label'], item.get('type', ''), state))

    def _request(self, ip, action, topic=None):
        key = (ip, action, topic or '')
        if not ip or key in self.busy:
            return
        options = self.app.current_options()
        stop = threading.Event()
        self.busy.add(key)
        self.requests.append(stop)

        def run():
            client = None
            try:
                client = self.workbench.connect(ip, options)
                if action == 'topics':
                    # ``rostopic type`` performs one master lookup per topic and
                    # becomes very slow on a full fleet.  List first; resolve a
                    # type only when the user asks for a specific message.
                    output = run_remote(client, _ros_topic_command(options, ip, 'timeout 7 rostopic list'),
                                        stop, timeout=10)
                    self.events.put((ip, action, parse_topic_listing(output)))
                else:
                    quoted = shlex.quote(topic)
                    command = ('type=$(rostopic type %s); printf "__ROS_TYPE__%%s\\n" "$type"; '
                               'printf "__ROS_BEGIN__\\n"; timeout 5 rostopic echo -n 1 %s || test $? -eq 124' % (quoted, quoted))
                    output = run_remote(client, _ros_topic_command(options, ip, command), stop, timeout=10)
                    msg_type, value, raw = parse_topic_query(output)
                    self.events.put((ip, action, dict(topic=topic, type=msg_type, value=value, raw=raw)))
            except Exception as error:
                self.events.put((ip, 'error', str(error)))
            finally:
                if client:
                    client.close()
                self.events.put((ip, 'finished', key))

        threading.Thread(target=run, daemon=True).start()

    def refresh_all(self):
        addresses = [ip for ip, row in self.app.robots.items() if row.get('robot_id')]
        if not addresses:
            self.status.set('暂无已识别车辆，请先扫描并连接。')
            return
        for ip in addresses:
            self.states[ip] = '正在查询'
            self._request(ip, 'topics')
        self._refresh_vehicles()
        self.status.set('正在查询 %d 辆车的 ROS Master 和话题列表…' % len(addresses))

    def refresh_selected(self):
        ip = self._selected_ip()
        if not ip:
            self.status.set('请先选择车辆')
            return
        self.states[ip] = '正在查询'
        self._request(ip, 'topics')
        self._refresh_vehicles()

    def query_selected(self):
        ip = self._selected_ip()
        selected = self.topics.selection()
        if not ip or not selected:
            self.status.set('请先选择车辆和话题')
            return
        topic = selected[0]
        self.status.set('%s 正在读取 %s 的最新消息…' % (ip, topic))
        self._request(ip, 'message', topic)

    def clear_message(self):
        self.message.configure(state='normal')
        self.message.delete('1.0', 'end')
        self.message.configure(state='disabled')

    def _show_message(self, ip, data):
        self.message.configure(state='normal')
        self.message.delete('1.0', 'end')
        self.message.insert('end', '车辆：%s\n话题：%s · %s\n消息类型：%s\n读取时间：%s\n\n' %
                            (ip, data['topic'], topic_label(data['topic']), data.get('type') or '未知',
                             datetime.now().strftime('%Y-%m-%d %H:%M:%S')))
        if data.get('value') is None:
            self.message.insert('end', '当前没有收到消息（话题可能未发布或数据暂时过期）。\n')
            if data.get('raw'):
                self.message.insert('end', '\n原始输出：\n' + data['raw'][:10000])
        else:
            rows = flatten_ros_message(data['value'])
            if rows:
                width = max(len(key) for key, _ in rows)
                self.message.insert('end', '\n'.join(('%-*s : %s' % (width, key, value)) for key, value in rows))
            else:
                self.message.insert('end', _friendly_scalar(data['value']))
        self.message.configure(state='disabled')
        for item in self.topic_cache.get(ip, ()):
            if item['topic'] == data['topic']:
                if data.get('type'):
                    item['type'] = data['type']
                item['state'] = '已获取'
        if ip == self.current_ip:
            self._show_topics()

    def poll(self):
        if self.closed:
            return
        self._refresh_vehicles()
        changed = False
        while True:
            try:
                ip, event, value = self.events.get_nowait()
            except queue.Empty:
                break
            if event == 'topics':
                self.topic_cache[ip] = value
                self.states[ip] = '在线'
                changed = True
            elif event == 'message':
                self._show_message(ip, value)
                self.status.set('%s · 已读取 %s' % (ip, value['topic']))
            elif event == 'error':
                self.states[ip] = '离线：' + value
                self.status.set('%s ROS 查询失败：%s' % (ip, value))
                changed = True
            elif event == 'finished':
                self.busy.discard(value)
        if changed:
            self._refresh_vehicles()
            self._show_topics()

    def close(self):
        self.closed = True
        for stop in self.requests:
            stop.set()
