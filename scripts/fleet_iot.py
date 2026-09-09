"""UWB port editor and independent IOT charts for the Tk fleet console."""
from collections import deque
from datetime import datetime
import json
import math
from pathlib import Path
import queue
import shlex
import threading
import time
import uuid

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
        ttk.Button(toolbar, text='显示全部链路', command=lambda: self.table.selection_remove(self.table.selection())).pack(side='left', padx=5)
        ttk.Label(toolbar, text='距离 m / 水平角 ° · 来源板局部坐标 · 最近 60 秒 · 红点表示超出 ±50°').pack(side='left', padx=12)
        self.status = tk.StringVar(value='正在启动所选车辆的 IOT…')
        ttk.Label(self.dialog, textvariable=self.status, wraplength=1230).pack(fill='x', padx=10)
        self.device_status = tk.StringVar()
        ttk.Label(self.dialog, textvariable=self.device_status, wraplength=1230).pack(fill='x', padx=10, pady=5)
        table_frame = ttk.Frame(self.dialog)
        table_frame.pack(fill='x', padx=10)
        self.table = ttk.Treeview(table_frame, columns=('link', 'distance', 'angle', 'count', 'state'),
                                 show='headings', height=7, selectmode='extended')
        for key, label, width in [('link', '有向链路（可多选曲线）', 340), ('distance', '距离 m', 150),
                                  ('angle', '水平角 °', 150), ('count', '记录数', 100), ('state', '数据状态', 210)]:
            self.table.heading(key, text=label)
            self.table.column(key, width=width)
        scroll = ttk.Scrollbar(table_frame, command=self.table.yview)
        self.table.configure(yscrollcommand=scroll.set)
        scroll.pack(side='right', fill='y')
        self.table.pack(fill='x')
        ttk.Label(self.dialog, text='未选择行时显示全部链路；缺测留空，未收到其他车数据时保持等待。').pack(anchor='w', padx=10)
        self.canvas = tk.Canvas(self.dialog, background='#f7fafc', highlightthickness=0)
        self.canvas.pack(fill='both', expand=True, padx=10, pady=6)
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
                self.states[ip] = '采集中 · bag: ' + data['directory']
                self.devices[ip] = data['sensors']
                robot = next(s.row['robot_id'] for s in self.sessions if s.ip == ip)
                for sensor in data['sensors']:
                    self.names.setdefault(sensor['uid'], robot)
            elif event == 'error':
                self.states[ip] = '失败：' + data['message']
            elif event == 'stopped':
                self.states[ip] = '已保存 bag · ' + data['directory']
            elif event == 'closed' and not self.states[ip].startswith(('失败', '已保存')):
                self.states[ip] = '已停止'
        self.status.set('  |  '.join(ip + ' ' + state for ip, state in self.states.items()))
        devices = []
        for session in self.sessions:
            for sensor in self.devices.get(session.ip, sensors_for(session.row)):
                uid = sensor['uid']
                count = self.sensor_counts.get(uid, 0)
                stamp = self.history.sources.get(uid, (-1e9, 0))[0]
                devices.append('%s · 0x%08X · %d 帧 · %s' % (self.names.get(uid, session.row['robot_id']), uid, count,
                               '在线' if now - stamp < 1 else '等待数据 / 已停止'))
        self.device_status.set('    '.join(devices))
        for index, (key, points) in enumerate(self.history.links.items()):
            iid = '%d:%d' % key
            color = COLORS[index % len(COLORS)]
            self.table.tag_configure(iid, foreground=color)
            latest = self.history.latest.get(key)
            current = latest and now - latest[0] < 1 and points and points[-1][1] is not None
            label = '等待数据' if latest is None else '缺测 / 已过期'
            if current:
                label = '超出 ±50°' if latest[2] is not None and abs(latest[2]) > 50 else '实时'
            values = (self.names.get(key[0], uid_name(key[0])) + ' → ' + self.names.get(key[1], uid_name(key[1])),
                      '%.3f' % latest[1] if current and latest[1] is not None else '—',
                      '%.2f' % latest[2] if current and latest[2] is not None else '—',
                      self.history.counts.get(key, 0), label)
            if self.table.exists(iid):
                self.table.item(iid, values=values)
            else:
                self.table.insert('', 'end', iid=iid, values=values, tags=(iid,))
        if self.stopped_at is None and not any(s.thread.is_alive() for s in self.sessions):
            self.stopped_at = now
        self.paint(self.stopped_at or now)
        self.timer = self.dialog.after(200, self.poll)

    def paint(self, now):
        canvas = self.canvas
        canvas.delete('all')
        width, height = max(canvas.winfo_width(), 600), max(canvas.winfo_height(), 200)
        selected = set(self.table.selection())
        curves = [(key, points, COLORS[i % len(COLORS)]) for i, (key, points) in enumerate(self.history.links.items())
                  if not selected or '%d:%d' % key in selected]
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
