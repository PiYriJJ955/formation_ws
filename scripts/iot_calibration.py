#!/usr/bin/env python3
"""Standalone, guided UWB IOT angle calibration for the fleet."""
import argparse
import csv
from datetime import datetime
import json
import math
import os
from pathlib import Path
import queue
import shutil
import tempfile
import threading
import time
import uuid

from fleet_console import CONFIG_PATH, connect_ssh, load_settings
from fleet_iot import IotSession, sensors_for, uid_name


ROOT = Path(__file__).resolve().parents[1]
DATA_ROOT = ROOT / 'logs' / 'iot_calibration'
ANGLES = tuple(range(-60, 61, 5))
CAPTURE_SECONDS = 60.0
OBSERVATION_FIELDS = (
    'capture_id', 'actual_angle_deg', 'source_uid', 'target_uid',
    'host_received', 'host_monotonic', 'elapsed_s', 'system_time',
    'target_present', 'valid_angle', 'distance_m', 'measured_horizontal_deg',
    'fp_rssi', 'rx_rssi',
)
POINT_FIELDS = (
    'actual_angle_deg', 'measured_mean_deg', 'measured_std_deg',
    'distance_mean_m', 'distance_std_m', 'source_frames', 'target_present_frames',
    'valid_angle_samples', 'missing_pct', 'capture_id', 'captured_at',
)


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def population_stats(values):
    values = [float(value) for value in values if finite(value)]
    if not values:
        return None, None
    mean = sum(values) / len(values)
    return mean, math.sqrt(sum((value - mean) ** 2 for value in values) / len(values))


def linear_fit(points):
    """Fit actual angle y = a + b*x using the supplied MATLAB equations."""
    usable = [point for point in points
              if finite(point.get('measured_mean_deg')) and finite(point.get('actual_angle_deg'))]
    usable.sort(key=lambda point: float(point['actual_angle_deg']))
    if len(usable) < 3:
        raise ValueError('至少完成 3 个角度组才能拟合')
    x = [float(point['measured_mean_deg']) for point in usable]
    y = [float(point['actual_angle_deg']) for point in usable]
    n = len(x)
    x_average, y_average = sum(x) / n, sum(y) / n
    sxx = sum((value - x_average) ** 2 for value in x)
    syy = sum((value - y_average) ** 2 for value in y)
    if sxx <= 1e-15:
        raise ValueError('各组测量角均值没有变化，无法拟合')
    sxy = sum((x_value - x_average) * (y_value - y_average)
              for x_value, y_value in zip(x, y))
    b = sxy / sxx
    a = y_average - b * x_average
    yhat = [a + b * value for value in x]
    explained = sum((value - y_average) ** 2 for value in yhat)
    residual = max(0.0, syy - explained)
    r = sxy / math.sqrt(sxx * syy) if syy > 0 else None
    # Keep Syhat exactly as supplied by the user: sqrt(U / (n - 2)).
    syhat = math.sqrt(explained / (n - 2))
    sy = math.sqrt(syy / (n - 1))
    sq = math.sqrt(residual)
    sa = syhat * math.sqrt(1.0 / n + x_average ** 2 / sxx)
    sb = syhat / math.sqrt(sxx)
    rmse = math.sqrt(sum((estimate - actual) ** 2
                         for estimate, actual in zip(yhat, y)) / n)
    return {
        'method': 'ordinary_linear_y_equals_a_plus_bx_supplied_equations',
        'formula': 'actual_angle_deg = a + b * measured_horizontal_deg',
        'n': n, 'x_average': x_average, 'y_average': y_average,
        'a': a, 'b': b, 'r': r, 'Sy': sy, 'Sq': sq, 'Syhat': syhat,
        'Sa': sa, 'Sb': sb, 'U': explained, 'Q': residual, 'rmse_deg': rmse,
        'x': x, 'y': y, 'yhat': yhat,
    }


def prediction_band(fit, x_value):
    """Return fitted y and the error limits from the supplied plot formula."""
    x_value = float(x_value)
    sxx = sum((value - fit['x_average']) ** 2 for value in fit['x'])
    estimate = fit['a'] + fit['b'] * x_value
    delta = fit['Syhat'] * math.sqrt(
        1.0 + 1.0 / fit['n'] + (x_value - fit['x_average']) ** 2 / sxx)
    return estimate, estimate - delta, estimate + delta


def discover_modules(robots):
    """Build the six module choices from the main console's vehicle records."""
    modules = []
    for ip, original in robots.items():
        row = dict(original)
        name = row.get('robot_id') or row.get('name', '')
        if not name.startswith('ugv'):
            continue
        row['robot_id'] = name
        try:
            sensors = sensors_for(row)
        except (ValueError, IndexError):
            continue
        for sensor in sensors:
            if not sensor.get('uid'):
                continue
            board = uid_name(sensor['uid'])
            modules.append({
                'uid': int(sensor['uid']), 'board': board, 'robot': name,
                'ip': ip, 'sensor': sensor['name'], 'port': sensor['port'],
                'row': row,
                'label': '%s · %s · %s · 0x%08X' % (board, ip, sensor['port'], sensor['uid']),
            })
    modules.sort(key=lambda item: (item['robot'], item['sensor']))
    return modules


def available_targets(modules, source):
    """Return every module except the selected source, regardless of live graph."""
    return [module for module in modules if module['uid'] != source['uid']]


def fit_code(source, target):
    """Return the compact lab name, e.g. f23 means ugv2 measures ugv3."""
    source_number, target_number = str(source['robot'])[3:], str(target['robot'])[3:]
    if source_number != target_number:
        return 'f%s%s' % (source_number, target_number)
    return 'f%s_%s' % (source['board'], target['board'])


def atomic_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix='.' + path.name + '-', dir=str(path.parent))
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
        os.replace(temporary, str(path))
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def atomic_csv(path, fields, rows):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix='.' + path.name + '-', dir=str(path.parent))
    try:
        with os.fdopen(fd, 'w', encoding='utf-8', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(rows)
        os.replace(temporary, str(path))
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def read_session(directory):
    with (Path(directory) / 'session.json').open(encoding='utf-8') as stream:
        data = json.load(stream)
    data['_directory'] = str(Path(directory))
    return data


class SessionStore:
    def __init__(self, directory, source, target, config_path):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.observation_path = self.directory / 'observations.csv'
        self.stream = self.observation_path.open('a', encoding='utf-8', newline='')
        self.writer = csv.DictWriter(self.stream, fieldnames=OBSERVATION_FIELDS)
        self.writer.writeheader()
        self.stream.flush()
        now = datetime.now().astimezone().isoformat()
        self.data = {
            'format_version': 1, 'created_at': now, 'updated_at': now,
            'state': 'connecting', 'capture_seconds': CAPTURE_SECONDS,
            'angles_deg': list(ANGLES), 'config_path': str(Path(config_path)),
            'source': self.public_module(source), 'target': self.public_module(target),
            'fit_code': fit_code(source, target),
            'points': [], 'fit': None,
            'formula_note': 'Syhat follows supplied formula sqrt(U/(n-2)); band follows supplied plot formula.',
        }
        self.save()

    @staticmethod
    def public_module(module):
        return {key: module[key] for key in ('uid', 'board', 'robot', 'ip', 'sensor', 'port')}

    def save(self):
        self.data['updated_at'] = datetime.now().astimezone().isoformat()
        atomic_json(self.directory / 'session.json', self.data)
        atomic_csv(self.directory / 'points.csv', POINT_FIELDS, self.data['points'])
        if self.data.get('fit'):
            atomic_json(self.directory / 'fit.json', self.data['fit'])

    def write_observation(self, row):
        self.writer.writerow({key: row.get(key, '') for key in OBSERVATION_FIELDS})
        self.stream.flush()

    def set_point(self, point):
        angle = int(point['actual_angle_deg'])
        self.data['points'] = [old for old in self.data['points']
                               if int(old['actual_angle_deg']) != angle]
        self.data['points'].append(point)
        self.data['points'].sort(key=lambda item: int(item['actual_angle_deg']))
        try:
            self.data['fit'] = linear_fit(self.data['points'])
        except ValueError:
            self.data['fit'] = None
        self.save()

    def close(self, state='stopped'):
        if not self.stream.closed:
            self.data['state'] = state
            self.save()
            self.stream.close()


class ConnectionAdapter:
    def __init__(self, root, config_path):
        self.root = root
        self.host_keys = Path(config_path).with_name('known_hosts')

    def connect(self, ip, options):
        return connect_ssh(ip, options, self.host_keys)


class FitCanvas:
    def __init__(self, tk, parent):
        self.canvas = tk.Canvas(parent, background='#f7fafc', highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)
        self.fit, self.fit_code = None, ''
        self.canvas.bind('<Configure>', lambda unused: self.paint())

    def show(self, fit, fit_code=''):
        self.fit, self.fit_code = fit, fit_code
        self.paint()

    def paint(self):
        canvas = self.canvas
        canvas.delete('all')
        width, height = max(420, canvas.winfo_width()), max(300, canvas.winfo_height())
        fit = self.fit
        if not fit:
            canvas.create_text(width / 2, height / 2,
                               text='完成至少 3 个角度组后显示拟合图', fill='#526574')
            return
        samples = [prediction_band(fit, min(fit['x']) +
                                   (max(fit['x']) - min(fit['x'])) * index / 100.0)
                   for index in range(101)]
        xmin, xmax = min(fit['x']), max(fit['x'])
        if abs(xmax - xmin) < 1e-9:
            xmin, xmax = xmin - 1, xmax + 1
        xpad = max(1.0, (xmax - xmin) * 0.08)
        xmin, xmax = xmin - xpad, xmax + xpad
        yvalues = fit['y'] + [value for sample in samples for value in sample]
        ymin, ymax = min(yvalues), max(yvalues)
        ypad = max(2.0, (ymax - ymin) * 0.08)
        ymin, ymax = ymin - ypad, ymax + ypad
        # Keep equation, metrics and legend in dedicated rows above the axes;
        # drawing all three at the same coordinate caused the screenshot's ghosting.
        left, right, top, bottom = 66, width - 25, 76, height - 55

        def px(value):
            return left + (value - xmin) / (xmax - xmin) * (right - left)

        def py(value):
            return bottom - (value - ymin) / (ymax - ymin) * (bottom - top)

        for index in range(6):
            xvalue = xmin + (xmax - xmin) * index / 5.0
            yvalue = ymin + (ymax - ymin) * index / 5.0
            canvas.create_line(px(xvalue), top, px(xvalue), bottom, fill='#e0e7ec')
            canvas.create_text(px(xvalue), bottom + 17, text='%.1f' % xvalue, fill='#526574')
            canvas.create_line(left, py(yvalue), right, py(yvalue), fill='#e0e7ec')
            canvas.create_text(left - 8, py(yvalue), text='%.1f' % yvalue,
                               anchor='e', fill='#526574')
        canvas.create_line(left, bottom, right, bottom, fill='#526574')
        canvas.create_line(left, top, left, bottom, fill='#526574')
        canvas.create_text((left + right) / 2, height - 16,
                           text='IOT 水平角组均值 x（°）', fill='#243746')
        canvas.create_text(16, (top + bottom) / 2, text='实际角 y（°）', angle=90, fill='#243746')
        xline = [xmin + (xmax - xmin) * index / 100.0 for index in range(101)]
        triples = [prediction_band(fit, value) for value in xline]
        for component, color, dash, width_value in ((1, '#9aaab5', (3, 3), 1),
                                                     (2, '#9aaab5', (3, 3), 1),
                                                     (0, '#1976d2', (), 2)):
            coords = []
            for xvalue, values in zip(xline, triples):
                coords.extend((px(xvalue), py(values[component])))
            canvas.create_line(*coords, fill=color, dash=dash, width=width_value, smooth=True)
        for xvalue, yvalue in zip(fit['x'], fit['y']):
            canvas.create_rectangle(px(xvalue) - 4, py(yvalue) - 4,
                                    px(xvalue) + 4, py(yvalue) + 4,
                                    fill='#e76622', outline='white')
        canvas.create_text(left + 8, 14, anchor='nw', fill='#243746',
                           text='%s：y = %.6f %+.6f x' %
                           (self.fit_code or '拟合', fit['a'], fit['b']))
        canvas.create_text(left + 8, 38, anchor='nw', fill='#526574',
                           text='r=%s    RMSE=%.3f°    Syhat=%.3f' %
                           ('—' if fit['r'] is None else '%.5f' % fit['r'],
                            fit['rmse_deg'], fit['Syhat']))
        canvas.create_text(right, 62, anchor='ne', fill='#71818c',
                           text='橙：组均值   蓝：拟合   灰虚线：误差范围')


class CalibrationApp:
    def __init__(self, root, config_path, data_root=DATA_ROOT):
        import tkinter as tk
        from tkinter import ttk, messagebox
        self.tk, self.ttk, self.messagebox = tk, ttk, messagebox
        self.root, self.config_path, self.data_root = root, Path(config_path), Path(data_root)
        self.options, self.robots = load_settings(self.config_path)
        self.modules = discover_modules(self.robots)
        self.by_label = {module['label']: module for module in self.modules}
        self.adapter = ConnectionAdapter(root, self.config_path)
        self.events, self.sessions, self.states = queue.Queue(), [], {}
        self.store = None
        self.capture = None
        self.loaded = None
        self.last_directory = None
        self.closing = False
        self.close_deadline = None
        root.title('IOT 独立角度拟合测试')
        root.geometry('1500x900')
        root.minsize(1200, 760)
        style = ttk.Style(root)
        style.theme_use('clam')
        # Leave enough vertical pixels for the large CJK font used by Tk on the
        # vehicle test laptop; the old 27px rows clipped the descenders.
        style.configure('Treeview', rowheight=38, font=('TkDefaultFont', 10))
        style.configure('Treeview.Heading', font=('TkDefaultFont', 10, 'bold'))
        self.make_ui()
        self.refresh_history()
        self.timer = root.after(100, self.poll)
        root.protocol('WM_DELETE_WINDOW', self.close)

    def make_ui(self):
        tk, ttk = self.tk, self.ttk
        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill='both', expand=True)
        setup = ttk.LabelFrame(outer, text='1 · 选择任意两个不同 IoT 模块（读取主控制台车辆信息）', padding=8)
        setup.pack(fill='x')
        ttk.Label(setup, text='来源模块').grid(row=0, column=0, sticky='w')
        self.source = ttk.Combobox(setup, state='readonly', width=55,
                                   values=[module['label'] for module in self.modules])
        self.source.grid(row=0, column=1, sticky='ew', padx=6)
        ttk.Label(setup, text='测得').grid(row=0, column=2)
        self.target = ttk.Combobox(setup, state='readonly', width=55)
        self.target.grid(row=0, column=3, sticky='ew', padx=6)
        setup.columnconfigure(1, weight=1)
        setup.columnconfigure(3, weight=1)
        self.source.bind('<<ComboboxSelected>>', self.source_changed)
        if self.modules:
            self.source.current(0)
            self.source_changed()
        ttk.Button(setup, text='连接 IoT / 新建记录', command=self.connect).grid(row=1, column=0, pady=8)
        ttk.Button(setup, text='停止并保存', command=self.disconnect).grid(row=1, column=1, sticky='w', pady=8)
        self.connection_status = tk.StringVar(value='未连接')
        ttk.Label(setup, textvariable=self.connection_status, wraplength=850).grid(
            row=1, column=1, columnspan=3, sticky='e', pady=8)

        capture = ttk.LabelFrame(outer, text='2 · 摆放实际车辆后，静止采集 60 秒', padding=8)
        capture.pack(fill='x', pady=8)
        ttk.Label(capture, text='实际夹角').pack(side='left')
        self.actual_angle = ttk.Combobox(capture, state='readonly', width=8,
                                         values=['%d°' % angle for angle in ANGLES])
        self.actual_angle.current(0)
        self.actual_angle.pack(side='left', padx=6)
        self.capture_button = ttk.Button(capture, text='开始本组 60 秒采集', command=self.start_capture)
        self.capture_button.pack(side='left')
        self.cancel_button = ttk.Button(capture, text='取消本组', command=self.cancel_capture, state='disabled')
        self.cancel_button.pack(side='left', padx=6)
        self.progress = ttk.Progressbar(capture, maximum=CAPTURE_SECONDS, length=260)
        self.progress.pack(side='left', padx=10)
        self.capture_status = tk.StringVar(value='请连接 IoT；每组必须由人工确认摆放后再开始。')
        ttk.Label(capture, textvariable=self.capture_status, wraplength=650).pack(side='left', fill='x', expand=True)

        panes = ttk.Panedwindow(outer, orient='horizontal')
        panes.pack(fill='both', expand=True)
        left, right = ttk.Frame(panes), ttk.LabelFrame(panes, text='拟合效果', padding=5)
        self.fit_frame = right
        panes.add(left, weight=3)
        panes.add(right, weight=2)
        self.points = ttk.Treeview(left, columns=POINT_FIELDS[:-2], show='headings', height=14)
        headings = {
            'actual_angle_deg': '实际角°', 'measured_mean_deg': '测量均值°',
            'measured_std_deg': '角标准差°', 'distance_mean_m': '距离均值m',
            'distance_std_m': '距离标准差m', 'source_frames': '来源帧',
            'target_present_frames': '目标出现', 'valid_angle_samples': '有效样本',
            'missing_pct': '缺测率%',
        }
        for key in POINT_FIELDS[:-2]:
            self.points.heading(key, text=headings[key])
            self.points.column(key, width=96, minwidth=84, anchor='center', stretch=True)
        scroll = ttk.Scrollbar(left, command=self.points.yview)
        xscroll = ttk.Scrollbar(left, orient='horizontal', command=self.points.xview)
        self.points.configure(yscrollcommand=scroll.set)
        self.points.configure(xscrollcommand=xscroll.set)
        scroll.pack(side='right', fill='y')
        xscroll.pack(side='bottom', fill='x')
        self.points.pack(fill='both', expand=True)
        self.fit_canvas = FitCanvas(tk, right)

        tools = ttk.Frame(outer)
        tools.pack(fill='x', pady=(8, 0))
        ttk.Button(tools, text='查看原始逐帧记录', command=self.view_raw).pack(side='left')
        ttk.Button(tools, text='导出汇总 CSV', command=self.export_csv).pack(side='left', padx=5)
        ttk.Button(tools, text='导出完整 ZIP', command=self.export_zip).pack(side='left')
        ttk.Button(tools, text='复制拟合结果', command=self.copy_fit).pack(side='left', padx=5)
        ttk.Label(tools, text='历史记录').pack(side='left', padx=(20, 5))
        self.history = ttk.Combobox(tools, state='readonly', width=44)
        self.history.pack(side='left', fill='x', expand=True)
        ttk.Button(tools, text='打开历史', command=self.open_history).pack(side='left', padx=5)
        ttk.Button(tools, text='刷新', command=self.refresh_history).pack(side='left')
        self.fit_status = tk.StringVar(value='尚无拟合结果')
        ttk.Label(outer, textvariable=self.fit_status, wraplength=1270).pack(fill='x', pady=(7, 0))

    def source_changed(self, unused=None):
        source = self.by_label.get(self.source.get())
        if not source:
            self.target['values'] = ()
            self.target.set('')
            return
        # Calibration is deliberately more general than the live formation graph:
        # any two different modules can be put in a static pose and measured.
        targets = [module['label'] for module in available_targets(self.modules, source)]
        self.target['values'] = targets
        if targets:
            self.target.set(targets[0])
        else:
            self.target.set('')

    def selected_modules(self):
        source, target = self.by_label.get(self.source.get()), self.by_label.get(self.target.get())
        if not source or not target:
            raise ValueError('请选择来源模块和目标模块')
        if source['uid'] == target['uid']:
            raise ValueError('来源模块和目标模块必须不同')
        return source, target

    def connect(self):
        if any(session.thread.is_alive() for session in self.sessions):
            self.messagebox.showinfo('IOT 标定', '当前 IOT 已连接；请先停止并保存。', parent=self.root)
            return
        try:
            source, target = self.selected_modules()
            if not self.options.get('password'):
                raise ValueError('主控制台尚未保存 SSH 密码，请先在主软件填写并保存')
            stamp = datetime.now().strftime('cal_%Y%m%d_%H%M%S_') + uuid.uuid4().hex[:6]
            store = SessionStore(self.data_root / stamp, source, target, self.config_path)
            sessions, states = [], {}
            rows = {source['ip']: source['row'], target['ip']: target['row']}
            for ip, row in rows.items():
                states[ip] = '正在连接'
                sessions.append(IotSession(self.adapter, ip, row, self.options,
                                           self.events, store.directory))
            self.store, self.sessions, self.states = store, sessions, states
            self.last_directory = store.directory
            self.loaded = store.data
            self.source.configure(state='disabled')
            self.target.configure(state='disabled')
            self.connection_status.set('正在启动 %s → %s；记录：%s' %
                                       (source['board'], target['board'], store.directory))
            self.show_data(store.data)
            self.refresh_history()
        except (OSError, ValueError) as error:
            self.messagebox.showerror('不能启动 IOT 标定', str(error), parent=self.root)

    def disconnect(self):
        if self.capture:
            self.cancel_capture()
        for session in self.sessions:
            session.stop.set()
        if self.sessions:
            self.connection_status.set('正在停止并等待车端保存 bag…')
        elif self.store:
            self.store.close()
            self.store = None
        self.source.configure(state='readonly')
        self.target.configure(state='readonly')

    def start_capture(self):
        if self.capture:
            return
        if not self.store or not self.sessions or not all(state == '采集中' for state in self.states.values()):
            self.messagebox.showerror('不能采集', '请等待所选车辆全部显示“采集中”。', parent=self.root)
            return
        source, target = self.selected_modules()
        angle = ANGLES[self.actual_angle.current()]
        existing = [point for point in self.store.data['points']
                    if int(point['actual_angle_deg']) == angle]
        if existing and not self.messagebox.askyesno(
                '重新采集', '%d° 已有均值。重新采集将替换均值；旧逐帧记录仍保留。' % angle,
                parent=self.root):
            return
        started = time.monotonic()
        self.capture = {
            'id': uuid.uuid4().hex, 'actual': angle, 'source_uid': source['uid'],
            'target_uid': target['uid'], 'started': started,
            'deadline': started + CAPTURE_SECONDS, 'finalize_at': started + CAPTURE_SECONDS + 0.6,
            'frames': 0, 'present': 0, 'angles': [], 'distances': [],
        }
        self.store.data['state'] = 'capturing'
        self.store.save()
        self.capture_button.configure(state='disabled')
        self.cancel_button.configure(state='normal')
        self.progress['value'] = 0
        self.capture_status.set('%d°：采集中，车辆必须保持静止…' % angle)

    def cancel_capture(self):
        if not self.capture:
            return
        angle, frames = self.capture['actual'], self.capture['frames']
        self.capture = None
        self.capture_button.configure(state='normal')
        self.cancel_button.configure(state='disabled')
        self.progress['value'] = 0
        self.capture_status.set('%d° 已取消（%d 个来源帧仍保留在原始 CSV）。' % (angle, frames))
        if self.store:
            self.store.data['state'] = 'recording'
            self.store.save()

    def record_frame(self, data):
        capture = self.capture
        if not capture or int(data.get('uid', -1)) != capture['source_uid']:
            return
        stamp = float(data.get('host_monotonic', time.monotonic()))
        if stamp < capture['started'] or stamp > capture['deadline']:
            return
        capture['frames'] += 1
        node = next((item for item in data.get('nodes', [])
                     if int(item.get('uid', -1)) == capture['target_uid']), None)
        present = node is not None
        if present:
            capture['present'] += 1
        angle = node.get('horizontal') if node else None
        distance = node.get('distance') if node else None
        valid_angle = finite(angle)
        if valid_angle:
            capture['angles'].append(float(angle))
        if finite(distance):
            capture['distances'].append(float(distance))
        row = {
            'capture_id': capture['id'], 'actual_angle_deg': capture['actual'],
            'source_uid': capture['source_uid'], 'target_uid': capture['target_uid'],
            'host_received': data.get('host_received', ''), 'host_monotonic': stamp,
            'elapsed_s': stamp - capture['started'], 'system_time': data.get('system_time', ''),
            'target_present': int(present), 'valid_angle': int(valid_angle),
            'distance_m': distance if finite(distance) else '',
            'measured_horizontal_deg': angle if valid_angle else '',
            'fp_rssi': node.get('fp_rssi', '') if node else '',
            'rx_rssi': node.get('rx_rssi', '') if node else '',
        }
        self.store.write_observation(row)

    def finish_capture(self):
        capture = self.capture
        if not capture:
            return
        self.capture = None
        angle_mean, angle_std = population_stats(capture['angles'])
        distance_mean, distance_std = population_stats(capture['distances'])
        if angle_mean is None:
            self.capture_status.set('%d° 完成，但没有有效目标角度；请检查摆放、方向和串口后重测。' %
                                    capture['actual'])
        else:
            frames = capture['frames']
            missing = 100.0 * (1.0 - capture['present'] / frames) if frames else 100.0
            point = {
                'actual_angle_deg': capture['actual'], 'measured_mean_deg': angle_mean,
                'measured_std_deg': angle_std, 'distance_mean_m': distance_mean,
                'distance_std_m': distance_std, 'source_frames': frames,
                'target_present_frames': capture['present'],
                'valid_angle_samples': len(capture['angles']), 'missing_pct': missing,
                'capture_id': capture['id'],
                'captured_at': datetime.now().astimezone().isoformat(),
            }
            self.store.set_point(point)
            self.loaded = self.store.data
            self.show_data(self.loaded)
            self.capture_status.set('%d° 完成：测量均值 %.3f°，有效 %d/%d，缺测 %.1f%%。' %
                                    (capture['actual'], angle_mean, len(capture['angles']), frames, missing))
            remaining = [value for value in ANGLES
                         if value not in {int(item['actual_angle_deg']) for item in self.store.data['points']}]
            if remaining:
                self.actual_angle.current(ANGLES.index(remaining[0]))
        self.store.data['state'] = 'recording'
        self.store.save()
        self.capture_button.configure(state='normal')
        self.cancel_button.configure(state='disabled')
        self.progress['value'] = 0

    def handle_event(self, ip, data):
        event = data.get('event')
        if event == 'frame':
            self.record_frame(data)
        elif event == 'started':
            self.states[ip] = '采集中'
        elif event == 'error':
            self.states[ip] = '失败：' + data.get('message', '未知错误')
            if self.capture:
                self.cancel_capture()
        elif event == 'stopped':
            self.states[ip] = '已保存'
        elif event == 'closed' and not self.states.get(ip, '').startswith(('失败', '已保存')):
            self.states[ip] = '已停止'

    def poll(self):
        now = time.monotonic()
        for session in self.sessions:
            session.ui_heartbeat = now
        for unused in range(2000):
            try:
                ip, data = self.events.get_nowait()
            except queue.Empty:
                break
            self.handle_event(ip, data)
        if self.states:
            self.connection_status.set('  |  '.join('%s %s' % item for item in self.states.items()) +
                                       ('  ·  ' + str(self.store.directory) if self.store else ''))
        if self.capture:
            elapsed = min(CAPTURE_SECONDS, max(0.0, now - self.capture['started']))
            self.progress['value'] = elapsed
            remaining = max(0, int(math.ceil(self.capture['deadline'] - now)))
            self.capture_status.set('%d°：还剩 %d 秒 · 来源帧 %d · 有效角度 %d' %
                                    (self.capture['actual'], remaining, self.capture['frames'],
                                     len(self.capture['angles'])))
            if now >= self.capture['finalize_at']:
                self.finish_capture()
        if self.sessions and not any(session.thread.is_alive() for session in self.sessions):
            if self.capture:
                self.cancel_capture()
            if self.store:
                self.store.close('stopped')
                self.loaded = self.store.data
                self.store = None
            self.sessions = []
            self.source.configure(state='readonly')
            self.target.configure(state='readonly')
            self.refresh_history()
        if self.closing:
            if not self.sessions or now >= self.close_deadline:
                if self.store:
                    self.store.close('interrupted')
                self.root.destroy()
                return
        self.timer = self.root.after(100, self.poll)

    def show_data(self, data):
        self.loaded = data
        self.points.delete(*self.points.get_children())
        for point in sorted(data.get('points', []), key=lambda item: int(item['actual_angle_deg'])):
            values = []
            for key in POINT_FIELDS[:-2]:
                value = point.get(key)
                if value is None:
                    values.append('—')
                elif key in ('measured_mean_deg', 'measured_std_deg'):
                    values.append('%.3f' % float(value))
                elif key in ('distance_mean_m', 'distance_std_m'):
                    values.append('%.4f' % float(value))
                elif key == 'missing_pct':
                    values.append('%.1f' % float(value))
                else:
                    values.append(value)
            self.points.insert('', 'end', values=values)
        fit = data.get('fit')
        code = data.get('fit_code', '')
        if not code and data.get('source') and data.get('target'):
            code = fit_code(data['source'], data['target'])
        self.fit_frame.configure(text='拟合效果' + ((' · ' + code) if code else ''))
        self.fit_canvas.show(fit, code)
        if fit:
            self.fit_status.set('%s（来源 %s → 目标 %s）：actual = %.8f %+.8f × measured；n=%d，r=%s，RMSE=%.4f°，Syhat=%.4f' %
                                (code or '拟合', data.get('source', {}).get('board', '未知'),
                                 data.get('target', {}).get('board', '未知'), fit['a'], fit['b'], fit['n'],
                                 '—' if fit['r'] is None else '%.6f' % fit['r'],
                                 fit['rmse_deg'], fit['Syhat']))
        else:
            total_angles = len(data.get('angles_deg') or ANGLES)
            self.fit_status.set('已完成 %d/%d 组；至少 3 组后生成拟合，建议完成全部 -60–60°。' %
                                (len(data.get('points', [])), total_angles))

    def current_directory(self):
        if not self.loaded:
            raise ValueError('当前没有采集或历史记录')
        if self.store and self.loaded is self.store.data:
            return self.store.directory
        directory = self.loaded.get('_directory') or self.last_directory
        if not directory:
            raise ValueError('记录目录不可用')
        return Path(directory)

    def refresh_history(self):
        self.data_root.mkdir(parents=True, exist_ok=True)
        directories = sorted((path for path in self.data_root.iterdir()
                              if path.is_dir() and (path / 'session.json').exists()), reverse=True)
        self.history['values'] = [str(path) for path in directories]
        if directories and not self.history.get():
            self.history.set(str(directories[0]))

    def open_history(self):
        if not self.history.get():
            self.messagebox.showinfo('历史记录', '没有已保存的标定记录。', parent=self.root)
            return
        try:
            data = read_session(self.history.get())
            self.show_data(data)
            self.connection_status.set('已打开历史记录：' + self.history.get())
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as error:
            self.messagebox.showerror('历史记录读取失败', str(error), parent=self.root)

    def view_raw(self):
        try:
            path = self.current_directory() / 'observations.csv'
            text = path.read_text(encoding='utf-8', errors='replace')
        except (OSError, ValueError) as error:
            self.messagebox.showerror('原始记录读取失败', str(error), parent=self.root)
            return
        dialog = self.tk.Toplevel(self.root)
        dialog.title('原始逐帧记录 · ' + str(path))
        dialog.geometry('1150x650')
        editor = self.tk.Text(dialog, wrap='none', padx=8, pady=8)
        yscroll = self.ttk.Scrollbar(dialog, orient='vertical', command=editor.yview)
        xscroll = self.ttk.Scrollbar(dialog, orient='horizontal', command=editor.xview)
        editor.configure(yscrollcommand=yscroll.set, xscrollcommand=xscroll.set)
        yscroll.pack(side='right', fill='y')
        xscroll.pack(side='bottom', fill='x')
        editor.pack(fill='both', expand=True)
        editor.insert('1.0', text)
        editor.configure(state='disabled')

    def export_csv(self):
        from tkinter import filedialog
        try:
            directory = self.current_directory()
        except ValueError as error:
            self.messagebox.showerror('导出失败', str(error), parent=self.root)
            return
        target = filedialog.asksaveasfilename(parent=self.root, title='导出拟合汇总 CSV',
                                              defaultextension='.csv',
                                              initialfile=directory.name + '_summary.csv',
                                              filetypes=[('CSV', '*.csv')])
        if not target:
            return
        try:
            with open(target, 'w', encoding='utf-8-sig', newline='') as stream:
                fields = list(POINT_FIELDS) + ['fit_code', 'fit_a', 'fit_b', 'fit_r', 'fit_rmse_deg', 'fit_yhat_deg']
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                fit = self.loaded.get('fit')
                code = self.loaded.get('fit_code', '')
                if not code and self.loaded.get('source') and self.loaded.get('target'):
                    code = fit_code(self.loaded['source'], self.loaded['target'])
                for point in sorted(self.loaded.get('points', []),
                                    key=lambda item: int(item['actual_angle_deg'])):
                    row = dict(point)
                    row['fit_code'] = code
                    if fit:
                        row.update(fit_a=fit['a'], fit_b=fit['b'], fit_r=fit['r'],
                                   fit_rmse_deg=fit['rmse_deg'],
                                   fit_yhat_deg=fit['a'] + fit['b'] * point['measured_mean_deg'])
                    writer.writerow(row)
            self.fit_status.set('已导出：' + target)
        except OSError as error:
            self.messagebox.showerror('导出失败', str(error), parent=self.root)

    def export_zip(self):
        from tkinter import filedialog
        try:
            directory = self.current_directory()
        except ValueError as error:
            self.messagebox.showerror('导出失败', str(error), parent=self.root)
            return
        target = filedialog.asksaveasfilename(parent=self.root, title='导出完整记录 ZIP',
                                              defaultextension='.zip',
                                              initialfile=directory.name + '.zip',
                                              filetypes=[('ZIP', '*.zip')])
        if not target:
            return
        try:
            target_path = Path(target)
            base = str(target_path.with_suffix('')) if target_path.suffix.lower() == '.zip' else str(target_path)
            archive = shutil.make_archive(base, 'zip', root_dir=str(directory))
            self.fit_status.set('已导出完整记录：' + archive)
        except OSError as error:
            self.messagebox.showerror('导出失败', str(error), parent=self.root)

    def copy_fit(self):
        fit = self.loaded.get('fit') if self.loaded else None
        if not fit:
            self.messagebox.showinfo('复制拟合结果', '完成至少 3 个有效角度组后才能复制。', parent=self.root)
            return
        x = ' '.join('%.8g' % value for value in fit['x'])
        y = ' '.join('%.8g' % value for value in fit['y'])
        code = self.loaded.get('fit_code', '')
        if not code and self.loaded.get('source') and self.loaded.get('target'):
            code = fit_code(self.loaded['source'], self.loaded['target'])
        text = ('%% %s: source measures target\n' % (code or 'fit') +
                'x=[%s];\ny=[%s];\na=%.12g;\nb=%.12g;\nr=%s;\n'
                '%% actual_angle_deg = a + b * measured_horizontal_deg\n' %
                (x, y, fit['a'], fit['b'],
                 'NaN' if fit['r'] is None else '%.12g' % fit['r']))
        self.root.clipboard_clear()
        self.root.clipboard_append(text)
        self.root.update_idletasks()
        self.fit_status.set('拟合数组、a/b/r 和公式已复制到剪贴板。')

    def close(self):
        if self.closing:
            return
        self.closing = True
        if self.capture:
            self.cancel_capture()
        for session in self.sessions:
            session.stop.set()
        self.close_deadline = time.monotonic() + 15
        self.connection_status.set('正在停止 IoT 并保存原始 bag，完成后自动关闭…')
        if not self.sessions:
            if self.store:
                self.store.close()
            self.root.destroy()


def self_test():
    fit = linear_fit([
        {'measured_mean_deg': 6.5563, 'actual_angle_deg': 0},
        {'measured_mean_deg': 14.8223, 'actual_angle_deg': 5},
        {'measured_mean_deg': 22.3621, 'actual_angle_deg': 10},
        {'measured_mean_deg': 35.9828, 'actual_angle_deg': 15},
        {'measured_mean_deg': 44.2294, 'actual_angle_deg': 20},
    ])
    assert abs(fit['a'] - (-2.7138)) < 0.02, fit
    assert abs(fit['b'] - 0.5128) < 0.002, fit
    estimate, low, high = prediction_band(fit, fit['x_average'])
    assert low < estimate < high
    assert discover_modules({'192.168.0.109': {'robot_id': 'ugv3'}})[0]['uid'] in (
        0x2A003200, 0x30006E00)
    print('IOT calibration checks passed: modules, supplied linear fit and error band.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=CONFIG_PATH,
                        help='主控制台 settings.json（默认读取主软件配置）')
    parser.add_argument('--data-root', type=Path, default=DATA_ROOT,
                        help='标定记录保存目录')
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--check-gui', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    try:
        import tkinter as tk
        root = tk.Tk()
    except Exception as error:
        parser.exit(1, 'GUI 不可用：%s；需要桌面会话与 python3-tk\n' % error)
    if args.check_gui:
        root.withdraw()
        root.update()
        print('IOT 标定 GUI 可用：Tk %s' % tk.TkVersion)
        root.destroy()
        return
    try:
        app = CalibrationApp(root, args.config, args.data_root)
        if len(app.modules) < 2:
            raise ValueError('主控制台配置中没有足够的已编号车辆 / IOT 模块')
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as error:
        from tkinter import messagebox
        messagebox.showerror('IOT 标定软件启动失败', '%s\n配置：%s' % (error, args.config))
        root.destroy()
        return
    root.mainloop()


if __name__ == '__main__':
    main()
