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
# Live monitor: frames are buffered as compact (elapsed, measured, reference)
# tuples and the canvas repaints on this cadence instead of once per frame.
LIVE_SAMPLE_LIMIT = 20000
LIVE_REFRESH_MS = 120
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


def comparison_series(points, fit):
    """Pair every stored angle group with its raw measurement and fitted angle.

    The comparison chart must plot both curves from exactly the same groups, so
    this returns one ordered row per group instead of two loose lists. Without a
    fit the fitted column stays ``None`` and the chart simply skips that series.
    """
    usable = [point for point in points
              if finite(point.get('actual_angle_deg')) and finite(point.get('measured_mean_deg'))]
    usable.sort(key=lambda point: float(point['actual_angle_deg']))
    series = []
    for point in usable:
        measured = float(point['measured_mean_deg'])
        series.append({
            'actual_angle_deg': float(point['actual_angle_deg']),
            'measured_mean_deg': measured,
            'fitted_angle_deg': None if not fit else fit['a'] + fit['b'] * measured,
        })
    return series


def live_series(samples, fit, window_seconds=None):
    """Build chart rows from the frames the IoT modules push in real time.

    ``samples`` are ``(elapsed_s, measured_deg_or_None, reference_angle_deg)`` in
    arrival order, exactly what :meth:`CalibrationApp.publish_live` stores. A
    frame without a valid target angle stays in the rows as ``None`` so the chart
    breaks the curve at the dropout instead of drawing a fake straight line
    across it. ``window_seconds`` keeps only the newest slice of the stream.
    """
    usable = [sample for sample in samples if finite(sample[0])]
    if not usable:
        return {'rows': [], 'summary': {
            'frames': 0, 'valid': 0, 'missing_pct': 100.0, 'mean_raw': None,
            'std_raw': None, 'mean_fitted': None, 'reference_angle_deg': None,
            'latest_s': None, 'span_s': 0.0}}
    if window_seconds is not None:
        newest = max(float(sample[0]) for sample in usable)
        usable = [sample for sample in usable
                  if newest - float(sample[0]) <= float(window_seconds)]
    rows, values = [], []
    for elapsed, measured, reference in usable:
        raw = float(measured) if finite(measured) else None
        if raw is not None:
            values.append(raw)
        rows.append({
            'x': float(elapsed), 'raw': raw,
            'fitted': None if (raw is None or not fit) else fit['a'] + fit['b'] * raw,
            'reference_angle_deg': float(reference) if finite(reference) else None,
        })
    mean, std = population_stats(values)
    reference = next((row['reference_angle_deg'] for row in reversed(rows)
                      if row['reference_angle_deg'] is not None), None)
    frames, valid = len(rows), len(values)
    return {'rows': rows, 'summary': {
        'frames': frames, 'valid': valid,
        'missing_pct': 100.0 * (1.0 - valid / frames) if frames else 100.0,
        'mean_raw': mean, 'std_raw': std,
        'mean_fitted': None if (mean is None or not fit) else fit['a'] + fit['b'] * mean,
        'reference_angle_deg': reference, 'latest_s': rows[-1]['x'],
        'span_s': rows[-1]['x'] - rows[0]['x']}}


def frame_extremes(path):
    """Return {actual_angle: (min_deg, max_deg, samples)} from the raw frame CSV.

    Only valid target angles are counted; frames where the peer was missing are
    already blank in ``measured_horizontal_deg`` and must not become a zero.
    """
    summary = {}
    with Path(path).open(encoding='utf-8', newline='') as stream:
        for row in csv.DictReader(stream):
            if not (finite(row.get('actual_angle_deg'))
                    and finite(row.get('measured_horizontal_deg'))):
                continue
            key = int(round(float(row['actual_angle_deg'])))
            value = float(row['measured_horizontal_deg'])
            low, high, count = summary.get(key, (value, value, 0))
            summary[key] = (min(low, value), max(high, value), count + 1)
    return summary


def module_text(row):
    """Format a stored module record for the angle chart window header."""
    if not row:
        return '未知'
    uid = row.get('uid')
    return '%s · %s · %s · %s' % (
        row.get('board', '未知'), row.get('ip', '—'), row.get('port', '—'),
        '未知' if uid is None else '0x%08X' % int(uid))


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


class AngleChart:
    """Draw angle data against the current fit on one canvas.

    Two modes share the same painter. ``groups`` charts the stored per-angle
    means against the reference angle; ``live`` charts the frames the IoT
    modules push while the operator watches, with the capture start as x = 0.
    Deliberately Tk-only: the calibration tool must keep running on the vehicle
    test laptop, which has no matplotlib installed, so this mirrors FitCanvas
    instead of importing a plotting library.
    """

    RAW_COLOR = '#e76622'
    FIT_COLOR = '#1976d2'
    SPREAD_COLOR = '#c3cfd8'
    IDEAL_COLOR = '#9aaab5'
    BAND_COLOR = '#e6eef4'
    # Cap the painted geometry so a long live stream still repaints smoothly.
    MAX_LINE_POINTS = 1400
    MAX_DOTS = 400
    X_TICKS = 6
    Y_STEP = 5.0

    def __init__(self, tk, parent):
        self.canvas = tk.Canvas(parent, background='#f7fafc', highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)
        self.series, self.fit, self.code, self.options = [], None, '', {}
        self.summary, self.mode, self.y_lock = {}, 'groups', None
        self.canvas.bind('<Configure>', lambda unused: self.paint())

    def show(self, series, fit, code, options):
        self.mode, self.series, self.fit, self.code, self.options = (
            'groups', series, fit, code, options)
        self.summary, self.y_lock = {}, None
        self.paint()

    def show_live(self, series, summary, fit, code, options):
        self.mode, self.series, self.fit, self.code, self.options = (
            'live', series, fit, code, options)
        self.summary = summary
        self.paint()

    def rows(self):
        """Return the rows to plot, resolved for the selected x axis."""
        if self.mode == 'live':
            return list(self.series)
        measured_axis = self.options.get('axis') == 'measured'
        spread = self.options.get('spread_points') or {}
        rows = []
        for item in self.series:
            low, high = spread.get(int(round(item['actual_angle_deg'])), (None, None, 0))[:2]
            rows.append({
                'x': item['measured_mean_deg'] if measured_axis else item['actual_angle_deg'],
                'raw': item['measured_mean_deg'], 'fitted': item['fitted_angle_deg'],
                'low': low, 'high': high,
            })
        return rows

    @staticmethod
    def segments(rows, key):
        """Split a series into unbroken runs so dropouts stay visible as gaps."""
        runs, run = [], []
        for row in rows:
            value = row.get(key)
            if value is None:
                if run:
                    runs.append(run)
                    run = []
            else:
                run.append((row['x'], value))
        if run:
            runs.append(run)
        return runs

    def thin(self, runs, limit=None):
        """Drop every n-th point of very long runs to keep the painter cheap."""
        limit = limit or self.MAX_LINE_POINTS
        total = sum(len(run) for run in runs)
        if total <= limit:
            return runs, total
        step = int(math.ceil(total / float(limit)))
        thinned = []
        for run in runs:
            piece = run[::step]
            if piece and piece[-1] != run[-1]:
                piece.append(run[-1])
            if piece:
                thinned.append(piece)
        return thinned, total

    def lock_y(self, ymin, ymax):
        """Keep one capture's y axis steady: widen it, never shrink it."""
        key = self.options.get('lock_key')
        if self.mode != 'live' or not key:
            self.y_lock = None
            return ymin, ymax
        if self.y_lock is None or self.y_lock[0] != key:
            self.y_lock = (key, ymin, ymax)
        else:
            self.y_lock = (key, min(self.y_lock[1], ymin), max(self.y_lock[2], ymax))
        return self.y_lock[1], self.y_lock[2]

    def draw_mean_lines(self, canvas, py, left, right):
        """Horizontal markers for the visible window's raw and fitted averages."""
        for key, color, label, offset in (
                ('mean_raw', self.RAW_COLOR, '窗口均值', -12),
                ('mean_fitted', self.FIT_COLOR, '拟合后均值', 12)):
            value = self.summary.get(key)
            if value is None:
                continue
            y_pos = py(value)
            canvas.create_line(left, y_pos, right, y_pos, fill=color, dash=(5, 3), width=2)
            canvas.create_text(right - 6, y_pos + offset, anchor='e', fill=color,
                               text='%s %.3f°' % (label, value))

    def paint(self):
        canvas = self.canvas
        canvas.delete('all')
        width, height = max(420, canvas.winfo_width()), max(300, canvas.winfo_height())
        live = self.mode == 'live'
        rows = self.rows()
        if not rows:
            canvas.create_text(width / 2, height / 2, fill='#526574',
                               text='等待 IoT 实时数据…请在主窗口连接模块并开始采集'
                               if live else '没有可显示的角度组')
            return
        options = self.options
        show_raw = bool(options.get('raw', True))
        show_fit = bool(options.get('fit', True)) and self.fit is not None
        show_spread = (not live and bool(options.get('spread', False))
                       and any(row['low'] is not None for row in rows))
        show_mean = live and bool(options.get('mean', True))
        show_ideal = bool(options.get('ideal', True)) and not live
        reference = self.summary.get('reference_angle_deg') if live else None
        if reference is not None and not options.get('reference', True):
            reference = None
        xvalues = [row['x'] for row in rows]
        yvalues = []
        for row in rows:
            if show_raw and row['raw'] is not None:
                yvalues.append(row['raw'])
            if show_fit and row['fitted'] is not None:
                yvalues.append(row['fitted'])
            if show_spread:
                yvalues.extend(value for value in (row['low'], row['high']) if value is not None)
            if show_ideal:
                yvalues.append(row['x'])
        if show_mean:
            for value in (self.summary.get('mean_raw'), self.summary.get('mean_fitted')):
                if value is not None:
                    yvalues.append(value)
        if reference is not None:
            yvalues.append(reference)
        if not yvalues:
            yvalues = list(xvalues)
        xmin, xmax = min(xvalues), max(xvalues)
        if live and xmax - xmin < 5.0:
            # Hold the left edge still for the first frames instead of letting
            # the x axis lurch on every repaint.
            xmax = xmin + 5.0
        if abs(xmax - xmin) < 1e-9:
            xmin, xmax = xmin - 1, xmax + 1
        xpad = max(1.0, (xmax - xmin) * (0.04 if live else 0.08))
        xmin, xmax = xmin - xpad, xmax + xpad
        ymin, ymax = min(yvalues), max(yvalues)
        if abs(ymax - ymin) < 1e-9:
            ymin, ymax = ymin - 1, ymax + 1
        ypad = max(2.0, (ymax - ymin) * 0.08)
        ymin, ymax = ymin - ypad, ymax + ypad
        # Snap to whole 5° steps so a live axis stops jittering frame by frame.
        ymin = math.floor(ymin / self.Y_STEP) * self.Y_STEP
        ymax = math.ceil(ymax / self.Y_STEP) * self.Y_STEP
        if ymax - ymin < 2 * self.Y_STEP:
            ymin, ymax = ymin - self.Y_STEP, ymax + self.Y_STEP
        ymin, ymax = self.lock_y(ymin, ymax)
        # Same layout rule as FitCanvas: keep the equation, the metrics and the
        # legend on separate rows above the axes so nothing overlaps.
        left, right, top, bottom = 66, width - 25, 84, height - 55

        def px(value):
            return left + (value - xmin) / (xmax - xmin) * (right - left)

        def py(value):
            return bottom - (value - ymin) / (ymax - ymin) * (bottom - top)

        def marker(x_pos, y_pos, color, shape):
            if shape == 'square':
                canvas.create_rectangle(x_pos - 4, y_pos - 4, x_pos + 4, y_pos + 4,
                                        fill=color, outline='white')
            else:
                canvas.create_oval(x_pos - 4, y_pos - 4, x_pos + 4, y_pos + 4,
                                   fill=color, outline='white')

        def draw_series(key, color, shape):
            runs = self.segments(rows, key)
            painted, total = self.thin(runs)
            for run in painted:
                if len(run) > 1:
                    coords = []
                    for value_x, value_y in run:
                        coords.extend((px(value_x), py(value_y)))
                    canvas.create_line(*coords, fill=color, width=2)
            if total <= self.MAX_DOTS:
                for run in runs:
                    for value_x, value_y in run:
                        marker(px(value_x), py(value_y), color, shape)

        span = options.get('capture_span') if live else None
        if span:
            band_left, band_right = max(xmin, min(span)), min(xmax, max(span))
            if band_left < band_right:
                canvas.create_rectangle(px(band_left), top, px(band_right), bottom,
                                        fill=self.BAND_COLOR, outline='')
        tick_format = '%.1f' if live else '%.2f'
        for index in range(self.X_TICKS):
            span_ratio = index / (self.X_TICKS - 1.0)
            xvalue = xmin + (xmax - xmin) * span_ratio
            yvalue = ymin + (ymax - ymin) * span_ratio
            canvas.create_line(px(xvalue), top, px(xvalue), bottom, fill='#e0e7ec')
            canvas.create_text(px(xvalue), bottom + 17, text=tick_format % xvalue, fill='#526574')
            canvas.create_line(left, py(yvalue), right, py(yvalue), fill='#e0e7ec')
            canvas.create_text(left - 8, py(yvalue), text='%.2f' % yvalue,
                               anchor='e', fill='#526574')
        canvas.create_line(left, bottom, right, bottom, fill='#526574')
        canvas.create_line(left, top, left, bottom, fill='#526574')
        if live:
            x_label = '时间 x（秒，自本次连接）'
        else:
            x_label = ('原始测量角 x（°）' if options.get('axis') == 'measured'
                       else '实际角（参考）x（°）')
        canvas.create_text((left + right) / 2, height - 16, fill='#243746', text=x_label)
        canvas.create_text(16, (top + bottom) / 2, text='角度 y（°）', angle=90, fill='#243746')
        if show_spread:
            for row in rows:
                if row['low'] is None:
                    continue
                canvas.create_line(px(row['x']), py(row['low']), px(row['x']), py(row['high']),
                                   fill=self.SPREAD_COLOR, width=2)
        if show_ideal:
            # Clip y = x to the axes box; py() extrapolates outside the plot area.
            low, high = max(xmin, ymin), min(xmax, ymax)
            if low < high:
                canvas.create_line(px(low), py(low), px(high), py(high),
                                   fill=self.IDEAL_COLOR, dash=(4, 4), width=1)
        if show_raw:
            draw_series('raw', self.RAW_COLOR, 'square')
        if show_fit:
            draw_series('fitted', self.FIT_COLOR, 'oval')
        if show_mean:
            self.draw_mean_lines(canvas, py, left, right)
        if reference is not None:
            y_pos = py(reference)
            canvas.create_line(left, y_pos, right, y_pos, fill='#7a8b98', dash=(4, 4), width=1)
            canvas.create_text(left + 8, y_pos - 11, anchor='w', fill='#526574',
                               text='实际角 %g°' % reference)
        if self.fit:
            title = '%s：拟合后角度 = %.6f %+.6f × 原始测量角' % (
                self.code or '拟合', self.fit['a'], self.fit['b'])
        elif live:
            title = '尚未生成拟合：完成至少 3 个角度组后，蓝色拟合曲线才会出现'
        else:
            title = '尚未生成拟合：完成至少 3 个角度组后才会出现拟合后角度'
        if live:
            metrics = '帧 %d · 有效 %d · 缺测 %.1f%% · 窗口均值 %s · 拟合后均值 %s' % (
                self.summary.get('frames', 0), self.summary.get('valid', 0),
                self.summary.get('missing_pct', 0.0),
                '—' if self.summary.get('mean_raw') is None
                else '%.3f°' % self.summary['mean_raw'],
                '—' if self.summary.get('mean_fitted') is None
                else '%.3f°' % self.summary['mean_fitted'])
        elif self.fit:
            metrics = 'n=%d    r=%s    RMSE=%.3f°    组数 %d' % (
                self.fit['n'], '—' if self.fit['r'] is None else '%.5f' % self.fit['r'],
                self.fit['rmse_deg'], len(rows))
        else:
            metrics = '当前仅显示原始测量角，共 %d 组' % len(rows)
        canvas.create_text(left + 8, 14, anchor='nw', fill='#243746', text=title)
        canvas.create_text(left + 8, 38, anchor='nw', fill='#526574', text=metrics)
        legend = ('橙方块：逐帧原始测量角   蓝圆点：逐帧拟合角度   橙/蓝虚线：窗口均值   '
                  '灰虚线：实际角参考   浅色区：本组采集窗口' if live else
                  '橙方块：原始测量角   蓝圆点：拟合后角度   '
                  '灰竖线：原始逐帧范围   灰虚线：理想 45°')
        canvas.create_text(right, 62, anchor='ne', fill='#71818c', text=legend)


class AngleChartWindow:
    """Live IoT angle monitor plus the stored-group comparison chart.

    Live mode is fed by the frames the IoT sessions push while a capture runs,
    so the operator watches the real sensor data and the current fit together
    instead of waiting for the 60 s group to finish. Both modes read the app's
    current state on every repaint, so the single window always shows whatever
    the main window is working on.
    """

    MODE_LIVE = '实时逐帧（IoT 实时数据）'
    MODE_GROUPS = '角度组均值对比'
    LIVE_WINDOWS = (('最近 10 秒', 10.0), ('最近 30 秒', 30.0), ('全部', None))
    AXIS_CHOICES = ('实际角（参考）', '原始测量角')

    def __init__(self, app, mode=None):
        tk, ttk = app.tk, app.ttk
        self.app = app
        self.tk, self.ttk = tk, ttk
        self.spread_points = None
        self.closed, self.dirty, self.last_draw, self.timer = False, True, 0.0, None
        self.rows_cache, self.summary_cache = [], {}
        self.group_vars = {
            'raw': tk.BooleanVar(value=True),
            'fit': tk.BooleanVar(value=True),
            'spread': tk.BooleanVar(value=False),
            'ideal': tk.BooleanVar(value=True),
        }
        self.live_vars = {
            'raw': tk.BooleanVar(value=True),
            'fit': tk.BooleanVar(value=True),
            'mean': tk.BooleanVar(value=True),
            'reference': tk.BooleanVar(value=True),
        }
        self.axis = tk.StringVar(value=self.AXIS_CHOICES[0])
        self.live_window = tk.StringVar(value=self.LIVE_WINDOWS[0][0])
        self.mode = tk.StringVar(value=mode or self.MODE_LIVE)
        self.info_source, self.info_target = tk.StringVar(), tk.StringVar()
        self.fit_detail, self.status = tk.StringVar(), tk.StringVar()
        self.window = tk.Toplevel(app.root)
        self.window.geometry('1180x820')
        self.window.minsize(900, 600)
        self.window.transient(app.root)
        self.make_ui()
        self.window.protocol('WM_DELETE_WINDOW', self.close)
        self.mode_changed()
        self.tick()

    def make_ui(self):
        tk, ttk = self.tk, self.ttk
        outer = ttk.Frame(self.window, padding=10)
        outer.pack(fill='both', expand=True)
        info = ttk.LabelFrame(outer, text='当前选中模块与拟合函数', padding=8)
        info.pack(fill='x')
        for row, (label, variable) in enumerate((('来源模块：', self.info_source),
                                                 ('测得模块：', self.info_target),
                                                 ('当前拟合：', self.fit_detail))):
            ttk.Label(info, text=label).grid(row=row, column=0, sticky='nw')
            ttk.Label(info, textvariable=variable, wraplength=1040, justify='left').grid(
                row=row, column=1, sticky='w')

        mode = ttk.Frame(outer)
        mode.pack(fill='x', pady=(8, 0))
        ttk.Label(mode, text='视图').pack(side='left')
        for choice in (self.MODE_LIVE, self.MODE_GROUPS):
            ttk.Radiobutton(mode, text=choice, value=choice, variable=self.mode,
                            command=self.mode_changed).pack(side='left', padx=(10, 0))

        self.control_holder = ttk.Frame(outer)
        self.control_holder.pack(fill='x', pady=(6, 0))
        self.live_controls = ttk.Frame(self.control_holder)
        ttk.Checkbutton(self.live_controls, text='逐帧原始测量角',
                        variable=self.live_vars['raw'],
                        command=self.redraw).pack(side='left')
        ttk.Checkbutton(self.live_controls, text='逐帧拟合后角度',
                        variable=self.live_vars['fit'],
                        command=self.redraw).pack(side='left', padx=(12, 0))
        ttk.Checkbutton(self.live_controls, text='窗口均值', variable=self.live_vars['mean'],
                        command=self.redraw).pack(side='left', padx=(12, 0))
        ttk.Checkbutton(self.live_controls, text='实际角参考线',
                        variable=self.live_vars['reference'],
                        command=self.redraw).pack(side='left', padx=(12, 0))
        ttk.Label(self.live_controls, text='时间窗口').pack(side='left', padx=(18, 4))
        window_box = ttk.Combobox(self.live_controls, state='readonly', width=11,
                                  textvariable=self.live_window,
                                  values=[label for label, unused in self.LIVE_WINDOWS])
        window_box.pack(side='left')
        window_box.bind('<<ComboboxSelected>>', lambda unused: self.redraw())

        self.group_controls = ttk.Frame(self.control_holder)
        ttk.Checkbutton(self.group_controls, text='原始测量角（组均值）',
                        variable=self.group_vars['raw'],
                        command=self.redraw).pack(side='left')
        ttk.Checkbutton(self.group_controls, text='拟合后角度', variable=self.group_vars['fit'],
                        command=self.redraw).pack(side='left', padx=(12, 0))
        ttk.Checkbutton(self.group_controls, text='原始逐帧范围（最小–最大）',
                        variable=self.group_vars['spread'],
                        command=self.toggle_spread).pack(side='left', padx=(12, 0))
        ttk.Checkbutton(self.group_controls, text='理想 45° 参考线',
                        variable=self.group_vars['ideal'],
                        command=self.redraw).pack(side='left', padx=(12, 0))
        ttk.Label(self.group_controls, text='x 轴').pack(side='left', padx=(18, 4))
        axis = ttk.Combobox(self.group_controls, state='readonly', width=13,
                            textvariable=self.axis, values=list(self.AXIS_CHOICES))
        axis.pack(side='left')
        axis.bind('<<ComboboxSelected>>', lambda unused: self.redraw())

        bottom = ttk.Frame(outer)
        bottom.pack(side='bottom', fill='x', pady=(8, 0))
        ttk.Button(bottom, text='导出当前视图 CSV', command=self.export_csv).pack(side='left')
        ttk.Button(bottom, text='关闭', command=self.close).pack(side='right')
        ttk.Label(bottom, textvariable=self.status, wraplength=900).pack(side='left', padx=10)

        chart_frame = ttk.Frame(outer)
        chart_frame.pack(fill='both', expand=True, pady=(8, 0))
        self.chart = AngleChart(self.tk, chart_frame)

    def mode_changed(self):
        if self.mode.get() == self.MODE_LIVE:
            self.group_controls.pack_forget()
            self.live_controls.pack(fill='x')
        else:
            self.live_controls.pack_forget()
            self.group_controls.pack(fill='x')
        self.redraw()

    def accept(self, unused_sample):
        """Called by the app for every live frame; the timer does the painting."""
        self.dirty = True

    def tick(self):
        """Repaint in batches so a fast sensor stream stays smooth."""
        if self.closed:
            return
        now = time.monotonic()
        # Repaint when new frames arrived, plus twice a second during a capture
        # so the countdown and the rolling time window keep moving.
        if self.dirty or (self.app.capture and now - self.last_draw >= 0.5):
            self.dirty = False
            self.last_draw = now
            self.redraw()
        self.timer = self.window.after(LIVE_REFRESH_MS, self.tick)

    def live_options(self):
        return {
            'raw': self.live_vars['raw'].get(), 'fit': self.live_vars['fit'].get(),
            'mean': self.live_vars['mean'].get(),
            'reference': self.live_vars['reference'].get(),
            'lock_key': self.app.capture['id'] if self.app.capture else None,
            'capture_span': self.app.live_span,
        }

    def group_options(self):
        return {
            'raw': self.group_vars['raw'].get(), 'fit': self.group_vars['fit'].get(),
            'spread': self.group_vars['spread'].get(), 'ideal': self.group_vars['ideal'].get(),
            'axis': 'measured' if self.axis.get() == self.AXIS_CHOICES[1] else 'actual',
            'spread_points': self.spread_points or {},
        }

    def redraw(self):
        if self.closed:
            return
        fit, code = self.app.current_fit()
        data = self.app.loaded or {}
        self.info_source.set(module_text(data.get('source')))
        self.info_target.set(module_text(data.get('target')))
        if fit:
            self.fit_detail.set('%s：拟合后角度 = %.6f %+.6f × 原始测量角'
                                '（n=%d，r=%s，RMSE=%.4f°，Syhat=%.4f）' %
                                (code or '未知', fit['a'], fit['b'], fit['n'],
                                 '—' if fit['r'] is None else '%.6f' % fit['r'],
                                 fit['rmse_deg'], fit['Syhat']))
        else:
            self.fit_detail.set('尚无拟合结果：完成至少 3 个角度组后才会出现拟合后角度曲线。')
        if self.mode.get() == self.MODE_LIVE:
            live_window = dict(self.LIVE_WINDOWS)[self.live_window.get()]
            built = live_series(self.app.live_samples, fit, live_window)
            self.rows_cache, self.summary_cache = built['rows'], built['summary']
            self.chart.show_live(built['rows'], built['summary'], fit, code,
                                 self.live_options())
            self.update_live_status(built['summary'])
        else:
            self.rows_cache = comparison_series(data.get('points') or [], fit)
            self.summary_cache = {}
            self.chart.show(self.rows_cache, fit, code, self.group_options())
            self.status.set('%d 组角度均值；橙方块为原始测量角，蓝圆点为按当前拟合函数换算后的角度。'
                            % len(self.rows_cache))
        self.window.title('%s · %s' % ('实时角度图' if self.mode.get() == self.MODE_LIVE
                                       else '角度组均值对比图', code or '当前记录'))

    def update_live_status(self, summary):
        capture = self.app.capture
        if capture:
            remaining = max(0, int(math.ceil(capture['deadline'] - time.monotonic())))
            head = '采集中 %d° · 还剩 %d 秒' % (capture['actual'], remaining)
        elif not self.app.sessions:
            head = '未连接：请先在主窗口连接 IoT 模块，再开始采集'
        else:
            head = '已连接，等待开始下一组采集'
        self.status.set('%s · 帧 %d · 有效 %d · 缺测 %.1f%% · 窗口均值 %s · 拟合后均值 %s' % (
            head, summary.get('frames', 0), summary.get('valid', 0),
            summary.get('missing_pct', 0.0),
            '—' if summary.get('mean_raw') is None else '%.3f°' % summary['mean_raw'],
            '—' if summary.get('mean_fitted') is None else '%.3f°' % summary['mean_fitted']))

    def toggle_spread(self):
        if self.group_vars['spread'].get() and self.spread_points is None:
            try:
                raw_frames = self.app.current_directory() / 'observations.csv'
                self.spread_points = frame_extremes(raw_frames)
            except (OSError, ValueError) as error:
                self.group_vars['spread'].set(False)
                self.spread_points = {}
                self.app.messagebox.showerror('原始逐帧数据读取失败', str(error),
                                              parent=self.window)
                return
            total = sum(count for unused_low, unused_high, count in self.spread_points.values())
            self.status.set('已读取 %d 个原始角度样本；灰竖线为该组逐帧最小–最大范围。' % total)
        self.redraw()

    def export_csv(self):
        from tkinter import filedialog
        live = self.mode.get() == self.MODE_LIVE
        target = filedialog.asksaveasfilename(
            parent=self.window, title='导出角度数据', defaultextension='.csv',
            initialfile='%s_%s.csv' % (self.app.current_fit()[1] or 'angles',
                                       'live' if live else 'compare'),
            filetypes=[('CSV', '*.csv')])
        if not target:
            return
        if live:
            fields = ['elapsed_s', 'measured_deg', 'fitted_deg', 'reference_angle_deg']
        else:
            fields = ['actual_angle_deg', 'measured_mean_deg', 'fitted_angle_deg']
        try:
            with open(target, 'w', encoding='utf-8-sig', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
                writer.writeheader()
                for row in self.rows_cache:
                    writer.writerow({key: ('' if row.get(key) is None else row.get(key))
                                     for key in fields})
        except OSError as error:
            self.app.messagebox.showerror('导出失败', str(error), parent=self.window)
            return
        self.status.set('已导出 %d 行：%s' % (len(self.rows_cache), target))

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.timer:
            try:
                self.window.after_cancel(self.timer)
            except self.tk.TclError:
                pass
            self.timer = None
        self.app.live_window = None
        self.window.destroy()


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
        # Live frame stream behind the real-time monitor: compact
        # (elapsed_s, measured_deg, reference_deg) tuples plus the clock they are
        # relative to, so the chart can start as soon as the first frame lands.
        self.live_samples = []
        self.live_origin = None
        self.live_span = None
        self.live_window = None
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
        # Group the three session actions in one strip so the live chart button
        # sits directly after 停止并保存 without colliding with the right-aligned
        # status text.
        actions = ttk.Frame(setup)
        actions.grid(row=1, column=0, columnspan=4, sticky='w', pady=(8, 2))
        ttk.Button(actions, text='连接 IoT / 新建记录', command=self.connect).pack(side='left')
        ttk.Button(actions, text='停止并保存', command=self.disconnect).pack(side='left', padx=6)
        ttk.Button(actions, text='实时角度图', command=self.open_angle_chart).pack(side='left')
        self.connection_status = tk.StringVar(value='未连接')
        ttk.Label(setup, textvariable=self.connection_status, wraplength=1250).grid(
            row=2, column=0, columnspan=4, sticky='w', padx=6)

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
            # A new session starts the live monitor clock at zero and drops the
            # previous stream, so the chart never mixes two records.
            self.live_samples = []
            self.live_origin = time.monotonic()
            self.live_span = None
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

    def current_fit(self):
        """Return the fit and its code for whatever record is currently loaded."""
        data = self.loaded or {}
        fit = data.get('fit')
        code = data.get('fit_code', '')
        if not code and data.get('source') and data.get('target'):
            code = fit_code(data['source'], data['target'])
        return fit, code

    def session_uids(self):
        """Source/target UIDs of the running session, or (None, None)."""
        if not self.store:
            return None, None
        return self.store.data['source']['uid'], self.store.data['target']['uid']

    def publish_live(self, stamp, measured):
        """Push one frame to the real-time monitor.

        Every frame of the selected source module is forwarded while a session
        is connected, not only during the 60 s capture, so the operator can watch
        the sensor live while lining the vehicle up.
        """
        if self.live_origin is None:
            self.live_origin = stamp
        capture = self.capture
        if capture:
            reference = capture['actual']
        else:
            index = self.actual_angle.current()
            reference = ANGLES[index] if 0 <= index < len(ANGLES) else None
        self.live_samples.append((stamp - self.live_origin, measured, reference))
        if len(self.live_samples) > LIVE_SAMPLE_LIMIT:
            del self.live_samples[:len(self.live_samples) - LIVE_SAMPLE_LIMIT]
        if self.live_window and not self.live_window.closed:
            self.live_window.accept(self.live_samples[-1])

    def open_angle_chart(self):
        """Open the live angle monitor for the selected module.

        The live view charts the frames the IoT modules report while the operator
        watches, with the current fit applied to every frame. The stored group
        means stay available from the view switch for reviewing a finished record.
        """
        if not self.loaded and not self.live_samples:
            self.messagebox.showerror('不能显示角度图',
                                      '当前没有采集或历史记录；请先连接 IoT 采集，或打开一条历史记录。',
                                      parent=self.root)
            return
        if self.live_window and self.live_window.window.winfo_exists():
            self.live_window.window.deiconify()
            self.live_window.window.lift()
            self.live_window.window.focus_set()
            return
        default = AngleChartWindow.MODE_LIVE
        if not self.sessions and self.loaded and not self.loaded.get('fit'):
            default = AngleChartWindow.MODE_GROUPS
        self.live_window = AngleChartWindow(self, default)

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
        # Shade the 60 s window of this group on the live chart.
        if self.live_origin is not None:
            self.live_span = (started - self.live_origin,
                              started + CAPTURE_SECONDS - self.live_origin)
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
        source_uid, target_uid = self.session_uids()
        if source_uid is None or int(data.get('uid', -1)) != source_uid:
            return
        stamp = float(data.get('host_monotonic', time.monotonic()))
        node = next((item for item in data.get('nodes', [])
                     if int(item.get('uid', -1)) == target_uid), None)
        present = node is not None
        angle = node.get('horizontal') if node else None
        distance = node.get('distance') if node else None
        valid_angle = finite(angle)
        # The live monitor sees every frame; only the capture window is stored.
        self.publish_live(stamp, float(angle) if valid_angle else None)
        capture = self.capture
        if (not capture or capture['source_uid'] != source_uid
                or stamp < capture['started'] or stamp > capture['deadline']):
            return
        capture['frames'] += 1
        if present:
            capture['present'] += 1
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
    # The comparison chart must apply the same a/b to the raw group means.
    series = comparison_series(
        [{'actual_angle_deg': 5, 'measured_mean_deg': 14.8223},
         {'actual_angle_deg': 0, 'measured_mean_deg': 6.5563},
         {'actual_angle_deg': 10, 'measured_mean_deg': None}], fit)
    assert [row['actual_angle_deg'] for row in series] == [0, 5]
    assert abs(series[1]['fitted_angle_deg'] - (fit['a'] + fit['b'] * 14.8223)) < 1e-9
    assert comparison_series([{'actual_angle_deg': 0}], fit) == []
    # The live stream keeps invalid frames as gaps and applies the same a/b.
    stream = live_series([(0.0, 6.5563, 0), (0.1, None, 0), (0.2, 14.8223, 5)], fit)
    assert [row['raw'] for row in stream['rows']] == [6.5563, None, 14.8223]
    assert [row['fitted'] for row in stream['rows']][0] == fit['a'] + fit['b'] * 6.5563
    assert stream['rows'][1]['fitted'] is None
    assert stream['summary']['frames'] == 3 and stream['summary']['valid'] == 2
    assert abs(stream['summary']['missing_pct'] - 100.0 / 3.0) < 1e-9
    assert stream['summary']['reference_angle_deg'] == 5
    windowed = live_series([(0.0, 6.5, 0), (30.0, 14.8, 0), (31.0, 15.0, 0)], fit, 10.0)
    assert [row['x'] for row in windowed['rows']] == [30.0, 31.0]
    assert live_series([], fit)['rows'] == []
    assert live_series([], fit)['summary']['frames'] == 0
    print('IOT calibration checks passed: modules, supplied linear fit, error band, '
          'raw-versus-fitted comparison series and live frame stream.')


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
