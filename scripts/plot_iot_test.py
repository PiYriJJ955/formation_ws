#!/usr/bin/env python3
"""Plot NIU_B01 distance and horizontal angle from the fleet test CSV files."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

LINKS = [
    (0x2A003200, 0x15003B00, 'ugv3 左板 → ugv2'),
    (0x15003B00, 0x2A003200, 'ugv2 → ugv3 左板'),
    (0x30006E00, 0x30003000, 'ugv3 右板 → ugv4'),
    (0x30003000, 0x30006E00, 'ugv4 → ugv3 右板'),
    (0x15003B00, 0x30003000, 'ugv2 → ugv4'),
    (0x30003000, 0x15003B00, 'ugv4 → ugv2'),
]

HORIZONTAL_LIMIT_DEG = 50.  # IOT_User_Manual_V1.0, section 4.1.2.


def make_series(frames, measurements, source, target):
    """Keep a slot for every source frame so missing peers break the curve."""
    selected = [r for r in frames if int(r['source_uid']) == source]
    if not selected:
        raise ValueError('No frames for UID 0x%08X' % source)
    selected.sort(key=lambda r: int(r['frame_index']))
    ids = [int(r['frame_index']) for r in selected]
    if len(set(ids)) != len(ids):
        raise ValueError('Duplicate source frame indices')
    system_time = np.array([int(r['system_time']) for r in selected], dtype=np.int64)
    if np.any(np.diff(system_time) <= 0):
        raise ValueError('Device clock repeated, wrapped or restarted')
    index = {frame_id: i for i, frame_id in enumerate(ids)}
    present = np.zeros(len(ids), dtype=bool)
    distance = np.full(len(ids), np.nan)
    angle = np.full(len(ids), np.nan)
    for row in measurements:
        if int(row['source_uid']) == source and int(row['target_uid']) == target:
            i = index[int(row['frame_index'])]
            if present[i]:
                raise ValueError('Duplicate peer in source frame')
            if int(row['system_time']) != system_time[i]:
                raise ValueError('Measurement and frame timestamps disagree')
            present[i] = True
            distance[i], angle[i] = float(row['distance_m']), float(row['horizontal_deg'])
    if not present.any():
        raise ValueError('No observations for 0x%08X -> 0x%08X' % (source, target))
    if not (np.isfinite(distance[present]).all() and np.isfinite(angle[present]).all()):
        raise ValueError('Nonfinite observation; inspect raw CSV before plotting')
    return dict(t=(system_time - system_time[0]) / 1000., present=present,
                distance_m=distance, horizontal_deg=angle,
                within_horizontal_range=present & (np.abs(angle) <= HORIZONTAL_LIMIT_DEG))


def self_test():
    frames = [dict(source_uid='1', frame_index=str(i), system_time=str(1000+100*i)) for i in range(3)]
    rows = [dict(frames[i], target_uid='2', distance_m=str(.5+i*.1), horizontal_deg=str(-4+i))
            for i in (0, 2)]
    rows.append(dict(frames[1], target_uid='3', distance_m='99', horizontal_deg='88'))
    result = make_series(frames, rows, 1, 2)
    assert result['present'].tolist() == [True, False, True]
    assert np.isnan(result['distance_m'][1])
    np.testing.assert_allclose(result['t'], [0, .1, .2])
    np.testing.assert_allclose(result['horizontal_deg'][[0, 2]], [-4, -2])
    try:
        make_series(frames, rows + [rows[0]], 1, 2)
    except ValueError:
        pass
    else:
        raise AssertionError('duplicate observation accepted')
    frames = [dict(source_uid='1', frame_index=str(i), system_time=str(100*i)) for i in range(4)]
    rows = [dict(frames[i], target_uid='2', distance_m='.5', horizontal_deg=str(value))
            for i, value in enumerate([-50.,50.,50.01,-50.01])]
    assert make_series(frames, rows, 1, 2)['within_horizontal_range'].tolist() == [True,True,False,False]
    print('PASS: UID selection, frame alignment, missing data, duplicate rejection, manual angle limits')


def main(directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties

    font_path = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
    if Path(font_path).exists():
        plt.rcParams['font.family'] = FontProperties(fname=font_path).get_name()
    plt.rcParams.update({'axes.unicode_minus': False, 'font.size': 11, 'axes.spines.top': False,
                         'axes.spines.right': False, 'savefig.facecolor': 'white', 'pdf.fonttype': 42})
    frames, measurements = [], []
    for robot in ('ugv2', 'ugv3', 'ugv4'):
        for filename, dest in [('frames.csv', frames), ('measurements.csv', measurements)]:
            with (directory / robot / filename).open() as stream:
                dest.extend(csv.DictReader(stream))
    output = directory / 'plots'
    output.mkdir(exist_ok=True)
    series, summaries, exported = [], [], []
    for source, target, label in LINKS:
        data = make_series(frames, measurements, source, target)
        series.append(data)
        present = data['present']
        summary = dict(link=label, source_uid=source, target_uid=target,
                       frames=len(present), samples=int(present.sum()),
                       presence_pct=float(present.mean()*100), device_span_s=float(data['t'][-1]),
                       max_observation_gap_s=float(np.max(np.diff(data['t'][present]))),
                       nonpositive_distance_count=int(np.sum(data['distance_m'][present] <= 0)),
                       horizontal_within_range_count=int(data['within_horizontal_range'].sum()),
                       horizontal_out_of_range_count=int(np.sum(present & ~data['within_horizontal_range'])))
        for field in ('distance_m', 'horizontal_deg'):
            values = data[field][present]
            for name, value in [('mean', np.mean(values)), ('std', np.std(values)),
                                ('min', np.min(values)), ('max', np.max(values)),
                                ('p05', np.percentile(values, 5)), ('p95', np.percentile(values, 95))]:
                summary[field + '_' + name] = float(value)
        summaries.append(summary)
        for i, t in enumerate(data['t']):
            exported.append(dict(link=label, source_uid=source, target_uid=target,
                                 t_device_s=t, present=int(present[i]),
                                 distance_m=data['distance_m'][i], horizontal_deg=data['horizontal_deg'][i],
                                 within_horizontal_range=int(data['within_horizontal_range'][i])))
    for name, rows in [('selected_links.csv', exported), ('selected_summary.csv', summaries)]:
        with (directory / name).open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    (directory / 'selected_summary.json').write_text(json.dumps(summaries, ensure_ascii=False, indent=2)+'\n')

    colors = ['#1666a8', '#d36b20']
    titles = ['左侧链路：ugv3 左板（0x2A003200） ↔ ugv2',
              '右侧链路：ugv3 右板（0x30006E00） ↔ ugv4', '跨车链路：ugv2 ↔ ugv4']
    fig, axes = plt.subplots(3, 3, figsize=(20, 10), sharex='col', gridspec_kw={'height_ratios': [1, 1, .65]})
    for column in range(3):
        for direction in range(2):
            i = column*2 + direction
            data, summary = series[i], summaries[i]
            color, label = colors[direction], LINKS[i][2]
            for row, field in enumerate(('distance_m', 'horizontal_deg')):
                axes[row, column].plot(data['t'], data[field], color=color, lw=.55, marker='.',
                                       markersize=1.7, alpha=.8, label=label)
            outside = data['present'] & ~data['within_horizontal_range']
            axes[1, column].scatter(data['t'][outside], data['horizontal_deg'][outside], s=14,
                                    facecolors='none', edgecolors='#b71c1c', linewidths=.7, zorder=5)
            edges = np.arange(0, data['t'][-1]+5, 5.)
            totals, _ = np.histogram(data['t'], bins=edges)
            hits, _ = np.histogram(data['t'][data['present']], bins=edges)
            axes[2, column].plot((edges[:-1]+edges[1:])/2, 100*hits/totals,
                                 color=color, lw=1.3, marker='o', markersize=3,
                                 label='%s：%.1f%% (%d/%d)' %
                                 (label, summary['presence_pct'], summary['samples'], summary['frames']))
        axes[0, column].set_title(titles[column], fontsize=13, pad=12)
        for row in range(3):
            axes[row, column].grid(alpha=.2)
        axes[0, column].legend(loc='upper right', fontsize=9, framealpha=.85)
        axes[2, column].legend(loc='lower left', fontsize=8.5, framealpha=.85)
        axes[2, column].set_ylim(0, 105)
        axes[2, column].set_xlabel('各来源设备从首帧起的经过时间（s）')
        axes[0, column].set_ylabel('距离（m）')
        axes[1, column].set_ylabel('设备局部水平角（°）')
        axes[2, column].set_ylabel('每 5 s 对端出现率（%）')
    fig.suptitle('NIU_B01 六条有向链路 · 静止测距与水平角', fontsize=18, y=.99)
    fig.text(.5, .025, '原始输出；缺测保留为空隙。手册有效测角范围 [-50°, 50°]，超出范围的角度用红圈标记。\n各设备时间轴独立归零，未做跨设备时钟同步。', ha='center', color='#555555')
    fig.tight_layout(rect=[0, .05, 1, .955])
    for extension in ('png', 'pdf', 'svg'):
        fig.savefig(output / ('six_links_timeseries.'+extension), dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(2, 3, figsize=(20, 8))
    for column in range(3):
        for row, field in enumerate(('distance_m', 'horizontal_deg')):
            values = [series[column*2+d][field][series[column*2+d]['present']] for d in range(2)]
            bins = np.histogram_bin_edges(np.concatenate(values), bins=30)
            for direction in range(2):
                i = column*2+direction
                summary = summaries[i]
                label = '%s\n均值 %.3f，标准差 %.3f' % (LINKS[i][2], summary[field+'_mean'], summary[field+'_std'])
                axes[row, column].hist(values[direction], bins=bins, density=True, histtype='step',
                                        lw=1.7, color=colors[direction], label=label)
            axes[row, column].set_xlabel('距离（m）' if row == 0 else '设备局部水平角（°）')
            axes[row, column].set_ylabel('概率密度')
            axes[row, column].grid(alpha=.2)
            axes[row, column].legend(fontsize=9)
        axes[0, column].set_title(titles[column], fontsize=13, pad=12)
    fig.suptitle('NIU_B01 六条有向链路 · 静止数据分布', fontsize=18, y=.99)
    fig.text(.5, .025, '标准差描述本次输出的散布；绝对精度需要独立距离与角度真值验证。', ha='center', color='#555555')
    fig.tight_layout(rect=[0, .05, 1, .95])
    for extension in ('png', 'pdf', 'svg'):
        fig.savefig(output / ('six_links_distributions.'+extension), dpi=180)
    plt.close(fig)
    print(json.dumps(summaries, ensure_ascii=False, indent=2))
    print('Plots:', output)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', nargs='?', type=Path)
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        self_test()
    else:
        if args.directory is None:
            parser.error('directory is required')
        main(args.directory)
