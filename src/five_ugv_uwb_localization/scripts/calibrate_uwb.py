#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Fit fixed per-anchor biases from stationary, independently measured tag positions.

Input manifest YAML: samples: [{csv: /path/raw_linktrack.csv, tag: [x,y,z]}, ...]
Use at least three distinct positions, separated by >= 0.5 m, under LOS.
"""
from __future__ import print_function
import argparse
import csv
import os
import numpy as np
import yaml


def calibrate(config, manifest, root='.'):
    samples = manifest['samples']
    points = np.array([s['tag'] for s in samples], dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.all(np.isfinite(points)):
        raise ValueError('Each sample requires a finite measured tag [x,y,z]')
    distinct = []
    for point in points:
        if all(np.linalg.norm(point-v) >= 0.5 for v in distinct):
            distinct.append(point)
    if len(distinct) < 3:
        raise ValueError('At least three reference positions separated by 0.5 m are required')
    errors = {int(a['id']): [] for a in config['anchors']}
    for sample, point in zip(samples, points):
        with open(os.path.join(root, sample['csv'])) as stream:
            rows = list(csv.DictReader(stream))
        for a in config['anchors']:
            aid = int(a['id'])
            expected = np.linalg.norm(point-np.array([a['x'], a['y'], a['z']]))
            values = [float(r['range_id%d' % aid]) for r in rows]
            values = [v-expected for v in values if np.isfinite(v) and v > 0]
            if len(values) < 30:
                raise ValueError('Need >=30 valid LOS samples per reference and anchor %d' % aid)
            errors[aid].append(values)
    biases, sigmas = {}, {}
    for aid, groups in errors.items():
        medians = np.array([np.median(v) for v in groups])
        bias = float(np.median(medians))
        if max(abs(medians-bias)) > 0.15:
            raise ValueError('Anchor %d has position-dependent bias; inspect LOS/reference measurements' % aid)
        scatter = [1.4826*np.median(abs(np.array(v)-np.median(v))) for v in groups]
        biases[aid] = bias
        sigmas[aid] = float(max(0.10, max(scatter), 1.4826*np.median(abs(medians-bias))))
    return {'range_biases': biases, 'range_stddevs': sigmas}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest')
    parser.add_argument('--config', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    with open(args.config) as stream:
        config = yaml.safe_load(stream)
    with open(args.manifest) as stream:
        manifest = yaml.safe_load(stream)
    result = calibrate(config, manifest, os.path.dirname(os.path.abspath(args.manifest)))
    if os.path.exists(args.output):
        parser.error('Output already exists; select a new file and review before installing')
    with open(args.output, 'w') as stream:
        yaml.safe_dump(result, stream, default_flow_style=False)
    print('Fixed calibration saved:', args.output)


if __name__ == '__main__':
    main()
