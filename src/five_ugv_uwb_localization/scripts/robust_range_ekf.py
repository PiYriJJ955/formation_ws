#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Planar range EKF with fixed noise, robust innovations and link hysteresis."""
from __future__ import division
from collections import deque
import math
import numpy as np


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class RangeEKF(object):
    def __init__(self, config):
        self.config = config
        self.anchors = {int(a['id']): np.array([a['x'], a['y'], a['z']], dtype=float)
                        for a in config['anchors']}
        self.min_anchors = int(config.get('min_anchors', 4))
        self.tag_height = float(config.get('tag_height', 0.25))
        self.lever = np.array(config.get('tag_offset_xy', [0.0, 0.0]), dtype=float)
        self.sigma = float(config.get('range_sigma', 0.20))
        self.biases = {int(k): float(v) for k, v in config.get('range_biases', {}).items()}
        self.sigmas = {int(k): float(v) for k, v in config.get('range_stddevs', {}).items()}
        self.gate = float(config.get('innovation_gate_sigma', 3.0))
        self.huber = float(config.get('innovation_huber_sigma', 1.5))
        self.bad_frames = int(config.get('link_bad_frames', 3))
        self.good_frames = int(config.get('link_recovery_frames', 15))
        self.recovery_step = float(config.get('link_recovery_step', 0.05))
        self.init_frames = int(config.get('initialization_frames', 8))
        self.init_spread = float(config.get('initialization_spread', 0.15))
        self.max_rms = float(config.get('valid_max_residual_rms', 0.35))
        self.max_condition = float(config.get('max_geometry_condition', 20.0))
        self.max_correction = float(config.get('max_position_correction', 0.06))
        self.max_stddev = float(config.get('max_position_stddev', 0.35))
        self.systematic_stddev = float(config.get('systematic_position_stddev', 0.10))
        self.position_noise = float(config.get('position_process_noise', 0.025))
        self.yaw_noise = float(config.get('yaw_process_noise', 0.015))
        self.wheel_noise = float(config.get('wheel_speed_stddev', 0.03))
        self.gyro_noise = float(config.get('gyro_stddev', 0.025))
        self.initial_yaw = float(config.get('initial_uwb_yaw', 0.0))
        numbers = [self.sigma, self.gate, self.huber, self.max_rms, self.max_condition,
                   self.max_correction, self.max_stddev, self.init_spread,
                   self.position_noise, self.yaw_noise, self.wheel_noise, self.gyro_noise]
        if (len(self.anchors) != len(config['anchors']) or self.min_anchors < 4 or
                len(self.anchors) < self.min_anchors or self.lever.shape != (2,) or
                not np.all(np.isfinite(self.lever)) or
                any(not np.all(np.isfinite(a)) for a in self.anchors.values()) or
                any(not np.isfinite(v) or v <= 0 for v in numbers + list(self.sigmas.values())) or
                any(not np.isfinite(v) for v in list(self.biases.values()) + [self.tag_height, self.initial_yaw]) or
                not 0 <= self.systematic_stddev < self.max_stddev or
                self.bad_frames < 1 or self.good_frames < 1 or self.init_frames < 2 or
                not 0 < self.recovery_step <= 1 or self.huber > self.gate or
                not set(self.biases).issubset(self.anchors) or not set(self.sigmas).issubset(self.anchors)):
            raise ValueError('Invalid EKF geometry, noise or quality settings')
        self.links = {i: {'bad': 0, 'good': 0, 'quarantined': False, 'weight': 1.0}
                      for i in self.anchors}
        self.x = self.P = None
        self.candidates = deque(maxlen=self.init_frames)

    def model(self, aid, state=None):
        state = self.x if state is None else state
        c, s = math.cos(state[2]), math.sin(state[2])
        lx, ly = self.lever
        offset = np.array([c*lx-s*ly, s*lx+c*ly])
        d = np.r_[state[:2] + offset, self.tag_height] - self.anchors[aid]
        distance = max(float(np.linalg.norm(d)), 1e-6)
        yaw_derivative = np.array([-s*lx-c*ly, c*lx-s*ly])
        H = np.array([d[0]/distance, d[1]/distance, d[:2].dot(yaw_derivative)/distance])
        return distance, H

    def ranges(self, measurements):
        result = {}
        for aid, value in measurements.items():
            if aid not in self.anchors:
                continue
            value = float(value) - self.biases.get(aid, 0.0)
            if (np.isfinite(value) and float(self.config.get('min_valid_range', 1.05)) <= value <=
                    float(self.config.get('max_valid_range', 20.0)) and
                    value + 0.05 >= abs(self.tag_height - self.anchors[aid][2])):
                result[aid] = value
        return result

    def geometry(self, H):
        if len(H) < self.min_anchors:
            return float('inf')
        values = np.linalg.eigvalsh(np.asarray(H)[:, :2].T.dot(np.asarray(H)[:, :2]))
        return float(values[-1]/values[0]) if values[0] > 1e-8 else float('inf')

    def initialize(self, measurements):
        """Robust all-anchor bootstrap, accepted only after a stable sequence."""
        if len(measurements) < self.min_anchors:
            self.candidates.clear()
            return False
        ids = sorted(measurements)
        ref = self.anchors[ids[0]]
        A, b = [], []
        for aid in ids[1:]:
            a = self.anchors[aid]
            A.append(2*(a[:2]-ref[:2]))
            b.append(measurements[ids[0]]**2-measurements[aid]**2 + a[:2].dot(a[:2])-
                     ref[:2].dot(ref[:2])+(self.tag_height-a[2])**2-(self.tag_height-ref[2])**2)
        pos = np.linalg.lstsq(np.asarray(A), np.asarray(b), rcond=-1)[0]
        c, s = math.cos(self.initial_yaw), math.sin(self.initial_yaw)
        pos -= np.array([[c, -s], [s, c]]).dot(self.lever)
        state = np.r_[pos, self.initial_yaw]
        for _ in range(10):
            models = [self.model(i, state) for i in ids]
            H = np.array([v[1][:2] for v in models])
            residual = np.array([measurements[i]-v[0] for i, v in zip(ids, models)])
            sigmas = np.array([self.sigmas.get(i, self.sigma) for i in ids])
            weights = np.minimum(1.0, self.huber*sigmas/np.maximum(abs(residual), 1e-9))/sigmas**2
            delta = np.linalg.lstsq(H*np.sqrt(weights[:, None]), residual*np.sqrt(weights), rcond=-1)[0]
            state[:2] += delta * min(1.0, 0.5/max(float(np.linalg.norm(delta)), 1e-9))
            if np.linalg.norm(delta) < 1e-5:
                break
        # Bootstrap also requires a consistent set of at least four links.
        ids = [i for i in ids if abs(measurements[i]-self.model(i, state)[0]) <=
               self.gate*self.sigmas.get(i, self.sigma)]
        if len(ids) < self.min_anchors:
            self.candidates.clear()
            return False
        H = [self.model(i, state)[1] for i in ids]
        rms = math.sqrt(np.mean([(measurements[i]-self.model(i, state)[0])**2 for i in ids]))
        if not self.inside(state[:2]) or rms > self.max_rms or self.geometry(H) > self.max_condition:
            self.candidates.clear()
            return False
        self.candidates.append(state[:2].copy())
        if len(self.candidates) < self.init_frames:
            return False
        center = np.median(np.array(self.candidates), axis=0)
        if max(np.linalg.norm(v-center) for v in self.candidates) > self.init_spread:
            return False
        self.x = np.r_[center, self.initial_yaw]
        self.P = np.diag([0.10**2, 0.10**2, math.radians(5)**2])
        return True

    def inside(self, xy):
        margin = float(self.config.get('workspace_margin', 0.8))
        return (float(self.config.get('workspace_x_min', 0))-margin <= xy[0] <=
                float(self.config.get('workspace_x_max', 6.4))+margin and
                float(self.config.get('workspace_y_min', 0))-margin <= xy[1] <=
                float(self.config.get('workspace_y_max', 4.4))+margin)

    def predict(self, dt, vx, vy, omega):
        if self.x is None or dt <= 0:
            return
        angle = self.x[2] + omega*dt/2
        c, s = math.cos(angle), math.sin(angle)
        dx, dy = (c*vx-s*vy)*dt, (s*vx+c*vy)*dt
        F = np.eye(3)
        F[0, 2], F[1, 2] = -dy, dx
        self.x += np.array([dx, dy, omega*dt])
        self.x[2] = wrap(self.x[2])
        Q = np.diag([self.position_noise**2*dt+self.wheel_noise**2*dt**2]*2 +
                    [self.yaw_noise**2*dt+self.gyro_noise**2*dt**2])
        self.P = F.dot(self.P).dot(F.T) + Q

    def position_stddev(self):
        if self.P is None:
            return float('inf')
        return math.sqrt(max(np.linalg.eigvalsh(self.P[:2, :2])) + self.systematic_stddev**2)

    def update(self, measurements):
        measurements = self.ranges(measurements)
        result = {'status': 'INITIALIZING', 'accepted_ids': [], 'rejected_ids': [],
                  'quarantined_ids': [], 'innovations': {}, 'normalized_innovations': {},
                  'weights': {}, 'residual_rms': None, 'full_residual_rms': None,
                  'geometry_condition': None, 'valid': False}
        if self.x is None:
            if not self.initialize(measurements):
                return result
        ids, residuals, jacobians, variances = [], [], [], []
        for aid in sorted(self.anchors):
            health = self.links[aid]
            if aid not in measurements:
                health['good'] = 0
                result['rejected_ids'].append(aid)
                continue
            distance, H = self.model(aid)
            innovation = measurements[aid]-distance
            R = self.sigmas.get(aid, self.sigma)**2
            normalized = abs(innovation)/math.sqrt(max(float(H.dot(self.P).dot(H)) + R, 1e-12))
            result['innovations'][aid] = float(innovation)
            result['normalized_innovations'][aid] = float(normalized)
            if normalized > self.gate:
                health['bad'] += 1
                health['good'] = 0
                if health['bad'] >= self.bad_frames:
                    health['quarantined'], health['weight'] = True, self.recovery_step
                result['rejected_ids'].append(aid)
                continue
            health['bad'] = 0
            if health['quarantined']:
                health['good'] += 1
                if health['good'] < self.good_frames:
                    result['rejected_ids'].append(aid)
                    continue
                health['quarantined'] = False
            weight = min(1.0, self.huber/max(normalized, 1e-9)) * health['weight']
            health['weight'] = min(1.0, health['weight'] + self.recovery_step)
            result['weights'][aid] = float(weight)
            ids.append(aid); residuals.append(innovation); jacobians.append(H); variances.append(R/weight)
        result['quarantined_ids'] = [i for i, h in sorted(self.links.items()) if h['quarantined']]
        result['accepted_ids'] = ids
        condition = self.geometry(jacobians)
        result['geometry_condition'] = condition if np.isfinite(condition) else None
        if len(ids) < self.min_anchors:
            result['status'] = 'INSUFFICIENT_ANCHORS'
            return result
        if condition > self.max_condition:
            result['status'] = 'POOR_GEOMETRY'
            return result
        H, residual, R = np.array(jacobians), np.array(residuals), np.diag(variances)
        S = H.dot(self.P).dot(H.T) + R
        K = np.linalg.solve(S, H.dot(self.P)).T
        correction = K.dot(residual)
        proposed = self.x + correction
        proposed[2] = wrap(proposed[2])
        rms = math.sqrt(np.mean([(measurements[i]-self.model(i, proposed)[0])**2 for i in ids]))
        result['residual_rms'] = float(rms)
        result['full_residual_rms'] = float(math.sqrt(np.mean(
            [(measurements[i]-self.model(i, proposed)[0])**2 for i in measurements])))
        if (np.linalg.norm(correction[:2]) > self.max_correction or abs(correction[2]) > 0.15 or
                not self.inside(proposed[:2])):
            result['status'] = 'INCONSISTENT_POSITION'
            return result
        if rms > self.max_rms:
            result['status'] = 'HIGH_RESIDUAL'
            return result
        self.x = proposed
        I_KH = np.eye(3)-K.dot(H)
        self.P = I_KH.dot(self.P).dot(I_KH.T)+K.dot(R).dot(K.T)
        self.P = (self.P+self.P.T)/2
        result['valid'] = self.position_stddev() <= self.max_stddev
        result['status'] = 'TRACKING' if result['valid'] else 'UNCERTAIN'
        return result


class FrameClock(object):
    """Unwrap tag-local milliseconds; reject duplicates and relative backlog.

    The earliest receipt estimates the clock offset. Constant transport latency
    and individual anchor sample ages are unobservable in Nodeframe2.
    """
    def __init__(self):
        self.last = self.received = self.elapsed = self.offset = None

    def accept(self, ticks, received):
        ticks = int(ticks)
        reset = False
        if self.last is None:
            self.elapsed, self.offset = 0.0, received
        else:
            delta = (ticks-self.last) % (2**32)
            if delta == 0:
                return None
            if delta >= 2**31:
                if received-self.received < 1.0:
                    return None
                self.elapsed, self.offset, reset = 0.0, received, True
            else:
                self.elapsed += delta/1000.0
                # Slowly follow oscillator drift; never conceal a sudden backlog.
                self.offset = min(received-self.elapsed, self.offset + 0.0001*max(0, received-self.received))
        self.last, self.received = ticks, received
        return self.elapsed+self.offset, reset


def predict_motion(ekf, start, end, histories, timeout):
    """Causal zero-order integration of timestamped body wheel speed and gyro.

    Returns false on a missing interval. Caller discards the partial candidate.
    """
    if end < start or end-start > 5.0:
        return False
    if ekf.x is None:
        start = end
    boundaries = sorted(set([start, end] + [t for samples in histories.values()
                                             for t, _ in samples if start < t < end]))
    for index, t in enumerate(boundaries):
        values = {}
        for kind in ('odom', 'imu'):
            sample = next(((stamp, value) for stamp, value in reversed(histories[kind]) if stamp <= t+1e-6), None)
            if sample is None or t-sample[0] > timeout:
                return False
            values[kind] = sample[1]
        if index == len(boundaries)-1:
            break
        vx, vy, wheel_omega = values['odom']
        omega = values['imu'][0] - float(ekf.config.get('gyro_bias_z', 0.0))
        # Zero angular rate only when independent wheel and gyro readings agree
        # that the base is stationary; fixed thresholds, no noise adaptation.
        if max(abs(vx), abs(vy)) < 0.008 and abs(wheel_omega) < 0.015 and abs(omega) < 0.035:
            omega = 0.0
        ekf.predict(boundaries[index+1]-t, vx, vy, omega)
    return True
