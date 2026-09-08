#!/usr/bin/env python
"""Polyline Pure Pursuit, used by fleet_ros's single velocity output (Python 2/3)."""
from __future__ import division
import csv
import json
import math
import os
import sys
import tempfile
import time


def finite(value):
    return not math.isnan(value) and not math.isinf(value)


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def parse_points(text):
    try:
        return validate_points([[float(v) for v in line.replace(',', ' ').split()]
                                for line in text.splitlines() if line.strip()])
    except (TypeError, ValueError):
        raise ValueError('PATH_POINTS')


def validate_points(points):
    if not isinstance(points, (list, tuple)) or not 2 <= len(points) <= 50:
        raise ValueError('PATH_POINTS')
    result = []
    for point in points:
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise ValueError('PATH_POINTS')
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or
               not finite(v) or abs(v) > 1000 for v in point):
            raise ValueError('PATH_POINTS')
        point = tuple(float(v) for v in point)
        if result and math.hypot(point[0]-result[-1][0], point[1]-result[-1][1]) < 0.15:
            raise ValueError('PATH_SEGMENT_SHORT')
        result.append(point)
    return result


def check_bounds(points, bounds, offsets=()):
    """Conservative footprint for all headings, including turns at vertices."""
    if bounds is None:
        return
    margin = 0.15 + max([math.hypot(*d) for d in offsets] or [0.0])
    xmin, ymin, xmax, ymax = bounds
    if any(not (xmin+margin <= x <= xmax-margin and ymin+margin <= y <= ymax-margin)
           for x, y in points):
        raise ValueError('PATH_OUTSIDE')


class LeaderTracker(object):
    ACTIVE = ('TRACKING', 'ALIGNING')

    def __init__(self):
        self.points, self.segments = [], []
        self.progress = self.total = 0.0
        self.state, self.reason = 'IDLE', ''
        self.v = self.w = 0.0
        self.last_pose = self.last_time = None
        self.target = self.reference = None
        self.cross_track = self.heading_error = 0.0
        self.acceleration, self.angular_acceleration = 0.15, 0.8

    def start(self, points, speed, lookahead, pose, now):
        points = validate_points(points)
        if not finite(speed) or not 0 < speed <= 0.5 or not finite(lookahead) or not 0.3 <= lookahead <= 0.5:
            raise ValueError('PATH_SETTINGS')
        if math.hypot(points[0][0]-pose[0], points[0][1]-pose[1]) > 0.5:
            raise ValueError('START_TOO_FAR')
        self.__init__()
        self.points, self.speed, self.lookahead = points, speed, lookahead
        for a, b in zip(points, points[1:]):
            length = math.hypot(b[0]-a[0], b[1]-a[1])
            self.segments.append((a, b, self.total, length))
            self.total += length
        self.resume(now)

    def resume(self, now):
        if not self.points or self.state in ('DONE', 'STOPPED'):
            raise ValueError('NO_PAUSED_PATH')
        self.state, self.reason = 'TRACKING', ''
        self.v = self.w = 0.0
        self.last_time, self.last_pose = now, None

    def pause(self, reason='USER_PAUSE'):
        self.state, self.reason = 'PAUSED', reason
        self.v = self.w = 0.0

    def stop(self):
        self.state, self.reason = 'STOPPED', ''
        self.v = self.w = 0.0

    def point_at(self, s):
        for a, b, start, length in self.segments:
            if s <= start+length:
                t = clamp((s-start)/length, 0.0, 1.0)
                return [a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1])]
        return list(self.points[-1])

    def step(self, pose, now, limit, followers=()):
        """followers contains (offset_x, offset_y, speed_limit) of enabled followers."""
        if self.state not in self.ACTIVE:
            return 0.0, 0.0
        x, y, yaw = pose
        if not all(finite(v) for v in pose):
            self.pause('INVALID_POSE')
            return 0.0, 0.0
        if self.last_pose is not None:
            px, py, at = self.last_pose
            if math.hypot(x-px, y-py) > 0.20 + 3*limit*max(0, now-at):
                self.pause('POSE_JUMP')
                return 0.0, 0.0
        self.last_pose = (x, y, now)
        dt = clamp(now-self.last_time, 0.0, 0.1)
        self.last_time = now
        if limit <= 0 or any(cap <= 0 for _, _, cap in followers):
            self.pause('ZERO_LIMIT')
            return 0.0, 0.0

        # Search only the local arc-length window; ties favour earlier segments.
        best = None
        low, high = max(0, self.progress-0.10), min(self.total, self.progress+self.lookahead)
        for a, b, start, length in self.segments:
            if start > high or start+length < low:
                continue
            t = ((x-a[0])*(b[0]-a[0])+(y-a[1])*(b[1]-a[1]))/(length*length)
            s = clamp(start+t*length, max(start, low), min(start+length, high))
            t = (s-start)/length
            px, py = a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1])
            distance = math.hypot(x-px, y-py)
            if best is None or distance < best[0]-1e-9:
                best = distance, s, [px, py]
        self.cross_track, s, self.reference = best
        if self.cross_track > 0.6:
            self.pause('PATH_ERROR')
            return 0.0, 0.0
        self.progress = max(self.progress, s)
        end_distance = math.hypot(x-self.points[-1][0], y-self.points[-1][1])
        if end_distance <= 0.10 and self.progress >= self.total-0.15:
            self.stop()
            self.state = 'DONE'
            return 0.0, 0.0
        self.target = self.point_at(min(self.total, self.progress+self.lookahead))
        dx, dy = self.target[0]-x, self.target[1]-y
        self.heading_error = wrap(math.atan2(dy, dx)-yaw)
        curvature = 2*(-math.sin(yaw)*dx+math.cos(yaw)*dy)/max(dx*dx+dy*dy, 1e-6)
        angular_limit = 0.5
        if abs(self.heading_error) > 0.7:
            self.state = 'ALIGNING'
            # In-place leader rotation moves followers at |omega| * |offset|.
            for ox, oy, cap in followers:
                radius = math.hypot(ox, oy)
                if radius > 1e-6:
                    angular_limit = min(angular_limit, 0.8*cap/radius)
            v, w = 0.0, clamp(1.5*self.heading_error, -angular_limit, angular_limit)
        else:
            self.state = 'TRACKING'
            hard_limit = limit
            for ox, oy, cap in followers:
                gain = math.hypot(1-curvature*oy, curvature*ox)
                hard_limit = min(hard_limit, 0.8*cap/max(1.0, gain))
            hard_limit = min(hard_limit, angular_limit/max(abs(curvature), 1e-6))
            speed = min(self.speed, hard_limit,
                        math.sqrt(2*self.acceleration*max(0, max(self.total-self.progress, end_distance)-0.08)))
            v = clamp(speed, self.v-self.acceleration*dt, self.v+self.acceleration*dt)
            v = clamp(v, 0.0, hard_limit)
            w = curvature*v
        self.v = v
        self.w = clamp(clamp(w, self.w-self.angular_acceleration*dt, self.w+self.angular_acceleration*dt),
                       -angular_limit, angular_limit)
        scale = 1.0
        for ox, oy, cap in followers:
            required = math.hypot(self.v-self.w*oy, self.w*ox)
            if required > 1e-9:
                scale = min(scale, 0.8*cap/required)
        self.v *= scale
        self.w *= scale
        return self.v, self.w

    def status(self):
        return dict(state=self.state, reason=self.reason, points=self.points, progress=self.progress,
                    total=self.total, target=self.target, reference=self.reference,
                    cross_track=self.cross_track, heading_error=self.heading_error,
                    linear=self.v, angular=self.w)


class TrackingLog(object):
    def __init__(self, root, metadata, now):
        if not os.path.isdir(root):
            os.makedirs(root)
        self.directory = tempfile.mkdtemp(prefix=time.strftime('%Y%m%d-%H%M%S-'), dir=root)
        with open(os.path.join(self.directory, 'path.json'), 'w') as stream:
            json.dump(metadata, stream, indent=2)
        self.stream = open(os.path.join(self.directory, 'tracking.csv'), 'wb' if sys.version_info[0] < 3 else 'w')
        self.writer = csv.writer(self.stream)
        self.writer.writerow(['t', 'x', 'y', 'yaw', 'progress', 'cross_track', 'heading_error',
                              'target_x', 'target_y', 'linear', 'angular', 'max_linear', 'state', 'reason'])
        self.started = self.flushed = now

    def record(self, now, pose, tracker, limit, command):
        self.writer.writerow([now-self.started] + list(pose or [None]*3) +
                             [tracker.progress, tracker.cross_track, tracker.heading_error] +
                             list(tracker.target or [None]*2) +
                             list(command) + [limit, tracker.state, tracker.reason])
        if now-self.flushed >= 1.0:
            self.stream.flush()
            self.flushed = now

    def close(self):
        self.stream.close()
