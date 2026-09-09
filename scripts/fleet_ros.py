#!/usr/bin/env python
"""Shared-master telemetry and leader deadman; compatible with ROS Melodic/Python 2."""
from __future__ import print_function
import argparse
import glob
import json
import math
import os
import select
import socket
import sys
import threading
import time

from fleet_bridge import monotonic, velocity
from leader_tracker import LeaderTracker, TrackingLog, check_bounds, finite, wrap, sample_path


def emit(data):
    print(json.dumps(data, allow_nan=False))
    sys.stdout.flush()


def master_proxy():
    try:
        from xmlrpc.client import ServerProxy
    except ImportError:
        from xmlrpclib import ServerProxy
    socket.setdefaulttimeout(2)
    return ServerProxy(os.environ['ROS_MASTER_URI'])


def master_ready():
    try:
        result = master_proxy().getPid('/formation_console_probe')
        return result[0] == 1
    except Exception:
        return False


def check_serial():
    ports = ('/dev/wheeltec_controller', os.environ.get('UWB_PORT', ''))
    if ports[1] and any(os.path.exists(alias) and os.path.realpath(ports[1]) == os.path.realpath(alias)
                        for alias in ('/dev/uwb_iot', '/dev/uwb_iot_aux')):
        raise RuntimeError('UWB_PORT points to an IOT device; select /dev/uwb_linktrack for localization')
    if ports[1] and os.path.realpath(ports[0]) == os.path.realpath(ports[1]):
        raise RuntimeError('UWB_PORT points to the chassis serial device: ' + ports[1] +
                           '; configure a separate UWB serial port before starting')
    for port in ports:
        if not port or not os.access(port, os.R_OK | os.W_OK):
            raise RuntimeError('Serial unavailable; configure UWB_PORT / permissions: ' + port)
        serial = os.path.realpath(port)
        for fd in glob.glob('/proc/[0-9]*/fd/*'):
            try:
                if os.readlink(fd) == serial:
                    raise RuntimeError('Serial already in use: ' + port + ' (' + fd + ')')
            except OSError:
                pass


def yaw(orientation):
    q = orientation
    norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
    if not finite(norm) or norm < 1e-12:
        return float('nan')
    return math.atan2(2 * (q.w * q.z + q.x * q.y), norm - 2 * (q.y * q.y + q.z * q.z))


def parse_limits(encoded, ids):
    data = json.loads(encoded) if isinstance(encoded, str) else encoded
    if not isinstance(data, dict) or set(data) != set(str(n) for n in ids):
        raise ValueError('Speed limits must contain exactly the participating robot IDs')
    result = {}
    for number, value in data.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0 <= value <= 0.5:
            raise ValueError('Speed limits must be finite numbers in [0, 0.5] m/s')
        result[str(number)] = float(value)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=['master', 'check', 'wait', 'monitor'])
    parser.add_argument('--ids', default='0,1,2')
    parser.add_argument('--leader', type=int, default=0)
    parser.add_argument('--step', choices=['chassis', 'follower'], default='chassis')
    parser.add_argument('--linear-limits', default='')
    parser.add_argument('--offsets', default='{}')
    parser.add_argument('--bounds', default='')
    parser.add_argument('--timeout', type=float, default=45)
    args = parser.parse_args()
    ids = [int(value) for value in args.ids.split(',')]
    if args.mode == 'check':
        if args.step == 'chassis':
            check_serial()
        result = master_proxy().getSystemState('/formation_console_probe')
        if result[0] != 1:
            raise RuntimeError('Cannot read ROS Master state')
        registered = set(node for group in result[2] for _, nodes in group for node in nodes)
        for number in ids:
            node = '/ugv%d/%s' % (number, 'formation_controller' if args.step == 'follower' else 'uwb_localizer')
            if node in registered:
                raise RuntimeError('Node already registered; stop its existing launch first: ' + node)
        return
    if args.mode == 'master':
        deadline = monotonic() + args.timeout
        while monotonic() < deadline:
            if master_ready():
                print('ROS Master ready')
                return
            time.sleep(0.2)
        raise RuntimeError('ROS Master unavailable: ' + os.environ.get('ROS_MASTER_URI', ''))

    import rospy
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, Vector3Stamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String, Float64
    rospy.init_node('formation_console', anonymous=True, disable_signals=True)
    if args.mode == 'wait':
        deadline = monotonic() + args.timeout
        for number in ids:
            prefix = '/ugv%d/' % number
            topics = [(prefix + 'formation_controller/state', String)] if args.step == 'follower' else [
                (prefix + 'odom', Odometry), (prefix + 'uwb/pose', PoseStamped), (prefix + 'uwb/valid', Bool)]
            for topic, kind in topics:
                while True:
                    left = deadline - monotonic()
                    if left <= 0:
                        raise RuntimeError('Readiness timed out: ' + topic)
                    message = rospy.wait_for_message(topic, kind, timeout=left)
                    if kind is not Bool or message.data:
                        break
                print('Ready: ' + topic)
        return

    cache, lock = {}, threading.Lock()
    publishers = {number: rospy.Publisher('/ugv%d/cmd_vel' % number, Twist, queue_size=1) for number in ids}
    enable = rospy.Publisher('/five_ugv_formation/enable', Bool, queue_size=1, latch=True)
    subscribers = []
    limits = parse_limits(args.linear_limits, ids) if args.linear_limits else {str(n): 0.15 for n in ids}
    limit_publishers, limit_subscribers = {}, {}
    last_limit_publish = -1e9
    offsets = json.loads(args.offsets)
    for number, point in offsets.items():
        if str(number) not in limits or len(point) != 2 or not all(finite(float(v)) for v in point):
            raise ValueError('Invalid formation offsets')
    bounds = [float(v) for v in args.bounds.split(',')] if args.bounds else None
    if bounds and (len(bounds) != 4 or not all(finite(v) for v in bounds) or
                   bounds[0] >= bounds[2] or bounds[1] >= bounds[3]):
        raise ValueError('Invalid path bounds')
    tracker, tracking_log = LeaderTracker(), None
    log_directory = ''
    control_mode, last_control, control_error = 'keyboard', -1, ''
    alignment = {'offset': None, 'stationary_since': None}

    def record(number, key, convert):
        def callback(message):
            value = convert(message)
            try:
                json.dumps(value, allow_nan=False)
            except ValueError:
                return
            with lock:
                age = 0.0
                if hasattr(message, 'header') and message.header.stamp.to_sec():
                    age = max(0.0, (rospy.Time.now() - message.header.stamp).to_sec())
                cache.setdefault(str(number), {})[key] = (value, monotonic() - age)
        return callback

    def position(message):
        p = message.pose.position
        return [p.x, p.y]

    for number in ids:
        prefix = '/ugv%d/' % number
        fields = [('uwb/pose', PoseStamped, 'pose', position),
                  ('uwb/pose', PoseStamped, 'fusion_yaw', lambda m: yaw(m.pose.orientation)),
                  ('uwb/valid', Bool, 'valid', lambda m: m.data),
                  ('uwb/status', String, 'uwb_status', lambda m: m.data),
                  ('odom_combined', PoseWithCovarianceStamped, 'yaw', lambda m: yaw(m.pose.pose.orientation)),
                  ('odom', Odometry, 'velocity', lambda m: [m.twist.twist.linear.x, m.twist.twist.linear.y,
                                                          m.twist.twist.angular.z]),
                  ('formation_controller/target_pose', PoseStamped, 'target', position),
                  ('formation_controller/tracking_error', Vector3Stamped, 'error', lambda m: m.vector.z),
                  ('formation_controller/state', String, 'state', lambda m: m.data)]
        for topic, kind, key, convert in fields:
            subscribers.append(rospy.Subscriber(prefix + topic, kind, record(number, key, convert), queue_size=1))
    for number in ids:
        if number == args.leader:
            continue
        prefix = '/ugv%d/formation_controller/' % number
        limit_publishers[number] = rospy.Publisher(prefix+'set_max_linear', Float64, queue_size=1, latch=True)
        limit_subscribers[number] = rospy.Subscriber(prefix+'max_linear', Float64,
                                                      record(number, 'max_linear', lambda m: m.data), queue_size=1)

    def limit_status():
        with lock:
            result = {}
            for number in ids:
                requested = limits[str(number)]
                if number == args.leader:
                    applied, connected = requested, True
                else:
                    applied, at = cache.get(str(number), {}).get('max_linear', (None, 0))
                    connected = (limit_publishers[number].get_num_connections() > 0 and
                                 limit_subscribers[number].get_num_connections() > 0 and monotonic() - at < 2.0)
                result[str(number)] = {'requested': requested, 'applied': applied,
                                       'ready': connected and applied == requested}
            return result

    def control_pose(now):
        with lock:
            row = dict(cache.get(str(args.leader), {}))
        if not row.get('valid', (False, 0))[0]:
            alignment['stationary_since'] = None
            return None, 'UWB_INVALID'
        if any(now-row.get(key, (None, -1e9))[1] > 0.5
               for key in ('pose', 'valid', 'yaw', 'fusion_yaw', 'velocity')):
            alignment['stationary_since'] = None
            return None, 'STALE_INPUT'
        if alignment['offset'] is None:
            vx, vy, omega = row['velocity'][0]
            if math.hypot(vx, vy) > 0.02 or abs(omega) > 0.03:
                alignment['stationary_since'] = None
            elif alignment['stationary_since'] is None:
                alignment['stationary_since'] = now
            elif now-alignment['stationary_since'] >= 0.4:
                # Anchor odom heading to the EKF's UWB frame once while stationary.
                alignment['offset'] = wrap(row['fusion_yaw'][0]-row['yaw'][0])
            if alignment['offset'] is None:
                return None, 'WAIT_ALIGNMENT'
        return list(row['pose'][0]) + [wrap(row['yaw'][0]+alignment['offset'])], ''

    def formation_problem(now):
        if not enable_sent:
            return ''
        for number in ids:
            if number == args.leader:
                continue
            if str(number) not in offsets:
                return 'FORMATION_OFFSETS'
            with lock:
                row = dict(cache.get(str(number), {}))
            if (not row.get('valid', (False, 0))[0] or
                    any(now-row.get(key, (None, -1e9))[1] > 0.5 for key in ('pose', 'valid'))):
                return 'FOLLOWER_INVALID'
        if not all(row['ready'] for row in limit_status().values()):
            return 'LIMIT_PENDING'
        return ''

    def read_offsets():
        for number in ids:
            if number == args.leader:
                continue
            default = offsets.get(str(number), [None, None])
            actual = [float(rospy.get_param('/ugv%d/formation_controller/offset_%s' % (number, axis), default[i]))
                      for i, axis in enumerate(('x', 'y'))]
            if not all(finite(v) for v in actual):
                raise ValueError('FORMATION_OFFSETS')
            offsets[str(number)] = actual

    # Capture launch parameters before entering the velocity loop; RPCs must not
    # block the 20 Hz output. Offset changes require reconnecting the monitor.
    try:
        read_offsets()
    except (ValueError, TypeError):
        pass  # Missing offsets prevent formation path control below.
    last_input, command, buffer, last_enable = monotonic(), {}, '', None
    last_heartbeat, last_sample = -1e9, -1e9
    enable_sent = False
    enable.publish(False)
    emit({'event': 'ready'})
    try:
        while not rospy.is_shutdown():
            started = monotonic()
            readable = select.select([sys.stdin], [], [], 0)[0]
            if readable:
                chunk = os.read(sys.stdin.fileno(), 65536)
                if not chunk:
                    break
                buffer += chunk.decode('utf-8')
                if len(buffer) > 65536:
                    raise RuntimeError('Input too long')
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    command = json.loads(line)
                    if command.get('quit'):
                        return
                    last_input = monotonic()
                    tick = command.get('tick', 0)
                    fresh = isinstance(tick, (int, float)) and 0 <= monotonic() - tick <= 0.4
                    if fresh:
                        last_heartbeat = monotonic()
                    if fresh and 'linear_limits' in command:
                        updated = parse_limits(command['linear_limits'], ids)
                        if updated != limits:
                            limits, last_limit_publish = updated, -1e9
                    sequence = command.get('enable_sequence')
                    if fresh and sequence is not None and sequence != last_enable:
                        ready = all(row['ready'] for row in limit_status().values())
                        if ready and command.get('enable'):
                            try:
                                if control_mode == 'path' and tracker.points:
                                    check_bounds(tracker.points, bounds, list(offsets.values()))
                            except (ValueError, TypeError) as error:
                                ready = False
                                if tracker.state in tracker.ACTIVE:
                                    tracker.pause(str(error))
                        enable_sent = bool(command.get('enable')) and ready
                        enable.publish(enable_sent)
                        last_enable = sequence
                    request = command.get('control', {})
                    sequence = request.get('sequence', -1)
                    if not fresh and isinstance(sequence, int) and sequence > last_control:
                        last_control, control_error = sequence, 'CONTROL_TIMEOUT'
                        if request.get('action') in ('start', 'resume', 'path'):
                            control_mode = 'path'
                        if tracker.state in tracker.ACTIVE:
                            tracker.pause(control_error)
                    if fresh and isinstance(sequence, int) and sequence > last_control:
                        last_control, control_error = sequence, ''
                        action = request.get('action')
                        try:
                            if action in ('keyboard', 'path', 'stop'):
                                tracker.stop()
                                control_mode = 'stopped' if action == 'stop' else action
                            elif action == 'pause':
                                if tracker.state in tracker.ACTIVE or tracker.state == 'PAUSED':
                                    tracker.pause()
                                control_mode = 'path'
                            elif action in ('start', 'resume'):
                                control_mode = 'path'
                                if action == 'start':
                                    tracker.pause()
                                elif tracker.state != 'PAUSED':
                                    raise ValueError('NO_PAUSED_PATH')
                                pose, problem = control_pose(monotonic())
                                problem = problem or formation_problem(monotonic())
                                if publishers[args.leader].get_num_connections() < 1:
                                    problem = 'NO_CHASSIS'
                                if problem:
                                    raise ValueError(problem)
                                if action == 'start':
                                    points = request.get('points', [])
                                    bends = request.get('bends')
                                    check_bounds(sample_path(points, bends), bounds, list(offsets.values()) if enable_sent else [])
                                    tracker.start(points, float(request.get('speed', 0.1)),
                                                  float(request.get('lookahead', 0.4)), pose, monotonic(), bends=bends)
                                    if tracking_log:
                                        tracking_log.close()
                                        tracking_log = None
                                    root = os.path.join(os.environ.get('FORMATION_WS', os.getcwd()), 'logs', 'leader_tracking')
                                    tracking_log = TrackingLog(root, dict(request, leader=args.leader,
                                                                         offsets=offsets, bounds=bounds,
                                                                         yaw_offset=alignment['offset']), monotonic())
                                    log_directory = tracking_log.directory
                                else:
                                    tracker.resume(monotonic())
                            else:
                                raise ValueError('CONTROL_ACTION')
                        except (ValueError, TypeError, OSError) as error:
                            if tracker.state in tracker.ACTIVE:
                                tracker.pause(str(error))
                            else:
                                tracker.reason = str(error)
                            control_mode, control_error = 'path', str(error)
                    if fresh and command.get('stop'):
                        tracker.stop()
                        control_mode = 'stopped'
                        enable_sent = False
                        enable.publish(False)
                        for publisher in publishers.values():
                            publisher.publish(Twist())
            now = monotonic()
            if now - last_input > 3:
                break
            if now-last_limit_publish >= 1.0:
                for number, publisher in limit_publishers.items():
                    publisher.publish(Float64(data=limits[str(number)]))
                last_limit_publish = now
            linear, angular = velocity(command, now)
            pose, problem = control_pose(now)
            if now-last_heartbeat > 0.4:
                if tracker.state in tracker.ACTIVE:
                    tracker.pause('CONTROL_TIMEOUT')
                if enable_sent:
                    enable_sent = False
                    enable.publish(False)
            if control_mode == 'path':
                problem = problem or formation_problem(now)
                if not problem and enable_sent and tracker.points:
                    try:
                        check_bounds(tracker.points, bounds, list(offsets.values()))
                    except ValueError as error:
                        problem = str(error)
                if publishers[args.leader].get_num_connections() < 1:
                    problem = 'NO_CHASSIS'
                if problem and tracker.state in tracker.ACTIVE:
                    tracker.pause(problem)
                following = [(float(offsets[str(n)][0]), float(offsets[str(n)][1]), limits[str(n)])
                             for n in ids if n != args.leader and str(n) in offsets] if enable_sent else []
                linear, angular = tracker.step(pose, now, limits[str(args.leader)], following)
            elif control_mode == 'stopped':
                linear = angular = 0.0
            linear = max(-limits[str(args.leader)], min(limits[str(args.leader)], linear))
            twist = Twist()
            twist.linear.x, twist.angular.z = linear, angular
            publishers[args.leader].publish(twist)
            if tracking_log:
                tracking_log.record(now, pose, tracker, limits[str(args.leader)], (linear, angular))
                if control_mode != 'path' or tracker.state in ('DONE', 'STOPPED'):
                    tracking_log.close()
                    tracking_log = None
            with lock:
                robots = {number: {key: {'value': value, 'age': now - at}
                                   for key, (value, at) in values.items()} for number, values in cache.items()}
            if now-last_sample >= 0.1:
                emit({'event': 'sample', 'tick': now, 'robots': robots,
                      'enable_sequence': last_enable, 'enable_sent': enable_sent,
                      'tracking': dict(tracker.status(), mode=control_mode, ack=last_control, error=control_error,
                                       yaw=pose[2] if pose else None, input_problem=problem, offsets=offsets,
                                       log_directory=log_directory),
                      'linear_limits': limit_status(), 'leader_subscribers': publishers[args.leader].get_num_connections()})
                last_sample = now
            time.sleep(max(0, 0.05 - (monotonic() - started)))
    finally:
        for _ in range(3):
            enable.publish(False)
            for publisher in publishers.values():
                publisher.publish(Twist())
            time.sleep(0.05)
        if tracking_log:
            tracking_log.close()


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        emit({'event': 'error', 'message': str(error)})
        sys.exit(1)
