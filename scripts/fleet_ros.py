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
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


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
                  ('uwb/valid', Bool, 'valid', lambda m: m.data),
                  ('uwb/status', String, 'uwb_status', lambda m: m.data),
                  ('odom_combined', PoseWithCovarianceStamped, 'yaw', lambda m: yaw(m.pose.pose.orientation)),
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

    last_input, command, buffer, last_enable = monotonic(), {}, '', None
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
                    if fresh and 'linear_limits' in command:
                        updated = parse_limits(command['linear_limits'], ids)
                        if updated != limits:
                            limits, last_limit_publish = updated, -1e9
                    sequence = command.get('enable_sequence')
                    if fresh and sequence is not None and sequence != last_enable:
                        ready = all(row['ready'] for row in limit_status().values())
                        enable_sent = bool(command.get('enable')) and ready
                        enable.publish(enable_sent)
                        last_enable = sequence
                    if fresh and command.get('stop'):
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
            linear = max(-limits[str(args.leader)], min(limits[str(args.leader)], linear))
            twist = Twist()
            twist.linear.x, twist.angular.z = linear, angular
            publishers[args.leader].publish(twist)
            with lock:
                robots = {number: {key: {'value': value, 'age': now - at}
                                   for key, (value, at) in values.items()} for number, values in cache.items()}
            emit({'event': 'sample', 'tick': now, 'robots': robots,
                  'enable_sequence': last_enable, 'enable_sent': enable_sent,
                  'linear_limits': limit_status(), 'leader_subscribers': publishers[args.leader].get_num_connections()})
            time.sleep(max(0, 0.1 - (monotonic() - started)))
    finally:
        for _ in range(3):
            enable.publish(False)
            for publisher in publishers.values():
                publisher.publish(Twist())
            time.sleep(0.05)


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        emit({'event': 'error', 'message': str(error)})
        sys.exit(1)
