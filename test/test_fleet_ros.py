#!/usr/bin/env python3
"""Loopback-only ROS integration: synthetic UWB, leader routing and lost-GUI stop.
Run: source scripts/env.sh && python3 test/test_fleet_ros.py
"""
import json
import csv
import math
import os
from pathlib import Path
import queue
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    with tempfile.TemporaryDirectory() as directory:
        os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % port, ROS_IP='127.0.0.1', ROS_LOG_DIR=directory,
                          FORMATION_WS=directory)
        os.environ.pop('ROS_HOSTNAME', None)
        stop = threading.Event()
        monitor = publisher_thread = None
        with open(Path(directory) / 'master.log', 'w') as log:
            master = subprocess.Popen(['roscore', '-p', str(port)], stdout=log, stderr=subprocess.STDOUT,
                                      start_new_session=True)
            try:
                deadline = time.monotonic() + 15
                while True:
                    try:
                        with socket.create_connection(('127.0.0.1', port), timeout=0.2):
                            break
                    except OSError:
                        if time.monotonic() > deadline:
                            raise RuntimeError('Local test ROS Master did not start')
                        time.sleep(0.1)
                import rospy
                from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, Vector3Stamped
                from nav_msgs.msg import Odometry
                from std_msgs.msg import Bool, String, Float64
                rospy.init_node('console_offline_test', anonymous=True, disable_signals=True)
                velocities, enable_values = {0: [], 1: []}, []
                subs = [rospy.Subscriber('/ugv%d/cmd_vel' % n, Twist,
                                         lambda message, n=n: velocities[n].append((time.monotonic(), message.linear.x)))
                        for n in velocities]
                subs.append(rospy.Subscriber('/five_ugv_formation/enable', Bool,
                                             lambda message: enable_values.append(message.data)))
                limit_ack = rospy.Publisher('/ugv1/formation_controller/max_linear', Float64, queue_size=1, latch=True)
                acknowledge = threading.Event()
                received_limits = []
                def apply_limit(message):
                    received_limits.append(message.data)
                    if acknowledge.is_set():
                        limit_ack.publish(message)
                subs.append(rospy.Subscriber('/ugv1/formation_controller/set_max_linear', Float64, apply_limit))
                pubs = {}
                validity = {0: True, 1: True}
                for n in (0, 1):
                    prefix = '/ugv%d/' % n
                    for topic, kind in [('uwb/pose', PoseStamped), ('uwb/valid', Bool), ('odom', Odometry),
                                        ('odom_combined', PoseWithCovarianceStamped),
                                        ('formation_controller/target_pose', PoseStamped),
                                        ('formation_controller/tracking_error', Vector3Stamped),
                                        ('formation_controller/state', String)]:
                        pubs[(n, topic)] = rospy.Publisher(prefix + topic, kind, queue_size=1)
                def publish():
                    while not stop.is_set():
                        for n in (0, 1):
                            pose = PoseStamped()
                            pose.header.stamp = rospy.Time.now()
                            pose.pose.position.x, pose.pose.position.y = 1.0 + n, 2.0
                            pose.pose.orientation.w = 1
                            heading = PoseWithCovarianceStamped()
                            heading.header.stamp = rospy.Time.now()
                            heading.pose.pose.orientation.z = math.sin(1.2/2)
                            heading.pose.pose.orientation.w = math.cos(1.2/2)
                            error = Vector3Stamped()
                            error.header.stamp = rospy.Time.now()
                            error.vector.z = 0.12
                            for topic, message in [('uwb/pose', pose), ('uwb/valid', Bool(validity[n])), ('odom', Odometry()),
                                                   ('odom_combined', heading),
                                                   ('formation_controller/target_pose', pose),
                                                   ('formation_controller/tracking_error', error),
                                                   ('formation_controller/state', String('DISABLED'))]:
                                pubs[(n, topic)].publish(message)
                        stop.wait(0.04)
                publisher_thread = threading.Thread(target=publish, daemon=True)
                publisher_thread.start()
                helper = str(ROOT / 'scripts/fleet_ros.py')
                for step in ('chassis', 'follower'):
                    subprocess.run([sys.executable, helper, 'wait', '--ids', '0,1', '--step', step, '--timeout', '8'],
                                   check=True, timeout=12, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                monitor = subprocess.Popen([sys.executable, '-u', helper, 'monitor', '--ids', '0,1', '--leader', '0',
                                            '--offsets', '{"1":[0.8,0.8]}'],
                                           stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                           text=True, bufsize=1)
                events, latest = queue.Queue(), {}
                def read():
                    for line in monitor.stdout:
                        try:
                            events.put(json.loads(line))
                        except ValueError:
                            pass
                threading.Thread(target=read, daemon=True).start()
                limits = {'0': 0.12, '1': 0.15}
                control = {'sequence': 0, 'action': 'keyboard'}
                def request(action, **values):
                    sequence = control['sequence']+1
                    control.clear()
                    control.update(values, sequence=sequence, action=action)
                def pump(seconds, linear=0.0, enabled=False, stale=False, send=True, sequence=None, requested=None):
                    deadline, sent = time.monotonic() + seconds, 0.0
                    while time.monotonic() < deadline:
                        while not events.empty():
                            data = events.get_nowait()
                            if data.get('event') == 'error':
                                raise AssertionError(data)
                            if data.get('event') == 'sample':
                                latest.update(data)
                        now = time.monotonic()
                        if send and latest and now - sent > 0.1:
                            payload = dict(tick=latest['tick'] - (10 if stale else 0), linear=linear, angular=0,
                                           enable_sequence=sequence if sequence is not None else (2 if enabled else 0),
                                           enable=enabled, linear_limits=limits if requested is None else requested,
                                           control=dict(control))
                            monitor.stdin.write(json.dumps(payload) + '\n')
                            monitor.stdin.flush()
                            sent = now
                        time.sleep(0.02)
                pump(2)
                assert latest['robots']['1']['pose']['value'] == [2.0, 2.0], latest
                assert latest['robots']['1']['error']['value'] == 0.12
                assert not latest['linear_limits']['1']['ready'], latest
                pump(0.5, enabled=True, sequence=1)
                assert True not in enable_values, enable_values
                assert latest['enable_sequence'] == 1 and not latest['enable_sent'], latest
                acknowledge.set()
                pump(1.3)
                assert latest['linear_limits']['1']['applied'] == 0.15, latest
                assert latest['linear_limits']['1']['ready'], latest
                pump(0.7, linear=0.1, stale=True)
                assert not any(value for _, value in velocities[0]), velocities
                pump(0.8, linear=0.1, enabled=True)
                assert any(value == 0.1 for _, value in velocities[0]), velocities
                assert not any(value for _, value in velocities[1]), velocities
                assert True in enable_values, enable_values
                assert latest['enable_sequence'] == 2 and latest['enable_sent'], latest
                limits.update({'0': 0.04, '1': 0.08})
                pump(1.2, linear=0.3, enabled=True)
                assert velocities[0][-1][1] == 0.04, velocities[0][-5:]
                assert latest['linear_limits']['1']['applied'] == 0.08 and 0.08 in received_limits, latest
                pump(0.4, linear=-0.3, enabled=True)
                assert velocities[0][-1][1] == -0.04, velocities[0][-5:]
                pump(0.4, stale=True, requested={'0': 0.4, '1': 0.4})
                assert latest['linear_limits']['0']['applied'] == 0.04, latest
                limits.update({'0': 0.0, '1': 0.0})
                pump(1.2, linear=0.3, enabled=True)
                assert velocities[0][-1][1] == 0.0, velocities[0][-5:]
                assert latest['linear_limits']['1']['applied'] == 0.0, latest
                # Auto control uses the same publisher at 20 Hz, ignoring manual input.
                limits.update({'0': 0.15, '1': 0.15})
                pump(1.2)
                assert abs(latest['tracking']['yaw']) < 1e-6, latest
                request('start', points=[[1, 2], [2, 2], [2, 3]], bends=[0.1, 0], speed=0.1, lookahead=0.4)
                started = time.monotonic()
                pump(1.2, linear=0.5)
                assert latest['tracking']['state'] == 'TRACKING', latest
                assert latest['tracking']['waypoints'] == [[1, 2], [2, 2], [2, 3]], latest
                assert [1.5, 2.1] in latest['tracking']['points'], latest
                auto = [(t, v) for t, v in velocities[0] if t > started+0.2]
                assert len(auto) >= 15 and all(0 <= v <= 0.1 for _, v in auto), auto
                assert any(v > 0.05 for _, v in auto), auto
                request('pause')
                pump(0.4, linear=0.5)
                assert velocities[0][-1][1] == 0 and latest['tracking']['state'] == 'PAUSED', latest
                request('resume')
                pump(0.5, linear=-0.5)
                assert velocities[0][-1][1] > 0, latest
                validity[0] = False
                pump(0.3)
                assert velocities[0][-1][1] == 0 and latest['tracking']['reason'] == 'UWB_INVALID', latest
                validity[0] = True
                pump(0.5, linear=0.5)
                assert velocities[0][-1][1] == 0 and latest['tracking']['state'] == 'PAUSED', latest
                request('resume')
                pump(0.5, stale=True)
                pump(0.5)
                assert velocities[0][-1][1] == 0 and latest['tracking']['error'] == 'CONTROL_TIMEOUT', latest
                request('resume')
                pump(0.5, enabled=True, sequence=5)
                validity[1] = False
                pump(0.3, enabled=True, sequence=5)
                assert velocities[0][-1][1] == 0 and latest['tracking']['reason'] == 'FOLLOWER_INVALID', latest
                validity[1] = True
                pump(0.3, enabled=True, sequence=5)  # Wait for recovered ROS data before explicit resume.
                request('resume')
                pump(0.5, enabled=True, sequence=5)
                assert latest['tracking']['state'] == 'TRACKING' and velocities[0][-1][1] > 0, latest
                # Stale heartbeats pause auto; repeating the same resume request cannot restart it.
                pump(2.3, stale=True)
                assert velocities[0][-1][1] == 0 and latest['tracking']['reason'] == 'CONTROL_TIMEOUT', latest
                pump(0.5)
                assert latest['tracking']['state'] == 'PAUSED', latest
                request('stop')
                pump(0.3)
                assert latest['tracking']['state'] == 'STOPPED', latest
                with open(Path(latest['tracking']['log_directory'])/'tracking.csv') as stream:
                    records = list(csv.DictReader(stream))
                with open(Path(latest['tracking']['log_directory'])/'path.json') as stream:
                    assert json.load(stream)['bends'] == [0.1, 0]
                assert {'TRACKING', 'PAUSED', 'STOPPED'} <= {r['state'] for r in records}, records[-4:]
                assert 'max_linear' in records[0] and 'heading_error' in records[0], records[0]
                request('resume')
                pump(0.3, linear=0.5)
                assert velocities[0][-1][1] == 0 and latest['tracking']['error'] == 'NO_PAUSED_PATH', latest
                request('keyboard')
                pump(0.4, linear=0.08)
                assert velocities[0][-1][1] == 0.08, latest
                # Host sends nothing: lease expires, then remote monitor disables followers and exits.
                pump(0.8, send=False)
                assert velocities[0][-1][1] == 0.0, velocities[0][-5:]
                monitor.wait(timeout=5)
                time.sleep(0.15)
                assert enable_values[-1] is False, enable_values
                assert velocities[1][-1][1] == 0.0
                print('Loopback ROS passed: 20 Hz path control, mode arbitration, UWB and follower fault pause, explicit resume, stale lease stop, CSV logging, online limits and lost-GUI disable.')
            finally:
                stop.set()
                if publisher_thread:
                    publisher_thread.join(2)
                if monitor:
                    if monitor.poll() is None:
                        monitor.terminate()
                        monitor.wait(timeout=5)
                    monitor.stdin.close()
                    monitor.stdout.close()
                try:
                    os.killpg(master.pid, signal.SIGINT)
                    master.wait(timeout=8)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    if master.poll() is None:
                        os.killpg(master.pid, signal.SIGKILL)
                        master.wait()


if __name__ == '__main__':
    main()
