#!/usr/bin/env python3
"""Loopback-only ROS integration: synthetic UWB, leader routing and lost-GUI stop.
Run: source scripts/env.sh && python3 test/test_fleet_ros.py
"""
import json
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
        os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % port, ROS_IP='127.0.0.1', ROS_LOG_DIR=directory)
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
                from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
                from nav_msgs.msg import Odometry
                from std_msgs.msg import Bool, String
                rospy.init_node('console_offline_test', anonymous=True, disable_signals=True)
                velocities, enable_values = {0: [], 1: []}, []
                subs = [rospy.Subscriber('/ugv%d/cmd_vel' % n, Twist,
                                         lambda message, n=n: velocities[n].append((time.monotonic(), message.linear.x)))
                        for n in velocities]
                subs.append(rospy.Subscriber('/five_ugv_formation/enable', Bool,
                                             lambda message: enable_values.append(message.data)))
                pubs = {}
                for n in (0, 1):
                    prefix = '/ugv%d/' % n
                    for topic, kind in [('uwb/pose', PoseStamped), ('uwb/valid', Bool), ('odom', Odometry),
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
                            error = Vector3Stamped()
                            error.header.stamp = rospy.Time.now()
                            error.vector.z = 0.12
                            for topic, message in [('uwb/pose', pose), ('uwb/valid', Bool(True)), ('odom', Odometry()),
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
                monitor = subprocess.Popen([sys.executable, '-u', helper, 'monitor', '--ids', '0,1', '--leader', '0'],
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
                def pump(seconds, linear=0.0, enabled=False, stale=False, send=True):
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
                                           enable_sequence=1 if enabled else 0, enable=enabled)
                            monitor.stdin.write(json.dumps(payload) + '\n')
                            monitor.stdin.flush()
                            sent = now
                        time.sleep(0.02)
                pump(2)
                assert latest['robots']['1']['pose']['value'] == [2.0, 2.0], latest
                assert latest['robots']['1']['error']['value'] == 0.12
                pump(0.7, linear=0.1, stale=True)
                assert not any(value for _, value in velocities[0]), velocities
                pump(0.8, linear=0.1, enabled=True)
                assert any(value == 0.1 for _, value in velocities[0]), velocities
                assert not any(value for _, value in velocities[1]), velocities
                assert True in enable_values, enable_values
                assert latest['enable_sequence'] == 1, latest
                # Host sends nothing: lease expires, then remote monitor disables followers and exits.
                pump(0.8, send=False)
                assert velocities[0][-1][1] == 0.0, velocities[0][-5:]
                monitor.wait(timeout=5)
                time.sleep(0.15)
                assert enable_values[-1] is False, enable_values
                assert velocities[1][-1][1] == 0.0
                print('Loopback ROS passed: readiness, live telemetry, leader-only motion, stale-command rejection, 0.4 s lease, lost-GUI disable.')
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
