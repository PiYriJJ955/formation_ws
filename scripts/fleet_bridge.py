#!/usr/bin/env python
"""ROS Melodic/Python 2 and Noetic/Python 3 bridge, run by fleet_console over SSH."""
from __future__ import print_function

import ctypes
import fcntl
import glob
import json
import math
import os
import select
import signal
import socket
import subprocess
import sys
import time


if hasattr(time, 'monotonic'):
    monotonic = time.monotonic
else:
    class Timespec(ctypes.Structure):
        _fields_ = [('seconds', ctypes.c_long), ('nanoseconds', ctypes.c_long)]

    _clock_gettime = ctypes.CDLL('librt.so.1', use_errno=True).clock_gettime

    def monotonic():
        stamp = Timespec()
        if _clock_gettime(1, ctypes.byref(stamp)):
            raise OSError(ctypes.get_errno(), 'clock_gettime')
        return stamp.seconds + stamp.nanoseconds / 1e9


COMMAND_TIMEOUT = 0.4


def velocity(command, now):
    """Echoed remote timestamps prevent delayed SSH data from restarting motion."""
    try:
        tick = float(command['tick'])
        linear, angular = float(command['linear']), float(command['angular'])
        if not all(not math.isnan(x) and not math.isinf(x) for x in (tick, linear, angular)):
            raise ValueError('nonfinite command')
        if not 0 <= now - tick <= COMMAND_TIMEOUT:
            return 0.0, 0.0
        if abs(linear) > 0.5 or abs(angular) > 1.5:
            return 0.0, 0.0
        return linear, angular
    except (KeyError, TypeError, ValueError, OverflowError):
        return 0.0, 0.0


def emit(**data):
    print(json.dumps(data))
    sys.stdout.flush()


def check_master_port(port=11321):
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        # Match ROS's reuse policy so TIME_WAIT from the previous master is allowed.
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        probe.bind(('127.0.0.1', port))
    finally:
        probe.close()


def main():
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float32

    cache = os.path.expanduser('~/.cache/formation-console')
    if not os.path.isdir(cache):
        os.makedirs(cache)
    lock = open(os.path.join(cache, 'chassis.lock'), 'w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    serial = os.path.realpath('/dev/wheeltec_controller')
    if not os.access(serial, os.R_OK | os.W_OK):
        raise RuntimeError('Cannot access /dev/wheeltec_controller')
    for fd in glob.glob('/proc/[0-9]*/fd/*'):
        try:
            if os.readlink(fd) == serial:
                raise RuntimeError('Chassis serial port already in use: ' + fd)
        except OSError:
            pass
    # roslaunch owns this private master and shuts it down with its child nodes.
    check_master_port()
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11321'
    os.environ['ROS_IP'] = '127.0.0.1'
    os.environ['CAR_MODE'] = 'mini_4wd'
    os.environ.pop('ROS_HOSTNAME', None)
    os.environ.pop('ROS_NAMESPACE', None)
    log_path = os.path.join(cache, 'chassis.log')
    log = open(log_path, 'w')
    launch = None
    publisher = None
    running = [True]
    telemetry = {'odom_time': 0.0, 'linear': 0.0, 'angular': 0.0, 'voltage': None}

    def stop_signal(*_):
        running[0] = False

    def odometry(message):
        telemetry.update(odom_time=monotonic(), linear=message.twist.twist.linear.x,
                         angular=message.twist.twist.angular.z)

    def voltage(message):
        telemetry['voltage'] = message.data

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, stop_signal)
    try:
        launch = subprocess.Popen(
            ['roslaunch', '-p', '11321', 'turn_on_wheeltec_robot',
             'turn_on_wheeltec_robot.launch', 'car_mode:=mini_4wd'],
            stdin=open(os.devnull), stdout=log, stderr=log, preexec_fn=os.setsid)
        # Bound startup even when ROS init cannot reach a working master.
        def startup_timeout(*_):
            raise RuntimeError('Chassis startup timeout; see ' + log_path)
        signal.signal(signal.SIGALRM, startup_timeout)
        signal.alarm(40)
        rospy.init_node('fleet_console_bridge', disable_signals=True)
        publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/odom', Odometry, odometry, queue_size=1)
        rospy.Subscriber('/PowerVoltage', Float32, voltage, queue_size=1)
        while running[0] and not rospy.is_shutdown():
            if launch.poll() is not None:
                raise RuntimeError('roslaunch exited; see ' + log_path)
            if select.select([sys.stdin], [], [], 0)[0]:
                pending = os.read(sys.stdin.fileno(), 4096)
                if not pending or b'"quit"' in pending:
                    return
            publisher.publish(Twist())
            if publisher.get_num_connections() and telemetry['odom_time']:
                break
            time.sleep(0.05)
        signal.alarm(0)
        if not running[0] or rospy.is_shutdown():
            return
        emit(event='ready', car_mode='mini_4wd', topic='/cmd_vel')
        command = {}
        received = monotonic()
        buffer = b''
        last_report = 0.0
        while running[0] and not rospy.is_shutdown():
            now = monotonic()
            if launch.poll() is not None:
                raise RuntimeError('roslaunch exited; see ' + log_path)
            if now - telemetry['odom_time'] > 1.0:
                raise RuntimeError('Chassis odometry stopped')
            if now - received > 3.0:
                raise RuntimeError('SSH heartbeat lost')
            if select.select([sys.stdin], [], [], 0.02)[0]:
                chunk = os.read(sys.stdin.fileno(), 4096)
                if not chunk:
                    break
                buffer += chunk
                if len(buffer) > 16384:
                    raise RuntimeError('Oversized command')
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    try:
                        command = json.loads(line.decode('utf-8'))
                        if not isinstance(command, dict):
                            command = {}
                    except (ValueError, UnicodeError):
                        command = {}
                    received = monotonic()
                    if command.get('quit'):
                        running[0] = False
                        command = {}
            now = monotonic()
            linear, angular = velocity(command, now) if running[0] else (0.0, 0.0)
            message = Twist()
            message.linear.x, message.angular.z = linear, angular
            publisher.publish(message)
            if now - last_report >= 0.1:
                emit(event='state', tick=now, linear=telemetry['linear'],
                     angular=telemetry['angular'], voltage=telemetry['voltage'],
                     commanded_linear=linear, commanded_angular=angular)
                last_report = now
    finally:
        signal.alarm(0)
        if publisher is not None:
            for _ in range(5):
                try:
                    publisher.publish(Twist())
                except Exception:
                    pass
                time.sleep(0.05)
        if launch is not None:
            try:
                os.killpg(launch.pid, signal.SIGINT)
                deadline = monotonic() + 8
                while launch.poll() is None and monotonic() < deadline:
                    time.sleep(0.1)
                # Clean descendants even if the roslaunch parent exited first.
                os.killpg(launch.pid, signal.SIGKILL)
            except OSError:
                pass
            launch.wait()
        rospy.signal_shutdown('SSH console closed')
        log.close()
        lock.close()


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        emit(event='error', message=str(error))
        sys.exit(1)
