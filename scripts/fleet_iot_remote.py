#!/usr/bin/env python
"""IOT discovery and recorded telemetry; ROS Melodic/Python 2 and Python 3."""
from __future__ import print_function
import argparse
import fcntl
import glob
import json
import math
import os
import select
import signal
import socket
import struct
import subprocess
import sys
import termios
import threading
import time
import tty
from fleet_bridge import monotonic

OUTPUT_LOCK = threading.Lock()


def emit(data):
    with OUTPUT_LOCK:
        print(json.dumps(data, allow_nan=False))
        sys.stdout.flush()


def identify(data):
    """Recognize checksum-valid NLink frames; measurement decoding stays in nlink_parser."""
    data = bytearray(data)
    sizes = {2: 11, 3: 27, 4: 119, 5: 21, 6: 21, 7: 21, 8: 24, 9: 14}
    for index in range(len(data) - 4):
        head, kind = data[index:index + 2]
        if head == 0x6a and kind == 0:
            length, minimum = struct.unpack_from('<H', data, index + 2)[0], 15
        elif head == 0x55 and kind in sizes:
            length, minimum = struct.unpack_from('<H', data, index + 2)[0], sizes[kind] + 1
        elif head == 0x55 and kind == 1:
            length = minimum = 128
        else:
            continue
        frame = data[index:index + length]
        if minimum <= length <= 4096 and len(frame) == length and sum(frame[:-1]) % 256 == frame[-1]:
            if head == 0x6a:
                return dict(kind='uwb_iot', uid=struct.unpack_from('<I', frame, 4)[0])
            return dict(kind='uwb_linktrack')
    return dict(kind='unknown')


def serial_busy(port):
    real = os.path.realpath(port)
    for fd in glob.glob('/proc/[0-9]*/fd/*'):
        try:
            if os.readlink(fd) == real:
                return True
        except OSError:
            pass
    return False


def probe_port(port, seconds=1.0):
    if serial_busy(port):
        return dict(kind='busy')
    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    original = None
    try:
        original = termios.tcgetattr(fd)
        tty.setraw(fd)
        attrs = termios.tcgetattr(fd)
        # Linux asm-generic/termbits.h; Python 2 omits the extended baud names.
        attrs[4] = attrs[5] = getattr(termios, 'B921600', 0x1007)
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        data = bytearray()
        deadline = monotonic() + seconds
        while monotonic() < deadline:
            if select.select([fd], [], [], 0.1)[0]:
                data.extend(os.read(fd, 8192))
                result = identify(data)
                if result['kind'] != 'unknown':
                    return result
                data = data[-8192:]
        return dict(kind='unknown')
    finally:
        try:
            if original is not None:
                termios.tcsetattr(fd, termios.TCSANOW, original)
        finally:
            os.close(fd)


def discover(extra=()):
    aliases = glob.glob('/dev/uwb*') + glob.glob('/dev/serial/by-id/*')
    excluded = {os.path.realpath(p) for p in glob.glob('/dev/wheeltec*')}
    ports = set(os.path.realpath(p) for p in list(extra) + aliases +
                glob.glob('/dev/ttyCH343USB*') + glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
    result = []
    for port in sorted(ports - excluded):
        row = dict(port=port, aliases=[p for p in aliases if os.path.realpath(p) == port])
        try:
            row.update(probe_port(port))
        except (OSError, termios.error) as error:
            row.update(kind='unavailable', error=str(error))
        result.append(row)
    return result


def stop_process(proc):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        for unused in range(50):
            if proc.poll() is not None:
                break
            time.sleep(0.1)
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()


def monitor(args):
    import rospy
    import rosbag
    from nlink_parser.msg import IotFrame0
    if os.environ.get('UGV_ID') != args.robot[3:]:
        raise RuntimeError('UGV_ID mismatch: expected ' + args.robot)
    sensors = json.loads(args.sensors)
    real_ports = [os.path.realpath(s['port']) for s in sensors]
    excluded = {os.path.realpath(p) for p in glob.glob('/dev/wheeltec*') + ['/dev/uwb_linktrack']}
    if len(set(real_ports)) != len(real_ports) or set(real_ports) & excluded:
        raise RuntimeError('IOT ports overlap each other, LinkTrack or chassis devices')
    for sensor in sensors:
        result = probe_port(sensor['port'])
        if result['kind'] != 'uwb_iot':
            raise RuntimeError(sensor['port'] + ': expected IOT, got ' + result['kind'])
        if sensor.get('uid') and sensor['uid'] != result['uid']:
            raise RuntimeError('%s: UID mismatch, received 0x%08X' % (sensor['port'], result['uid']))
        sensor['uid'] = result['uid']
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11341', ROS_IP='127.0.0.1')
    os.environ.pop('ROS_HOSTNAME', None)
    os.environ.pop('ROS_NAMESPACE', None)
    probe = socket.socket()
    try:
        if probe.connect_ex(('127.0.0.1', 11341)) == 0:
            raise RuntimeError('IOT monitor master port 11341 is already in use')
    finally:
        probe.close()
    directory = os.path.abspath(args.directory)
    os.makedirs(directory)
    os.environ['ROS_LOG_DIR'] = directory + '/ros'
    stop, lock = threading.Event(), threading.Lock()
    processes, handles, subscribers = [], [], []
    bag = None
    failure, last_frames = [], {}

    def start(name, command):
        handle = open(directory + '/' + name + '.log', 'w')
        handles.append(handle)
        proc = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, preexec_fn=os.setsid)
        processes.append(proc)
        return proc

    def callback(message, sensor):
        try:
            now = time.time()
            with lock:
                if bag is None:
                    return
                topic = '/%s/%s/nlink_iot_frame0' % (args.robot, sensor['name'])
                bag.write(topic, message, rospy.Time.from_sec(now))
                if message.uid != sensor['uid']:
                    raise RuntimeError('IOT source UID changed on ' + sensor['port'])
                last_frames[sensor['name']] = monotonic()
                nodes = [dict(uid=n.uid, distance=n.dis, horizontal=n.aoa_angle_horizontal,
                              fp_rssi=n.fp_rssi, rx_rssi=n.rx_rssi) for n in message.nodes]
                for node in nodes:
                    for key in ('distance', 'horizontal', 'fp_rssi', 'rx_rssi'):
                        if math.isnan(node[key]) or math.isinf(node[key]):
                            node[key] = None
                emit(dict(event='frame', robot=args.robot, sensor=sensor['name'], uid=message.uid,
                          system_time=message.system_time, received=now, nodes=nodes))
        except Exception as error:
            failure.append(str(error))
            stop.set()

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stop.set())
    try:
        start('master', ['roscore', '-p', '11341'])
        try:
            from xmlrpc.client import ServerProxy
        except ImportError:
            from xmlrpclib import ServerProxy
        socket.setdefaulttimeout(1)
        master = ServerProxy(os.environ['ROS_MASTER_URI'])
        deadline = monotonic() + 15
        while not stop.is_set():
            try:
                if master.getPid('/iot_monitor_probe')[0] == 1:
                    break
            except Exception:
                pass
            if monotonic() > deadline:
                raise RuntimeError('IOT master did not start')
            stop.wait(0.1)
        if stop.is_set():
            return
        rospy.init_node('iot_console', anonymous=True, disable_signals=True)
        bag = rosbag.Bag(directory + '/iot.bag', 'w')
        for sensor in sensors:
            subscribers.append(rospy.Subscriber('/%s/%s/nlink_iot_frame0' % (args.robot, sensor['name']),
                                               IotFrame0, callback, sensor, queue_size=200))
        command = ['roslaunch', args.launch, 'robot_name:=' + args.robot,
                   'port_name:=' + sensors[0]['port']]
        if len(sensors) == 2:
            command.append('aux_port:=' + sensors[1]['port'])
        start('iot', command)
        emit(dict(event='started', directory=directory, sensors=sensors))
        last_input = started = monotonic()
        pending = b''
        while not stop.is_set():
            if select.select([sys.stdin], [], [], 0.2)[0]:
                chunk = os.read(sys.stdin.fileno(), 4096)
                if not chunk:
                    break
                pending += chunk
                while b'\n' in pending:
                    line, pending = pending.split(b'\n', 1)
                    if json.loads(line.decode()).get('quit'):
                        stop.set()
                last_input = monotonic()
            if monotonic() - last_input > 6:
                raise RuntimeError('IOT SSH heartbeat timed out')
            if any(proc.poll() is not None for proc in processes):
                raise RuntimeError('IOT launch exited; see ' + directory)
            for sensor in sensors:
                if monotonic() - last_frames.get(sensor['name'], started) > 12:
                    raise RuntimeError('No IOT frames: ' + sensor['name'])
        if failure:
            raise RuntimeError(failure[0])
    finally:
        for proc in reversed(processes):
            stop_process(proc)
        for subscriber in subscribers:
            subscriber.unregister()
        with lock:
            if bag is not None:
                bag.close()
                bag = None
        for handle in handles:
            handle.close()
        rospy.signal_shutdown('IOT monitor closed')
    emit(dict(event='stopped', directory=directory))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=['probe', 'monitor'])
    parser.add_argument('--ports', default='[]')
    parser.add_argument('--robot')
    parser.add_argument('--sensors')
    parser.add_argument('--directory')
    parser.add_argument('--launch')
    args = parser.parse_args()
    cache = os.path.expanduser('~/.cache/formation-console')
    if not os.path.isdir(cache):
        os.makedirs(cache)
    with open(cache + '/iot.lock', 'w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if args.mode == 'probe':
            emit(dict(event='devices', devices=discover(json.loads(args.ports))))
        else:
            monitor(args)


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        emit(dict(event='error', message=str(error)))
        sys.exit(1)
