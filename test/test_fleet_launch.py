#!/usr/bin/env python
"""Resolve every vehicle's launch configuration without starting ROS nodes."""
from __future__ import print_function

import os
import tempfile

import yaml

import roslaunch

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def resolve(package, launch, args):
    config = roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(
        os.path.join(ROOT, 'src', package, 'launch', launch),
        config, argv=args, verbose=False)
    return config


for vehicle in range(3):
    os.environ['UGV_ID'] = str(vehicle)
    os.environ['UWB_PORT'] = '/dev/test_uwb'
    prefix = '/ugv%d/' % vehicle
    config = resolve('five_ugv_uwb_localization', 'ugv.launch', [])
    assert all(node.namespace == prefix for node in config.nodes)
    assert config.params[prefix + 'linktrack/port_name'].value == '/dev/test_uwb'
    assert config.params[prefix + 'uwb_localizer/input_topic'].value == prefix + 'nlink_linktrack_nodeframe2'
    assert any(node.package == 'robot_pose_ekf' for node in config.nodes)

    if vehicle:
        config = resolve('five_ugv_formation_control', 'follower.launch', [])
        assert all(node.namespace == prefix for node in config.nodes)
        for name in ('formation_controller', 'formation_logger'):
            assert config.params[prefix + name + '/robot_name'].value == 'ugv%d' % vehicle
            assert config.params[prefix + name + '/cmd_vel_topic'].value == prefix + 'cmd_vel'
        assert config.params[prefix + 'formation_controller/enabled'].value is False
        assert config.params[prefix + 'formation_controller/data_timeout'].value == 0.6
        assert config.params[prefix + 'formation_controller/self_velocity_odom_topic'].value == prefix + 'odom'
        assert config.params[prefix + 'formation_controller/k_position'].value == 0.5
        assert config.params[prefix + 'formation_controller/rotate_exit_threshold'].value < config.params[prefix + 'formation_controller/rotate_in_place_threshold'].value
        assert config.params[prefix + 'formation_logger/controller_namespace'].value == prefix + 'formation_controller'
        assert config.params[prefix + 'formation_controller/leader_pose_topic'].value == '/ugv0/uwb/pose'

config = resolve('five_ugv_formation_control', 'follower.launch', ['ugv_id:=1', 'leader_id:=2'])
assert config.params['/ugv1/formation_controller/leader_pose_topic'].value == '/ugv2/uwb/pose'
assert config.params['/ugv1/formation_logger/leader_name'].value == 'ugv2'
for limit in (0.0, 0.08, 0.5):
    config = resolve('five_ugv_formation_control', 'follower.launch',
                     ['ugv_id:=5', 'leader_id:=3', 'max_linear:=' + str(limit)])
    assert config.params['/ugv5/formation_controller/max_linear'].value == limit
config = resolve('five_ugv_formation_control', 'follower.launch',
                 ['ugv_id:=5', 'leader_id:=3', 'data_timeout:=0.85'])
assert config.params['/ugv5/formation_controller/data_timeout'].value == 0.85
print('Fleet launch checks passed for ugv0, ugv1 and ugv2.')

with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml') as custom:
    anchors = [{'id': n, 'x': float(n % 2) * 8, 'y': float(n // 2) * 5, 'z': 1.6} for n in range(4)]
    yaml.safe_dump({'anchors': anchors, 'tag_height': 0.33}, custom)
    custom.flush()
    config = resolve('five_ugv_uwb_localization', 'ugv.launch',
                     ['ugv_id:=7', 'localization_config:=' + custom.name])
    assert config.params['/ugv7/uwb_localizer/anchors'].value == anchors
    assert config.params['/ugv7/uwb_localizer/tag_height'].value == 0.33
print('Custom anchor config reaches the real localizer launch.')

for robot, aux, expected in [('ugv2', '', 1), ('ugv3', '/dev/uwb_iot_aux', 2)]:
    config = resolve('nlink_parser', 'iot_monitor.launch',
                     ['robot_name:=' + robot, 'port_name:=/dev/uwb_iot', 'aux_port:=' + aux])
    assert len(config.nodes) == expected
    assert all(node.package == 'nlink_parser' and node.type == 'iot' for node in config.nodes)
    assert config.params['/' + robot + '/uwb_iot/iot/port_name'].value == '/dev/uwb_iot'
    if aux:
        assert config.params['/ugv3/uwb_iot_aux/iot/port_name'].value == aux
print('IOT launch checks passed for one sensor and ugv3 left/right sensors.')
