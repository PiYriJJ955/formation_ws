#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Expose LinkTrack NodeFrame2 poses through the formation localization topics."""
from __future__ import print_function

import math

import rospy
from geometry_msgs.msg import PoseStamped
from nlink_parser.msg import LinktrackNodeframe2
from std_msgs.msg import Bool, String


class NLinkLocalizer(object):
    def __init__(self):
        self.timeout = float(rospy.get_param('~timeout', 0.4))
        if not self.is_finite(self.timeout) or self.timeout <= 0:
            raise ValueError('~timeout must be positive')
        self.world_frame = rospy.get_param('~world_frame', 'linktrack_map')
        self.last = None
        self.pose_pub = rospy.Publisher('uwb/pose', PoseStamped, queue_size=1)
        self.valid_pub = rospy.Publisher('uwb/valid', Bool, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher('uwb/status', String, queue_size=1, latch=True)
        topic = rospy.get_param('~input_topic', 'nlink_linktrack_nodeframe2')
        rospy.Subscriber(topic, LinktrackNodeframe2, self.update, queue_size=1)
        self.valid_pub.publish(Bool(data=False))
        self.status_pub.publish(String(data='WAIT_LINKTRACK'))
        rospy.Timer(rospy.Duration(0.05), self.watchdog)

    @staticmethod
    def is_finite(value):
        value = float(value)
        return not math.isnan(value) and not math.isinf(value)

    @staticmethod
    def finite(values):
        return all(NLinkLocalizer.is_finite(value) for value in values)

    def update(self, message):
        position = message.pos_3d
        quaternion = message.quaternion
        norm = sum(float(value) * float(value) for value in quaternion)
        if not self.finite(position) or not self.finite(quaternion) or norm < 1e-12:
            self.valid_pub.publish(Bool(data=False))
            self.status_pub.publish(String(data='INVALID_LINKTRACK'))
            return
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.world_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, position)
        pose.pose.orientation.w, pose.pose.orientation.x = float(quaternion[0]), float(quaternion[1])
        pose.pose.orientation.y, pose.pose.orientation.z = float(quaternion[2]), float(quaternion[3])
        self.pose_pub.publish(pose)
        self.last = pose.header.stamp
        self.valid_pub.publish(Bool(data=True))
        self.status_pub.publish(String(data='LINKTRACK'))

    def watchdog(self, _event):
        valid = self.last is not None and (rospy.Time.now() - self.last).to_sec() <= self.timeout
        if not valid:
            self.valid_pub.publish(Bool(data=False))
            self.status_pub.publish(String(data='LINKTRACK_TIMEOUT'))


if __name__ == '__main__':
    rospy.init_node('nlink_localizer')
    NLinkLocalizer()
    rospy.spin()
