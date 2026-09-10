#!/usr/bin/env python
"""ROS entry point; shared state and control code is installed by catkin."""
import sys
import rospy
from five_ugv_formation_control.follower import DisplacementFollower, self_test

if __name__ == '__main__':
    if '--self-test' in sys.argv:
        self_test()
    else:
        rospy.init_node('displacement_follower')
        DisplacementFollower()
        rospy.spin()
