#!/usr/bin/env python
import rospy
from five_ugv_mpc_formation_control.controller import MPCFollower

if __name__ == '__main__':
    rospy.init_node('mpc_follower')
    MPCFollower()
    rospy.spin()
