#!/usr/bin/env python
# -*- coding:utf-8 -*-
from __future__ import print_function
import math
import rospy
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

def yaw_from_msg(q):
    return euler_from_quaternion([q.x,q.y,q.z,q.w])[2]

def wrap(a):
    while a > math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

class FormationController:
    """
    UGV0 leader + UGV1-4 followers.
    Algorithm:
       pd_i = p0 + R(theta0) Delta_i
       ui = v0_feedforward + omega0*J*R(theta0)Delta_i
            + kp(pd_i-pi)
    """

    def __init__(self):
        self.id = int(rospy.get_param("~ugv_id",1))
        self.rate = rospy.get_param("~rate",20.0)
        self.kp = rospy.get_param("~kp",0.8)
        self.kyaw = rospy.get_param("~kyaw",1.5)
        self.vmax = rospy.get_param("~vmax",0.15)
        self.wmax = rospy.get_param("~wmax",0.8)

        self.offset = rospy.get_param(
            "~offset",
            {"x":-0.8,"y":0.5}
        )

        ns="/ugv%d"%self.id
        self.self_pose=None
        self.leader_pose=None
        self.self_yaw=0
        self.leader_yaw=0
        self.leader_twist=Twist()

        rospy.Subscriber(
            ns+"/uwb/pose",
            PoseStamped,
            self.self_cb)

        rospy.Subscriber(
            "/ugv0/uwb/pose",
            PoseStamped,
            self.leader_cb)

        rospy.Subscriber(
            ns+"/odom_combined",
            PoseWithCovarianceStamped,
            self.self_odom_cb)

        rospy.Subscriber(
            "/ugv0/odom_combined",
            PoseWithCovarianceStamped,
            self.leader_odom_cb)

        rospy.Subscriber(
            "/ugv0/cmd_vel",
            Twist,
            self.twist_cb)

        self.pub=rospy.Publisher(
            "/cmd_vel",
            Twist,
            queue_size=10)

        rospy.Timer(
            rospy.Duration(1.0/self.rate),
            self.control)

        rospy.loginfo("UGV%d formation controller started",self.id)

    def self_cb(self,m):
        self.self_pose=m.pose.position

    def leader_cb(self,m):
        self.leader_pose=m.pose.position

    def self_odom_cb(self,m):
        self.self_yaw=yaw_from_msg(
            m.pose.pose.orientation)

    def leader_odom_cb(self,m):
        self.leader_yaw=yaw_from_msg(
            m.pose.pose.orientation)

    def twist_cb(self,m):
        self.leader_twist=m

    def control(self,e):
        if self.self_pose is None or self.leader_pose is None:
            return

        th=self.leader_yaw
        c=math.cos(th)
        s=math.sin(th)

        dx=c*self.offset["x"]-s*self.offset["y"]
        dy=s*self.offset["x"]+c*self.offset["y"]

        xd=self.leader_pose.x+dx
        yd=self.leader_pose.y+dy

        ex=xd-self.self_pose.x
        ey=yd-self.self_pose.y

        # leader cmd_vel is body frame
        vx=self.leader_twist.linear.x*c
        vy=self.leader_twist.linear.x*s

        # rotation feedforward
        vx-=self.leader_twist.angular.z*dy
        vy+=self.leader_twist.angular.z*dx

        ux=vx+self.kp*ex
        uy=vy+self.kp*ey

        target=math.atan2(uy,ux)
        err=wrap(target-self.self_yaw)

        cmd=Twist()
        cmd.angular.z=max(
            -self.wmax,
            min(self.wmax,self.kyaw*err))
        rospy.loginfo_throttle(
            1.0,
            "UGV%d err=(%.3f, %.3f), cmd=(%.3f, %.3f)",
            self.id,
            ex,
            ey,
            cmd.linear.x,
            cmd.angular.z
        )
        if abs(err)>0.8:
            cmd.linear.x=0
        else:
            cmd.linear.x=min(
                self.vmax,
                math.hypot(ux,uy))

        self.pub.publish(cmd)

if __name__=="__main__":
    rospy.init_node("formation_controller")
    FormationController()
    rospy.spin()
