#!/usr/bin/env python
# coding=utf-8

import rospy
import socket
import struct
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

v3 = 0
w3 = 0
distance3_1 = 0
distance3_2 = 0
f_3_1 = 0
f_3_2 = 0
fz_3_1 = 0
fz_3_2 = 0
x_3_absolute = 0
y_3_absolute = 0
v3_sensor = 0
w3_sensor = 0

def imu_fusioncallback(msg):
    global w3_sensor
    w3_sensor = msg.data

def odom_callback(msg):
    global v3_sensor
    v3_sensor= msg.twist.twist.linear.x

def velocitycallback(msg):
    global v3
    global w3
    v3 = msg.linear.x
    w3 = msg.angular.z

def distance3_1_callback(msg):
    global distance3_1
    distance3_1 = msg.data

def distance3_2_callback(msg):
    global distance3_2
    distance3_2 = msg.data
 
def f_3_1_callback(msg):
    global f_3_1
    f_3_1 = msg.data   

def f_3_2_callback(msg):
    global f_3_2
    f_3_2 = msg.data    

def fz_3_1_callback(msg):
    global fz_3_1
    fz_3_1 = msg.data   

def fz_3_2_callback(msg):
    global fz_3_2
    fz_3_2 = msg.data       

def x_3_absoluteCallback(msg):
    global x_3_absolute
    x_3_absolute = msg.data   

def y_3_absoluteCallback(msg):
    global y_3_absolute
    y_3_absolute = msg.data     


def publishOdom():
    rospy.init_node('send')   
    rospy.Subscriber('distance3_1', Float32, distance3_1_callback) 
    rospy.Subscriber('distance3_2', Float32, distance3_2_callback) 
    rospy.Subscriber('f_3_1', Float32, f_3_1_callback)
    rospy.Subscriber('f_3_2', Float32, f_3_2_callback)
    rospy.Subscriber('fz_3_1', Float32, fz_3_1_callback)
    rospy.Subscriber('fz_3_2', Float32, fz_3_2_callback)    
    rospy.Subscriber('cmd_vel', Twist, velocitycallback)
    rospy.Subscriber('x_3_absolute', Float32, x_3_absoluteCallback)
    rospy.Subscriber('y_3_absolute', Float32, y_3_absoluteCallback) 
    rospy.Subscriber('imu_fusion', Float32, imu_fusioncallback)     
    rospy.Subscriber('odom',  Odometry, odom_callback)     
         
    soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    soc.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)  # UDP 广播
    network = '<broadcast>'
    rate = rospy.Rate(60.0)
    
    while not rospy.is_shutdown():
        send_data = struct.pack("ffffffffffff", v3, w3, distance3_1 , distance3_2 , f_3_1 , f_3_2 , fz_3_1 ,fz_3_2 , x_3_absolute , y_3_absolute,v3_sensor,w3_sensor)
        soc.sendto(send_data, (network, 10002))
        rate.sleep()

if __name__ == '__main__':
    try:
        publishOdom()
    except rospy.ROSInterruptException:
        soc.close()
        pass