#!/usr/bin/env python
# coding=utf-8

import rospy
import socket
import struct
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from std_msgs.msg import Bool
odom_v1 = 0
imu_w1 = 0

x_1_hat = 0
y_1_hat = 0
z_1_hat = 0
f_1_hat = 0

x_2_hat = 0
y_2_hat = 0
z_2_hat = 0
f_2_hat = 0

x_3_hat = 0
y_3_hat = 0
z_3_hat = 0
f_3_hat = 0

x_1_absolute = 0
y_1_absolute = 0

formation_change = 0

v2_sensor = 0
w2_sensor = 0

def odom_callback(msg):
    global odom_v1
    odom_v1 = msg.twist.twist.linear.x
    
def imu_callback(msg):
    global imu_w1
    imu_w1 = msg.data
  
def x1_callback(msg):
    global x_1_hat
    x_1_hat = msg.data   

def y1_callback(msg):
    global y_1_hat
    y_1_hat = msg.data  

def z1_callback(msg):
    global z_1_hat
    z_1_hat = msg.data      

def f1_callback(msg):
    global f_1_hat
    f_1_hat = msg.data   

def x2_callback(msg):
    global x_2_hat
    x_2_hat = msg.data   

def y2_callback(msg):
    global y_2_hat
    y_2_hat = msg.data  

def z2_callback(msg):
    global z_2_hat
    z_2_hat = msg.data  

def f2_callback(msg):
    global f_2_hat
    f_2_hat = msg.data  

def x3_callback(msg):
    global x_3_hat
    x_3_hat = msg.data   

def y3_callback(msg):
    global y_3_hat
    y_3_hat = msg.data  

def z3_callback(msg):
    global z_3_hat
    z_3_hat = msg.data 

def f3_callback(msg):
    global f_3_hat
    f_3_hat = msg.data  

def x_1_absolutecallback(msg):
    global x_1_absolute
    x_1_absolute = msg.data  

def y_1_absolutecallback(msg):
    global y_1_absolute
    y_1_absolute = msg.data  

def formation_changecallback(msg):
    global formation_change
    formation_change = msg.data  

def v2sensorcallback(msg):
    global v2_sensor
    v2_sensor = msg.data  

def w2sensorcallback(msg):
    global w2_sensor
    w2_sensor = msg.data  

def publishOdom():
    rospy.init_node('send')
    rospy.Subscriber('odom', Odometry, odom_callback)
    rospy.Subscriber('imu_fusion', Float32, imu_callback)    


    rospy.Subscriber('x_1_hat', Float32, x1_callback)
    rospy.Subscriber('y_1_hat', Float32, y1_callback) 
    rospy.Subscriber('z_1_hat', Float32, z1_callback)        
    rospy.Subscriber('f_1_hat', Float32, f1_callback)

    rospy.Subscriber('x_2_hat', Float32, x2_callback)
    rospy.Subscriber('y_2_hat', Float32, y2_callback)
    rospy.Subscriber('z_2_hat', Float32, z2_callback)         
    rospy.Subscriber('f_2_hat', Float32, f2_callback)

    rospy.Subscriber('x_3_hat', Float32, x3_callback)
    rospy.Subscriber('y_3_hat', Float32, y3_callback) 
    rospy.Subscriber('z_3_hat', Float32, z3_callback)        
    rospy.Subscriber('f_3_hat', Float32, f3_callback)

    rospy.Subscriber('x_1_absolute', Float32, x_1_absolutecallback)        
    rospy.Subscriber('y_1_absolute', Float32, y_1_absolutecallback)  

    rospy.Subscriber('formation_change', Bool, formation_changecallback)   

    rospy.Subscriber('v2_sensor', Float32, v2sensorcallback)           
    rospy.Subscriber('w2_sensor', Float32, w2sensorcallback)      

    soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    soc.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)  # UDP 广播
    network = '<broadcast>'
    rate = rospy.Rate(60.0)
    
    while not rospy.is_shutdown():
        send_data = struct.pack("fffffffffffffffffff", odom_v1, imu_w1, x_1_hat , y_1_hat , z_1_hat , f_1_hat , x_2_hat , y_2_hat , z_2_hat , f_2_hat , x_3_hat , y_3_hat , z_3_hat , f_3_hat , x_1_absolute , y_1_absolute,formation_change,v2_sensor,w2_sensor)
        soc.sendto(send_data, (network, 10000))

        # rospy.loginfo("w=%f",imu_w1)

        rate.sleep()

if __name__ == '__main__':
    try:
        publishOdom()
    except rospy.ROSInterruptException:
        soc.close()
        pass