#!/usr/bin/env python
# coding=utf-8
import rospy
import socket
import struct
from std_msgs.msg import Float32

def FrameListener():
    rospy.init_node('listenfollower2')

    # 发布不同的数据
    pub_v3 = rospy.Publisher('v3', Float32, queue_size=10)
    pub_w3 = rospy.Publisher('w3', Float32, queue_size=10)
    pub_distance3_1 = rospy.Publisher('distance3_1', Float32, queue_size=10)
    pub_distance3_2 = rospy.Publisher('distance3_2', Float32, queue_size=10)
    pub_f_3_1 = rospy.Publisher('f_3_1', Float32, queue_size=10)
    pub_f_3_2 = rospy.Publisher('f_3_2', Float32, queue_size=10)
    pub_fz_3_1 = rospy.Publisher('fz_3_1', Float32, queue_size=10)
    pub_fz_3_2 = rospy.Publisher('fz_3_2', Float32, queue_size=10)
    pub_x_3_absolute = rospy.Publisher('x_3_absolute', Float32, queue_size=10)    
    pub_y_3_absolute = rospy.Publisher('y_3_absolute', Float32, queue_size=10)  
    pub_v3_sensor = rospy.Publisher('v3_sensor', Float32, queue_size=10)  
    pub_w3_sensor = rospy.Publisher('w3_sensor', Float32, queue_size=10)  

    # 创建UDP套接字
    soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    soc.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 512)
    soc.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    soc.bind(('', 10002))

    rate = rospy.Rate(60.0)  # 设置频率为60Hz

    while not rospy.is_shutdown():
        # 接收UDP数据
        data, address = soc.recvfrom(1024)
        v3, w3, distance3_1, distance3_2 ,f_3_1, f_3_2 , fz_3_1 , fz_3_2 , x_3_absolute , y_3_absolute,v3_sensor,w3_sensor= struct.unpack("ffffffffffff", data)

        # print("v3, w3, distance3_1, distance3_2 ,f_3_1, f_3_2 , fz_3_1 , fz_3_2",v3, w3, distance3_1, distance3_2 ,f_3_1, f_3_2 , fz_3_1 , fz_3_2)

        # 创建自定义消息 Float32Stamped
        msg_v3 = Float32()
        msg_v3.data = v3  # 设置数据
        pub_v3.publish(msg_v3)

        msg_w3 = Float32()
        msg_w3.data = w3  # 设置数据
        pub_w3.publish(msg_w3)

        msg_distance_3_1 = Float32()
        msg_distance_3_1.data = distance3_1  # 设置数据
        pub_distance3_1.publish(msg_distance_3_1)

        msg_distance_3_2 = Float32()
        msg_distance_3_2.data = distance3_2  # 设置数据
        pub_distance3_2.publish(msg_distance_3_2)        

        msg_f_3_1 = Float32()
        msg_f_3_1.data = f_3_1  # 设置数据
        pub_f_3_1.publish(msg_f_3_1)

        msg_f_3_2 = Float32()
        msg_f_3_2.data = f_3_2  # 设置数据
        pub_f_3_2.publish(msg_f_3_2)

        msg_fz_3_1 = Float32()
        msg_fz_3_1.data = fz_3_1  # 设置数据
        pub_fz_3_1.publish(msg_fz_3_1)

        msg_fz_3_2 = Float32()
        msg_fz_3_2.data = fz_3_2  # 设置数据
        pub_fz_3_2.publish(msg_fz_3_2)   

        msg_x_3_absolute = Float32()
        msg_x_3_absolute.data = x_3_absolute
        pub_x_3_absolute.publish(msg_x_3_absolute)     

        msg_y_3_absolute = Float32()
        msg_y_3_absolute.data = y_3_absolute
        pub_y_3_absolute.publish(msg_y_3_absolute)              

        msg_v3_sensor = Float32()
        msg_v3_sensor.data = v3_sensor
        pub_v3_sensor.publish(msg_v3_sensor.data)  

        msg_w3_sensor = Float32()
        msg_w3_sensor.data = w3_sensor
        pub_w3_sensor.publish(msg_w3_sensor.data) 

        # 控制发布频率
        rate.sleep()

if __name__ == '__main__':
    try:
        FrameListener()
    except rospy.ROSInterruptException:
        pass
