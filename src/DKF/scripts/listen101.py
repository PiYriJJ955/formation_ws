#!/usr/bin/env python
# coding=utf-8
import rospy
import socket
import struct
from std_msgs.msg import Float32

def FrameListener():
    rospy.init_node('listenfollower1')

    # 发布不同的数据
    pub_v2 = rospy.Publisher('v2', Float32, queue_size=10)
    pub_w2 = rospy.Publisher('w2', Float32, queue_size=10)
    pub_distance2_1 = rospy.Publisher('distance2_1', Float32, queue_size=10)
    pub_distance2_3 = rospy.Publisher('distance2_3', Float32, queue_size=10)
    pub_f_2_1 = rospy.Publisher('f_2_1', Float32, queue_size=10)
    pub_f_2_3 = rospy.Publisher('f_2_3', Float32, queue_size=10)
    pub_fz_2_1 = rospy.Publisher('fz_2_1', Float32, queue_size=10)
    pub_fz_2_3 = rospy.Publisher('fz_2_3', Float32, queue_size=10)
    pub_x_2_absolute = rospy.Publisher('x_2_absolute', Float32, queue_size=10)    
    pub_y_2_absolute = rospy.Publisher('y_2_absolute', Float32, queue_size=10)  
    pub_v2_sensor = rospy.Publisher('v2_sensor', Float32, queue_size=10)  
    pub_w2_sensor = rospy.Publisher('w2_sensor', Float32, queue_size=10)  

   
    # 创建UDP套接字
    soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    soc.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 512)
    soc.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    soc.bind(('', 10001))

    rate = rospy.Rate(60.0)  # 设置频率为60Hz

    
    while not rospy.is_shutdown():
        # 接收UDP数据

        data, address = soc.recvfrom(1024)
        v2, w2, distance2_1, distance2_3 ,f_2_1, f_2_3 , fz_2_1 , fz_2_3 , x_2_absolute , y_2_absolute , v2_sensor , w2_sensor = struct.unpack("ffffffffffff", data)
        # print("v2, w2, distance2_1, distance2_3 ,f_2_1, f_2_3 , fz_2_1 , fz_2_3",v2, w2, distance2_1, distance2_3 ,f_2_1, f_2_3 , fz_2_1 , fz_2_3)

        msg_v2 = Float32()
        msg_v2.data = v2  # 设置数据
        pub_v2.publish(msg_v2)

        msg_w2 = Float32()
        msg_w2.data = w2  # 设置数据
        pub_w2.publish(msg_w2)

        msg_distance_2_1 = Float32()
        msg_distance_2_1.data = distance2_1  # 设置数据
        pub_distance2_1.publish(msg_distance_2_1)

        msg_distance_2_3 = Float32()
        msg_distance_2_3.data = distance2_3  # 设置数据
        pub_distance2_3.publish(msg_distance_2_3)        

        msg_f_2_1 = Float32()
        msg_f_2_1.data = f_2_1  # 设置数据
        pub_f_2_1.publish(msg_f_2_1)

        msg_f_2_3 = Float32()
        msg_f_2_3.data = f_2_3  # 设置数据
        pub_f_2_3.publish(msg_f_2_3)

        msg_fz_2_1 = Float32()
        msg_fz_2_1.data = fz_2_1  # 设置数据
        pub_fz_2_1.publish(msg_fz_2_1)

        msg_fz_2_3 = Float32()
        msg_fz_2_3.data = fz_2_3  # 设置数据
        pub_fz_2_3.publish(msg_fz_2_3)    

        msg_x_2_absolute = Float32()
        msg_x_2_absolute.data = x_2_absolute
        pub_x_2_absolute.publish(msg_x_2_absolute)     

        msg_y_2_absolute = Float32()
        msg_y_2_absolute.data = y_2_absolute
        pub_y_2_absolute.publish(msg_y_2_absolute)  

        msg_v2_sensor = Float32()
        msg_v2_sensor.data = v2_sensor
        pub_v2_sensor.publish(msg_v2_sensor.data)  

        msg_w2_sensor = Float32()
        msg_w2_sensor.data = w2_sensor
        pub_w2_sensor.publish(msg_w2_sensor.data) 

        # 控制发布频率
        rate.sleep()

if __name__ == '__main__':
    try:
        FrameListener()
    except rospy.ROSInterruptException:
        pass
