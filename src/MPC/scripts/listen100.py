#!/usr/bin/env python
# coding=utf-8
import rospy
import socket
import struct
from std_msgs.msg import Float32
from std_msgs.msg import Bool

def FrameListener():
    rospy.init_node('listenleader')

    # 发布不同的数据
    pub_v1 = rospy.Publisher('v1', Float32, queue_size=10)
    pub_w1 = rospy.Publisher('w1', Float32, queue_size=10)

    pub_x1 = rospy.Publisher('x_1_hat', Float32, queue_size=10)    
    pub_y1 = rospy.Publisher('y_1_hat', Float32, queue_size=10) 
    pub_z1 = rospy.Publisher('z_1_hat', Float32, queue_size=10)       
    pub_f1 = rospy.Publisher('f_1_hat', Float32, queue_size=10) 

    pub_x2 = rospy.Publisher('x_2_hat', Float32, queue_size=10)    
    pub_y2 = rospy.Publisher('y_2_hat', Float32, queue_size=10)
    pub_z2 = rospy.Publisher('z_2_hat', Float32, queue_size=10)       
    pub_f2 = rospy.Publisher('f_2_hat', Float32, queue_size=10) 

    pub_x3 = rospy.Publisher('x_3_hat', Float32, queue_size=10)    
    pub_y3 = rospy.Publisher('y_3_hat', Float32, queue_size=10) 
    pub_z3 = rospy.Publisher('z_3_hat', Float32, queue_size=10)       
    pub_f3 = rospy.Publisher('f_3_hat', Float32, queue_size=10)

    pub_v2_sensor = rospy.Publisher('v2_sensor', Float32, queue_size=10)
    pub_w2_sensor = rospy.Publisher('w2_sensor', Float32, queue_size=10)

    pub_x_1_absolute = rospy.Publisher('x_1_absolute', Float32, queue_size=10)  
    pub_y_1_absolute = rospy.Publisher('y_1_absolute', Float32, queue_size=10) 

    pub_formation_change = rospy.Publisher('formation_change', Bool, queue_size=10)


    # 创建UDP套接字
    soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    soc.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 512)
    soc.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    soc.bind(('', 10000))

    rate = rospy.Rate(60.0)  # 设置频率为60Hz

    while not rospy.is_shutdown():
        # 接收UDP数据
        data, address = soc.recvfrom(1024)
        v1,w1,x_1_hat,y_1_hat,z_1_hat,f_1_hat,x_2_hat,y_2_hat,z_2_hat,f_2_hat,x_3_hat,y_3_hat,z_3_hat,f_3_hat,x_1_absolute,y_1_absolute,formation_change,v2_sensor,w2_sensor = struct.unpack("fffffffffffffffffff", data)

        msg_v1 = Float32()
        msg_v1.data = v1  # 设置数据
        pub_v1.publish(msg_v1)

        msg_w1 = Float32()
        msg_w1.data = w1  # 设置数据
        pub_w1.publish(msg_w1)

        msg_x_1_hat = Float32()
        msg_x_1_hat.data = x_1_hat  # 设置数据
        pub_x1.publish(msg_x_1_hat)

        msg_y_1_hat = Float32()
        msg_y_1_hat.data = y_1_hat  # 设置数据
        pub_y1.publish(msg_y_1_hat)  

        msg_z_1_hat = Float32()
        msg_z_1_hat.data = z_1_hat  # 设置数据
        pub_z1.publish(msg_z_1_hat)    

        msg_f_1_hat = Float32()
        msg_f_1_hat.data = f_1_hat  # 设置数据
        pub_f1.publish(msg_f_1_hat) 

        msg_x_2_hat = Float32()
        msg_x_2_hat.data = x_2_hat  # 设置数据
        pub_x2.publish(msg_x_2_hat)

        msg_y_2_hat = Float32()
        msg_y_2_hat.data = y_2_hat  # 设置数据
        pub_y2.publish(msg_y_2_hat)  

        msg_z_2_hat = Float32()
        msg_z_2_hat.data = z_2_hat  # 设置数据
        pub_z2.publish(msg_z_2_hat)    

        msg_f_2_hat = Float32()
        msg_f_2_hat.data = f_2_hat  # 设置数据
        pub_f2.publish(msg_f_2_hat) 

        msg_x_3_hat = Float32()
        msg_x_3_hat.data = x_3_hat  # 设置数据
        pub_x3.publish(msg_x_3_hat)

        msg_y_3_hat = Float32()
        msg_y_3_hat.data = y_3_hat  # 设置数据
        pub_y3.publish(msg_y_3_hat)  

        msg_z_3_hat = Float32()
        msg_z_3_hat.data = z_3_hat  # 设置数据
        pub_z3.publish(msg_z_3_hat)    

        msg_f_3_hat = Float32()
        msg_f_3_hat.data = f_3_hat  # 设置数据
        pub_f3.publish(msg_f_3_hat) 

        msg_x_1_absolute = Float32()
        msg_x_1_absolute.data = x_1_absolute  # 设置数据
        pub_x_1_absolute.publish(msg_x_1_absolute) 

        msg_y_1_absolute = Float32()
        msg_y_1_absolute.data = y_1_absolute  # 设置数据
        pub_y_1_absolute.publish(msg_y_1_absolute)         

        msg_formation_change = Bool()
        msg_formation_change.data = formation_change  # 设置数据
        pub_formation_change.publish(msg_formation_change) 

        msg_v2_sensor = Float32()
        msg_v2_sensor.data = v2_sensor  # 设置数据
        pub_v2_sensor.publish(msg_v2_sensor)  

        msg_w2_sensor = Float32()
        msg_w2_sensor.data = w2_sensor  # 设置数据
        pub_w2_sensor.publish(msg_w2_sensor)  

        # 控制发布频率
        rate.sleep()

if __name__ == '__main__':
    try:
        FrameListener()
    except rospy.ROSInterruptException:
        pass
