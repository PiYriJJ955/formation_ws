#include "ros/ros.h"

#include <nlink_parser/LinktrackNodeframe2.h>

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>

#include <std_msgs/Float32.h>

#include <tf/tf.h>

#include <fstream>

#include <cmath>

#include <geometry_msgs/Twist.h>

#include <nlink_parser/IotFrame0.h>

#include <nlink_parser/LinktrackNode2.h>

#include <nav_msgs/Odometry.h>

#include <Eigen/Dense>

#include <std_msgs/Bool.h>

#include <math.h>

#include <iostream>

#include <vector>

#include <algorithm>









std::ofstream csv_file_absolute;



// 全局变量

float dis_1 = -1.0, dis_2 = -1.0, dis_3 = -1.0, dis_4 = -1.0, dis_5 = -1.0, dis_6 = -1.0 ,dis_7 = -1.0,dis_8 = -1.0;

int id_1 = 6, id_2 = 8, id_3 = 0, id_4 = 7, id_5 = 13, id_6 = 10, id_7 = 11 , id_8 = 2;



// 关键点：记录最后一次收到数据的时间

ros::Time last_update_time; 



void uwbCallBack(const nlink_parser::LinktrackNodeframe2::ConstPtr &msg)

{

    // 只要收到消息，就更新时间戳

    last_update_time = ros::Time::now();



    for (const auto &node : msg->nodes)

    {

        if (node.id == id_1) dis_1 = node.dis;

        else if (node.id == id_2) dis_2 = node.dis;

        else if (node.id == id_3) dis_3 = node.dis;

        else if (node.id == id_4) dis_4 = node.dis;

        else if (node.id == id_5) dis_5 = node.dis;

        else if (node.id == id_6) dis_6 = node.dis;

        else if (node.id == id_7) dis_7 = node.dis;

        else if (node.id == id_8) dis_8 = node.dis;

    }

}



int main(int argc, char **argv)

{

    ros::init(argc, argv, "uwb_clean_reader");

    ros::NodeHandle nh;



    csv_file_absolute.open("/home/wheeltec/wheeltec_robot/absolutedata.csv");



    ros::Subscriber uwb_sub = nh.subscribe("/nlink_linktrack_nodeframe2", 1, uwbCallBack);

    

    ros::Rate loop_rate(10);

    while (ros::ok())

    {

        ros::spinOnce();



    ROS_INFO("dis_1: %.2f, dis_2: %.2f, dis_3: %.2f, dis_4: %.2f, dis_5: %.2f, dis_6: %.2f, dis_7: %.2f, dis_8: %.2f", dis_1, dis_2, dis_3, dis_4, dis_5, dis_6, dis_7, dis_8);





    csv_file_absolute << dis_1 << ","<< dis_2 << ","<< dis_3 

                << ","<< dis_4 << ","<< dis_5 << ","<< dis_6 << ","<< dis_7 << ","<< dis_8<<",\n";



    loop_rate.sleep();

    }

    return 0;

}