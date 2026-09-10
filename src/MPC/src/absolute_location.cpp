#include "ros/ros.h"
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <nlink_parser/LinktrackNodeframe2.h>
#include <nlink_parser/LinktrackNode2.h>
#include <serial/serial.h>
#include <Eigen/Dense>
#include <std_msgs/Float32.h>
#include <tf/tf.h>
#include "sensor_msgs/Imu.h"
#include <nav_msgs/Odometry.h>

// 信标节点坐标（需与基站实际布置对应）
// 顺序：基站4 -> 基站5 -> 基站6
std::vector<Eigen::Vector2d> beacons;

int id_1, id_2, id_3;
float dis_1 = -1, dis_2 = -1, dis_3 = -1;

void uwbCallBack(const nlink_parser::LinktrackNodeframe2::ConstPtr& msg)
{
    // 初始化为无效
    dis_1 = dis_2 = dis_3 = -1;
    for (const auto& node : msg->nodes)
    {
        if (node.id == id_1)   // 基站1
        {
            dis_1 = node.dis;
        }
        else if (node.id == id_2)  // 基站2
        {
            dis_2 = node.dis;
        }
        else if (node.id == id_3)  // 基站3
        {
            dis_3 = node.dis;
        }
    }
}

Eigen::Vector2d trilateration(const std::vector<Eigen::Vector2d>& beacons, const std::vector<double>& distances)
{
    int numBeacons = beacons.size();
    Eigen::MatrixXd A(numBeacons - 1, 2);
    Eigen::VectorXd B(numBeacons - 1);

    for (int i = 1; i < numBeacons; ++i)
    {
        Eigen::Vector2d xi = beacons[i];
        Eigen::Vector2d xn = beacons[0];
        double di = distances[i];
        double dn = distances[0];

        A(i - 1, 0) = 2 * (xi.x() - xn.x());
        A(i - 1, 1) = 2 * (xi.y() - xn.y());
        B(i - 1) = xi.squaredNorm() - xn.squaredNorm() + dn * dn - di * di;
    }

    Eigen::Vector2d X = (A.transpose() * A).inverse() * A.transpose() * B;
    return X;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trilateration");
    ros::NodeHandle n("~");  // 使用私有命名空间以便从 launch 文件加载参数
    ros::NodeHandle nh;  
    // 从参数服务器读取基站 ID（默认值 10, 11, 12）
    n.param("id_1", id_1, 10);
    n.param("id_2", id_2, 11);
    n.param("id_3", id_3, 12);

    // 从参数服务器读取基站坐标
    std::vector<double> beacon1, beacon2, beacon3;
    n.param<std::vector<double>>("beacon1", beacon1, {0.0, 0.0});
    n.param<std::vector<double>>("beacon2", beacon2, {0.0, 2.4});
    n.param<std::vector<double>>("beacon3", beacon3, {2.4, 0.0});

    beacons.push_back(Eigen::Vector2d(beacon1[0], beacon1[1]));
    beacons.push_back(Eigen::Vector2d(beacon2[0], beacon2[1]));
    beacons.push_back(Eigen::Vector2d(beacon3[0], beacon3[1]));

    ros::Subscriber uwb_sub = n.subscribe("/nlink_linktrack_nodeframe2", 10, uwbCallBack);

    // 创建Publisher
    ros::Publisher locationx_pub = nh.advertise<std_msgs::Float32>("x_3_absolute", 10);
    ros::Publisher locationy_pub = nh.advertise<std_msgs::Float32>("y_3_absolute", 10);   

    ros::Rate loop_rate(10);


    while (ros::ok())
    {
        std_msgs::Float32 msgs1;
        std_msgs::Float32 msgs2;

        // 三个基站距离都有效才进行计算
        if (dis_1 > 0 && dis_2 > 0 && dis_3 > 0)
        {
            std::vector<double> distances = {
                1.027 * dis_1 + 0.2346,  // 基站4
                1.0163 * dis_2 + 0.2844, // 基站5
                0.9835 * dis_3 + 0.4461  // 基站6
            };

            Eigen::Vector2d pos = trilateration(beacons, distances);

            msgs1.data = pos.x();
            msgs2.data = pos.y();

            locationx_pub.publish(msgs1);
            locationy_pub.publish(msgs2);

            // ROS_INFO("-------");
            // ROS_INFO("d1:%.2f, d2:%.2f, d3:%.2f", distances[0], distances[1], distances[2]);
            // ROS_INFO("position: (%.2f, %.2f)", pos.x(), pos.y());

        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}