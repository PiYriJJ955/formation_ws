#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <nlink_parser/LinktrackNodeframe2.h>
#include <serial/serial.h>
#include <Eigen/Dense>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

// ====================== 全局变量 ======================
float anchor1_x = 0.0, anchor1_y = 0.0;
float anchor2_x = 0.0, anchor2_y = 0.0;
float anchor3_x = 0.0, anchor3_y = 0.0;

int id_1 = 10, id_2 = 11, id_3 = 12;

float dis_1 = 0.0, dis_2 = 0.0, dis_3 = 0.0;
float v1 = 0.0, w1 = 0.0;

// 状态估计
float x_hat = 0.0, y_hat = 0.0, f_hat = 0.0;

// 时间步
float dt = 0.1;

// ====================== 矩阵定义 ======================
Eigen::Matrix<float, 3, 3> Q, R, Pu, A, Pp, H, S, K;
Eigen::Matrix<float, 3, 1> measure, measure_hat;

// ====================== UWB 回调 ======================
void uwbCallBack(const nlink_parser::LinktrackNodeframe2::ConstPtr &msg)
{
    for (const auto &node : msg->nodes)
    {
        if (node.id == id_1)
            dis_1 = node.dis;
        else if (node.id == id_2)
            dis_2 = node.dis;
        else if (node.id == id_3)
            dis_3 = node.dis;
    }
}

// ====================== 里程计回调 ======================
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    v1 = msg->twist.twist.linear.x;
}

// ====================== IMU 回调 ======================
void imuCallback(const std_msgs::Float32::ConstPtr &msg)
{
    w1 = -msg->data;
}

// ====================== 初始化矩阵 ======================
void createMatrix()
{
    Q.setIdentity();
    Q *= 0.01;

    R.setIdentity();
    R *= 0.05;

    Pu.setIdentity();
    Pu *= 1.0;
}

// ====================== EKF ======================
void EKF()
{
    // 预测步骤
    float x_minus = x_hat + dt * v1 * sin(f_hat);
    float y_minus = y_hat + dt * v1 * cos(f_hat);
    float f_minus = f_hat + dt * w1;

    A << 1, 0, dt * v1 * cos(f_hat),
        0, 1, -dt * v1 * sin(f_hat),
        0, 0, 1;

    Pp = A * Pu * A.transpose() + Q;

    // 量测
    measure << dis_1, dis_2, dis_3;

    // 计算预估距离
    float tmp1 = (x_minus - anchor1_x) * (x_minus - anchor1_x) + (y_minus - anchor1_y) * (y_minus - anchor1_y);
    float tmp2 = (x_minus - anchor2_x) * (x_minus - anchor2_x) + (y_minus - anchor2_y) * (y_minus - anchor2_y);
    float tmp3 = (x_minus - anchor3_x) * (x_minus - anchor3_x) + (y_minus - anchor3_y) * (y_minus - anchor3_y);

    // 数值保护
    tmp1 = std::max(tmp1, 1e-6f);
    tmp2 = std::max(tmp2, 1e-6f);
    tmp3 = std::max(tmp3, 1e-6f);

    float d1_hat = sqrtf(tmp1);
    float d2_hat = sqrtf(tmp2);
    float d3_hat = sqrtf(tmp3);

    measure_hat << d1_hat, d2_hat, d3_hat;

    // 计算 H
    H << (x_minus - anchor1_x) / d1_hat, (y_minus - anchor1_y) / d1_hat, 0,
        (x_minus - anchor2_x) / d2_hat, (y_minus - anchor2_y) / d2_hat, 0,
        (x_minus - anchor3_x) / d3_hat, (y_minus - anchor3_y) / d3_hat, 0;

    // 卡尔曼增益
    S = H * Pp * H.transpose() + R;

    // 数值稳定性保护
    if (S.determinant() < 1e-9)
    {
        ROS_WARN("Matrix S nearly singular, skip update!");
        return;
    }

    K = Pp * H.transpose() * S.inverse();

    // 更新
    Eigen::Matrix<float, 3, 1> y = measure - measure_hat;
    Eigen::Matrix<float, 3, 1> delta = K * y;

    x_hat = x_minus + delta(0);
    y_hat = y_minus + delta(1);
    f_hat = f_minus + delta(2);

    Pu = (Eigen::Matrix3f::Identity() - K * H) * Pp;
}

// ====================== 主函数 ======================
int main(int argc, char **argv)
{
    ros::init(argc, argv, "trilateration_ekf");
    ros::NodeHandle n("~");
    ros::NodeHandle nh;

    // 从参数服务器读取基站信息
    n.param("id_1", id_1, 10);
    n.param("id_2", id_2, 11);
    n.param("id_3", id_3, 12);

    n.param("anchor1_x", anchor1_x, 0.0f);
    n.param("anchor1_y", anchor1_y, 0.0f);
    n.param("anchor2_x", anchor2_x, 2.0f);
    n.param("anchor2_y", anchor2_y, 0.0f);
    n.param("anchor3_x", anchor3_x, 1.0f);
    n.param("anchor3_y", anchor3_y, 1.732f); // 等边三角形

    n.param("x_hat", x_hat, 0.5f);
    n.param("y_hat", y_hat, 0.5f);
    n.param("f_hat", f_hat, 0.0f);

    createMatrix();

    ros::Subscriber uwb_sub = nh.subscribe("/nlink_linktrack_nodeframe2", 1, uwbCallBack);
    ros::Subscriber imu_sub = nh.subscribe<std_msgs::Float32>("imu_fusion", 1, imuCallback);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, odomCallback);
    ros::Publisher locationx_pub = nh.advertise<std_msgs::Float32>("x_1_absolute", 1);
    ros::Publisher locationy_pub = nh.advertise<std_msgs::Float32>("y_1_absolute", 1);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();

        if (dis_1 > 0 && dis_2 > 0 && dis_3 > 0)
        {
            EKF();

            std_msgs::Float32 msg_x, msg_y;
            msg_x.data = x_hat;
            msg_y.data = y_hat;
            locationx_pub.publish(msg_x);
            locationy_pub.publish(msg_y);

            // ROS_INFO("EKF: x=%.3f, y=%.3f, f=%.3f", x_hat, y_hat, f_hat);
        }
        else
        {
            ROS_WARN("Waiting for valid UWB distances...");
        }

        loop_rate.sleep();
    }

    return 0;
}
