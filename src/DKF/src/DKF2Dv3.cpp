#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>
#include <fstream>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <nlink_parser/IotFrame0.h>
#include <nlink_parser/LinktrackNodeframe2.h>
#include <nlink_parser/LinktrackNode2.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <std_msgs/Bool.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <algorithm>

std::ofstream csv_file;
std::ofstream csv_file_1;

const std::vector<uint32_t> TARGET_UIDS = {352336640,805319168,805334528,335564288};

struct NodeData
{
    uint32_t uid     = 0;
    float    dis     = 0.0f;
    float    aoa_h   = 0.0f;
    float    aoa_v   = 0.0f;
    float    fp_rssi = 0.0f;
    float    rx_rssi = 0.0f;
    std::string user_data;
};

std::map<uint32_t, NodeData> g_node_map;
std::map<uint32_t, NodeData> g_node_map2;

NodeData node_a, node_b;


nlink_parser::IotFrame0 uwb_msg0;
nlink_parser::IotFrame0 uwb_msg1;
std_msgs::Float32 msg1 , msg2 , msg3 , msg4;
std_msgs::Float32 msg5 , msg6 , msg7 , msg8 , msg9 , msg10 , msg11 , msg12 ,msg13;
geometry_msgs::Twist twist;
float uid1 , uid2 ,uid3;
float uid4 , uid5 ,uid6;
float distance1_2 = 0 , distance1_3 = 0 , f_1_2 = 0 , f_1_3 = 0 ;
float distance2_1 = 0 , distance2_3 = 0 , f_2_1 = 0 , f_2_3 = 0 ;
float distance3_1 = 0 , distance3_2 = 0 , f_3_1 = 0 , f_3_2 = 0 ;
float x_1_absolute = 0 , y_1_absolute = 0 , x_2_absolute = 0 , y_2_absolute = 0 , x_3_absolute = 0 , y_3_absolute = 0 ;

float v1 , w1;
float v2 , w2;
float v3 , w3;
float dt = 0.1;
float varQ1 = 0.01 ,varQ2 = 0.01 ,varQ3 = 0.01 , varR1 = 1 , varR2 = 1 , varR3 = 1;
float v2_sensor,w2_sensor,v3_sensor,w3_sensor,w1_sensor;

//过程噪声协方差
Eigen::Matrix<float,3,3> Q1;
Eigen::Matrix<float,3,3> Q2;
Eigen::Matrix<float,3,3> Q3;
Eigen::Matrix<float,9,9> Q;

// 初始先验误差协方差矩阵 Pu 9x9 单位矩阵
Eigen::Matrix<float,9,9> Pu = Eigen::Matrix<float,9,9>::Identity();
// 初始后验误差协方差矩阵 Pp 9x9 0矩阵
Eigen::Matrix<float,9,9> Pp = Eigen::Matrix<float,9,9>::Identity();

// 初始先验状态
float x_1_minus=0,y_1_minus=0,f_1_minus=0;
float x_2_minus=-0.2,y_2_minus=0,f_2_minus=0;
float x_3_minus=0.2,y_3_minus=0,f_3_minus=0;
// Xp
Eigen::Matrix<float,9,1> Xp;

// 初始后验坐标
float x_1_hat=0 , y_1_hat=0 , f_1_hat=0;
float x_2_hat=-0.2 , y_2_hat=0 , f_2_hat=0;
float x_3_hat=0.2 , y_3_hat=0 , f_3_hat=0;
// Xu滤波器输出
Eigen::Matrix<float,9,1> Xu;

// 量测坐标
float x_12,y_12,f_12,x_13,y_13,f_13,x_21,y_21,f_21,x_23,y_23,f_23,x_31,y_31,f_31,x_32,y_32,f_32;

// 伪量测坐标
float x_12_hat,y_12_hat,f_12_hat,x_13_hat,y_13_hat,f_13_hat,x_21_hat,y_21_hat,f_21_hat,x_23_hat,y_23_hat,f_23_hat,x_31_hat,y_31_hat,f_31_hat,x_32_hat,y_32_hat,f_32_hat;

//量测
Eigen::Matrix<float,18,1> measure;

//伪量测
Eigen::Matrix<float,18,1> measure_hat;

//状态转移矩阵
Eigen::Matrix<float,3,3> A1;
Eigen::Matrix<float,3,3> A2;
Eigen::Matrix<float,3,3> A3;
Eigen::Matrix<float,3,3> M;
Eigen::Matrix<float,9,9> A;

float hc1,hc2,hc3,hc4,hc5,hc6,hc7,hc8,hc9,hc10,hc11,hc12;

Eigen::Matrix<float,18,3> D1;
Eigen::Matrix<float,18,3> D2;
Eigen::Matrix<float,18,3> D3;
Eigen::Matrix<float,18,9> H;
Eigen::Matrix<float,18,18> S;
Eigen::Matrix<float,9,18> K;

void DKF()
{

    Q1 << varQ1, 0, 0,
          0, varQ1, 0,
          0, 0, varQ1;
    Q2 << varQ2, 0, 0,
          0, varQ2, 0,
          0, 0, varQ2;
    Q3 << varQ3, 0, 0,
          0, varQ3, 0,
          0, 0, varQ3;

    Eigen::Matrix<float,18,18> R = Eigen::Matrix<float,18,18>::Identity();
/*    R = R*varR;*/
    R << varR1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, varR2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, varR3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, varR1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, varR2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, varR3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, varR1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, varR2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, varR3, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, varR1, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, varR2, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, varR3, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, varR1, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, varR2, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, varR3, 0, 0, 0, 
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, varR1, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, varR2, 0,          
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, varR3; 

    // 状态转移方程
    x_1_minus = x_1_hat+dt*v1*sin(f_1_hat);
    y_1_minus = y_1_hat+dt*v1*cos(f_1_hat);
    f_1_minus = f_1_hat + dt*w1;

    x_2_minus = x_2_hat+dt*v2_sensor*sin(f_2_hat);
    y_2_minus = y_2_hat+dt*v2_sensor*cos(f_2_hat);
    f_2_minus = f_2_hat + dt*w2_sensor;       

    x_3_minus = x_3_hat+dt*v3*sin(f_3_hat);
    y_3_minus = y_3_hat+dt*v3*cos(f_3_hat);
    f_3_minus = f_3_hat + dt*w3;

    Xp << x_1_minus, y_1_minus, f_1_minus, x_2_minus, y_2_minus, f_2_minus, x_3_minus, y_3_minus, f_3_minus;

    // 量测坐标转换
    // 1->2
    x_12 = -distance1_2*sin(f_1_2);
    y_12 = distance1_2*cos(f_1_2);
    f_12 = -f_1_2+f_2_1;

    // 1->3
    x_13 = distance1_3*sin(f_1_3);
    y_13 = -distance1_3*cos(f_1_3);
    f_13 = -f_1_3+f_3_1;

    // 2->1
    x_21 = distance2_1*sin(f_2_1);
    y_21 = -distance2_1*cos(f_2_1);
    f_21 = -f_2_1+f_1_2;

    // 2->3
    x_23 = distance2_3*sin(f_2_3);
    y_23 = -distance2_3*cos(f_2_3);
    f_23 = -f_2_3+f_3_2;  

    // 3->1
    x_31 = -distance3_1*sin(f_3_1);
    y_31 = distance3_1*cos(f_3_1);
    f_31 = -f_3_1+f_1_3;

    // 3->2
    x_32 = -distance3_2*sin(f_3_2);
    y_32 = distance3_2*cos(f_3_2);
    f_32 = -f_3_2+f_2_3;

    measure << x_12, y_12, f_12, x_13, y_13, f_13, x_21, y_21, f_21, x_23, y_23, f_23, x_31, y_31, f_31, x_32, y_32, f_32;

    A1 << 1 , 0 , dt*v1*cos(f_1_hat),
          0 , 1 , dt*v1*(-sin(f_1_hat)),
          0 , 0 , 1;
    A2 << 1 , 0 , dt*v2*cos(f_2_hat),
          0 , 1 , dt*v2*(-sin(f_2_hat)),
          0 , 0 , 1;       
    A3 << 1 , 0 , dt*v3*cos(f_3_hat),
          0 , 1 , dt*v3*(-sin(f_3_hat)),
          0 , 0 , 1;  

    M.setZero(3,3);
    A.block<3,3>(0,0) = A1;
    A.block<3,3>(0,3) = M;
    A.block<3,3>(0,6) = M;
    A.block<3,3>(3,0) = M;
    A.block<3,3>(3,3) = A2;
    A.block<3,3>(3,6) = M;
    A.block<3,3>(6,0) = M;
    A.block<3,3>(6,3) = M;
    A.block<3,3>(6,6) = A3;

    Q.block<3,3>(0,0) = Q1;
    Q.block<3,3>(0,3) = M;
    Q.block<3,3>(0,6) = M;
    Q.block<3,3>(3,0) = M;
    Q.block<3,3>(3,3) = Q2;
    Q.block<3,3>(3,6) = M;
    Q.block<3,3>(6,0) = M;
    Q.block<3,3>(6,3) = M;
    Q.block<3,3>(6,6) = Q3; 

    Pp = A*Pu*A.transpose()+Q; 

    // 构建伪量测
    // 1->2
    x_12_hat = cos(f_1_minus)*(x_2_minus-x_1_minus)-sin(f_1_minus)*(y_2_minus-y_1_minus);
    y_12_hat = sin(f_1_minus)*(x_2_minus-x_1_minus)+cos(f_1_minus)*(y_2_minus-y_1_minus);
    f_12_hat = f_2_minus - f_1_minus;

    // 1->3
    x_13_hat = cos(f_1_minus)*(x_3_minus-x_1_minus)-sin(f_1_minus)*(y_3_minus-y_1_minus);
    y_13_hat = sin(f_1_minus)*(x_3_minus-x_1_minus)+cos(f_1_minus)*(y_3_minus-y_1_minus);
    f_13_hat = f_3_minus - f_1_minus;

    // 2->1
    x_21_hat = cos(f_2_minus)*(x_1_minus-x_2_minus)-sin(f_2_minus)*(y_1_minus-y_2_minus);
    y_21_hat = sin(f_2_minus)*(x_1_minus-x_2_minus)+cos(f_2_minus)*(y_1_minus-y_2_minus);
    f_21_hat = f_1_minus - f_2_minus;

    // 2->3
    x_23_hat = cos(f_2_minus)*(x_3_minus-x_2_minus)-sin(f_2_minus)*(y_3_minus-y_2_minus);
    y_23_hat = sin(f_2_minus)*(x_3_minus-x_2_minus)+cos(f_2_minus)*(y_3_minus-y_2_minus);
    f_23_hat = f_3_minus - f_2_minus;

    // 3->1
    x_31_hat = cos(f_3_minus)*(x_1_minus-x_3_minus)-sin(f_3_minus)*(y_1_minus-y_3_minus);
    y_31_hat = sin(f_3_minus)*(x_1_minus-x_3_minus)+cos(f_3_minus)*(y_1_minus-y_3_minus);
    f_31_hat = f_1_minus - f_3_minus;

    // 3->2
    x_32_hat = cos(f_3_minus)*(x_2_minus-x_3_minus)-sin(f_3_minus)*(y_2_minus-y_3_minus);
    y_32_hat = sin(f_3_minus)*(x_2_minus-x_3_minus)+cos(f_3_minus)*(y_2_minus-y_3_minus);
    f_32_hat = f_2_minus - f_3_minus;

    measure_hat << x_12_hat, y_12_hat, f_12_hat, x_13_hat, y_13_hat, f_13_hat, x_21_hat, y_21_hat, f_21_hat, x_23_hat, y_23_hat, f_23_hat, x_31_hat, y_31_hat, f_31_hat, x_32_hat, y_32_hat, f_32_hat;

    hc1 = -sin(f_1_minus)*(x_2_minus-x_1_minus)-cos(f_1_minus)*(y_2_minus-y_1_minus);
    hc2 = cos(f_1_minus)*(x_2_minus-x_1_minus)-sin(f_1_minus)*(y_2_minus-y_1_minus);
    hc3 = -sin(f_1_minus)*(x_3_minus-x_1_minus)-cos(f_1_minus)*(y_3_minus-y_1_minus);
    hc4 = cos(f_1_minus)*(x_3_minus-x_1_minus)-sin(f_1_minus)*(y_3_minus-y_1_minus);

    D1 << -cos(f_1_minus), sin(f_1_minus),  hc1,
          -sin(f_1_minus), -cos(f_1_minus), hc2,
          0, 0, -1,
          -cos(f_1_minus), sin(f_1_minus),  hc3,
          -sin(f_1_minus), -cos(f_1_minus), hc4,
          0, 0, -1,
          cos(f_2_minus),  -sin(f_2_minus), 0,
          sin(f_2_minus),  cos(f_2_minus),  0,
          0, 0,  1,
          0, 0,  0,
          0, 0,  0,
          0, 0,  0,
          cos(f_3_minus),  -sin(f_3_minus), 0,
          sin(f_3_minus),  cos(f_3_minus),  0,
          0, 0,  1,
          0, 0,  0,
          0, 0,  0,
          0, 0,  0; 
          
    hc5 = -sin(f_2_minus)*(x_1_minus-x_2_minus)-cos(f_2_minus)*(y_1_minus-y_2_minus);
    hc6 = cos(f_2_minus)*(x_1_minus-x_2_minus)-sin(f_2_minus)*(y_1_minus-y_2_minus);
    hc7 = -sin(f_2_minus)*(x_3_minus-x_2_minus)-cos(f_2_minus)*(y_3_minus-y_2_minus);
    hc8 = cos(f_2_minus)*(x_3_minus-x_2_minus)-sin(f_2_minus)*(y_3_minus-y_2_minus);

    D2 << cos(f_1_minus), -sin(f_1_minus), 0,
          sin(f_1_minus),  cos(f_1_minus), 0,
          0, 0, 1,
          0, 0, 0,
          0, 0, 0,
          0, 0, 0,
          -cos(f_2_minus),  sin(f_2_minus), hc5,
          -sin(f_2_minus), -cos(f_2_minus), hc6,
          0, 0, -1,
          -cos(f_2_minus),  sin(f_2_minus), hc7,
          -sin(f_2_minus), -cos(f_2_minus), hc8,
          0, 0, -1,
          0, 0, 0,
          0, 0, 0,
          0, 0, 0,
          cos(f_3_minus),  -sin(f_3_minus), 0,
          sin(f_3_minus),   cos(f_3_minus), 0,
          0, 0, 1;

    hc9 = -sin(f_3_minus)*(x_1_minus-x_3_minus)-cos(f_3_minus)*(y_1_minus-y_3_minus);
    hc10 = cos(f_3_minus)*(x_1_minus-x_3_minus)-sin(f_3_minus)*(y_1_minus-y_3_minus);
    hc11 = -sin(f_3_minus)*(x_2_minus-x_3_minus)-cos(f_3_minus)*(y_2_minus-y_3_minus);
    hc12 = cos(f_3_minus)*(x_2_minus-x_3_minus)-sin(f_3_minus)*(y_2_minus-y_3_minus);

    D3 << 0, 0, 0,
          0, 0, 0,
          0, 0, 0,  
          cos(f_1_minus), -sin(f_1_minus), 0,
          sin(f_1_minus),  cos(f_1_minus), 0,
          0, 0, 1,
          0, 0, 0,
          0, 0, 0,
          0, 0, 0,
          cos(f_2_minus), -sin(f_2_minus), 0,
          sin(f_2_minus),  cos(f_2_minus), 0,
          0, 0, 1,
          -cos(f_3_minus), sin(f_3_minus),  hc9,
          -sin(f_3_minus), -cos(f_3_minus), hc10,
          0, 0, -1,
          -cos(f_3_minus), sin(f_3_minus),  hc11,
          -sin(f_3_minus), -cos(f_3_minus), hc12,
          0, 0, -1;

    H << D1, D2, D3;

    S = H*Pp*H.transpose() + R;

    K = Pp*H.transpose()*S.inverse();

    Xu = Xp + K*(measure - measure_hat);

    x_1_hat = Xu(0);
    y_1_hat = Xu(1);
    f_1_hat = Xu(2);
    x_2_hat = Xu(3);
    y_2_hat = Xu(4);
    f_2_hat = Xu(5);
    x_3_hat = Xu(6);
    y_3_hat = Xu(7);
    f_3_hat = Xu(8);  

    Pu = Pp - K*H*Pp;
}


void imuCallback(const std_msgs::Float32::ConstPtr& msg)
{
    w1 = msg->data;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    v1 = msg->twist.twist.linear.x;
}

void uwb0Callback(const nlink_parser::IotFrame0::ConstPtr& msg) 
{
    for (const auto& n : msg->nodes)
    {
        NodeData d;
        d.uid     = n.uid;
        d.dis     = n.dis;
        d.aoa_h   = n.aoa_angle_horizontal;
        d.aoa_v   = n.aoa_angle_vertical;
        d.fp_rssi = n.fp_rssi;
        d.rx_rssi = n.rx_rssi;
        d.user_data = n.user_data;

        g_node_map[n.uid] = d;
    }

    /* 用统一的 find/assign，避免重复代码 */
    auto get_node = [](uint32_t id) -> NodeData 
    {
        auto it = g_node_map.find(id);
        return (it != g_node_map.end()) ? it->second : NodeData{};
    };

    node_a = get_node(TARGET_UIDS[0]);
}

void uwb1Callback(const nlink_parser::IotFrame0::ConstPtr& msg) 
{
    for (const auto& n : msg->nodes)
    {
        NodeData d;
        d.uid     = n.uid;
        d.dis     = n.dis;
        d.aoa_h   = n.aoa_angle_horizontal;
        d.aoa_v   = n.aoa_angle_vertical;
        d.fp_rssi = n.fp_rssi;
        d.rx_rssi = n.rx_rssi;
        d.user_data = n.user_data;

        g_node_map2[n.uid] = d;
    }

    /* 用统一的 find/assign，避免重复代码 */
    auto get_node = [](uint32_t id) -> NodeData 
    {
        auto it = g_node_map2.find(id);
        return (it != g_node_map2.end()) ? it->second : NodeData{};
    };

    node_b = get_node(TARGET_UIDS[3]);
}

void v2Callback(const std_msgs::Float32::ConstPtr& msg)
{
    v2 = msg->data;
}

void w2Callback(const std_msgs::Float32::ConstPtr& msg)
{
    w2 = msg->data;
}

void v3Callback(const std_msgs::Float32::ConstPtr& msg)
{
    v3 = msg->data;
}

void w3Callback(const std_msgs::Float32::ConstPtr& msg)
{
    w3 = msg->data;
}

void d21Callback(const std_msgs::Float32::ConstPtr& msg)
{
    distance2_1 = msg->data;
}

void d23Callback(const std_msgs::Float32::ConstPtr& msg)
{
    distance2_3 = msg->data;
}

void d31Callback(const std_msgs::Float32::ConstPtr& msg)
{
    distance3_1 = msg->data;
}

void d32Callback(const std_msgs::Float32::ConstPtr& msg)
{
    distance3_2 = msg->data;
}

void f21Callback(const std_msgs::Float32::ConstPtr& msg)
{
    f_2_1 = msg->data;
}

void f23Callback(const std_msgs::Float32::ConstPtr& msg)
{
    f_2_3 = msg->data;
}

void f31Callback(const std_msgs::Float32::ConstPtr& msg)
{
    f_3_1 = msg->data;
}

void f32Callback(const std_msgs::Float32::ConstPtr& msg)
{
    f_3_2 = msg->data;
}

void x_1_absoluteCallback(const std_msgs::Float32::ConstPtr& msg)
{
    x_1_absolute = msg->data;
}

void y_1_absoluteCallback(const std_msgs::Float32::ConstPtr& msg)
{
    y_1_absolute = msg->data;
}

void x_2_absoluteCallback(const std_msgs::Float32::ConstPtr& msg)
{
    x_2_absolute = msg->data;
}

void y_2_absoluteCallback(const std_msgs::Float32::ConstPtr& msg)
{
    y_2_absolute = msg->data;
}

void x_3_absoluteCallback(const std_msgs::Float32::ConstPtr& msg)
{
    x_3_absolute = msg->data;
}

void y_3_absoluteCallback(const std_msgs::Float32::ConstPtr& msg)
{
    y_3_absolute = msg->data;
}


int StartLocation;
void StartLocationCallback(const std_msgs::Bool::ConstPtr& msg) 
{
    StartLocation = msg->data;
}


void v2_sensorCallback(const std_msgs::Float32::ConstPtr& msg)
{
    v2_sensor = msg->data;
}

void w2_sensorCallback(const std_msgs::Float32::ConstPtr& msg)
{
    w2_sensor = msg->data;
}

void v3_sensorCallback(const std_msgs::Float32::ConstPtr& msg)
{
    v3_sensor = msg->data;
}

void w3_sensorCallback(const std_msgs::Float32::ConstPtr& msg)
{
    w3_sensor = msg->data;
}

void w1_sensorCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    w1_sensor = msg->angular_velocity.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DKF2D");
    ros::NodeHandle n;
	//打开文件
	csv_file.open("/home/wheeltec/wheeltec_robot/data_dkf2D.csv");
    csv_file_1.open("/home/wheeltec/wheeltec_robot/data_mpc.csv");

    ros::Subscriber uwb_sub0 = n.subscribe<nlink_parser::IotFrame0>("nlink_iot_frame0", 1, uwb0Callback);
    ros::Subscriber uwb_sub1 = n.subscribe<nlink_parser::IotFrame0>("nlink_iot_frame1", 1, uwb1Callback);   
    ros::Subscriber imu_sub = n.subscribe<std_msgs::Float32>("imu_fusion", 1, imuCallback);
    ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("odom", 1, odomCallback);
    ros::Subscriber v2_sub = n.subscribe<std_msgs::Float32>("/v2",1,v2Callback);
    ros::Subscriber w2_sub = n.subscribe<std_msgs::Float32>("/w2",1,w2Callback);
    ros::Subscriber v3_sub = n.subscribe<std_msgs::Float32>("/v3",1,v3Callback);
    ros::Subscriber w3_sub = n.subscribe<std_msgs::Float32>("/w3",1,w3Callback);
    ros::Subscriber d21_sub = n.subscribe<std_msgs::Float32>("/distance2_1",1,d21Callback);
    ros::Subscriber f21_sub = n.subscribe<std_msgs::Float32>("/f_2_1",1,f21Callback);
    ros::Subscriber d23_sub = n.subscribe<std_msgs::Float32>("/distance2_3",1,d23Callback);
    ros::Subscriber f23_sub = n.subscribe<std_msgs::Float32>("/f_2_3",1,f23Callback);
    ros::Subscriber d31_sub = n.subscribe<std_msgs::Float32>("/distance3_1",1,d31Callback);
    ros::Subscriber f31_sub = n.subscribe<std_msgs::Float32>("/f_3_1",1,f31Callback);
    ros::Subscriber d32_sub = n.subscribe<std_msgs::Float32>("/distance3_2",1,d32Callback);
    ros::Subscriber f32_sub = n.subscribe<std_msgs::Float32>("/f_3_2",1,f32Callback);
    ros::Subscriber sub = n.subscribe("StartLocation", 1, StartLocationCallback);
    ros::Subscriber x_1_absolute_sub = n.subscribe<std_msgs::Float32>("/x_1_absolute",1,x_1_absoluteCallback);
    ros::Subscriber y_1_absolute_sub = n.subscribe<std_msgs::Float32>("/y_1_absolute",1,y_1_absoluteCallback);    
    ros::Subscriber x_2_absolute_sub = n.subscribe<std_msgs::Float32>("/x_2_absolute",1,x_2_absoluteCallback);
    ros::Subscriber y_2_absolute_sub = n.subscribe<std_msgs::Float32>("/y_2_absolute",1,y_2_absoluteCallback);   
    ros::Subscriber x_3_absolute_sub = n.subscribe<std_msgs::Float32>("/x_3_absolute",1,x_3_absoluteCallback);
    ros::Subscriber y_3_absolute_sub = n.subscribe<std_msgs::Float32>("/y_3_absolute",1,y_3_absoluteCallback); 

    ros::Subscriber v2_sensor_sub = n.subscribe<std_msgs::Float32>("/v2_sensor",1,v2_sensorCallback);
    ros::Subscriber w2_sensor_sub = n.subscribe<std_msgs::Float32>("/w2_sensor",1,w2_sensorCallback);  
    ros::Subscriber v3_sensor_sub = n.subscribe<std_msgs::Float32>("/v3_sensor",1,v3_sensorCallback);
    ros::Subscriber w3_sensor_sub = n.subscribe<std_msgs::Float32>("/w3_sensor",1,w3_sensorCallback);

    ros::Subscriber w1_sensor_sub  = n.subscribe("imu",  1, w1_sensorCallback);



    ros::Publisher d12_pub = n.advertise<std_msgs::Float32>("/distance1_2", 1);
    ros::Publisher d13_pub = n.advertise<std_msgs::Float32>("/distance1_3", 1);   
    ros::Publisher f12_pub = n.advertise<std_msgs::Float32>("/f_1_2", 1);
    ros::Publisher f13_pub = n.advertise<std_msgs::Float32>("/f_1_3", 1);

    ros::Publisher x1_pub = n.advertise<std_msgs::Float32>("/x_1_hat", 1);
    ros::Publisher y1_pub = n.advertise<std_msgs::Float32>("/y_1_hat", 1);   
    ros::Publisher f1_pub = n.advertise<std_msgs::Float32>("/f_1_hat", 1);
    ros::Publisher x2_pub = n.advertise<std_msgs::Float32>("/x_2_hat", 1);
    ros::Publisher y2_pub = n.advertise<std_msgs::Float32>("/y_2_hat", 1);   
    ros::Publisher f2_pub = n.advertise<std_msgs::Float32>("/f_2_hat", 1);
    ros::Publisher x3_pub = n.advertise<std_msgs::Float32>("/x_3_hat", 1);
    ros::Publisher y3_pub = n.advertise<std_msgs::Float32>("/y_3_hat", 1);   
    ros::Publisher f3_pub = n.advertise<std_msgs::Float32>("/f_3_hat", 1);    

    n.getParam("x_1_hat",x_1_hat);
    n.getParam("y_1_hat",y_1_hat);    
    n.getParam("f_1_hat",f_1_hat);
    n.getParam("x_2_hat",x_2_hat);
    n.getParam("y_2_hat",y_2_hat);    
    n.getParam("f_2_hat",f_2_hat);
    n.getParam("x_3_hat",x_3_hat);
    n.getParam("y_3_hat",y_3_hat);    
    n.getParam("f_3_hat",f_3_hat);

    n.getParam("x_1_minus",x_1_minus);
    n.getParam("y_1_minus",y_1_minus);    
    n.getParam("f_1_minus",f_1_minus);
    n.getParam("x_2_minus",x_2_minus);
    n.getParam("y_2_minus",y_2_minus);    
    n.getParam("f_2_minus",f_2_minus);
    n.getParam("x_3_minus",x_3_minus);
    n.getParam("y_3_minus",y_3_minus);    
    n.getParam("f_3_minus",f_3_minus);    

    n.getParam("varQ1",varQ1);
    n.getParam("varQ2",varQ2);
    n.getParam("varQ3",varQ3);

    n.getParam("varR1",varR1);          
    n.getParam("varR2",varR2);     
    n.getParam("varR3",varR3); 





    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        distance1_2 = node_a.dis;   // A→B 距离
        f_1_2       = node_a.aoa_h; // A→B 水平角

        distance1_3 = node_b.dis;   // A→B 距离
        f_1_3       = node_b.aoa_h; // A→B 水平角
  
        msg1.data = distance1_2;
        msg2.data = distance1_3;
        msg3.data = f_1_2;
        msg4.data = f_1_3;
        
        d12_pub.publish(msg1);
        d13_pub.publish(msg2);
        f12_pub.publish(msg3);
        f13_pub.publish(msg4);


        f_1_2 = (0.6518*f_1_2-8.9788)*M_PI/180;
        f_2_1 = (0.6421*f_2_1-9.5092)*M_PI/180;

        f_1_3 = (0.715*f_1_3-11.4498)*M_PI/180;
        f_3_1 = (0.6408*f_3_1-10.6376)*M_PI/180;

        f_2_3 = (0.5482*f_2_3-7.0443)*M_PI/180;
        f_3_2 = (0.489*f_3_2-4.7975)*M_PI/180;

        w1 = -w1;
        w1_sensor = -w1_sensor;
        w2_sensor = -w2_sensor;
        w3_sensor = -w3_sensor;

        if (StartLocation == 1)
        {

            DKF();
        }

        msg5.data  = x_1_hat;
        msg6.data  = y_1_hat;
        msg7.data  = f_1_hat;
        msg8.data  = x_2_hat;
        msg9.data  = y_2_hat;
        msg10.data = f_2_hat;
        msg11.data = x_3_hat;
        msg12.data = y_3_hat;
        msg13.data = f_3_hat;

        x1_pub.publish(msg5);
        y1_pub.publish(msg6);
        f1_pub.publish(msg7);
        x2_pub.publish(msg8);
        y2_pub.publish(msg9);
        f2_pub.publish(msg10);
        x3_pub.publish(msg11);
        y3_pub.publish(msg12);
        f3_pub.publish(msg13);        


        ROS_INFO("---------------------------------");
        ROS_INFO("v1:%f",v1);
        ROS_INFO("v2:%f",v2);
        ROS_INFO("v3:%f",v3);
        ROS_INFO("w1:%f",w1); 
        ROS_INFO("w2:%f",w2); 
        ROS_INFO("w3:%f",w3); 
        ROS_INFO("distance1_2:%f",distance1_2);
        ROS_INFO("distance1_3:%f",distance1_3);  
        ROS_INFO("distance2_1:%f",distance2_1);
        ROS_INFO("distance2_3:%f",distance2_3);
        ROS_INFO("distance3_1:%f",distance3_1);
        ROS_INFO("distance3_2:%f",distance3_2);
        ROS_INFO("f_1_2:%f",f_1_2);              
        ROS_INFO("f_1_3:%f",f_1_3); 
        ROS_INFO("f_2_1:%f",f_2_1);              
        ROS_INFO("f_2_3:%f",f_2_3);
        ROS_INFO("f_3_1:%f",f_3_1);              
        ROS_INFO("f_3_2:%f",f_3_2);

        ROS_INFO("w1_sensor:%f",w1_sensor);        
        ROS_INFO("v2_sensor:%f",v2_sensor);
        ROS_INFO("w2_sensor:%f",w2_sensor);
        ROS_INFO("v3_sensor:%f",v3_sensor);
        ROS_INFO("w3_sensor:%f",w3_sensor);

        if (StartLocation == 0)
        {
            ROS_INFO("Collaborative Location is not activated");
        }
        if (StartLocation == 1)
        {
            ROS_INFO("Collaborative Location is activated");
        }        
        ROS_INFO("Leader:CL(%f,%f,%f),absolute(%f,%f)",x_1_hat,y_1_hat,f_1_hat,x_1_absolute,y_1_absolute);  
        ROS_INFO("Follower1:CL(%f,%f,%f),absolute(%f,%f)",x_2_hat,y_2_hat,f_2_hat,x_2_absolute,y_2_absolute);  
        ROS_INFO("Follower2:CL(%f,%f,%f),absolute(%f,%f)",x_3_hat,y_3_hat,f_3_hat,x_3_absolute,y_3_absolute);  
     
        csv_file << distance1_2 << ","<< f_1_2 << ","<< distance1_3 << ","<< f_1_3
        << ","<< distance2_1 << ","<< f_2_1 << ","<< distance2_3 << ","<< f_2_3
        << ","<< distance3_1 << ","<< f_3_1 << ","<< distance3_2 << ","<< f_3_2 
        << ","<< v1<< ","<< v2 << ","<< v3 
        << ","<< w1<< ","<< w2 << ","<< w3
        << ","<< x_1_hat << ","<< y_1_hat << ","<< f_1_hat 
        << ","<< x_2_hat << ","<< y_2_hat << ","<< f_2_hat
        << ","<< x_3_hat << ","<< y_3_hat << ","<< f_3_hat 
        << ","<< x_1_absolute << ","<< y_1_absolute
        << ","<< x_2_absolute << ","<< y_2_absolute 
        << ","<< x_3_absolute << ","<< y_3_absolute         
        << ","<< v2_sensor << "," << w2_sensor
        << ","<< v3_sensor << "," << w3_sensor << "," << w1_sensor <<",\n"; 

        csv_file_1 << v1 << "," << v2 << "," << v3 << "," << w1 << "," << w2 << "," << w3 << ",\n";


        ros::spinOnce();
        loop_rate.sleep();
    }
    csv_file.close();
    csv_file_1.close();    
    return 0;
}