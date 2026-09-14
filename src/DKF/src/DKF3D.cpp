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


const std::vector<uint32_t> TARGET_UIDS = {805318144,805319168 ,805334528, 822107392};

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
float distance1_2 , distance1_3 , f_1_2 , f_1_3 , fz_1_2 , fz_1_3;
float distance2_1 , distance2_3 , f_2_1 , f_2_3 , fz_2_1 , fz_2_3;
float distance3_1 , distance3_2 , f_3_1 , f_3_2 , fz_3_1 , fz_3_2;

float v1 , w1;
float v2 , w2;
float v3 , w3;
float vz1 , vz2 , vz3;
float dt = 0.1;
float varQ = 0.01 , varR1 = 1 , varR2 = 1 , varR3 = 1;

//过程噪声协方差
Eigen::Matrix<float,4,4> Q1;
Eigen::Matrix<float,4,4> Q2;
Eigen::Matrix<float,4,4> Q3;
Eigen::Matrix<float,12,12> Q;

// 初始先验误差协方差矩阵 Pu 9x9 单位矩阵
Eigen::Matrix<float,12,12> Pu = Eigen::Matrix<float,12,12>::Identity();
// 初始后验误差协方差矩阵 Pp 9x9 0矩阵
Eigen::Matrix<float,12,12> Pp = Eigen::Matrix<float,12,12>::Identity();

// 初始先验状态
float x_1_minus=0,y_1_minus=0,f_1_minus=0,z_1_minus=0;
float x_2_minus=-0.2,y_2_minus=0,f_2_minus=0,z_2_minus=0;
float x_3_minus=0.2,y_3_minus=0,f_3_minus=0,z_3_minus=0;
// Xp
Eigen::Matrix<float,12,1> Xp;

// 初始后验坐标
float x_1_hat=0,y_1_hat=0,f_1_hat=0,z_1_hat;
float x_2_hat=-0.2,y_2_hat=0,f_2_hat=0,z_2_hat;
float x_3_hat=0.2,y_3_hat=0,f_3_hat=0,z_3_hat;
// Xu滤波器输出
Eigen::Matrix<float,12,1> Xu;

// 量测坐标
float x_12,y_12,f_12,z_12,x_13,y_13,f_13,z_13,x_21,y_21,f_21,z_21,x_23,y_23,f_23,z_23,x_31,y_31,f_31,z_31,x_32,y_32,f_32,z_32;

// 伪量测坐标
float x_12_hat,y_12_hat,f_12_hat,z_12_hat,x_13_hat,y_13_hat,f_13_hat,z_13_hat,x_21_hat,y_21_hat,f_21_hat,z_21_hat,x_23_hat,y_23_hat,f_23_hat,z_23_hat,x_31_hat,y_31_hat,f_31_hat,z_31_hat,x_32_hat,y_32_hat,f_32_hat,z_32_hat;


//量测
Eigen::Matrix<float,24,1> measure;

//伪量测
Eigen::Matrix<float,24,1> measure_hat;

//状态转移矩阵
Eigen::Matrix<float,4,4> A1;
Eigen::Matrix<float,4,4> A2;
Eigen::Matrix<float,4,4> A3;
Eigen::Matrix<float,4,4> M;
Eigen::Matrix<float,12,12> A;

float hc1,hc2,hc3,hc4,hc5,hc6,hc7,hc8,hc9,hc10,hc11,hc12;

Eigen::Matrix<float,24,4> D1;
Eigen::Matrix<float,24,4> D2;
Eigen::Matrix<float,24,4> D3;
Eigen::Matrix<float,24,12> H;
Eigen::Matrix<float,24,24> S;
Eigen::Matrix<float,12,24> K;

void DKF()
{
    Q1 << varQ, 0, 0, 0,
          0, varQ, 0, 0,
          0, 0, varQ, 0,
          0, 0, 0, varQ;

    Q2 << varQ, 0, 0, 0,
          0, varQ, 0, 0,
          0, 0, varQ, 0,
          0, 0, 0, varQ;

    Q3 << varQ, 0, 0, 0,
          0, varQ, 0, 0,
          0, 0, varQ, 0,
          0, 0, 0, varQ;

    Eigen::Matrix<float,24,24> R = Eigen::Matrix<float,24,24>::Identity();
/*    R = R*varR;*/
    R << varR1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, varR2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, varR3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, varR1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, varR2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, varR3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, varR1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, varR2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, varR3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, varR1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, varR2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, varR3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, varR1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, varR2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, varR3, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, varR1, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, varR2, 0, 0, 0, 0, 0, 0, 0,         
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, varR3, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, varR3, 0, 0, 0, 0, 0,         
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, varR3, 0, 0, 0, 0, 
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, varR3, 0, 0, 0,      
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, varR3, 0, 0,  
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, varR3, 0, 
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, varR3;                                  

    // 状态转移方程
    x_1_minus = x_1_hat+dt*v1*sin(f_1_hat);
    y_1_minus = y_1_hat+dt*v1*cos(f_1_hat);
    z_1_minus = z_1_hat+dt*vz1;
    f_1_minus = f_1_hat + dt*w1;

    x_2_minus = x_2_hat+dt*v2*sin(f_2_hat);
    y_2_minus = y_2_hat+dt*v2*cos(f_2_hat);
    z_2_minus = z_2_hat+dt*vz2;    
    f_2_minus = f_2_hat + dt*w2;       

    x_3_minus = x_3_hat+dt*v3*sin(f_3_hat);
    y_3_minus = y_3_hat+dt*v3*cos(f_3_hat);
    z_3_minus = z_3_hat+dt*vz3;    
    f_3_minus = f_3_hat + dt*w3;


    Xp << x_1_minus, y_1_minus, z_1_minus, f_1_minus, x_2_minus, y_2_minus, z_2_minus, f_2_minus, x_3_minus, y_3_minus, z_3_minus ,f_3_minus;

    // 量测坐标转换
    // 1->2
    x_12 = -distance1_2*cos(f_1_2);
    y_12 = -distance1_2*sin(f_1_2);
    z_12 = -distance1_2*sin(fz_1_2);
    f_12 = -f_1_2+f_2_1;

    // 1->3
    x_13 = distance1_3*cos(f_1_3);
    y_13 = distance1_3*sin(f_1_3);
    z_13 = distance1_3*sin(fz_1_3);
    f_13 = -f_1_3+f_3_1;

    // 2->1
    x_21 = distance2_1*cos(f_2_1);
    y_21 = distance2_1*sin(f_2_1);
    z_21 = distance2_1*sin(fz_2_1);    
    f_21 = -f_2_1+f_1_2;

    // 2->3
    x_23 = distance2_3*cos(f_2_3);
    y_23 = distance2_3*sin(f_2_3);
    z_23 = distance2_3*sin(fz_2_3);    
    f_23 = -f_2_3+f_3_2;  

    // 3->1
    x_31 = -distance3_1*cos(f_3_1);
    y_31 = -distance3_1*sin(f_3_1);
    z_31 = -distance3_1*sin(fz_3_1);    
    f_31 = -f_3_1+f_1_3;

    // 3->2
    x_32 = -distance3_2*cos(f_3_2);
    y_32 = -distance3_2*sin(f_3_2);
    z_32 = -distance3_2*sin(fz_3_2);    
    f_32 = -f_3_2+f_2_3;

    measure << x_12, y_12, z_12, f_12, x_13, y_13, z_13, f_13, x_21, y_21, z_21, f_21, x_23, y_23, z_23, f_23, x_31, y_31, z_31, f_31, x_32, y_32, z_32, f_32;

    A1 << 1 , 0 , 0, dt*v1*cos(f_1_hat),
          0 , 1 , 0, dt*v1*(-sin(f_1_hat)),
          0 , 0 , 1 , 0,
          0 , 0 , 0 , 1;

    A2 << 1 , 0 , 0 , dt*v2*cos(f_2_hat),
          0 , 1 , 0 , dt*v2*(-sin(f_2_hat)),
          0 , 0 , 1 , 0,
          0 , 0 , 0 , 1;  

    A3 << 1 , 0 , 0 , dt*v3*cos(f_3_hat),
          0 , 1 , 0 , dt*v3*(-sin(f_3_hat)),
          0 , 0 , 1 , 0,
          0 , 0 , 0 , 1;  

    M.setZero(4,4);
    A.block<4,4>(0,0) = A1;
    A.block<4,4>(0,4) = M;
    A.block<4,4>(0,8) = M;
    A.block<4,4>(4,0) = M;
    A.block<4,4>(4,4) = A2;
    A.block<4,4>(4,8) = M;
    A.block<4,4>(8,0) = M;
    A.block<4,4>(8,4) = M;
    A.block<4,4>(8,8) = A3;

    Q.block<4,4>(0,0) = Q1;
    Q.block<4,4>(0,4) = M;
    Q.block<4,4>(0,8) = M;
    Q.block<4,4>(4,0) = M;
    Q.block<4,4>(4,4) = Q2;
    Q.block<4,4>(4,8) = M;
    Q.block<4,4>(8,0) = M;
    Q.block<4,4>(8,4) = M;
    Q.block<4,4>(8,8) = Q3; 

    Pp = A*Pu*A.transpose()+Q; 

    // 构建伪量测
    // 1->2
    x_12_hat = cos(f_1_minus)*(x_2_minus-x_1_minus)-sin(f_1_minus)*(y_2_minus-y_1_minus);
    y_12_hat = sin(f_1_minus)*(x_2_minus-x_1_minus)+cos(f_1_minus)*(y_2_minus-y_1_minus);
    z_12_hat = z_2_minus-z_1_minus;
    f_12_hat = f_2_minus - f_1_minus;

    // 1->3
    x_13_hat = cos(f_1_minus)*(x_3_minus-x_1_minus)-sin(f_1_minus)*(y_3_minus-y_1_minus);
    y_13_hat = sin(f_1_minus)*(x_3_minus-x_1_minus)+cos(f_1_minus)*(y_3_minus-y_1_minus);
    z_13_hat = z_3_minus-z_1_minus;    
    f_13_hat = f_3_minus - f_1_minus;

    // 2->1
    x_21_hat = cos(f_2_minus)*(x_1_minus-x_2_minus)-sin(f_2_minus)*(y_1_minus-y_2_minus);
    y_21_hat = sin(f_2_minus)*(x_1_minus-x_2_minus)+cos(f_2_minus)*(y_1_minus-y_2_minus);
    z_21_hat = z_1_minus-z_2_minus;     
    f_21_hat = f_1_minus - f_2_minus;

    // 2->3
    x_23_hat = cos(f_2_minus)*(x_3_minus-x_2_minus)-sin(f_2_minus)*(y_3_minus-y_2_minus);
    y_23_hat = sin(f_2_minus)*(x_3_minus-x_2_minus)+cos(f_2_minus)*(y_3_minus-y_2_minus);
    z_23_hat = z_3_minus-z_2_minus;
    f_23_hat = f_3_minus - f_2_minus;

    // 3->1
    x_31_hat = cos(f_3_minus)*(x_1_minus-x_3_minus)-sin(f_3_minus)*(y_1_minus-y_3_minus);
    y_31_hat = sin(f_3_minus)*(x_1_minus-x_3_minus)+cos(f_3_minus)*(y_1_minus-y_3_minus);
    z_31_hat = z_1_minus-z_3_minus;
    f_31_hat = f_1_minus - f_3_minus;

    // 3->2
    x_32_hat = cos(f_3_minus)*(x_2_minus-x_3_minus)-sin(f_3_minus)*(y_2_minus-y_3_minus);
    y_32_hat = sin(f_3_minus)*(x_2_minus-x_3_minus)+cos(f_3_minus)*(y_2_minus-y_3_minus);
    z_32_hat = z_2_minus-z_3_minus;
    f_32_hat = f_2_minus - f_3_minus;

    measure_hat << x_12_hat, y_12_hat, z_12_hat ,f_12_hat, x_13_hat, y_13_hat, z_13_hat , f_13_hat, x_21_hat, y_21_hat, z_21_hat, f_21_hat, x_23_hat, y_23_hat, z_23_hat, f_23_hat, x_31_hat, y_31_hat, z_31_hat, f_31_hat, x_32_hat, y_32_hat, z_32_hat, f_32_hat;

    hc1 = -sin(f_1_minus)*(x_2_minus-x_1_minus)-cos(f_1_minus)*(y_2_minus-y_1_minus);
    hc2 = cos(f_1_minus)*(x_2_minus-x_1_minus)-sin(f_1_minus)*(y_2_minus-y_1_minus);
    hc3 = -sin(f_1_minus)*(x_3_minus-x_1_minus)-cos(f_1_minus)*(y_3_minus-y_1_minus);
    hc4 = cos(f_1_minus)*(x_3_minus-x_1_minus)-sin(f_1_minus)*(y_3_minus-y_1_minus);

    D1 << -cos(f_1_minus), sin(f_1_minus),  0, hc1,
          -sin(f_1_minus), -cos(f_1_minus), 0, hc2,
          0, 0, -1, 0,
          0, 0, 0, -1,

          -cos(f_1_minus), sin(f_1_minus),  0, hc3,
          -sin(f_1_minus), -cos(f_1_minus), 0, hc4,
          0, 0, -1, 0,
          0, 0, 0, -1,

          cos(f_2_minus),  -sin(f_2_minus), 0, 0,
          sin(f_2_minus),  cos(f_2_minus),  0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1,

          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0,

          cos(f_3_minus),  -sin(f_3_minus), 0, 0,
          sin(f_3_minus),  cos(f_3_minus),  0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1,

          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0;
          
    hc5 = -sin(f_2_minus)*(x_1_minus-x_2_minus)-cos(f_2_minus)*(y_1_minus-y_2_minus);
    hc6 = cos(f_2_minus)*(x_1_minus-x_2_minus)-sin(f_2_minus)*(y_1_minus-y_2_minus);
    hc7 = -sin(f_2_minus)*(x_3_minus-x_2_minus)-cos(f_2_minus)*(y_3_minus-y_2_minus);
    hc8 = cos(f_2_minus)*(x_3_minus-x_2_minus)-sin(f_2_minus)*(y_3_minus-y_2_minus);

    D2 << cos(f_1_minus), -sin(f_1_minus), 0, 0,
          sin(f_1_minus),  cos(f_1_minus), 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1,

          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0,

          -cos(f_2_minus),  sin(f_2_minus), 0, hc5,
          -sin(f_2_minus), -cos(f_2_minus), 0, hc6,
          0, 0, -1, 0,
          0, 0, 0, -1,

          -cos(f_2_minus),  sin(f_2_minus), 0, hc7,
          -sin(f_2_minus), -cos(f_2_minus), 0, hc8,
          0, 0, -1, 0,
          0, 0, 0, -1,

          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0,

          cos(f_3_minus),  -sin(f_3_minus), 0, 0,
          sin(f_3_minus),   cos(f_3_minus), 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;

    hc9 = -sin(f_3_minus)*(x_1_minus-x_3_minus)-cos(f_3_minus)*(y_1_minus-y_3_minus);
    hc10 = cos(f_3_minus)*(x_1_minus-x_3_minus)-sin(f_3_minus)*(y_1_minus-y_3_minus);
    hc11 = -sin(f_3_minus)*(x_2_minus-x_3_minus)-cos(f_3_minus)*(y_2_minus-y_3_minus);
    hc12 = cos(f_3_minus)*(x_2_minus-x_3_minus)-sin(f_3_minus)*(y_2_minus-y_3_minus);

    D3 << 
          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0,

          cos(f_1_minus), -sin(f_1_minus), 0, 0,
          sin(f_1_minus),  cos(f_1_minus), 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1,

          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0,

          cos(f_2_minus), -sin(f_2_minus), 0, 0,
          sin(f_2_minus),  cos(f_2_minus), 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1,

          -cos(f_3_minus), sin(f_3_minus),  0, hc9,
          -sin(f_3_minus), -cos(f_3_minus), 0, hc10,
          0, 0, -1, 0,
          0, 0, 0, -1,

          -cos(f_3_minus), sin(f_3_minus), 0,  hc11,
          -sin(f_3_minus), -cos(f_3_minus), 0, hc12,
          0, 0, -1, 0,
          0, 0, 0, -1;

    H << D1, D2, D3;

    S = H*Pp*H.transpose() + R;

    K = Pp*H.transpose()*S.inverse();

    Xu = Xp + K*(measure - measure_hat);

    x_1_hat = Xu(0);
    y_1_hat = Xu(1);
    z_1_hat = Xu(2);
    f_1_hat = Xu(3);

    x_2_hat = Xu(4);
    y_2_hat = Xu(5);
    z_2_hat = Xu(6);
    f_2_hat = Xu(7);

    x_3_hat = Xu(8);
    y_3_hat = Xu(9);
    z_3_hat = Xu(10);
    f_3_hat = Xu(11);  

    Pu = Pp - K*H*Pp;
}


void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    w1 = msg->angular_velocity.z;
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

void fz12Callback(const std_msgs::Float32::ConstPtr& msg)
{
    fz_1_2 = msg->data;
}

void fz13Callback(const std_msgs::Float32::ConstPtr& msg)
{
    fz_1_3 = msg->data;
}

void fz21Callback(const std_msgs::Float32::ConstPtr& msg)
{
    fz_2_1 = msg->data;
}

void fz23Callback(const std_msgs::Float32::ConstPtr& msg)
{
    fz_2_3 = msg->data;
}

void fz31Callback(const std_msgs::Float32::ConstPtr& msg)
{
    fz_3_1 = msg->data;
}

void fz32Callback(const std_msgs::Float32::ConstPtr& msg)
{
    fz_3_2 = msg->data;
}

int StartLocation;
void StartLocationCallback(const std_msgs::Bool::ConstPtr& msg) 
{
    StartLocation = msg->data;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "DKF3D");
    ros::NodeHandle n;
	//打开文件
	csv_file.open("/home/wheeltec/wheeltec_robot/data_3ddkf.csv");

    ros::Subscriber uwb_sub0 = n.subscribe<nlink_parser::IotFrame0>("nlink_iot_frame0", 1000, uwb0Callback);
    ros::Subscriber uwb_sub1 = n.subscribe<nlink_parser::IotFrame0>("nlink_iot_frame1", 1000, uwb1Callback);   
    ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("imu", 1000, imuCallback);
    ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("odom", 1000, odomCallback);
    ros::Subscriber v2_sub = n.subscribe<std_msgs::Float32>("/v2",10,v2Callback);
    ros::Subscriber w2_sub = n.subscribe<std_msgs::Float32>("/w2",10,w2Callback);
    ros::Subscriber v3_sub = n.subscribe<std_msgs::Float32>("/v3",10,v3Callback);
    ros::Subscriber w3_sub = n.subscribe<std_msgs::Float32>("/w3",10,w3Callback);
    ros::Subscriber d21_sub = n.subscribe<std_msgs::Float32>("/distance2_1",10,d21Callback);
    ros::Subscriber f21_sub = n.subscribe<std_msgs::Float32>("/f_2_1",10,f21Callback);
    ros::Subscriber d23_sub = n.subscribe<std_msgs::Float32>("/distance2_3",10,d23Callback);
    ros::Subscriber f23_sub = n.subscribe<std_msgs::Float32>("/f_2_3",10,f23Callback);
    ros::Subscriber d31_sub = n.subscribe<std_msgs::Float32>("/distance3_1",10,d31Callback);
    ros::Subscriber f31_sub = n.subscribe<std_msgs::Float32>("/f_3_1",10,f31Callback);
    ros::Subscriber d32_sub = n.subscribe<std_msgs::Float32>("/distance3_2",10,d32Callback);
    ros::Subscriber f32_sub = n.subscribe<std_msgs::Float32>("/f_3_2",10,f32Callback);
    ros::Subscriber fz12_sub = n.subscribe<std_msgs::Float32>("/fz_1_2",10,fz12Callback);
    ros::Subscriber fz13_sub = n.subscribe<std_msgs::Float32>("/fz_1_3",10,fz13Callback);
    ros::Subscriber fz21_sub = n.subscribe<std_msgs::Float32>("/fz_2_1",10,fz21Callback);
    ros::Subscriber fz23_sub = n.subscribe<std_msgs::Float32>("/fz_2_3",10,fz23Callback);
    ros::Subscriber fz31_sub = n.subscribe<std_msgs::Float32>("/fz_3_1",10,fz31Callback);
    ros::Subscriber fz32_sub = n.subscribe<std_msgs::Float32>("/fz_3_2",10,fz32Callback);    

    ros::Subscriber sub = n.subscribe("StartLocation", 10, StartLocationCallback);

    // ros::Publisher d12_pub = n.advertise<std_msgs::Float32>("/distance1_2", 10);
    // ros::Publisher d13_pub = n.advertise<std_msgs::Float32>("/distance1_3", 10);   
    // ros::Publisher f12_pub = n.advertise<std_msgs::Float32>("/f_1_2", 10);
    // ros::Publisher f13_pub = n.advertise<std_msgs::Float32>("/f_1_3", 10);

    ros::Publisher x1_pub = n.advertise<std_msgs::Float32>("/x_1_hat", 10);
    ros::Publisher y1_pub = n.advertise<std_msgs::Float32>("/y_1_hat", 10);   
    ros::Publisher f1_pub = n.advertise<std_msgs::Float32>("/f_1_hat", 10);
    ros::Publisher x2_pub = n.advertise<std_msgs::Float32>("/x_2_hat", 10);
    ros::Publisher y2_pub = n.advertise<std_msgs::Float32>("/y_2_hat", 10);   
    ros::Publisher f2_pub = n.advertise<std_msgs::Float32>("/f_2_hat", 10);
    ros::Publisher x3_pub = n.advertise<std_msgs::Float32>("/x_3_hat", 10);
    ros::Publisher y3_pub = n.advertise<std_msgs::Float32>("/y_3_hat", 10);   
    ros::Publisher f3_pub = n.advertise<std_msgs::Float32>("/f_3_hat", 10);    

    n.getParam("x_1_hat",x_1_hat);
    n.getParam("y_1_hat",y_1_hat);    
    n.getParam("f_1_hat",f_1_hat);
    n.getParam("z_1_hat",z_1_hat);    
    n.getParam("x_2_hat",x_2_hat);
    n.getParam("y_2_hat",y_2_hat);    
    n.getParam("f_2_hat",f_2_hat);
    n.getParam("z_2_hat",z_2_hat);    
    n.getParam("x_3_hat",x_3_hat);
    n.getParam("y_3_hat",y_3_hat);    
    n.getParam("f_3_hat",f_3_hat);
    n.getParam("z_3_hat",z_3_hat);    

    n.getParam("x_1_minus",x_1_minus);
    n.getParam("y_1_minus",y_1_minus);    
    n.getParam("f_1_minus",f_1_minus);
    n.getParam("z_1_minus",z_1_minus);    
    n.getParam("x_2_minus",x_2_minus);
    n.getParam("y_2_minus",y_2_minus);    
    n.getParam("f_2_minus",f_2_minus);
    n.getParam("z_2_minus",z_2_minus);    
    n.getParam("x_3_minus",x_3_minus);
    n.getParam("y_3_minus",y_3_minus);    
    n.getParam("f_3_minus",f_3_minus);    
    n.getParam("z_3_minus",z_3_minus); 

    n.getParam("varQ",varQ);
    n.getParam("varR1",varR1);          
    n.getParam("varR2",varR2);     
    n.getParam("varR3",varR3); 
    n.getParam("varR4",varR3);     

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        distance1_2 = node_a.dis;   // A→B 距离
        f_1_2       = node_a.aoa_h; // A→B 水平角
        fz_1_2      = node_a.aoa_v; // A→B 垂直角

        distance1_3 = node_b.dis;   // A→B 距离
        f_1_3       = node_b.aoa_h; // A→B 水平角
        fz_1_3      = node_b.aoa_v; // A→B 垂直角        


        // msg1.data = distance1_2;
        // msg2.data = distance1_3;
        // msg3.data = f_1_2;
        // msg4.data = f_1_3;
        
        // d12_pub.publish(msg1);
        // d13_pub.publish(msg2);
        // f12_pub.publish(msg3);
        // f13_pub.publish(msg4);

        f_1_2 = (0.46*f_1_2-1.4555)*M_PI/180;
        f_1_3 = (0.7901*f_1_3-10.272)*M_PI/180;
        f_2_1 = (0.4572*f_2_1-0.2671)*M_PI/180;
        f_2_3 = (0.5242*f_2_3-5.261)*M_PI/180;
        f_3_1 = (0.7517*f_3_1-7.7062)*M_PI/180;
        f_3_2 = (0.5592*f_3_2-7.5059)*M_PI/180;


/*        f_1_2 = (0.5803*f_1_2-8.4887)*M_PI/180;
        f_1_3 = (0.7462*f_1_3-10.3736)*M_PI/180;
        f_2_1 = (0.5620*f_2_1-3.1042)*M_PI/180;
        f_2_3 = (0.6904*f_2_3-8.2845)*M_PI/180;
        f_3_1 = (0.8867*f_3_1-5.6167)*M_PI/180;
        f_3_2 = (0.6353*f_3_2-7.9225)*M_PI/180;*/


/*        fz_1_2 = fz_1_2*M_PI/180;
        fz_1_3 = fz_1_3*M_PI/180;
        fz_2_1 = fz_2_1*M_PI/180;
        fz_2_3 = fz_2_3*M_PI/180;
        fz_3_1 = fz_3_1*M_PI/180;
        fz_3_2 = fz_3_2*M_PI/180;*/

        fz_1_2 = 0;
        fz_1_3 = 0;
        fz_2_1 = 0;
        fz_2_3 = 0;
        fz_3_1 = 0;
        fz_3_2 = 0;

        w1 = -w1;
        w2 = -w2;
        w3 = -w3;  

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
        ROS_INFO("f_1_2:%f",f_1_2);              
        ROS_INFO("fz_1_2:%f",fz_1_2);              

        ROS_INFO("distance1_3:%f",distance1_3); 
        ROS_INFO("f_1_3:%f",f_1_3); 
        ROS_INFO("fz_1_3:%f",fz_1_3); 

        ROS_INFO("distance2_1:%f",distance2_1);
        ROS_INFO("f_2_1:%f",f_2_1); 
        ROS_INFO("fz_2_1:%f",fz_2_1);   

        ROS_INFO("distance2_3:%f",distance2_3);
        ROS_INFO("f_2_3:%f",f_2_3);
        ROS_INFO("fz_2_3:%f",fz_2_3);        

        ROS_INFO("distance3_1:%f",distance3_1);
        ROS_INFO("f_3_1:%f",f_3_1);  
        ROS_INFO("fz_3_1:%f",fz_3_1); 

        ROS_INFO("distance3_2:%f",distance3_2);
        ROS_INFO("f_3_2:%f",f_3_2);
        ROS_INFO("fz_3_2:%f",fz_3_2);


        if (StartLocation == 0)
        {
            ROS_INFO("Collaborative Location is not activated");
        }
        if (StartLocation == 1)
        {
            ROS_INFO("Collaborative Location is activated");
        }        
        ROS_INFO("Leader(%f,%f,%f,%f)",x_1_hat,y_1_hat,z_1_hat,f_1_hat);  
        ROS_INFO("Follower1(%f,%f,%f,%f)",x_2_hat,y_2_hat,z_2_hat,f_2_hat);  
        ROS_INFO("Follower2(%f,%f,%f,%f)",x_3_hat,y_3_hat,z_3_hat,f_3_hat);  

     
        csv_file << distance1_2 << ","<< f_1_2 << ","<< fz_1_2
        << ","<< distance1_3 << ","<< f_1_3 << ","<< fz_1_3
        << ","<< distance2_1 << ","<< f_2_1 << ","<< fz_2_1
        << ","<< distance2_3 << ","<< f_2_3 << ","<< fz_2_3
        << ","<< distance3_1 << ","<< f_3_1 << ","<< fz_3_1
        << ","<< distance3_2 << ","<< f_3_2 << ","<< fz_3_2
        << ","<< v1<< ","<< v2 << ","<< v3 
        << ","<< w1<< ","<< w2 << ","<< w3
        << ","<< x_1_hat << ","<< y_1_hat << ","<< z_1_hat << ","<<f_1_hat 
        << ","<< x_2_hat << ","<< y_2_hat << ","<< z_2_hat << ","<<f_2_hat
        << ","<< x_3_hat << ","<< y_3_hat << ","<< z_3_hat << ","<<f_3_hat << ","<< ",\n"; 

        ros::spinOnce();
        loop_rate.sleep();
    }
    csv_file.close();  
    return 0;
}

