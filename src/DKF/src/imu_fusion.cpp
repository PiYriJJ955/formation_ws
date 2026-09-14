#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <fstream>
#include <std_msgs/Float32.h>


std::ofstream csv_file;
std_msgs::Float32 msg1;
ros::Publisher  imu_fusion_pub;

// 传感器数据
double w_imu  = 0;
double w_odom = 0;
ros::Time stamp_odom;          // 保存最新 odom 时间戳

// EKF 状态
Eigen::Vector3d X;   // [theta, omega, bias]
Eigen::Matrix3d P;
Eigen::Matrix3d F;
Eigen::Matrix3d Q;
Eigen::Matrix2d R;
Eigen::Matrix<double, 2, 3> H;

// 噪声参数
double var_imu_omega   = 0.05;
double var_odom_omega  = 0.1;

/* 初始化 EKF 矩阵 */
void initializeEKF()
{
    X << 0, 0, 0;
    P << 0.1, 0,   0,
         0,   0.1, 0,
         0,   0,   0.01;

    H << 0, 1, 0,
         0, 1, 1;

    R << var_odom_omega, 0,
         0, var_imu_omega;
}

/* odom 回调：只保存最新值 */
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    w_odom     = msg->twist.twist.angular.z;
    stamp_odom = msg->header.stamp;
}

/* IMU 回调：事件驱动 EKF */
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // 1. 计算真实 dt
    static ros::Time t_last = msg->header.stamp;
    ros::Time  t_now = msg->header.stamp;
    double dt_real = (t_now - t_last).toSec();
    t_last = t_now;
    if (dt_real <= 0 || dt_real > 0.05) dt_real = 0.05;  // 安全钳位

    // 2. 构造 F 和 Q
    F << 1, dt_real, 0,
         0, 1,       0,
         0, 0,       1;
    Q << 0.1, 0, 0,
         0, 0.01 , 0,
         0, 0, 0.01;

    // 3. 预测
    X = F * X;
    P = F * P * F.transpose() + Q;

    // 4. 如果 odom 也是“新鲜”的（±25 ms），就 update
    if (!stamp_odom.isZero() && std::abs((t_now - stamp_odom).toSec()) < 0.025)
    {
        Eigen::Vector2d Z;
        Z << w_odom, msg->angular_velocity.z;
        Eigen::Vector2d y       = Z - H * X;
        Eigen::Matrix2d S       = H * P * H.transpose() + R;
        Eigen::Matrix<double, 3, 2> K = P * H.transpose() * S.inverse();
        X += K * y;
        P  = (Eigen::Matrix3d::Identity() - K * H) * P;
    }

    // 5. 角度归一化
    if (X(0) > M_PI) X(0) -= 2 * M_PI;
    if (X(0) < -M_PI) X(0) += 2 * M_PI;

    // 6. 记录 & 打印
    csv_file << w_odom << "," << msg->angular_velocity.z << "," << X(1) << ","
             << X(2) << "," << X(0) << "\n";
    // ROS_INFO("Odom: %.3f  IMU: %.3f  Fused: %.3f  Bias: %.6f  Yaw: %.3f  dt: %.3f ms",
    //          w_odom, msg->angular_velocity.z, X(1), X(2), X(0), dt_real * 1000);

    msg1.data = X(1);
    imu_fusion_pub.publish(msg1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "omega_fusion");
    ros::NodeHandle nh;

    ros::Subscriber sub_imu  = nh.subscribe("imu",  1, imuCallback);
    ros::Subscriber sub_odom = nh.subscribe("odom", 1, odomCallback);
    imu_fusion_pub = nh.advertise<std_msgs::Float32>("imu_fusion",1);

    csv_file.open("/home/wheeltec/wheeltec_robot/imu_fusion.csv");
    csv_file << "odom_omega,imu_omega,fused_omega,bias,Yaw\n";

    initializeEKF();

    ros::spin();
    csv_file.close();
    return 0;
}