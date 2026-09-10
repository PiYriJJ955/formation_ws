#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>
#include <limits>
#include <qpOASES.hpp>
#include <qpOASES/Options.hpp>
#include <qpOASES/Types.hpp>
#include <std_msgs/Bool.h>
geometry_msgs::Twist twist;

// 全局变量定义
float v1 = 0.0, w1 = 0.0;  // 机器人1的速度和角速度
float v2 = 0.0, w2 = 0.0;  // 机器人2的速度和角速度
float v3 = 0.0, w3 = 0.0;  // 机器人2的速度和角速度
float x_1_hat = 0.0, y_1_hat = 0.0, f_1_hat = 0.0;  // 机器人1的状态估计
float x_2_hat = -0.5, y_2_hat = 0.0, f_2_hat = 0.0;  // 机器人2的状态估计
float x_3_hat = 0.5, y_3_hat = 0.0, f_3_hat = 0.0;  // 机器人3的状态估计
float dt = 0.1;
float d = 0.055;

// 预测时域和控制时域
int Np = 5, Nc = 1;

// 编队参考点
float dx = 0, dy = 0.5, k_dtheta = 0.0;
float dx_beforechange = 0, dy_beforechange = 0;
float dx_afterchange = 0, dy_afterchange = 0;

// 
bool formation_change = 0;

// 权重矩阵
float weight_Q1 = 1.0;
float weight_Q2 = 1.0;
float weight_Q3 = 0.1;
float weight_R = 2.0;
float weight_S = 10.0;
Eigen::Matrix<float, 3, 3> Q;
Eigen::Matrix<float, 2, 2> R;
Eigen::Matrix<float, 3, 3> S;

// 约束
float v_low = -std::numeric_limits<float>::infinity();
float v_high = std::numeric_limits<float>::infinity();
float w_low = -std::numeric_limits<float>::infinity();
float w_high = std::numeric_limits<float>::infinity();
float xyf_low = -std::numeric_limits<float>::infinity();
float xyf_high = std::numeric_limits<float>::infinity();
Eigen::Matrix<float, 2, 1> derta_u_low;
Eigen::Matrix<float, 2, 1> derta_u_high;
Eigen::Matrix<float, 3, 1> e_low;
Eigen::Matrix<float, 3, 1> e_high;

// 约束用矩阵（全局定义）
Eigen::MatrixXf I_2x2;
Eigen::MatrixXf I_3x3;
Eigen::MatrixXf O_2x3;
Eigen::MatrixXf O_3x2;
Eigen::MatrixXf O_10x3;
Eigen::MatrixXf O_10x2;
Eigen::MatrixXf O_6x3;
Eigen::MatrixXf O_6x2;
Eigen::MatrixXf M_10x3;
Eigen::MatrixXf F_10x2;
Eigen::MatrixXf Beita;
Eigen::MatrixXf M_Np;
Eigen::MatrixXf Beita_Np;
Eigen::MatrixXf A_constraint;
Eigen::MatrixXf B_constraint;

//
Eigen::Matrix<float,3,1> X_ref;
Eigen::Matrix<float,3,3> A;
Eigen::Matrix<float,3,2> B;
Eigen::Matrix<float,3,1>X_follower1;
Eigen::Matrix<float,3,1> E_follower1;
Eigen::MatrixXf Omega;
Eigen::MatrixXf Psi;
Eigen::MatrixXf Fai;
Eigen::MatrixXf Gamma;
Eigen::MatrixXf F;
Eigen::MatrixXf H;
Eigen::MatrixXf M1;
Eigen::MatrixXf M2;
Eigen::MatrixXf F2;
Eigen::MatrixXf Beita_1;

Eigen::MatrixXf createOmega(int Np, Eigen::Matrix<float, 3, 3> Q, Eigen::Matrix<float, 3, 3> S)
{
    // 创建一个大小为NpxNp的二维向量，用于存储每个位置的矩阵块
    std::vector<std::vector<Eigen::MatrixXf>> Omega_cell(Np, std::vector<Eigen::MatrixXf>(Np));

    for (int i = 0; i < Np; ++i)
    {
        for (int j = 0; j < Np; ++j)
        {
            if (i == j && i < Np - 1)
            {
                Omega_cell[i][j] = Q;
            }
            else if (i == j && i == Np - 1)
            {
                Omega_cell[i][j] = S;
            }
            else
            {
                Omega_cell[i][j] = Eigen::MatrixXf::Zero(3, 3);
            }
        }
    }

    // 计算Omega矩阵的行数和列数
    int rows = 0, cols = 0;
    for (int i = 0; i < Np; ++i)
    {
        rows += Omega_cell[i][0].rows();
    }
    for (int j = 0; j < Np; ++j)
    {
        cols += Omega_cell[0][j].cols();
    }

    // 创建并填充Omega矩阵
    Eigen::MatrixXf Omega(rows, cols);
    int rowStart = 0, colStart = 0;
    for (int i = 0; i < Np; ++i)
    {
        for (int j = 0; j < Np; ++j)
        {
            Omega.block(rowStart, colStart, Omega_cell[i][j].rows(), Omega_cell[i][j].cols()) = Omega_cell[i][j];
            colStart += Omega_cell[i][j].cols();
        }
        colStart = 0;
        rowStart += Omega_cell[i][0].rows();
    }

    return Omega;
}

Eigen::MatrixXf createPsi(int Nc, Eigen::Matrix<float, 2, 2> R)
{
    // 创建一个大小为NcxNc的二维向量，用于存储每个位置的矩阵块
    std::vector<std::vector<Eigen::MatrixXf>> Psi_cell(Nc, std::vector<Eigen::MatrixXf>(Nc));

    for (int i = 0; i < Nc; ++i)
    {
        for (int j = 0; j < Nc; ++j)
        {
            if (i == j)
            {
                Psi_cell[i][j] = R;
            }
            else
            {
                // 创建一个2x2的零矩阵
                Eigen::MatrixXf zero_block(2, 2);
                zero_block.setZero();
                Psi_cell[i][j] = zero_block;
            }
        }
    }

    // 计算Psi矩阵的行数和列数
    int rows = 0, cols = 0;
    for (int i = 0; i < Nc; ++i)
    {
        rows += Psi_cell[i][0].rows();
    }
    for (int j = 0; j < Nc; ++j)
    {
        cols += Psi_cell[0][j].cols();
    }

    // 创建并填充Psi矩阵
    Eigen::MatrixXf Psi(rows, cols);
    int rowStart = 0, colStart = 0;
    for (int i = 0; i < Nc; ++i)
    {
        for (int j = 0; j < Nc; ++j)
        {
            Psi.block(rowStart, colStart, Psi_cell[i][j].rows(), Psi_cell[i][j].cols()) = Psi_cell[i][j];
            colStart += Psi_cell[i][j].cols();
        }
        colStart = 0;
        rowStart += Psi_cell[i][0].rows();
    }

    return Psi;
}

Eigen::MatrixXf createM1(int Np)
{
    std::vector<std::vector<Eigen::MatrixXf>> M1_cell(Np+1 , std::vector<Eigen::MatrixXf>(1));

    for (int i = 0; i < Np+1 ; ++i)
    {
        if (i == 0)
        {
            M1_cell[i][0] = M_10x3;
        }
        else if (i > 0 && i < Np )
        {
            M1_cell[i][0] = O_10x3;
        }
        else
        {
            M1_cell[i][0] = O_6x3;
        }
    }

    // 计算M1矩阵的行数和列数
    int rows = 0, cols = 3; // 所有块的列数都是3
    for (int i = 0; i < Np+1 ; ++i)
    {
        rows += M1_cell[i][0].rows();
    }

    // 创建并填充M1矩阵
    Eigen::MatrixXf M1(rows, cols);
    int rowStart = 0;
    for (int i = 0; i < Np+1 ; ++i)
    {
        M1.block(rowStart, 0, M1_cell[i][0].rows(), cols) = M1_cell[i][0];
        rowStart += M1_cell[i][0].rows();
    }

    return M1;
}

Eigen::MatrixXf createM2(int Np)
{
    std::vector<std::vector<Eigen::MatrixXf>> M2_cell(Np+1, std::vector<Eigen::MatrixXf>(Np));

    for (int i = 0; i < Np+1; ++i)
    {
        for (int j = 0; j < Np; ++j)
        {
            if (i < Np)
            {
                if (i==j+1)
                M2_cell[i][j] = M_10x3;
                else
                M2_cell[i][j] = O_10x3;
            }
            else
            {
                if (j==Np-1)
                {
                    M2_cell[i][j] = M_Np;
                }
                else
                {
                    M2_cell[i][j] = O_6x3;
                }
            }
        }
    }

    int rows = 0, cols = 0;
    for (int i = 0; i < Np+1; ++i)
    {
        rows += M2_cell[i][0].rows();
    }
    for (int j = 0; j < Np; ++j)
    {
        cols += M2_cell[0][j].cols();
    }

    Eigen::MatrixXf M2(rows, cols);
    int rowStart = 0, colStart = 0;
    for (int i = 0; i < Np+1; ++i)
    {
        for (int j = 0; j < Np; ++j)
        {
            M2.block(rowStart, colStart, M2_cell[i][j].rows(), M2_cell[i][j].cols()) = M2_cell[i][j];
            colStart += M2_cell[i][j].cols();
        }
        colStart = 0;
        rowStart += M2_cell[i][0].rows();
    }

    return M2;
}

Eigen::MatrixXf createF2(int Np, int Nc)
{
    std::vector<std::vector<Eigen::MatrixXf>> F2_cell(Np + 1, std::vector<Eigen::MatrixXf>(Nc));

    for (int i = 0; i < Np + 1; ++i)
    {
        for (int j = 0; j < Nc; ++j)
        {
            if (i < Nc)
            {
                if (i == j)
                {
                    F2_cell[i][j] = F_10x2;
                }
                else
                {
                    F2_cell[i][j] = O_10x2;
                }
            }
            else if (i < Np)
            {
                if (j == Nc - 1)
                {
                    F2_cell[i][j] = F_10x2;
                }
                else
                {
                    F2_cell[i][j] = O_10x2;
                }
            }
            else
            {
                F2_cell[i][j] = O_6x2;
            }
        }
    }

    int rows = 0, cols = 0;
    for (int i = 0; i < Np + 1; ++i)
    {
        rows += F2_cell[i][0].rows();
    }
    for (int j = 0; j < Nc; ++j)
    {
        cols += F2_cell[0][j].cols();
    }

    Eigen::MatrixXf F2(rows, cols);
    int rowStart = 0, colStart = 0;
    for (int i = 0; i < Np + 1; ++i)
    {
        for (int j = 0; j < Nc; ++j)
        {
            F2.block(rowStart, colStart, F2_cell[i][j].rows(), F2_cell[i][j].cols()) = F2_cell[i][j];
            colStart += F2_cell[i][j].cols();
        }
        colStart = 0;
        rowStart += F2_cell[i][0].rows();
    }

    return F2;
}

Eigen::MatrixXf createBeita_1(int Np)
{
    std::vector<std::vector<Eigen::MatrixXf>> Beita_1_cell(Np+1 , std::vector<Eigen::MatrixXf>(1));

    for (int i = 0; i < Np+1 ; ++i)
    {
        if (i < Np)
        {
            Beita_1_cell[i][0] = Beita;
        }
        else
        {
            Beita_1_cell[i][0] = Beita_Np;
        }
    }

    int rows = 0, cols = 1; 
    for (int i = 0; i < Np+1 ; ++i)
    {
        rows += Beita_1_cell[i][0].rows();
    }

    Eigen::MatrixXf Beita_1(rows, cols);
    int rowStart = 0;
    for (int i = 0; i < Np+1 ; ++i)
    {
        Beita_1.block(rowStart, 0, Beita_1_cell[i][0].rows(), cols) = Beita_1_cell[i][0];
        rowStart += Beita_1_cell[i][0].rows();
    }

    return Beita_1;
}

Eigen::MatrixXf createFai(int Np)
{
    // 定义 Fai_cell
    std::vector<std::vector<Eigen::Matrix3f>> Fai_cell(Np, std::vector<Eigen::Matrix3f>(1));

    // 计算 A 的幂次并存储到 Fai_cell 中
    for (int i = 0; i < Np; ++i)
    {
        Fai_cell[i][0] = A.pow(i + 1);
    }

    // 计算总行数和列数
    int rows = 0, cols = 3;
    for (int i = 0; i < Np; ++i)
    {
        rows += Fai_cell[i][0].rows();
    }

    // 将 Fai_cell 中的所有矩阵拼接成一个大的矩阵
    Eigen::MatrixXf result(rows, cols);
    int current_row = 0;
    for (int i = 0; i < Np; ++i)
    {
        result.block(current_row, 0, Fai_cell[i][0].rows(), cols) = Fai_cell[i][0];
        current_row += Fai_cell[i][0].rows();
    }

    // 返回结果矩阵
    return result;
}

Eigen::MatrixXf createGamma(int Np, int Nc)
{
    std::vector<std::vector<Eigen::MatrixXf>> Gamma_cell(Np, std::vector<Eigen::MatrixXf>(Nc));

    for (int i = 0; i < Np; ++i)
    {
        for (int j = 0; j < Nc; ++j)
        {
            if (i < Nc)
            {
                if (i > j)
                {
                    Gamma_cell[i][j] = A.pow(i - j) * B;
                }
                else if (i == j)
                {
                    Gamma_cell[i][j] = B;
                }
                else
                {
                    Gamma_cell[i][j] = O_3x2;
                }
            }
            else
            {
                if (j < Nc - 1)
                {
                    Gamma_cell[i][j] = A.pow(i - j) * B;
                }
                else
                {
                    if (i == Nc)
                    {
                        Gamma_cell[i][j] = A * B + B;
                    }
                    else
                    {
                        // 确保 i-1 有效
                        if (i - 1 >= 0)
                        {
                            Gamma_cell[i][j] = A * Gamma_cell[i - 1][j] + B;
                        }
                        else
                        {
                            Gamma_cell[i][j] = O_3x2;  // 处理边界情况
                        }
                    }
                }
            }
        }
    }

    int rows = 0, cols = 0;
    for (int i = 0; i < Np; ++i)
    {
        rows += Gamma_cell[i][0].rows();
    }
    for (int j = 0; j < Nc; ++j)
    {
        cols += Gamma_cell[0][j].cols();
    }

    Eigen::MatrixXf Gamma(rows, cols);
    int rowStart = 0, colStart = 0;
    for (int i = 0; i < Np; ++i)
    {
        for (int j = 0; j < Nc; ++j)
        {
            Gamma.block(rowStart, colStart, Gamma_cell[i][j].rows(), Gamma_cell[i][j].cols()) = Gamma_cell[i][j];
            colStart += Gamma_cell[i][j].cols();
        }
        colStart = 0;
        rowStart += Gamma_cell[i][0].rows();
    }

    return Gamma;
}

float x_ref, y_ref, f_ref;
float cmd_v, cmd_w;
using namespace qpOASES;
real_t*H_qpOASES = nullptr;
real_t*F_qpOASES = nullptr;
real_t*A_constraint_qpOASES = nullptr; //Np+1 * Nc
real_t*B_constraint_qpOASES = nullptr;
real_t*g_qpOASES = nullptr;
void MPC()
{
    // 计算参考点
    x_ref = x_1_hat + dx * sin(f_1_hat) - dy * cos(f_1_hat);
    y_ref = y_1_hat + dx * cos(f_1_hat) + dy * sin(f_1_hat);
    f_ref = (1-k_dtheta)*f_1_hat + k_dtheta*atan2(x_ref-x_3_hat,y_ref-y_3_hat);
    X_ref << x_ref, y_ref, f_ref;

    // 状态转移矩阵A和控制矩阵B
    A << 1, 0, v1 * cos(f_1_hat) * dt,
         0, 1, -v1 * sin(f_1_hat) * dt,
         0, 0, 1;

    B << sin(f_1_hat) * dt, 0,
         cos(f_1_hat) * dt, 0,
         0, dt;

    // 跟随者状态
    X_follower1 << x_3_hat, y_3_hat, f_3_hat;

    // 误差计算
    E_follower1(0) = X_ref(0) - (X_follower1(0) + d * sin(X_follower1(2)));
    E_follower1(1) = X_ref(1) - (X_follower1(1) + d * cos(X_follower1(2)));
    E_follower1(2) = X_ref(2) - X_follower1(2);

    // 构造Fai和Gamma矩阵
    Fai = createFai(Np);
    Gamma = createGamma(Np, Nc);

    // 构造二次规划问题的矩阵
    F = Gamma.transpose() * Omega * Fai;
    H = Gamma.transpose() * Omega * Gamma + Psi;
    H = (H + H.transpose()) / 2; // 确保H矩阵对称

    // 计算线性项
    Eigen::MatrixXf g = F * E_follower1;

    // 约束矩阵
    A_constraint = M2 * Gamma + F2;
    B_constraint = Beita_1 - (M1 + M2 * Fai) * E_follower1;

    //
    int rowH = H.rows();
    int colH = H.cols();
    int rowF = F.rows();
    int colF = F.cols();
    int rowAcons = A_constraint.rows();
    int colAcons = A_constraint.cols();
    int rowBcons = B_constraint.rows();
    int colBcons = B_constraint.cols();
    int rowg = g.rows();
    //
    if (H_qpOASES != nullptr) delete[] H_qpOASES;
    if (F_qpOASES != nullptr) delete[] F_qpOASES;   
    if (A_constraint_qpOASES != nullptr) delete[] A_constraint_qpOASES;
    if (B_constraint_qpOASES != nullptr) delete[] B_constraint_qpOASES; 
    if (g_qpOASES != nullptr) delete[] g_qpOASES;    
    // 
    H_qpOASES = new real_t[rowH*colH];
    F_qpOASES = new real_t[rowF*colF];
    A_constraint_qpOASES = new real_t[rowAcons*colAcons];
    B_constraint_qpOASES = new real_t[rowBcons*colBcons];
    g_qpOASES = new real_t[rowg*1];



    for (int i=0;i<rowH;++i)
    {
        for (int j=0;j<colH;++j)
        {
            H_qpOASES[i+j*rowH] = H(i,j);
        }
    }
    for (int i=0;i<rowF;++i)
    {
        for (int j=0;j<colF;++j)
        {
            F_qpOASES[i+j*rowF] = F(i,j);
        }
    } 
    for (int i=0;i<rowAcons;++i)
    {
        for (int j=0;j<colAcons;++j)
        {
            A_constraint_qpOASES[i+j*rowAcons] = A_constraint(i,j);
        }
    }
    for (int i=0;i<rowBcons;++i)
    {
        for (int j=0;j<colBcons;++j)
        {
            B_constraint_qpOASES[i+j*rowBcons] = B_constraint(i,j);
        }
    }
    for (int i=0;i<rowg;++i)
    {
        g_qpOASES[i] = g(i);
    }

    //定义维度
    int nV = Nc*2;
    int nC = (Np*10)+6;

    //创建求解器
    SQProblem qp(nV,nC);
    qp.setPrintLevel(qpOASES::PL_NONE);

    //设置约束上下界
    real_t*lbA = new real_t[nC];
    real_t*ubA = new real_t[nC];
    for (int i=0;i<nC;++i)
    {
        lbA[i] = -std::numeric_limits<real_t>::infinity();
        ubA[i] = B_constraint_qpOASES[i];
    }

    //初始化并且求解QP
    int nWSR = 1000;
    qp.init(H_qpOASES,g_qpOASES,A_constraint_qpOASES,nullptr,nullptr,lbA,ubA,nWSR);

    //获取最优解
    real_t* u_opt = new real_t[nV];
    qp.getPrimalSolution(u_opt);

    //转换为Eigen向量
    Eigen::VectorXf delta_u(nV);
    for(int i=0;i<nV;++i)
    {
        delta_u(i) = u_opt[i];
    }
    cmd_v = v1-delta_u(0);
    cmd_w = w1-delta_u(1);

    cmd_w = -cmd_w;

    twist.linear.x =cmd_v;
    twist.angular.z = cmd_w;



    ROS_INFO("weight_Q1=%f",weight_Q1); 
    ROS_INFO("weight_Q2=%f",weight_Q2);    
    ROS_INFO("weight_Q3=%f",weight_Q3);
    ROS_INFO("weight_R=%f",weight_R);    
    ROS_INFO("weight_S=%f",weight_S);

    ROS_INFO("dx=%f",dx);    
    ROS_INFO("dy=%f",dy);  
    ROS_INFO("k_dtheta=%f",k_dtheta);  
    ROS_INFO("Leader(%f,%f,%f)",x_1_hat,y_1_hat,f_1_hat);  
    ROS_INFO("Follower2(%f,%f,%f)",x_3_hat,y_3_hat,f_3_hat);  
    ROS_INFO("v1=%f",v1);
    ROS_INFO("v3=%f",v3);
    ROS_INFO("w1=%f",w1);
    ROS_INFO("w3=%f",w3);    
    ROS_INFO("cmd_v=%f",cmd_v);
    ROS_INFO("cmd_w=%f",cmd_w);
    ROS_INFO("----------");



}

// 回调函数定义
void v1Callback(const std_msgs::Float32::ConstPtr& msg)
{
    v1 = msg->data;
}

void w1Callback(const std_msgs::Float32::ConstPtr& msg)
{
    w1 = msg->data;
    w1 = -w1;
}

void v2Callback(const std_msgs::Float32::ConstPtr& msg)
{
    v2 = msg->data;
}

void w2Callback(const std_msgs::Float32::ConstPtr& msg)
{
    w2 = msg->data;
    w2 = -w2;
}

void imuCallback(const std_msgs::Float32::ConstPtr& msg)
{
    w3 = -msg->data;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    v3 = msg->twist.twist.linear.x;
}

void x1hatCallback(const std_msgs::Float32::ConstPtr& msg)
{
    x_1_hat = msg->data;
}

void y1hatCallback(const std_msgs::Float32::ConstPtr& msg)
{
    y_1_hat = msg->data;
}

void f1hatCallback(const std_msgs::Float32::ConstPtr& msg)
{
    f_1_hat = msg->data;
}

void x2hatCallback(const std_msgs::Float32::ConstPtr& msg)
{
    x_2_hat = msg->data;
}

void y2hatCallback(const std_msgs::Float32::ConstPtr& msg)
{
    y_2_hat = msg->data;
}

void f2hatCallback(const std_msgs::Float32::ConstPtr& msg)
{
    f_2_hat = msg->data;
}

void x3hatCallback(const std_msgs::Float32::ConstPtr& msg)
{
    x_3_hat = msg->data;
}

void y3hatCallback(const std_msgs::Float32::ConstPtr& msg)
{
    y_3_hat = msg->data;
}

void f3hatCallback(const std_msgs::Float32::ConstPtr& msg)
{
    f_3_hat = msg->data;
}

void formation_changeCallback(const std_msgs::Bool::ConstPtr& msg)
{
    formation_change = msg->data;
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "mpc_controller");
    ros::NodeHandle nh;

    // 订阅话题
    ros::Subscriber imu_sub = nh.subscribe<std_msgs::Float32>("imu_fusion", 1, imuCallback);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, odomCallback);
    ros::Subscriber v1_sub = nh.subscribe<std_msgs::Float32>("/v1", 1, v1Callback);
    ros::Subscriber w1_sub = nh.subscribe<std_msgs::Float32>("/w1", 1, w1Callback);
    ros::Subscriber v2_sub = nh.subscribe<std_msgs::Float32>("/v2", 1, v2Callback);
    ros::Subscriber w2_sub = nh.subscribe<std_msgs::Float32>("/w2", 1, w2Callback);
    ros::Subscriber x_1_hat_sub = nh.subscribe<std_msgs::Float32>("/x_1_hat", 1, x1hatCallback);
    ros::Subscriber y_1_hat_sub = nh.subscribe<std_msgs::Float32>("/y_1_hat", 1, y1hatCallback);
    ros::Subscriber f_1_hat_sub = nh.subscribe<std_msgs::Float32>("/f_1_hat", 1, f1hatCallback);
    ros::Subscriber x_2_hat_sub = nh.subscribe<std_msgs::Float32>("/x_2_hat", 1, x2hatCallback);
    ros::Subscriber y_2_hat_sub = nh.subscribe<std_msgs::Float32>("/y_2_hat", 1, y2hatCallback);
    ros::Subscriber f_2_hat_sub = nh.subscribe<std_msgs::Float32>("/f_2_hat", 1, f2hatCallback);
    ros::Subscriber x_3_hat_sub = nh.subscribe<std_msgs::Float32>("/x_3_hat", 1, x3hatCallback);
    ros::Subscriber y_3_hat_sub = nh.subscribe<std_msgs::Float32>("/y_3_hat", 1, y3hatCallback);
    ros::Subscriber f_3_hat_sub = nh.subscribe<std_msgs::Float32>("/f_3_hat", 1, f3hatCallback);
    ros::Subscriber formation_change_sub = nh.subscribe<std_msgs::Bool>("/formation_change", 1, formation_changeCallback);    
    
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // 设置循环频率
    ros::Rate loop_rate(10);

    nh.getParam("Np", Np);
    nh.getParam("Nc", Nc);
    nh.getParam("v_low", v_low);
    nh.getParam("w_low", w_low);
    nh.getParam("dx_beforechange", dx_beforechange);
    nh.getParam("dy_beforechange", dy_beforechange);  
    nh.getParam("dx_afterchange", dx_afterchange);
    nh.getParam("dy_afterchange", dy_afterchange);  
    nh.getParam("k_dtheta", k_dtheta);      

    // 初始化权重矩阵
    Q << weight_Q1, 0, 0,
         0, weight_Q2, 0,
         0, 0, weight_Q3;

    R << weight_R, 0,
         0, weight_R;

    S << weight_S, 0, 0,
         0, weight_S, 0,
         0, 0, weight_S;

    // 创建Omega矩阵
    Omega = createOmega(Np, Q, S);
    Psi = createPsi(Nc, R);

    //
    derta_u_low << v_low, w_low;
    derta_u_high << v_high, w_high;
    e_low << xyf_low, xyf_low, xyf_low;
    e_high << xyf_high, xyf_high, xyf_high;

    // 约束用矩阵初始化
    I_2x2 = Eigen::MatrixXf::Identity(2, 2);
    I_3x3 = Eigen::MatrixXf::Identity(3, 3);
    O_2x3 = Eigen::MatrixXf::Zero(2, 3);
    O_3x2 = Eigen::MatrixXf::Zero(3, 2);
    O_10x3 = Eigen::MatrixXf::Zero(10, 3);
    O_10x2 = Eigen::MatrixXf::Zero(10, 2);
    O_6x3 = Eigen::MatrixXf::Zero(6, 3);
    O_6x2 = Eigen::MatrixXf::Zero(6, 2);

    M_10x3 = Eigen::MatrixXf::Zero(10, 3);
    M_10x3.block(0, 0, 2, 3) = O_2x3;
    M_10x3.block(2, 0, 2, 3) = O_2x3;
    M_10x3.block(4, 0, 3, 3) = -I_3x3;
    M_10x3.block(7, 0, 3, 3) = I_3x3;

    F_10x2 = Eigen::MatrixXf::Zero(10, 2);
    F_10x2.block(0, 0, 2, 2) = -I_2x2;
    F_10x2.block(2, 0, 2, 2) = I_2x2;
    F_10x2.block(4, 0, 3, 2) = O_3x2;
    F_10x2.block(7, 0, 3, 2) = O_3x2;

    Beita = Eigen::MatrixXf::Zero(10, 1);
    Beita.block(0, 0, 2, 1) = -derta_u_low;
    Beita.block(2, 0, 2, 1) = derta_u_high;
    Beita.block(4, 0, 3, 1) = -e_low;
    Beita.block(7, 0, 3, 1) = e_high;

    M_Np = Eigen::MatrixXf::Zero(6, 3);
    M_Np.block(0, 0, 3, 3) = -I_3x3;
    M_Np.block(3, 0, 3, 3) = I_3x3;

    Beita_Np = Eigen::MatrixXf::Zero(6, 1);
    Beita_Np.block(0, 0, 3, 1) = -e_low;
    Beita_Np.block(3, 0, 3, 1) = e_high;

    M1 = createM1(Np);
    M2 = createM2(Np);
    F2 = createF2(Np,Nc); 
    Beita_1 = createBeita_1(Np);   
    // 主循环
    while (ros::ok())
    {
        if(formation_change==1)
        {
            dx = dx_afterchange;
            dy = dy_afterchange;
        }
        if(formation_change==0)
        {
            dx = dx_beforechange;
            dy = dy_beforechange;
        }     
        // 执行MPC算法
        MPC();
        cmd_vel_pub.publish(twist);
    
        // 处理ROS消息
        ros::spinOnce();
        loop_rate.sleep();
    }

    if (H_qpOASES != nullptr) delete[] H_qpOASES;
    if (F_qpOASES != nullptr) delete[] F_qpOASES; 
    if (A_constraint_qpOASES != nullptr) delete[] A_constraint_qpOASES;
    if (B_constraint_qpOASES != nullptr) delete[] B_constraint_qpOASES; 
    if (g_qpOASES != nullptr) delete[] g_qpOASES;     
    return 0;
}
