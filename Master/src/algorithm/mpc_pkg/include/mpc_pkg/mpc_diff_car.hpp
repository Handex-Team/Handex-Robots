#ifndef OPTIMIZER_H
#define OPTIMIZER_H
#include <Eigen/Dense>
#include "OsqpEigen/OsqpEigen.h"
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <iostream>
#include <cmath>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "tf2_msgs/msg/tf_message.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// 实现了差分动力学模型的MPC算法，线性化该NMPC问题,用OSQP求解
// 包含状态类、轨迹点类、MPC控制器类
namespace Mpc_diff_car {

    class MPCState{
    public:
        double x = 0;
        double y = 0;
        double v = 0;
        double theta = 0;
        double w = 0;
    };

    class TrajPoint{
    public:
        double x = 0;
        double y = 0;
        double v = 0;
        double a = 0;
        double theta = 0;
        double w = 0;
        double t = 0;// 相对于轨迹起点的时间间隔
    };
    class MPC_Controller {
    public:
        MPC_Controller(const MPCState &s, const nav_msgs::msg::Path &path);
        //  void get_linear_model(const MPCState s);
        //  void state_trans(MPCState &s, double v, double w);
        void init_controller();
        // 根据当前状态展开，计算线性化矩阵
        void get_Ak_Bk_Ok(const MPCState &s);
        void get_Abar_Bbar_Cbar_O();

        // 生成OSQP的API：二次型矩阵H和线性型矩阵q,等式和不等式约束l<=Ax<=u
        void get_H_q_A_l_u();

        // 找到轨迹中里程最接近当前时间戳最近的轨迹点索引
        int find_closest_trajectory_point();

        // 获取当前时间戳的目标状态
        MPCState get_target_state(const MPCState &s);

        // 获取当前时间戳的状态向量(这里的状态向量是指N个预测步长的状态，通过名义轨迹点的输入计算得到的)和参考状态向量
        void get_X_r_X_k();

        void set_Q_R();

        Eigen::VectorXd OSQP_solve();

     


    
        // MPC参数
        double dt; // 时间步长
        double v_max; // 最大速度
        double a_max; // 最大加速度
        double w_max; // 最大角速度
        double j_max; // 最大角加速度
        int N; // 预测步数
        MPCState s_now;// 当前状态
        nav_msgs::msg::Path::SharedPtr path_; // 轨迹点序列
        Eigen::MatrixXd Q; // 状态权重矩阵
        Eigen::MatrixXd R; // 控制权重矩阵
        Eigen::VectorXd s_eigen, s_ref_eigen, X_k, X_r; // 预测模型中，当前状态和参考状态的误差向量;eigen接口的当前状态和参考状态
        OsqpEigen::Solver solver;
        bool is_initialized = false; // 求解器是否初始化
        rclcpp::Time now_time;


        
        // 线性化矩阵
        Eigen::MatrixXd Ak;
        Eigen::MatrixXd Bk;
        Eigen::MatrixXd Ok;
                /*
                ↓
                ↓
                ↓
                */
        Eigen::MatrixXd A_hat;
        Eigen::MatrixXd B_hat;
        Eigen::MatrixXd O_hat;
                /*
                ↓
                ↓
                ↓
                */
        Eigen::MatrixXd A_bar;
        Eigen::MatrixXd B_bar;
        Eigen::MatrixXd C_bar;
        Eigen::MatrixXd O;
                /*
                ↓
                ↓
                ↓
                */
        // OSQP入口参数
        Eigen::MatrixXd H;
        Eigen::MatrixXd A;
        Eigen::VectorXd q;
        Eigen::VectorXd l;
        Eigen::VectorXd u;
        

        


    };



}


#endif