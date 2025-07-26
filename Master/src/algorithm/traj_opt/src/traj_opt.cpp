#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "gcopter/trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "OsqpEigen/OsqpEigen.h"
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cstring>
#include "string.h"
#include "visualization_msgs/msg/marker.hpp"
// 欧拉角转换
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// 线程有关
#include <thread>
#include <atomic>




#define POLY_ORDER 5    // 五次多项式
#define DIMENSIONS 2    // 二维空间

using namespace std::chrono_literals;
using namespace std::placeholders;

struct state_odom{
    double vel;
    double acc;
};
class TrajectoryOptimizer : public rclcpp::Node
{
public:
    TrajectoryOptimizer() : Node("trajectory_optimizer"),clock(RCL_SYSTEM_TIME)
    {
        // 初始化参数
        params_.max_vel = 1.0; 
        params_.max_acc = 0.5;
        params_.stable_vel = params_.max_vel / 1.2;
        params_.dt = 0.1;// 这里必须和mpc中的dt保持一致

        // 订阅原始路径
        raw_traj_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/straight_plan", 10,
            std::bind(&TrajectoryOptimizer::raw_traj_callback, this, _1));

        // 发布优化后的路径
        RDP_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("rdp_path", 10);
        optimized_traj_pub_ = this->create_publisher<nav_msgs::msg::Path>("optimized_path", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 1,
            std::bind(&TrajectoryOptimizer::odom_callback, this, _1));
        

        RCLCPP_INFO(this->get_logger(), "轨迹优化节点初始化完成");

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&TrajectoryOptimizer::goal_callback, this, _1));
    }

private:
    // 优化参数结构体
    struct OptimizationParams {
        double max_vel;        // 最大线速度 (m/s)
        double max_acc;        // 最大加速度 (m/s²)
        double stable_vel;     // 巡航速度
        double dt;             // 时间步长
    } params_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr raw_traj_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimized_traj_pub_, RDP_path_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;// 发布当前时刻小车的理想位置
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;// 订阅目标点
    rclcpp::Clock clock;// 时钟

    // OsqpEigen::Solver solver;

    // 运动状态
    nav_msgs::msg::Odometry odom_;
    double global_yaw = 0.0;
    double global_vel_x = 0.0;
    double global_vel_y = 0.0;
    double global_acc_x = 0.0;
    double global_acc_y = 0.0;
    double abs_vel_min = 0.05; // 小于这个速度认为速度为0，给定轨迹初速度的时候，初速度约束都为正值


    double prev_vel_x_ = 0.0, prev_vel_y_ = 0.0;
    rclcpp::Time prev_time_;

    nav_msgs::msg::Path path_msg;// 发布的优化后的路径
    bool is_init = false; // 是否初始化

    geometry_msgs::msg::PoseStamped goal_pose;
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        goal_pose = *msg;
    }
    // 获取当前的x,y轴运动状态
    void cul_vel_xy(){

        tf2::Quaternion imu_quat(
        odom_.pose.pose.orientation.x,
        odom_.pose.pose.orientation.y,
        odom_.pose.pose.orientation.z,
        odom_.pose.pose.orientation.w);
        double roll, pitch, yaw;//定义存储r\p\y的容器
        tf2::Matrix3x3 m(imu_quat);
        m.getRPY(roll, pitch, yaw);//进行转换
        global_yaw = yaw;
        global_vel_x = odom_.twist.twist.linear.x * cos(global_yaw);
        global_vel_y = odom_.twist.twist.linear.x * sin(global_yaw);
    }

    void cul_acc_xy()
    {
        auto current_time = clock.now();
        double dt = (current_time - prev_time_).seconds();  // 直接获取秒数
        if (dt > 0) // 避免 dt = 0 导致的除零错误
        {
            global_acc_x = (global_vel_x - prev_vel_x_) / dt;
            global_acc_y = (global_vel_y - prev_vel_y_) / dt;
        }
        else
        {
            global_acc_x = 0.0;
            global_acc_y = 0.0;
        }

        // 更新存储的速度和时间
        prev_vel_x_ = global_vel_x;
        prev_vel_y_ = global_vel_y;
        prev_time_ = current_time;
    }

    // OSQP接口
    Eigen::MatrixXd get_jerk_quadratic_matrix(const Trajectory<POLY_ORDER, DIMENSIONS>& traj){
        // 计算每个轨迹段jerk的二次项矩阵
        int num_segments = traj.getPieceNum();
        int matrix_size = POLY_ORDER + 1;
        // 6*6矩阵
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(matrix_size, matrix_size);
        Eigen::MatrixXd Q_total = Eigen::MatrixXd::Zero(num_segments * matrix_size, num_segments * matrix_size);
        auto t = traj.getDurations();
        for (int k = 0; k < num_segments; ++k){
            // 遍历每个轨迹段，但是求完3次导(jerk)之后，只需要操作右下角3*3矩阵即可
            double t_k = t(k);
            double t_k2 = t_k * t_k;
            double t_k3 = t_k2 * t_k;
            double t_k4 = t_k2 * t_k2;
            double t_k5 = t_k4 * t_k;

            Eigen::Block<Eigen::MatrixXd> Q_block = Q.block(3, 3, 3, 3);
            Q_block(0, 0) = 36 * t_k;
            Q_block(0, 1) = 72 * t_k2;
            Q_block(0, 2) = 120 * t_k3;
            Q_block(1, 0) = 72 * t_k2;
            Q_block(1, 1) = 192 * t_k3;
            Q_block(1, 2) = 360 * t_k4;
            Q_block(2, 0) = 120 * t_k3;
            Q_block(2, 1) = 360 * t_k4;
            Q_block(2, 2) = 720 * t_k5;

            Q_total.block(k * matrix_size, k * matrix_size, matrix_size, matrix_size) = Q;
        }
        return Q_total;

    }

    Eigen::MatrixXd get_A_expand_matrix(const Trajectory<POLY_ORDER, DIMENSIONS>& traj,
                     std::vector<double> path, state_odom &s) {
        
        int num_segments = traj.getPieceNum();
        int matrix_size = POLY_ORDER + 1;
        auto T = traj.getDurations();
        
        // 计算段首位置约束矩阵
        Eigen::MatrixXd Head_pos = Eigen::MatrixXd::Zero(num_segments, matrix_size * num_segments);
        Eigen::VectorXd Head_pos_l = Eigen::VectorXd::Zero(num_segments);
        Eigen::VectorXd Head_pos_u = Eigen::VectorXd::Zero(num_segments);
        for (int i = 0; i < num_segments; i++) {
            Head_pos(i, i * matrix_size) = 1.0;
            Head_pos_l(i) = path[i];
            Head_pos_u(i) = path[i];
        }
        Head_pos.conservativeResize(Eigen::NoChange, num_segments * matrix_size + 2);
        Head_pos.col(matrix_size * num_segments) = Head_pos_l;
        Head_pos.col(matrix_size * num_segments + 1) = Head_pos_u;
        // std::cout << "Head_pos:\n" << Head_pos << std::endl;

        // 计算段末位置约束矩阵
        Eigen::MatrixXd Tail_pos = Eigen::MatrixXd::Zero(num_segments, matrix_size * num_segments);
        Eigen::VectorXd Tail_pos_l = Eigen::VectorXd::Zero(num_segments);
        Eigen::VectorXd Tail_pos_u = Eigen::VectorXd::Zero(num_segments);
        
        for (int i = 0; i < num_segments; i++) {
            double T_2 = T(i) * T(i);
            double T_3 = T_2 * T(i);
            double T_4 = T_3 * T(i);
            double T_5 = T_4 * T(i);
            Tail_pos(i, i * matrix_size + 0) = 1.0;
            Tail_pos(i, i * matrix_size + 1) = T(i);
            Tail_pos(i, i * matrix_size + 2) = T_2;
            Tail_pos(i, i * matrix_size + 3) = T_3;
            Tail_pos(i, i * matrix_size + 4) = T_4;
            Tail_pos(i, i * matrix_size + 5) = T_5;

            Tail_pos_l(i) = path[i+1];
            Tail_pos_u(i) = path[i+1];
        }
        Tail_pos.conservativeResize(Eigen::NoChange, num_segments * matrix_size + 2);
        Tail_pos.col(matrix_size * num_segments) = Tail_pos_l;
        Tail_pos.col(matrix_size * num_segments + 1) = Tail_pos_l;
        // std::cout << "Tail_pos:\n" << Tail_pos << std::endl;

        
        // 起止位置的速度和加速度约束矩阵  start_goal_vel_acc 4*6M
        Eigen::MatrixXd start_goal_vel_acc = Eigen::MatrixXd::Zero( 4, matrix_size * num_segments);
        Eigen::VectorXd start_goal_vel_acc_l = Eigen::VectorXd::Zero(4);//起止速度和加速度都为0
        Eigen::VectorXd start_goal_vel_acc_u = Eigen::VectorXd::Zero(4);//起止速度和加速度都为0
        start_goal_vel_acc(0, 1) = 1.0;//选择起始速度
        start_goal_vel_acc(1, 2) = 1.0;//选择起点加速度

        
        start_goal_vel_acc_l(0) = s.vel;//起始速度
        start_goal_vel_acc_u(0) = s.vel;
        
        

        start_goal_vel_acc_l(3) = s.acc;//起始加速度
        start_goal_vel_acc_u(3) = s.acc;

        // start_goal_vel_acc_l(0) = 0;//起始速度
        // start_goal_vel_acc_u(0) = 0;

        // start_goal_vel_acc_l(3) = 0;//起始加速度
        // start_goal_vel_acc_u(3) = 0;

        //终点速度
        start_goal_vel_acc(2, matrix_size * (num_segments - 1) + 1) = 1.0;
        start_goal_vel_acc(2, matrix_size * (num_segments - 1) + 2) = 2.0 * T(num_segments - 1);
        start_goal_vel_acc(2, matrix_size * (num_segments - 1) + 3) = 3.0 * T(num_segments - 1) * T(num_segments - 1);
        start_goal_vel_acc(2, matrix_size * (num_segments - 1) + 4) = 4.0 * T(num_segments - 1) * T(num_segments - 1) * T(num_segments - 1);
        start_goal_vel_acc(2, matrix_size * (num_segments - 1) + 5) = 5.0 * T(num_segments - 1) * T(num_segments - 1) * 
                T(num_segments - 1) * T(num_segments - 1);
        
        //终点加速度
        start_goal_vel_acc(3, matrix_size * (num_segments - 1) + 2) = 2.0;
        start_goal_vel_acc(3, matrix_size * (num_segments - 1) + 3) = 6.0 * T(num_segments - 1);
        start_goal_vel_acc(3, matrix_size * (num_segments - 1) + 4) = 12.0 * T(num_segments - 1) * T(num_segments - 1);
        start_goal_vel_acc(3, matrix_size * (num_segments - 1) + 5) = 20.0 * T(num_segments - 1) * T(num_segments - 1) * T(num_segments - 1);

        start_goal_vel_acc.conservativeResize(Eigen::NoChange, num_segments * matrix_size + 2);
        start_goal_vel_acc.col(matrix_size * num_segments) = start_goal_vel_acc_l;
        start_goal_vel_acc.col(matrix_size * num_segments + 1) = start_goal_vel_acc_u;

        // 速度连续性约束矩阵V_cont   (M-1)*6M
        Eigen::MatrixXd V_cont = Eigen::MatrixXd::Zero(num_segments - 1, matrix_size * num_segments);
        Eigen::VectorXd V_cont_l = Eigen::VectorXd::Zero(num_segments - 1);
        Eigen::VectorXd V_cont_u = Eigen::VectorXd::Zero(num_segments - 1);
        for (int i = 0; i < num_segments - 1; i++) {
            double T_2 = T(i) * T(i);
            double T_3 = T_2 * T(i);
            double T_4 = T_3 * T(i);

            // 前一段轨迹的末端速度
            V_cont(i, i * matrix_size + 1) = 1.0;
            V_cont(i, i * matrix_size + 2) = 2.0 * T(i);
            V_cont(i, i * matrix_size + 3) = 3.0 * T_2;
            V_cont(i, i * matrix_size + 4) = 4.0 * T_3;
            V_cont(i, i * matrix_size + 5) = 5.0 * T_4;
            // 后一段轨迹的初始速度
            V_cont(i, i * matrix_size + 7) = -1.0;
        }
        V_cont.conservativeResize(Eigen::NoChange, num_segments * matrix_size + 2);
        V_cont.col(matrix_size * num_segments) = V_cont_l;
        V_cont.col(matrix_size * num_segments + 1) = V_cont_u;

        // 加速度连续性约束矩阵a_cont   (M-1)*6M
        Eigen::MatrixXd a_cont = Eigen::MatrixXd::Zero(num_segments - 1, matrix_size * num_segments);
        Eigen::VectorXd a_cont_l = Eigen::VectorXd::Zero(num_segments - 1);
        Eigen::VectorXd a_cont_u = Eigen::VectorXd::Zero(num_segments - 1);
        for (int i = 0; i < num_segments - 1; i++) {
            double T_2 = T(i) * T(i);
            double T_3 = T_2 * T(i);

            // 前一段轨迹的末端速度
            a_cont(i, i * matrix_size + 2) = 2.0;
            a_cont(i, i * matrix_size + 3) = 3.0 * 2.0 * T(i);
            a_cont(i, i * matrix_size + 4) = 4.0 * 3.0 * T_2;
            a_cont(i, i * matrix_size + 5) = 5.0 * 4.0 *T_3;
            // 后一段轨迹的初始速度
            a_cont(i, i * matrix_size + 8) = -2.0;
        }
        a_cont.conservativeResize(Eigen::NoChange, num_segments * matrix_size + 2);
        a_cont.col(matrix_size * num_segments) = a_cont_l;
        a_cont.col(matrix_size * num_segments + 1) = a_cont_u;

        // 速度大小约束矩阵v_max_min   
        Eigen::MatrixXd v_max_min = Eigen::MatrixXd::Zero(num_segments, matrix_size * num_segments);
        Eigen::VectorXd v_max_min_l = Eigen::VectorXd::Ones(num_segments) * -params_.max_vel;
        // Eigen::VectorXd v_max_min_l = Eigen::VectorXd::Zero(num_segments);
        Eigen::VectorXd v_max_min_u = Eigen::VectorXd::Ones(num_segments) * params_.max_vel;
        for (int i = 0; i < num_segments; i++) {
            double T_2 = T(i) * T(i);
            double T_3 = T_2 * T(i);
            double T_4 = T_3 * T(i);

            // 轨迹的末端速度
            v_max_min(i, i * matrix_size + 1) = 1.0;
            v_max_min(i, i * matrix_size + 2) = 2.0 * T(i);
            v_max_min(i, i * matrix_size + 3) = 3.0 * T_2;
            v_max_min(i, i * matrix_size + 4) = 4.0 * T_3;
            v_max_min(i, i * matrix_size + 5) = 5.0 * T_4;
        }
        v_max_min.conservativeResize(Eigen::NoChange, num_segments * matrix_size + 2);
        v_max_min.col(matrix_size * num_segments) = v_max_min_l;
        v_max_min.col(matrix_size * num_segments + 1) = v_max_min_u;

        // 加速度大小约束矩阵a_max_min   
        Eigen::MatrixXd a_max_min = Eigen::MatrixXd::Zero(num_segments, matrix_size * num_segments);
        Eigen::VectorXd a_max_min_l = Eigen::VectorXd::Ones(num_segments) * -params_.max_acc;
        // Eigen::VectorXd a_max_min_l = Eigen::VectorXd::Zero(num_segments); // 不允许加速度方向为负
        Eigen::VectorXd a_max_min_u = Eigen::VectorXd::Ones(num_segments) * params_.max_acc;
        for (int i = 0; i < num_segments; i++) {
            double T_2 = T(i) * T(i);
            double T_3 = T_2 * T(i);

            // 轨迹的末端加速度
            a_max_min(i, i * matrix_size + 2) = 2.0;
            a_max_min(i, i * matrix_size + 3) = 3.0 * 2.0 * T(i);
            a_max_min(i, i * matrix_size + 4) = 4.0 * 3.0 * T_2;
            a_max_min(i, i * matrix_size + 5) = 5.0 * 4.0 *T_3;
        }
        a_max_min.conservativeResize(Eigen::NoChange, num_segments * matrix_size + 2);
        a_max_min.col(matrix_size * num_segments) = a_max_min_l;
        a_max_min.col(matrix_size * num_segments + 1) = a_max_min_u;

        // 总约束矩阵
        Eigen::MatrixXd A_total = Eigen::MatrixXd::Zero(
            Head_pos.rows() + Tail_pos.rows() + start_goal_vel_acc.rows() + V_cont.rows() + a_cont.rows() + v_max_min.rows() + a_max_min.rows(),
            num_segments * matrix_size + 2
            );  
        A_total.block(0, 0, Head_pos.rows(), num_segments * matrix_size + 2) = Head_pos;
        A_total.block(Head_pos.rows(), 0, Tail_pos.rows(), num_segments * matrix_size + 2) = Tail_pos;
        A_total.block(Head_pos.rows() + Tail_pos.rows(), 0, start_goal_vel_acc.rows(), num_segments * matrix_size + 2) = start_goal_vel_acc;
        A_total.block(Head_pos.rows() + Tail_pos.rows() + start_goal_vel_acc.rows(), 0, V_cont.rows(), num_segments * matrix_size + 2) = V_cont;
        A_total.block(Head_pos.rows() + Tail_pos.rows() + start_goal_vel_acc.rows() + V_cont.rows(),
                                 0, a_cont.rows(), num_segments * matrix_size + 2) = a_cont;

        A_total.block(Head_pos.rows() + Tail_pos.rows() + start_goal_vel_acc.rows() + V_cont.rows() + a_cont.rows(),
                                0, v_max_min.rows(), num_segments * matrix_size + 2) = v_max_min;  
        A_total.block(Head_pos.rows() + Tail_pos.rows() + start_goal_vel_acc.rows() + V_cont.rows() + a_cont.rows() + v_max_min.rows(),
                                0, a_max_min.rows(), num_segments * matrix_size + 2) = a_max_min;  

        return A_total;
    }

    Eigen::VectorXd OSQP_expand(const Eigen::MatrixXd &Q, const Eigen::MatrixXd &A_expand){
        // 创建 OSQP 优化器
        int dim = Q.cols();
        OsqpEigen::Solver solver;
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(false);//这里的QP问题前后解的维度会发生改变，不能简单延用热启动
        // 定义二次项矩阵 P
        // Eigen::MatrixXd P = 2 * Q;//标准的接口是1/2XPX,所以要乘2
        Eigen::MatrixXd P = 2 * Q + 1e-6 * Eigen::MatrixXd::Identity(Q.rows(), Q.cols());

        Eigen::SparseMatrix<double> P_sparse = P.sparseView();

        // 线性约束矩阵 A
        Eigen::MatrixXd A = A_expand.block(0,0,A_expand.rows(),A_expand.cols()-2);
        Eigen::SparseMatrix<double> A_sparse = A.sparseView();

        // 线性项 q
        Eigen::VectorXd q = Eigen::VectorXd::Zero(dim);

        // // 约束下界 l 和上界 u
        Eigen::VectorXd l = A_expand.col(A_expand.cols()-2);
        Eigen::VectorXd u = A_expand.col(A_expand.cols()-1);

        // 初始化 OSQP 求解器
        solver.data()->setNumberOfVariables(dim);
        solver.data()->setNumberOfConstraints(A.rows());
        solver.data()->setHessianMatrix(P_sparse);
        solver.data()->setGradient(q);
        solver.data()->setLinearConstraintsMatrix(A_sparse);
        solver.data()->setLowerBound(l);
        solver.data()->setUpperBound(u);

        if (!solver.initSolver()) {
            std::cout << "OSQP solver initialization failed!" << std::endl;
            return Eigen::VectorXd::Zero(dim);
        }
        Eigen::VectorXd solution = Eigen::VectorXd::Zero(dim);
        // 求解
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
        {
            RCLCPP_ERROR(this->get_logger(), "OSQP solver failed to find a solution");
            return solution;
        }

        // 获取解
        solution = solver.getSolution();
        return solution;
    }


    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        odom_ = *msg;
        double v_limit = 0.0001, w_limit = 0.0001;
        if(odom_.twist.twist.linear.x>0 && odom_.twist.twist.linear.x<v_limit){
            odom_.twist.twist.linear.x = v_limit;
        }
        else if(odom_.twist.twist.linear.x<0 && odom_.twist.twist.linear.x>-v_limit){
            odom_.twist.twist.linear.x = -v_limit;
        }
        
        if(odom_.twist.twist.angular.z>0 && odom_.twist.twist.angular.z<w_limit){
            odom_.twist.twist.angular.z = w_limit;
        }
        else if(odom_.twist.twist.angular.z<0 && odom_.twist.twist.angular.z>-w_limit){
            odom_.twist.twist.angular.z = -w_limit;
        }
        cul_vel_xy();
        cul_acc_xy();
    }
    // 路径回调函数
    void raw_traj_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {   
        path_msg.header.stamp = msg->header.stamp;
        // 路径预处理，如果路径点数大于50，先对路径进行rdp处理，然后再进行优化
        nav_msgs::msg::Path sparse_path;
        if (msg->poses.size() > 100){
            double epsilon = 0.0001; // 允许的最大误差（单位：米）
            sparse_path = sparsifyPath(msg, epsilon);
        }
        else{
            sparse_path = *msg;
        }
        RDP_path_pub_->publish(sparse_path);

        const int num_points = sparse_path.poses.size();
        if (num_points < 2) {
            RCLCPP_WARN(this->get_logger(), "路径点数不足,至少需要2个点");
            return;
        }
        std::vector<double> path_x, path_y;
        for (int i = 0; i < num_points; ++i) {
            path_x.push_back(sparse_path.poses[i].pose.position.x);
            path_y.push_back(sparse_path.poses[i].pose.position.y);
        }
        
        /* 步骤1：路径预处理 */
        // 计算段数和时间分配
        const int num_segments = num_points - 1;
        RCLCPP_INFO(this->get_logger(), "sparse路径共%d段", num_segments);
        std::vector<double> segment_lengths(num_segments);
        std::vector<double> durs(num_segments);
        for (int i = 0; i < num_segments; ++i) {
            const auto& p1 = sparse_path.poses[i].pose.position;
            const auto& p2 = sparse_path.poses[i+1].pose.position;
            double v_total_max = 1.414 * params_.max_vel; // 最大速度
            double a_total_max = 1.414 * params_.max_acc; // 最大加速度
            
            segment_lengths[i] = std::hypot(p2.x - p1.x, 
                                           p2.y - p1.y);
            
            // durs[i] = segment_lengths[i] / params_.stable_vel;
            durs[i] = segment_lengths[i] / v_total_max + v_total_max / a_total_max;
        }

        /* 步骤2：构造初始多项式系数 */
        std::vector<Eigen::Matrix<double, DIMENSIONS, POLY_ORDER + 1>> coeffs;
        for (auto i = durs.begin(); i != durs.end(); ++i){
            Eigen::Matrix<double, DIMENSIONS, POLY_ORDER + 1> coeff;
            coeff.setRandom();
            coeffs.push_back(coeff);
        }
        Trajectory<POLY_ORDER, DIMENSIONS> traj(durs, coeffs);

        /* 步骤3：优化多项式系数(用前端话题的轨迹) */
        Eigen::MatrixXd Q = get_jerk_quadratic_matrix(traj);
        state_odom state_x;
        state_odom state_y;
        state_x.vel = global_vel_x;
        state_y.vel = global_vel_y;
        state_x.acc = global_acc_x;
        state_y.acc = global_acc_y;

        // 此时传进去的traj只有durs是有用的，coeffs初始是随机的
        Eigen::MatrixXd A_expand_x = get_A_expand_matrix(traj, path_x,state_x);
        Eigen::MatrixXd A_expand_y = get_A_expand_matrix(traj, path_y,state_y);  
        
        // path_msg.header.stamp = clock.now();
        
        auto solution_x = OSQP_expand(Q, A_expand_x);
        auto solution_y = OSQP_expand(Q, A_expand_y);

        if(!isSolutionValid(solution_x) ||!isSolutionValid(solution_y)){
            RCLCPP_ERROR(rclcpp::get_logger("trajectory_publisher"), "由于等式约束不满足，优化失败，延用上一次的结果");
            return;
        }
        path_msg.poses.clear();

        // 生成优化后的轨迹
        std::vector<Eigen::Matrix<double, DIMENSIONS, POLY_ORDER + 1>> opt_coeffs(durs.size());
        for (int i = 0; i < num_segments; i++){
            opt_coeffs[i].row(0) = solution_x.segment(i * (POLY_ORDER + 1), POLY_ORDER + 1).reverse();
            opt_coeffs[i].row(1) = solution_y.segment(i * (POLY_ORDER + 1), POLY_ORDER + 1).reverse();
        }
        Trajectory<POLY_ORDER, DIMENSIONS> opt_traj(durs, opt_coeffs);

        publishOptimizedTrajectory(opt_traj);
       
    }

    bool isSolutionValid(const Eigen::VectorXd &solution_x) {
        if (!solution_x.allFinite()) {
            std::cout << "Error: solution_x contains NaN or Inf!" << std::endl;
            return false;
        }
        if (solution_x.isZero()) {
            std::cout << "Warning: solution_x is all zeros!" << std::endl;
        }
        double max_value = 1e6, min_value = -1e6;
        if ((solution_x.array() > max_value).any() || (solution_x.array() < min_value).any()) {
            std::cout << "Error: solution_x contains values out of range!" << std::endl;
            return false;
        }
        return true;
    }


  
    void publishOptimizedTrajectory(const Trajectory<POLY_ORDER, DIMENSIONS> &traj)
    {
        // 1. 重写 Path 消息
        path_msg.header.frame_id = "odom"; // 确保坐标系正确"轨迹时间：%f, 轨迹点：%f, %f, 速度：%f, 角速度：%f, 偏航角：%f", 
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = path_msg.header.stamp;// 第一个点的时间戳
        pose_msg.header.frame_id = path_msg.header.frame_id;

        // 2. 遍历轨迹段
        /*一定要注意,这里轨迹的时间步长一定要和MPC中的dt一致，因为MPC查询参考状态的时候需要dt来快速索引*/
        double t = 0.0;
        double linearVel = 0.0, angularVel = 0.0, yaw = 0.0;
        double total_time = traj.getTotalDuration();
        while (t < total_time) {
            pose_msg.pose.position.x = traj.getPos(t)[0];
            pose_msg.pose.position.y = traj.getPos(t)[1];
            getVelocitiesAtTime(t, traj, linearVel, angularVel, yaw);
            pose_msg.pose.orientation.x = yaw;
            pose_msg.pose.orientation.w = angularVel;
            pose_msg.pose.orientation.z = linearVel;
            path_msg.poses.push_back(pose_msg);
            t += params_.dt;
            rclcpp::Time current_time = pose_msg.header.stamp;  // 转换为 rclcpp::Time
            pose_msg.header.stamp = current_time + rclcpp::Duration::from_seconds(params_.dt);
        }
        path_msg.poses.push_back(goal_pose);  // 最后一个点的时间戳
        
        
        // 5. 发布消息
        RCLCPP_INFO(rclcpp::get_logger("trajectory_publisher"), "optimized traj 时间戳：%f s",path_msg.header.stamp.sec + path_msg.header.stamp.nanosec * 1e-9);
        optimized_traj_pub_->publish(path_msg);

        RCLCPP_INFO(rclcpp::get_logger("trajectory_publisher"), "优化后的轨迹点数： %ld poses", path_msg.poses.size());
    }

    void getVelocitiesAtTime(double t, const Trajectory<POLY_ORDER, DIMENSIONS>& trajectory, double& linearVel, double& angularVel, double& yaw)
    {
        // 获取时间 t 时刻的速度
        Eigen::VectorXd velocity = trajectory.getVel(t);
        
        // 假设是二维轨迹：velocity(0) 是 x 方向速度，velocity(1) 是 y 方向速度
        linearVel = velocity.norm();  // 线速度是速度向量的模长
        
        // 计算角速度（这里假设是平面内运动，角速度通过前进方向的变化来估计）
        // 角速度可以通过轨迹的变化（如前一时刻和当前时刻的速度差）来估算
        static Eigen::VectorXd prevVel(2);
        if (t <= params_.dt){
            prevVel = trajectory.getVel(t);  // 如果是初始时刻，就等于当前时刻的速度
        }
        prevVel = trajectory.getVel(t - params_.dt);  // 前一时刻的速度
        
 
        angularVel = (velocity(0) * prevVel(1) - velocity(1) * prevVel(0)) / (velocity.norm() * prevVel.norm());
        
        

        // 通过速度向量计算偏航角（Yaw）
        yaw = atan2(velocity(1), velocity(0));  // 偏航角由速度向量的方向计算
        
        // 如果需要将yaw转换为角度，可以使用：yaw = yaw * 180.0 / M_PI;
    }
    
    double perpendicularDistance(const geometry_msgs::msg::PoseStamped& point,
                                const geometry_msgs::msg::PoseStamped& lineStart,
                                const geometry_msgs::msg::PoseStamped& lineEnd) {
        double x0 = point.pose.position.x, y0 = point.pose.position.y;
        double x1 = lineStart.pose.position.x, y1 = lineStart.pose.position.y;
        double x2 = lineEnd.pose.position.x, y2 = lineEnd.pose.position.y;

        double num = std::abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1);
        double den = std::sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));
        return (den == 0.0) ? 0.0 : (num / den);
    }

    // 递归实现 Douglas-Peucker 算法
    void rdpRecursive(const std::vector<geometry_msgs::msg::PoseStamped>& points, 
                    double epsilon, 
                    std::vector<geometry_msgs::msg::PoseStamped>& result) {
        if (points.size() < 2) {
            return;
        }

        // 找到距离最大的点
        double maxDist = 0.0;
        size_t index = 0;
        for (size_t i = 1; i < points.size() - 1; ++i) {
            double dist = perpendicularDistance(points[i], points.front(), points.back());
            if (dist > maxDist) {
                maxDist = dist;
                index = i;
            }
        }

        // 如果最大距离大于阈值 epsilon，则保留该点并递归处理两部分路径
        if (maxDist > epsilon) {
            std::vector<geometry_msgs::msg::PoseStamped> leftSubpath(points.begin(), points.begin() + index + 1);
            std::vector<geometry_msgs::msg::PoseStamped> rightSubpath(points.begin() + index, points.end());

            std::vector<geometry_msgs::msg::PoseStamped> leftResult, rightResult;
            rdpRecursive(leftSubpath, epsilon, leftResult);
            rdpRecursive(rightSubpath, epsilon, rightResult);

            // 合并结果（注意去重）
            result.insert(result.end(), leftResult.begin(), leftResult.end() - 1);
            result.insert(result.end(), rightResult.begin(), rightResult.end());
        } else {
            // 直接保留起点和终点
            result.push_back(points.front());
            result.push_back(points.back());
        }
    }

    // 主函数：稀疏化路径
    nav_msgs::msg::Path sparsifyPath(const nav_msgs::msg::Path::SharedPtr msg, double epsilon) {
        nav_msgs::msg::Path sparse_path;
        sparse_path.header = msg->header;

        if (msg->poses.size() < 2) {
            return *msg;
        }

        std::vector<geometry_msgs::msg::PoseStamped> result;
        rdpRecursive(msg->poses, epsilon, result);

        sparse_path.poses = result;
        return sparse_path;
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryOptimizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}