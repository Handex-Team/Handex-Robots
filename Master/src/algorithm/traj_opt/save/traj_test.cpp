
// 2025-03-14 21:48 本代码完成了轨迹优化的测试，成功接收了原始路径，并发布了优化后的路径。
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "gcopter/trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "OsqpEigen/OsqpEigen.h"
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>

#define POLY_ORDER 5    // 五次多项式
#define DIMENSIONS 2    // 二维空间

using namespace std::chrono_literals;
using namespace std::placeholders;

class TrajectoryOptimizer : public rclcpp::Node
{
public:
    TrajectoryOptimizer() : Node("trajectory_optimizer")
    {
        // 初始化参数
        params_.total_time = 5.0;
        params_.max_vel = 1.0; 
        params_.max_acc = 1.0;
        params_.stable_vel = params_.max_vel / 1.2;

        // 订阅原始路径
        raw_traj_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "A_star_planned_path", 10,
            std::bind(&TrajectoryOptimizer::raw_traj_callback, this, _1));

        // 发布优化后的路径
        optimized_traj_pub_ = this->create_publisher<nav_msgs::msg::Path>("optimized_path", 10);
            
        // // 初始化OSQP求解器
        // osqp_solver_ = std::make_unique<OsqpEigen::Solver>();
        // setup_osqp_solver();
        // Test();
        RCLCPP_INFO(this->get_logger(), "轨迹优化节点初始化完成");
    }

private:
    // 优化参数结构体
    struct OptimizationParams {
        double total_time;     // 总时间
        double max_vel;        // 最大线速度 (m/s)
        double max_acc;        // 最大加速度 (m/s²)
        double stable_vel;     // 巡航速度
    } params_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr raw_traj_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimized_traj_pub_;
    
    



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

    Eigen::MatrixXd get_A_expand_matrix(const Trajectory<POLY_ORDER, DIMENSIONS>& traj, std::vector<double> path) {
        
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
        Eigen::VectorXd start_goal_vel_acc_l = Eigen::VectorXd::Zero(4);//起始速度和加速度都为0
        Eigen::VectorXd start_goal_vel_acc_u = Eigen::VectorXd::Zero(4);//起始速度和加速度都为0
        start_goal_vel_acc(0, 1) = 1.0;//起始速度
        start_goal_vel_acc(1, 2) = 1.0;//起点加速度

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
        
        // std::cout << "start_goal_vel_acc:\n" << start_goal_vel_acc << std::endl;


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
        // std::cout << "V_cont:\n" << V_cont << std::endl;

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
        // std::cout << "a_cont:\n" << a_cont << std::endl;

        // 拼接成A
        Eigen::MatrixXd A_total = Eigen::MatrixXd::Zero(
            Head_pos.rows() + Tail_pos.rows() + start_goal_vel_acc.rows() + V_cont.rows() + a_cont.rows(),
            num_segments * matrix_size + 2
            );    
        A_total.block(0, 0, Head_pos.rows(), num_segments * matrix_size + 2) = Head_pos;
        A_total.block(Head_pos.rows(), 0, Tail_pos.rows(), num_segments * matrix_size + 2) = Tail_pos;
        A_total.block(Head_pos.rows() + Tail_pos.rows(), 0, start_goal_vel_acc.rows(), num_segments * matrix_size + 2) = start_goal_vel_acc;
        A_total.block(Head_pos.rows() + Tail_pos.rows() + start_goal_vel_acc.rows(), 0, V_cont.rows(), num_segments * matrix_size + 2) = V_cont;
        A_total.block(Head_pos.rows() + Tail_pos.rows() + start_goal_vel_acc.rows() + V_cont.rows(), 0, a_cont.rows(), num_segments * matrix_size + 2) = a_cont;
        // std::cout << "A_total:\n" << A_total << std::endl;

        return A_total;
    }

    Eigen::VectorXd OSQP_expand(const Eigen::MatrixXd &Q, const Eigen::MatrixXd &A_expand){
        // 创建 OSQP 优化器
        int dim = Q.cols();
        // std::cout << "dim:" << dim << std::endl;
        OsqpEigen::Solver solver;
        solver.settings()->setVerbosity(true);
        solver.settings()->setWarmStart(true);
        // 定义二次项矩阵 P
        Eigen::MatrixXd P = 2 * Q;//标准的接口是1/2XPX,所以要乘2
        Eigen::SparseMatrix<double> P_sparse = P.sparseView();
        // std::cout << "OSQP_P:\n" << P << std::endl;

        // 线性约束矩阵 A
        Eigen::MatrixXd A = A_expand.block(0,0,A_expand.rows(),A_expand.cols()-2);
        Eigen::SparseMatrix<double> A_sparse = A.sparseView();
        // std::cout << "OSQP_A:\n" << A << std::endl;

        // 线性项 q
        Eigen::VectorXd q = Eigen::VectorXd::Zero(dim);

        // // 约束下界 l 和上界 u
        Eigen::VectorXd l = A_expand.col(A_expand.cols()-2);
        Eigen::VectorXd u = A_expand.col(A_expand.cols()-1);
        // std::cout << "OSQP_l:\n" << l << std::endl;
        // std::cout << "OSQP_u:\n" << u << std::endl;

        // 初始化 OSQP 求解器
        solver.data()->setNumberOfVariables(dim);
        solver.data()->setNumberOfConstraints(A.rows());
        solver.data()->setHessianMatrix(P_sparse);
        solver.data()->setGradient(q);
        solver.data()->setLinearConstraintsMatrix(A_sparse);
        solver.data()->setLowerBound(l);
        solver.data()->setUpperBound(u);

        Eigen::VectorXd solution = Eigen::VectorXd::Zero(dim);

        if (!solver.initSolver())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize OSQP solver");
            return solution;
        }

        // 求解
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
        {
            RCLCPP_ERROR(this->get_logger(), "OSQP solver failed to find a solution");
            return solution;
        }

        // 获取解
        solution = solver.getSolution();
        // std::cout << "Solution:\n" << solution.transpose() << std::endl;
        return solution;
    }

    // 路径回调函数
    void raw_traj_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {   
     

        const int num_points = msg->poses.size();
        if (num_points < 2) {
            RCLCPP_WARN(this->get_logger(), "路径点数不足，至少需要2个点");
            return;
        }
        std::vector<double> path_x, path_y;
        for (int i = 0; i < num_points; ++i) {
            path_x.push_back(msg->poses[i].pose.position.x);
            path_y.push_back(msg->poses[i].pose.position.y);
        }
        

        /* 步骤1：路径预处理 */
        // 计算段数和时间分配
        const int num_segments = num_points - 1;
        RCLCPP_INFO(this->get_logger(), "路径共%d段", num_segments);
        std::vector<double> segment_lengths(num_segments);
        std::vector<double> durs(num_segments);
        for (int i = 0; i < num_segments; ++i) {
            const auto& p1 = msg->poses[i].pose.position;
            const auto& p2 = msg->poses[i+1].pose.position;
            
            segment_lengths[i] = std::hypot(p2.x - p1.x, 
                                           p2.y - p1.y);
            
            durs[i] = segment_lengths[i] / params_.stable_vel;
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
        Eigen::MatrixXd A_expand_x = get_A_expand_matrix(traj, path_x);
        Eigen::MatrixXd A_expand_y = get_A_expand_matrix(traj, path_y);

        auto solution_x = OSQP_expand(Q, A_expand_x);
        auto solution_y = OSQP_expand(Q, A_expand_y);

        // 生成优化后的轨迹
        std::vector<Eigen::Matrix<double, DIMENSIONS, POLY_ORDER + 1>> opt_coeffs(durs.size());
        for (int i = 0; i < num_segments; i++){
            opt_coeffs[i].row(0) = solution_x.segment(i * (POLY_ORDER + 1), POLY_ORDER + 1).reverse();
            opt_coeffs[i].row(1) = solution_y.segment(i * (POLY_ORDER + 1), POLY_ORDER + 1).reverse();
        }
        Trajectory<POLY_ORDER, DIMENSIONS> opt_traj(durs, opt_coeffs);
        // opt_traj.vis_x_y();
        // std::cout<< opt_traj.getPos(0.0)<<std::endl;
        publishOptimizedTrajectory(opt_traj);
       
    }


    void Test()
    {   
        std::vector<double> test_x={0.0, 10.0, 20.0, -10.0};// 路径x坐标
        std::vector<double> test_y={0.0, 20.0, 10.0, -30.0};// 路径y坐标
        nav_msgs::msg::Path::SharedPtr msg = std::make_shared<nav_msgs::msg::Path>();
        for (int i = 0; i < test_x.size(); ++i) {
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.pose.position.x = test_x[i];
            pose_msg.pose.position.y = test_y[i];
            msg->poses.push_back(pose_msg);
        }

        const int num_points = msg->poses.size();
        if (num_points < 2) {
            RCLCPP_WARN(this->get_logger(), "路径点数不足，至少需要2个点");
            return;
        }
        std::vector<double> path_x, path_y;
        for (int i = 0; i < num_points; ++i) {
            path_x.push_back(msg->poses[i].pose.position.x);
            path_y.push_back(msg->poses[i].pose.position.y);
        }
        
        std::cout << "path_x: ";
        for (const double& val : path_x) {
            std::cout << val << " ";
        }
        std::cout << std::endl;

        std::cout << "path_y: ";
        for (const double& val : path_y) {
            std::cout << val << " ";
        }
        std::cout << std::endl;

        /* 步骤1：路径预处理 */
        // 计算段数和时间分配
        const int num_segments = num_points - 1;
        RCLCPP_INFO(this->get_logger(), "路径共%d段", num_segments);
        std::vector<double> segment_lengths(num_segments);
        std::vector<double> durs(num_segments);
        for (int i = 0; i < num_segments; ++i) {
            const auto& p1 = msg->poses[i].pose.position;
            const auto& p2 = msg->poses[i+1].pose.position;
            
            segment_lengths[i] = std::hypot(p2.x - p1.x, 
                                           p2.y - p1.y);
            
            durs[i] = segment_lengths[i] / params_.stable_vel;
        }
        durs={0.9, 5.0, 3.0};
        // 打印轨迹长度
        std::cout << "segment_lengths: ";
        for (const double& val : segment_lengths) {
            std::cout << val << " ";
        }
        std::cout << std::endl;

        // 打印轨迹长度
        std::cout << "durs: ";
        for (const double& val : durs) {
            std::cout << val << " ";
        }
        std::cout << std::endl;

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
        Eigen::MatrixXd A_expand_x = get_A_expand_matrix(traj, path_x);
        Eigen::MatrixXd A_expand_y = get_A_expand_matrix(traj, path_y);

        auto solution_x = OSQP_expand(Q, A_expand_x);
        auto solution_y = OSQP_expand(Q, A_expand_y);

        // 生成优化后的轨迹
        std::vector<Eigen::Matrix<double, DIMENSIONS, POLY_ORDER + 1>> opt_coeffs(durs.size());
        for (int i = 0; i < num_segments; i++){
            opt_coeffs[i].row(0) = solution_x.segment(i * (POLY_ORDER + 1), POLY_ORDER + 1).reverse();
            opt_coeffs[i].row(1) = solution_y.segment(i * (POLY_ORDER + 1), POLY_ORDER + 1).reverse();
        }

        // 把durs改一下试试
        durs={0.9, 5.0, 3.0};
        Trajectory<POLY_ORDER, DIMENSIONS> opt_traj(durs, opt_coeffs);
        std::cout<< opt_traj.getDurations()<<std::endl;
        opt_traj.vis_x_y();
        opt_traj.vis_x_t();
        // 轨迹点测试
        std::cout<< "轨迹点测试"<<std::endl;
        std::cout<< opt_traj.getPos(0.0)<<std::endl;
        std::cout<< opt_traj.getPos(durs[0])<<std::endl;
        std::cout<< opt_traj.getPos(durs[0]+0.1)<<std::endl;
        std::cout<< opt_traj.getPos(40.0)<<std::endl;

        publishOptimizedTrajectory(opt_traj);
       
    }

    void publishOptimizedTrajectory(const Trajectory<POLY_ORDER, DIMENSIONS> &traj)
    {
        // 1. 创建 Path 消息
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = rclcpp::Clock().now();
        path_msg.header.frame_id = "map"; // 确保坐标系正确

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = path_msg.header.stamp;
        pose_msg.header.frame_id = path_msg.header.frame_id;

        // 2. 遍历轨迹段
        double t = 0.0, t_step = 0.01;
        while (t < traj.getTotalDuration()) {
            pose_msg.pose.position.x = traj.getPos(t)[0];
            pose_msg.pose.position.y = traj.getPos(t)[1];
            path_msg.poses.push_back(pose_msg);
            t += t_step;
        }

        
        
        // 5. 发布消息
        optimized_traj_pub_->publish(path_msg);

        RCLCPP_INFO(rclcpp::get_logger("trajectory_publisher"), "Published optimized path with %ld poses", path_msg.poses.size());
    }


    
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryOptimizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}