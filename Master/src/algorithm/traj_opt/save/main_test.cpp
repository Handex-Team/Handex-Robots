
// 2025-3-14 21：00 本代码完成了轨迹优化的测试，并绘制了轨迹。
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "OsqpEigen/OsqpEigen.h"
#include "rclcpp/rclcpp.hpp"
#include "gcopter/trajectory.hpp"
#include "matplotlibcpp.h"
#include <numeric>


#define POLY_ORDER 5    // 五次多项式
#define DIMENSIONS 1    // 三维空间(x,y,z)
using namespace std;
namespace plt = matplotlibcpp;

class OSQP_TEST : public rclcpp::Node
{
public:
    OSQP_TEST() : Node("osqp_test")
    {
        RCLCPP_INFO(this->get_logger(), "OSQP_TEST node has been created");
        // OSQP_TEST1();
        // OSQP_TEST2();
        OSQP_TEST3();
       
    }

private:
    // 两端轨迹，x=0到x=1再到x=2
    std::vector<double> path_x={0.0, 10.0, 20.0, -10.0};// 路径x坐标
    std::vector<double> path_y={0.0, 20.0, 10.0, -30.0};// 路径y坐标
    std::vector<double> durs={0.9, 5.0, 3.0};// 轨迹持续时间
    
    Eigen::VectorXd solution_ = Eigen::VectorXd::Zero(POLY_ORDER+1); // 解向量
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


    // 计算A的增广矩阵A l u，这里实际上是要把等式和不等式约束全都包含进来，如果是等式的话就是l<=Ax<=l，如果是不等式的话就是l<=Gx<=u
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
        std::cout << "Head_pos:\n" << Head_pos << std::endl;
        
        



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
        std::cout << "Tail_pos:\n" << Tail_pos << std::endl;

        
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
        
        std::cout << "start_goal_vel_acc:\n" << start_goal_vel_acc << std::endl;


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
        std::cout << "V_cont:\n" << V_cont << std::endl;

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
        std::cout << "a_cont:\n" << a_cont << std::endl;

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
        std::cout << "A_total:\n" << A_total << std::endl;

        return A_total;
    }
    

    /*
    *@brief 实现OSQP的流程，优化一维轨迹
    *@param Q 多项式轨迹的二次项矩阵
    *@param A_expand 增广矩阵A | l | u
    *@return 多项式轨迹的最优系数向量
    */
    Eigen::VectorXd OSQP_expand(const Eigen::MatrixXd &Q, const Eigen::MatrixXd &A_expand){
        // 创建 OSQP 优化器
        int dim = Q.cols();
        std::cout << "dim:" << dim << std::endl;
        OsqpEigen::Solver solver;
        solver.settings()->setVerbosity(true);
        solver.settings()->setWarmStart(true);
        // 定义二次项矩阵 P
        Eigen::MatrixXd P = 2 * Q;//标准的接口是1/2XPX,所以要乘2
        Eigen::SparseMatrix<double> P_sparse = P.sparseView();
        std::cout << "OSQP_P:\n" << P << std::endl;

        // 线性约束矩阵 A
        Eigen::MatrixXd A = A_expand.block(0,0,A_expand.rows(),A_expand.cols()-2);
        Eigen::SparseMatrix<double> A_sparse = A.sparseView();
        std::cout << "OSQP_A:\n" << A << std::endl;

        // 线性项 q
        Eigen::VectorXd q = Eigen::VectorXd::Zero(dim);

        // // 约束下界 l 和上界 u
        Eigen::VectorXd l = A_expand.col(A_expand.cols()-2);
        Eigen::VectorXd u = A_expand.col(A_expand.cols()-1);
        std::cout << "OSQP_l:\n" << l << std::endl;
        std::cout << "OSQP_u:\n" << u << std::endl;

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
        std::cout << "Solution:\n" << solution.transpose() << std::endl;
        return solution;
    }
    
    void OSQP_TEST1(){
         // 变量维度
        const int dim = 100;

        // 定义 OSQP 求解器
        OsqpEigen::Solver solver;
        solver.settings()->setVerbosity(true);
        solver.settings()->setWarmStart(true);

        // 定义二次项矩阵 P
        Eigen::MatrixXd P = Eigen::MatrixXd::Identity(dim, dim);
        Eigen::SparseMatrix<double> P_sparse = P.sparseView();

        // 线性约束矩阵 A
        Eigen::MatrixXd A = Eigen::MatrixXd::Random(dim, dim);
        Eigen::SparseMatrix<double> A_sparse = A.sparseView();

        // 线性项 q
        Eigen::VectorXd q = Eigen::VectorXd::Ones(dim);

        // 约束下界 l 和上界 u
        Eigen::VectorXd l = Eigen::VectorXd::Constant(dim, -1.0);
        Eigen::VectorXd u = Eigen::VectorXd::Constant(dim, 1.0);

        // 初始化 OSQP 求解器
        solver.data()->setNumberOfVariables(dim);
        solver.data()->setNumberOfConstraints(dim);
        solver.data()->setHessianMatrix(P_sparse);
        solver.data()->setGradient(q);
        solver.data()->setLinearConstraintsMatrix(A_sparse);
        solver.data()->setLowerBound(l);
        solver.data()->setUpperBound(u);

        if (!solver.initSolver())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize OSQP solver");
            return;
        }

        // 求解
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
        {
            RCLCPP_ERROR(this->get_logger(), "OSQP solver failed to find a solution");
            return;
        }

        // 获取解
        Eigen::VectorXd solution = solver.getSolution();
        std::cout << "Solution:\n" << solution.transpose() << std::endl;
    }

    // 测试一维x-t轨迹
    void OSQP_TEST2(){
        
        
        std::vector<Eigen::Matrix<double, DIMENSIONS, POLY_ORDER + 1>> coeffs;
        for (auto i = durs.begin(); i != durs.end(); ++i){
            Eigen::Matrix<double, DIMENSIONS, POLY_ORDER + 1> coeff;
            coeff.setRandom();
            coeffs.push_back(coeff);
        }
        Trajectory<POLY_ORDER, DIMENSIONS> traj(durs, coeffs);
        Eigen::MatrixXd Q = get_jerk_quadratic_matrix(traj);
        Eigen::MatrixXd A_expand = get_A_expand_matrix(traj, path_x);
        // std::cout << "Q:\n" << Q << std::endl;


        // 创建 OSQP 优化器
        int dim = Q.cols();
        std::cout << "dim:" << dim << std::endl;
        OsqpEigen::Solver solver;
        solver.settings()->setVerbosity(true);
        solver.settings()->setWarmStart(true);
        // 定义二次项矩阵 P
        Eigen::MatrixXd P = 2 * Q;//标准的接口是1/2XPX,所以要乘2
        Eigen::SparseMatrix<double> P_sparse = P.sparseView();
        std::cout << "OSQP_P:\n" << P << std::endl;

        // 线性约束矩阵 A
        Eigen::MatrixXd A = A_expand.block(0,0,A_expand.rows(),A_expand.cols()-2);
        Eigen::SparseMatrix<double> A_sparse = A.sparseView();
        std::cout << "OSQP_A:\n" << A << std::endl;

        // 线性项 q
        Eigen::VectorXd q = Eigen::VectorXd::Zero(dim);

        // // 约束下界 l 和上界 u
        Eigen::VectorXd l = A_expand.col(A_expand.cols()-2);
        Eigen::VectorXd u = A_expand.col(A_expand.cols()-1);
        std::cout << "OSQP_l:\n" << l << std::endl;
        std::cout << "OSQP_u:\n" << u << std::endl;

        // 初始化 OSQP 求解器
        solver.data()->setNumberOfVariables(dim);
        solver.data()->setNumberOfConstraints(A.rows());
        solver.data()->setHessianMatrix(P_sparse);
        solver.data()->setGradient(q);
        solver.data()->setLinearConstraintsMatrix(A_sparse);
        solver.data()->setLowerBound(l);
        solver.data()->setUpperBound(u);

        if (!solver.initSolver())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize OSQP solver");
            return;
        }

        // 求解
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
        {
            RCLCPP_ERROR(this->get_logger(), "OSQP solver failed to find a solution");
            return;
        }

        // 获取解
        solution_ = solver.getSolution();
        std::cout << "Solution:\n" << solution_.transpose() << std::endl;

        
        // 可视化

        
        matplot_vis_t(solution_, durs);
    }

    
    // 测试二维x-y轨迹
    void OSQP_TEST3(){
        std::vector<Eigen::Matrix<double, DIMENSIONS, POLY_ORDER + 1>> coeffs;
        for (auto i = durs.begin(); i != durs.end(); ++i){
            Eigen::Matrix<double, DIMENSIONS, POLY_ORDER + 1> coeff;
            coeff.setRandom();
            coeffs.push_back(coeff);
        }
        Trajectory<POLY_ORDER, DIMENSIONS> traj(durs, coeffs);
        Eigen::MatrixXd Q = get_jerk_quadratic_matrix(traj);
        Eigen::MatrixXd A_expand_x = get_A_expand_matrix(traj, path_x);
        Eigen::MatrixXd A_expand_y = get_A_expand_matrix(traj, path_y);
        
        auto solution_x = OSQP_expand(Q, A_expand_x);
        auto solution_y = OSQP_expand(Q, A_expand_y);
        std::cout << "Solution_x:\n" << solution_x.transpose() << std::endl;
        std::cout << "Solution_y:\n" << solution_y.transpose() << std::endl;
    
        auto t = Eigen::VectorXd::LinSpaced(100, 0.0, std::accumulate(durs.begin(), durs.end(), 0.0));
        // 生成优化后的轨迹
        std::vector<Eigen::Matrix<double, 2, POLY_ORDER + 1>> opt_coeffs(durs.size());
        for (int i = 0; i < durs.size(); i++){
            opt_coeffs[i].row(0) = solution_x.segment(i * (POLY_ORDER + 1), POLY_ORDER + 1).reverse();
            opt_coeffs[i].row(1) = solution_y.segment(i * (POLY_ORDER + 1), POLY_ORDER + 1).reverse();
        }
        Trajectory<POLY_ORDER, 2> optimized_traj(durs, opt_coeffs);
        std::cout << "Optimized_traj:\n" << optimized_traj.getPos(5.9)[0] << std::endl;
        // optimized_traj.vis_x_t();
        // 目前的问题是  算出的系数没问题，但是用这个系数构造的轨迹  调用API时出了些问题*********最后找到问题是系数输入的时候要降序排列

        // 可视化
        optimized_traj.vis_x_y();
        
    }

    // 一维轨迹段的可视化
void matplot_vis_t(const Eigen::VectorXd &solution_, const std::vector<double> &durs) {
    std::vector<double> time_values; // 存储所有时间点
    std::vector<double> x_values;    // 存储所有轨迹点

    int poly_order = solution_.size() /durs.size() - 1;  // 假设是五次多项式
    std::cout << "poly_order: " << poly_order << std::endl;
    double t_start = 0.0; // 轨迹起始时间

    // 遍历所有轨迹段
    for (size_t i = 0; i < durs.size(); ++i) {
        double t_end = t_start + durs[i];  // 计算当前轨迹段的结束时间

        // 取出当前轨迹段的多项式系数（solution_ 里每 6 个系数为一段）
        Eigen::VectorXd coeffs = solution_.segment(i * (poly_order + 1), poly_order + 1);

        // 生成当前轨迹段的时间点
        Eigen::VectorXd t_segment = Eigen::VectorXd::LinSpaced(50, t_start, t_end);

        // 计算当前轨迹段的 x 值
        for (int j = 0; j < t_segment.size(); ++j) {
            double t_rel = t_segment(j) - t_start;  // 计算相对时间
            double x = 0.0;

            // 计算多项式值
            for (int k = 0; k < coeffs.size(); ++k) {
                x += coeffs(k) * std::pow(t_rel, k);
            }

            time_values.push_back(t_segment(j));
            x_values.push_back(x);
        }

        t_start = t_end; // 更新起始时间
    }

    // 绘制轨迹
    plt::plot(time_values, x_values, "r-");
    plt::xlabel("Time (t)");
    plt::ylabel("Position (x)");
    plt::title("1D Trajectory Visualization");
    plt::grid(true);
    plt::show();
}


    /**
     * @brief 可视化二维轨迹（X-Y 轨迹图）。
     * 
     * @param solution_x X 维度的多项式系数（从低次到高次）
     * @param solution_y Y 维度的多项式系数（从低次到高次）
     * @param t 时间向量，对应轨迹的时间采样点
     */
    void matplot_vis_xy(const Eigen::VectorXd &solution_x, 
                    const Eigen::VectorXd &solution_y, 
                    const Eigen::VectorXd &t) {
        
        std::vector<double> time_values; // 存储所有时间点
        std::vector<double> x_values;    // 存储所有轨迹点 x
        std::vector<double> y_values;    // 存储所有轨迹点 y

        int poly_order = solution_x.size() / durs.size() - 1;  // 假设是五次多项式
        std::cout << "poly_order: " << poly_order << std::endl;

        double t_start = 0.0; // 轨迹起始时间

        // 遍历所有轨迹段
        for (size_t i = 0; i < durs.size(); ++i) {
        double t_end = t_start + durs[i];  // 计算当前轨迹段的结束时间

        // 取出当前轨迹段的多项式系数
        Eigen::VectorXd coeffs_x = solution_x.segment(i * (poly_order + 1), poly_order + 1);
        Eigen::VectorXd coeffs_y = solution_y.segment(i * (poly_order + 1), poly_order + 1);

        // 生成当前轨迹段的时间点
        Eigen::VectorXd t_segment = Eigen::VectorXd::LinSpaced(50, t_start, t_end);

        // 计算当前轨迹段的 x 和 y 值
        for (int j = 0; j < t_segment.size(); ++j) {
            double t_rel = t_segment(j) - t_start;  // 计算相对时间
            double x = 0.0;
            double y = 0.0;

            // 计算 x 和 y 的多项式值
            for (int k = 0; k < coeffs_x.size(); ++k) {
                x += coeffs_x(k) * std::pow(t_rel, k);
            }
            for (int k = 0; k < coeffs_y.size(); ++k) {
                y += coeffs_y(k) * std::pow(t_rel, k);
            }

            time_values.push_back(t_segment(j));
            x_values.push_back(x);
            y_values.push_back(y);
        }

        t_start = t_end; // 更新起始时间
        }

        // 绘制轨迹
        plt::plot(x_values, y_values, "r-");
        plt::xlabel("Position (x)");
        plt::ylabel("Position (y)");
        plt::title("2D Trajectory Visualization");
        plt::grid(true);
        plt::show();
       
    }

    


};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OSQP_TEST>());
    rclcpp::shutdown();
    return 0;
}
