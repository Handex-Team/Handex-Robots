#include "gcopter/lbfgs.hpp" // 替换为你的路径

#include <iostream>
#include <cmath>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/SparseLU>

#include "traj_opt/mars_traj.hpp"
#include "traj_opt/mars_traj_opt.hpp"

using namespace Eigen;

// 高维 Rosenbrock 函数及其梯度
double rosenbrock_evaluate(void* instance,
                           const VectorXd& x,
                           VectorXd& grad)
{
    const int n = x.size();
    double fx = 0.0;

    grad.setZero(n);

    for (int i = 0; i < n - 1; ++i)
    {
        double xi = x[i];
        double xi1 = x[i + 1];
        double t1 = xi1 - xi * xi;
        double t2 = 1.0 - xi;

        fx += 100.0 * t1 * t1 + t2 * t2;

        grad[i] += -400.0 * t1 * xi - 2.0 * t2;
        grad[i + 1] += 200.0 * t1;
    }

    return fx;
}

// 可选：优化进度监控
int print_progress(void* instance,
                   const VectorXd& x,
                   const VectorXd& g,
                   const double fx,
                   const double step,
                   const int k,
                   const int ls)
{
    printf("[L-BFGS] Iter %3d: f(x) = %.6f, ||g|| = %.4f, step = %.4e\n", 
           k, fx, g.norm(), step);
    return 0;
}

void test_lbfgs(){
    const int dim = 100; // 🚀 高维维度设置（如100维）

    VectorXd x(dim);
    for (int i = 0; i < dim; ++i)
        x[i] = (i % 2 == 0) ? -1.2 : 1.0;  // 初始值交替设置

    lbfgs::lbfgs_parameter_t param;
    param.g_epsilon = 1e-5;
    param.delta = 1e-6;
    param.max_iterations = 1000;
    param.mem_size = 20;

    double fx;
    auto start_time = std::chrono::high_resolution_clock::now();
    int ret = lbfgs::lbfgs_optimize(
        x,
        fx,
        rosenbrock_evaluate,
        nullptr,
        print_progress,  // 你也可以设为 nullptr
        nullptr,
        param
    );
    auto end_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

    std::cout << "\n===== Optimization Result =====" << std::endl;
    std::cout << "Status: " << lbfgs::lbfgs_strerror(ret) << std::endl;
    std::cout << "Optimal f(x): " << fx << std::endl;
    std::cout << "Elapsed time: " << elapsed_time << " microseconds" << std::endl;
    std::cout << "||x - 1|| = " << (x - VectorXd::Ones(dim)).norm() << std::endl;
}

void banded_system(){
    constexpr int Dim = 3;
    uneven_planner::MinJerkOpt<Dim> opt;

    int N = 5; // 200 个中间点 → 201 段轨迹
    opt.reset(N);

    // 每段时间：均匀设置为 1.0（或可按需修改）
    Eigen::VectorXd ts = Eigen::VectorXd::Ones(N);

    // 起点状态：位置、速度、加速度
    Eigen::MatrixXd headPVA(Dim, 3);
    headPVA << 0, 0, 0,   // x
            0, 0, 0,   // y
            0, 0, 0;   // z

    // 终点状态：位置、速度、加速度
    Eigen::MatrixXd tailPVA(Dim, 3);
    tailPVA << 0, 0, 0,
            0, 0, 0,
            0, 0, 0;

    // 中间点：N 个，线性插值
    Eigen::MatrixXd inPs(Dim, N - 1);
    for (int i = 0; i < N - 1; ++i) {
        double ratio = static_cast<double>(i + 1) / N;
        inPs(0, i) = ratio;
        inPs(1, i) = ratio;
        inPs(2, i) = ratio;
    }
    std::cout << "中间点：" << inPs << std::endl;

    // 调用 generate 函数
    auto start_time = std::chrono::high_resolution_clock::now();
    opt.generate(inPs, ts, headPVA, tailPVA);
    

    // 获取轨迹系数
    Eigen::MatrixXd c = opt.getCoeffs();
    

    // 获取轨迹 jerk 代价
    double jerkCost = opt.getTrajJerkCost();
    
    Eigen::MatrixXd gdC;
    Eigen::VectorXd gdT;
    
    opt.calJerkGradCT(gdC, gdT);
    

    Eigen::MatrixXd gdP;
    opt.calGradCTtoQT(gdC, gdT, gdP);
     
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    std::cout << "生成轨迹耗时：" << elapsed_time << " microseconds" << std::endl;

    std::cout << "轨迹 jerk 导数矩阵 gdC:" << std::endl << gdC << std::endl;
    std::cout << "轨迹 jerk 导数向量 gdT:" << std::endl << gdT << std::endl;
    std::cout << "轨迹 jerk cost: " << jerkCost << std::endl;
    std::cout << "多项式系数矩阵 c:" << std::endl << c << std::endl;
    std::cout << "轨迹 jerk 导数矩阵 gdP:" << std::endl << gdP << std::endl;   

    

}


void traj_opt_test(){
    auto node = rclcpp::Node::make_shared("traj_opt_test");
    uneven_fast_planner::MarsTrajOpt opt(node);

}

int main()
{
    // test_lbfgs();

    banded_system();

    // traj_opt_test();
    

    return 0;
}
