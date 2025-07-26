#include "mpc_pkg/mpc_diff_car.hpp"

namespace Mpc_diff_car {
    MPC_Controller::MPC_Controller(const MPCState &s, const nav_msgs::msg::Path &path){
        
        dt = 0.1; // 时间步长
        v_max = 1.0; // 最大速度
        a_max = 0.3; // 最大加速度
        w_max = 1.0; // 最大角速度
        j_max = 0.8; // 最大角加速度
        s_now = s;
        N = 15; // 预测步数
        path_= std::make_shared<nav_msgs::msg::Path>(path);
        rclcpp::Clock clock(RCL_SYSTEM_TIME);
        now_time = clock.now(); // 当前时间

    }
    void MPC_Controller::init_controller(){
        Q = Eigen::MatrixXd::Identity(3 * N, 3 * N);
        R = Eigen::MatrixXd::Identity(2 * N, 2 * N);
        s_eigen = Eigen::Vector3d::Zero(); // 状态向量
        s_ref_eigen = Eigen::Vector3d::Zero(); // 参考状态向量
        X_k = Eigen::VectorXd::Zero(3); // 当前状态向量
        X_r = Eigen::VectorXd::Zero(3 * N); // 参考控制向量
        std::cout << "MPC控制器 初始化成功" << std::endl;
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);
        H = Eigen::MatrixXd::Zero(2 * N, 2 * N);
        q = Eigen::VectorXd::Zero(2 * N, 1);
        A = Eigen::MatrixXd::Zero(4 * N, 2 * N);
        l = Eigen::VectorXd::Zero(4 * N, 1);
        u = Eigen::VectorXd::Zero(4 * N, 1);
    }
    // 根据当前状态展开，计算线性化矩阵，因为后续要逐步线性化，所以这里的s并不是指的当前机器人的实际s，而是当前要线性化的s
    void MPC_Controller::get_Ak_Bk_Ok(const MPCState &s){
        Ak = Eigen::MatrixXd::Zero(3, 3);
        Bk = Eigen::MatrixXd::Zero(3, 2);
        Ok = Eigen::MatrixXd::Zero(3, 1);

        Eigen::Vector3d v = Eigen::Vector3d::Zero();
        v(0) = s.x;
        v(1) = s.y;
        v(2) = s.theta;

        Ak(0, 2) = -s.v * sin(s.theta);
        Ak(1, 2) = s.v * cos(s.theta);

        Bk(0, 0) = cos(s.theta);
        Bk(1, 0) = sin(s.theta);
        Bk(2, 1) = 1.0;

        Ok = - Ak * v;

        // 计算A_hat,B_hat,O_hat:
        A_hat = Eigen::MatrixXd::Zero(3, 3);
        B_hat = Eigen::MatrixXd::Zero(3, 2);
        O_hat = Eigen::MatrixXd::Zero(3, 1);

        A_hat = Eigen::MatrixXd::Identity(Ak.rows(), Ak.cols())+dt*Ak;
        // std::cout << "A_hat:" << std::endl << A_hat << std::endl;
        B_hat = dt*Bk;
        // std::cout << "B_hat:" << std::endl << B_hat << std::endl;
        O_hat = dt*Ok;
        // std::cout << "O_hat:" << std::endl << O_hat << std::endl;

    }

    int MPC_Controller::find_closest_trajectory_point() {
        // // 获取路径的起始时间
        auto path_start_time = rclcpp::Time(path_->header.stamp.sec, path_->header.stamp.nanosec);
        std::cout << "路径起始时间：" << (path_start_time.nanoseconds() - 17434214 * 1e11)/1e9 << "s" << std::endl;
        // 找到离当前时间戳最近的轨迹点对应的索引
        // rclcpp::Clock clock(RCL_SYSTEM_TIME);
        // rclcpp::Time now_time = clock.now(); // 当前时间
        std::cout << "当前时间戳：" << (now_time.nanoseconds() - 17434214 * 1e11)/1e9 << "s" << std::endl;
        std::cout << "当前时间差：" << (now_time - path_start_time).nanoseconds()/1e9 << "s" << std::endl;
        int false_index = -1; // 初始化最近轨迹点索引
        rclcpp::Duration min_diff = rclcpp::Duration::from_seconds(10000); // 初始化最小时间差为极大值

        for (int i = 0; i < path_->poses.size(); i++) {
            // 计算当前轨迹点时间戳与当前时间的时间差
            // rclcpp::Time path_time = path_->poses[i].header.stamp;
            rclcpp::Time path_time(path_->poses[i].header.stamp.sec, path_->poses[i].header.stamp.nanosec, RCL_SYSTEM_TIME);
            
            rclcpp::Duration time_diff = now_time - path_time;
            // std::cout << "当前轨迹点时间戳：" << path_time.nanoseconds() << std::endl;
            // std::cout << "当前时间戳：" << now_time.nanoseconds() << std::endl;
            // std::cout << "时间差：" << time_diff.nanoseconds() << std::endl;

            if(time_diff.nanoseconds() < 0){
                std::cout << "首次出现晚于当前时间戳的轨迹点，认为上一次时间点是最近的起点，index："<< i - 1 << std::endl;
                return i - 1;
            }
         
        }

        // 返回最近轨迹点的索引
        std::cout << "没有找到合适的轨迹点，返回false_index" << std::endl;
        return false_index;






    }



    // 这个函数是最复杂的一部分,是逐步线性化的过程
    void MPC_Controller::get_Abar_Bbar_Cbar_O(){
        A_bar = Eigen::MatrixXd::Zero(3 * N, 3);
        B_bar = Eigen::MatrixXd::Zero(3 * N, 2 * N);
        C_bar = Eigen::MatrixXd::Zero(3 * N, 3 * N);
        O = Eigen::MatrixXd::Zero(3 * N, 1);

        // 将预测N步的hat矩阵都存起来
        std::vector<Eigen::MatrixXd> A_hat_total;
        std::vector<Eigen::MatrixXd> B_hat_total;
        std::vector<Eigen::MatrixXd> O_hat_total;
        MPCState s_update = s_now;

        // 找到离当前时间戳最近的轨迹点对应的索引
        int closest_time_index = find_closest_trajectory_point();
        // 测试找到的索引和当前位置的误差
        std::cout << "当前位置索引：" << closest_time_index << std::endl;
        std::cout << "当前真实x,y,theta,v,w:"<< 
            s_now.x << " " << s_now.y << " " << s_now.theta << " " << s_now.v << " " << s_now.w << std::endl;
        std::cout << "索引的轨迹点的x,y,theta,v,w:"<<
            path_->poses[closest_time_index].pose.position.x << " " 
            << path_->poses[closest_time_index].pose.position.y << " " 
            << path_->poses[closest_time_index].pose.orientation.x << " " 
            << path_->poses[closest_time_index].pose.orientation.z << " " 
            << path_->poses[closest_time_index].pose.orientation.w << std::endl;
        geometry_msgs::msg::PoseStamped ref_pose;
        for (int i = 0; i < N; i++) {
            get_Ak_Bk_Ok(s_update);//这里会改变几个hat矩阵
            A_hat_total.push_back(A_hat);
            B_hat_total.push_back(B_hat);
            O_hat_total.push_back(O_hat);

            // std::cout << "第" << i << "步的A_hat:" << std::endl << A_hat << std::endl;
            // std::cout << "第" << i << "步的B_hat:" << std::endl << B_hat << std::endl;
            // std::cout << "第" << i << "步的O_hat:" << std::endl << O_hat << std::endl;
            // 找到离当前时间戳最近的轨迹点对应的索引
            if (closest_time_index + i + 1 >= path_->poses.size()) {
                ref_pose = path_->poses[path_->poses.size() - 1];
            }
            else ref_pose = path_->poses[closest_time_index + i + 1];
            
            // 后续最好检查一下这里的时间是否同步
           
            // 状态更新
            s_update.x += s_update.v * cos(s_update.theta) * dt; // 更新 x 坐标，这里我直接用的当前输入去更新状态，实际用起来，按道理说直接用轨迹点的状态问题也不大
            s_update.y += s_update.v * sin(s_update.theta) * dt; // 更新 y 坐标

            // 注意，这里是我自己定义的信息，orientation.x是欧拉角，orientation.w是角速度,orientation.z是线速度
            s_update.theta += s_update.w * dt; // 更新角度
            s_update.w = ref_pose.pose.orientation.w; // 更新角速度
            s_update.v = ref_pose.pose.orientation.z; // 更新线速度

            // std::cout << "第" << i << "步的s_update.x:" << std::endl << s_update.x << std::endl;
            // std::cout << "第" << i << "步的s_update.y:" << std::endl << s_update.y << std::endl;
            // std::cout << "第" << i << "步的s_update.theta:" << std::endl << s_update.theta << std::endl;
            // std::cout << "第" << i << "步的s_update.v:" << std::endl << s_update.v << std::endl;
            // std::cout << "第" << i << "步的s_update.w:" << std::endl << s_update.w << std::endl;

        }


        // 计算A_bar
        for (int i = 0; i < N; i++) {
            if (i == 0){
                A_bar.block(3 * i, 0, 3, 3) = A_hat_total[0];
            }
            else{
                A_bar.block(3 * i, 0, 3, 3) = A_bar.block(3 * (i - 1), 0, 3, 3) * A_hat_total[i];
            }
        }
        // std::cout << "A_bar:" << std::endl << A_bar << std::endl;


        // 计算B_bar，B_bar是一个下对角矩阵
        // 按列填充
        for (int i = 0; i < N ; i++) {
            for (int j = i; j < N; j++) {
                if (j == i){
                    B_bar.block(3 * j, 2 * i, 3, 2) = B_hat_total[i];
                }
                else{
                    B_bar.block(3 * j, 2 * i, 3, 2) = A_hat_total[j] * B_bar.block(3 * (j - 1), 2 * i, 3, 2);
                }
            }
        }
        // std::cout << "B_bar:" << std::endl << B_bar << std::endl;
        // 检查过，应该是对的

        // 计算C_bar
        // 按列填充
        for (int i = 0; i < N; i++) {
            for (int j = i; j < N; j++) {
                if (j == i) {
                    C_bar.block(3 * j, 3 * i, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
                    continue;
                }
                else{
                    C_bar.block(3 * j, 3 * i, 3, 3) = A_hat_total[j] * C_bar.block(3 * (j - 1), 3 * i, 3, 3);
                }
            }
        }
        // std::cout << "C_bar:" << std::endl << C_bar << std::endl;

        // 计算O_bar
        for (int i = 0; i < N; i++){
            O.block(3 * i, 0, 3, 1) = O_hat_total[i];
        }
        // std::cout << "O_bar:" << std::endl << O << std::endl;

        

    }


    void MPC_Controller::get_X_r_X_k(){
        int next_time_index = find_closest_trajectory_point() + 40; // 如果不加一，则返回的状态是当前状态，而不是下一状态
        if (next_time_index >= path_->poses.size() - 1) next_time_index = path_->poses.size() - 1; // 防止溢出
        auto target_pose = path_->poses[next_time_index];
        // std::cout << "Debug3"<< std::endl;
        
        
        s_eigen << s_now.x, s_now.y, s_now.theta;
        s_ref_eigen << target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.orientation.x;
        // std::cout << "Debug4"<< std::endl;
        X_k = s_eigen;
        for (int i = 0; i < N; i++) {
            s_ref_eigen(0) = target_pose.pose.position.x;
            s_ref_eigen(1) = target_pose.pose.position.y;
            s_ref_eigen(2) = target_pose.pose.orientation.x;
            X_r.block(3 * i, 0, 3, 1) = s_ref_eigen;
            int pre_time_index;
            if(next_time_index + i + 1 >= path_->poses.size() - 1){
                pre_time_index = path_->poses.size() - 1;
            }
            else{
                pre_time_index = next_time_index + i + 1;
            }
            
            target_pose = path_->poses[pre_time_index];
        }
        
        // std::cout << "X_k:" << std::endl << X_k << std::endl;
        // std::cout << "X_r:" << std::endl << X_r << std::endl;
     
    }

    void MPC_Controller::get_H_q_A_l_u(){

        Eigen::MatrixXd E = A_bar * X_k + C_bar * O - X_r;
        H = 2 * B_bar.transpose() * Q * B_bar + 2 * R;
        q = 2 * B_bar.transpose() * Q * E;
        A = Eigen::MatrixXd::Zero(4 * N, 2 * N);
        l = Eigen::VectorXd::Zero(4 * N);
        u = Eigen::VectorXd::Zero(4 * N);

        /*===========================速度约束==================================*/
        Eigen::MatrixXd vel_constrain = Eigen::MatrixXd::Identity(2 * N, 2 * N);
        Eigen::VectorXd vel_l = Eigen::VectorXd::Zero(2 * N);
        Eigen::VectorXd vel_u = Eigen::VectorXd::Zero(2 * N);
        for (int i = 0; i < 2*N; i += 2) {
            vel_l(i) = -v_max;      // 最小线速度
            vel_u(i) = v_max;    // 最大线速度
            vel_l(i+1) = -w_max; // 最小角速度
            vel_u(i+1) = w_max;  // 最大角速度
        }
        A.block(0, 0, 2 * N, 2 * N) = vel_constrain;
        l.block(0, 0, 2 * N, 1) = vel_l;
        u.block(0, 0, 2 * N, 1) = vel_u;

        /*===========================加速度约束=================================*/
        Eigen::MatrixXd acc_constrain = Eigen::MatrixXd::Zero(2 * N, 2 * N);
        Eigen::VectorXd acc_l = Eigen::VectorXd::Zero(2 * N);
        Eigen::VectorXd acc_u = Eigen::VectorXd::Zero(2 * N);
        acc_constrain(0,0) = 1.0;
        acc_constrain(1,1) = 1.0;
        acc_l(0) = s_now.v - a_max * dt;
        acc_u(0) = s_now.v + a_max * dt;
        acc_l(1) = s_now.w - j_max * dt;
        acc_u(1) = s_now.w + j_max * dt;
        for (int i = 2; i < 2 * N; i += 2) {
            acc_constrain(i,i - 2) = -1.0;
            acc_constrain(i,i) = 1.0;
            acc_constrain(i+1,i+1) = 1.0;
            acc_constrain(i+1,i-1) = -1.0;
            acc_l(i) = -a_max * dt;
            acc_l(i+1) = -j_max * dt;
            acc_u(i) = a_max * dt;
            acc_u(i+1) = j_max * dt;
        }

        A.block(2 * N, 0, 2 * N, 2 * N) = acc_constrain;
        l.block(2 * N, 0, 2 * N, 1) = acc_l;
        u.block(2 * N, 0, 2 * N, 1) = acc_u;

        
        
    }


    void MPC_Controller::set_Q_R(){
        Q = Eigen::MatrixXd::Zero(3 * N, 3 * N);
        R = Eigen::MatrixXd::Zero(2 * N, 2 * N);
        // Q对应的是3 * N个状态误差的权重;R对应的是2 * N个控制量惩罚的权重
        for (int i = 0; i < 3 * N; i += 3){
            Q(i, i) = 1.0 ; // x误差权重
            Q(i+1, i+1) = 1.0 ; // y误差权重
            // Q(i+2, i+2) = 0.5 + (i/3) * 0.2; // 角度误差权重
            Q(i+2, i+2) = 0.0; // 角度误差权重
        }
        for (int i = 0; i < 2 * N; i += 2){
            R(i, i) = 0.01; // 线速度惩罚权重
            R(i+1, i+1) = 0.01; // 角速度惩罚权重
        }
    }
    
    Eigen::VectorXd MPC_Controller::OSQP_solve(){
        int dim = 2 * N;

        Eigen::MatrixXd P = H; 
        Eigen::SparseMatrix<double> P_sparse = P.sparseView();
        Eigen::SparseMatrix<double> A_sparse = A.sparseView();

        // **第一次调用时，初始化 OSQP**
        if (!is_initialized) {
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
            is_initialized = true;
        } else {
            // **已经初始化，直接更新数据**
            solver.updateHessianMatrix(P_sparse);
            solver.updateGradient(q);
            solver.updateLinearConstraintsMatrix(A_sparse);
            solver.updateLowerBound(l);
            solver.updateUpperBound(u);
        }

        // **求解**
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
            std::cout << "OSQP solver failed!" << std::endl;
            return Eigen::VectorXd::Zero(dim);
        }

        // **返回解**
        Eigen::VectorXd solution = solver.getSolution();
        std::cout << "Solution:\n" << solution.transpose() << std::endl;
        return solution;
    }

}