#include "mpc_pkg/mpc_diff_car.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int8.hpp"
class MPC_test : public rclcpp::Node {
public:
    MPC_test() : Node("mpc_test") {
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 1, std::bind(&MPC_test::OdomCallback, this, std::placeholders::_1));

        opt_traj_sub_ = this->create_subscription<nav_msgs::msg::Path>("/optimized_path", 1,
                     std::bind(&MPC_test::OptTrajCallback, this, std::placeholders::_1));

        
        
        
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 1,std::bind(&MPC_test::GoalCallback, this, std::placeholders::_1) );
        
        control_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&MPC_test::TimerCallback, this));
        // 雷达用于障碍停止
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 1, std::bind(&MPC_test::ScanCallback, this, std::placeholders::_1));
        car_state_pub_ = this->create_publisher<std_msgs::msg::Int8>("/car_state", 1);
        car_state_pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MPC_test::CarStatePubTimerCallback, this));

        now_state = nav_msgs::msg::Odometry();
        car_state_msg = std_msgs::msg::Int8();
    }


private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr opt_traj_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr car_state_pub_timer_;

    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr car_state_pub_;
    std_msgs::msg::Int8 car_state_msg;
    nav_msgs::msg::Path opt_traj;
    double path_generate_time;// 轨迹生成的起始时间
    // 控制信号
    geometry_msgs::msg::Twist cmd_vel;
    geometry_msgs::msg::PoseStamped goal_pose;

    Eigen::VectorXd solution; //MPC控制结果
    bool is_traj_sub = false;
    double angle_kp = 0.5; // 角速度控制参数

    // 安全停止逻辑
    bool stop_flag = false;
    double safe_distance = 0.8; // 安全阈值（单位：米）
    nav_msgs::msg::Odometry now_state; // odom信息

    bool is_reach_goal  = false; // 是否到达目标点

    void CarStatePubTimerCallback() {
        if(stop_flag&&!is_reach_goal){
            car_state_msg.data = 3; // 危险标志
        }
        else if(stop_flag&&is_reach_goal){
            car_state_msg.data = 3; // 危险标志
        }
        else if(!stop_flag&&is_reach_goal){
            car_state_msg.data = 2; // 忙碌
        }
        else if((!stop_flag&&!is_reach_goal)){
            car_state_msg.data = 1; // 空闲
        }
        RCLCPP_INFO(this->get_logger(), "当前车辆状态: %d", car_state_msg.data);
        car_state_pub_ -> publish(car_state_msg);
    }



    void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        stop_flag = false;

        // 当前状态
        double x0 = now_state.pose.pose.position.x;
        double y0 = now_state.pose.pose.position.y;

        // 提取yaw角
        double qx = now_state.pose.pose.orientation.x;
        double qy = now_state.pose.pose.orientation.y;
        double qz = now_state.pose.pose.orientation.z;
        double qw = now_state.pose.pose.orientation.w;
        double theta0 = std::atan2(2.0 * (qw * qz + qx * qy),
                                1.0 - 2.0 * (qy * qy + qz * qz));

        // 1. 找到距离当前坐标最近的雷达点
        double min_dist = std::numeric_limits<double>::max();
        double angle_min = 0.0; //最小距离点对应的相对于车正前方的角度
        double nearest_x = 0.0, nearest_y = 0.0;

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            
            float r = msg->ranges[i];
            if (std::isinf(r) || std::isnan(r)) continue;

            float angle = msg->angle_min + i * msg->angle_increment;

            if (angle > -0.3) continue; //只关注雷达右侧的,屏蔽左侧
            float local_angle = theta0 + angle;

            // 转换到世界坐标
            double obs_x = x0 + r * std::cos(local_angle);
            double obs_y = y0 + r * std::sin(local_angle);

            double dist = std::hypot(obs_x - x0, obs_y - y0);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_x = obs_x;
                nearest_y = obs_y;
                angle_min = angle;
            }
        }

        RCLCPP_INFO(this->get_logger(), "当前雷达点最小距离: %f,相对于车正前方的角度: %f", min_dist, angle_min);

        // 如果没有有效雷达点，直接返回
        if (min_dist == std::numeric_limits<double>::max()) {
            return;
        }

        // 2. 用MPC结果做2秒内的轨迹预测
        double x = x0;
        double y = y0;
        double theta = theta0;

        double dt = 0.1;  // 每个控制指令的时间步
        int steps = std::min((int)(solution.size() / 2), 10); // 最多1秒（10步）

        for (int i = 0; i < steps; ++i) {
            double v = solution[2 * i];
            double w = solution[2 * i + 1];

            x += v * std::cos(theta) * dt;
            y += v * std::sin(theta) * dt;
            theta += w * dt;

            double d = std::hypot(x - nearest_x, y - nearest_y);
            if (d < safe_distance) {
                stop_flag = true;

                
                return;
            }

        }
        stop_flag = false;
    }


    double NormalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
    void GoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        goal_pose = *msg;
    }
    void TimerCallback() {
        if(stop_flag){
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            RCLCPP_WARN(this->get_logger(), "安全停止");
        }
        RCLCPP_INFO(this->get_logger(), "控制输出: : v= %f, w= %f", cmd_vel.linear.x, cmd_vel.angular.z);
        pub_cmd_vel_->publish(cmd_vel); 
    }

    void OptTrajCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        if(msg->poses.size() >= 3){
            // 检测轨迹有没有异常，如果第一个点和最后一个点的坐标位置在地图范围之内，认为轨迹正常
            if(msg->poses[0].pose.position.x > -15.0 && msg->poses[0].pose.position.x < 15.0 &&
                msg->poses[0].pose.position.y > -15.0 && msg->poses[0].pose.position.y < 15.0 )
                {
                    opt_traj = *msg;
                    is_traj_sub = true;
                }
            
        }
    }

    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        auto start_odom_time = std::chrono::steady_clock::now();
        now_state = *msg;
        if (!is_traj_sub){
            RCLCPP_WARN(this->get_logger(), "未收到优化轨迹，无法进行MPC控制");
            return;
        }
        double dis_err_max = 0.1, angle_err_max = 0.1;
        if(sqrt((goal_pose.pose.position.x - msg->pose.pose.position.x) * (goal_pose.pose.position.x - msg->pose.pose.position.x)
            + (goal_pose.pose.position.y - msg->pose.pose.position.y) * (goal_pose.pose.position.y - msg->pose.pose.position.y)) < dis_err_max){
                cmd_vel.linear.x = 0.0;
                // cmd_vel.angular.z = 0.0;
                // auto start_time = std::chrono::steady_clock::now();
                // 四元数转欧拉角
                tf2::Quaternion q(goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w);
                tf2::Quaternion odom_quat(
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);
                double roll, pitch, yaw;
                double roll_odom, pitch_odom, yaw_odom;
                tf2::Matrix3x3 m(q);
                m.getRPY(roll, pitch, yaw);
                tf2::Matrix3x3 m_odom(odom_quat);
                m_odom.getRPY(roll_odom, pitch_odom, yaw_odom);
                if(abs(yaw - yaw_odom) < angle_err_max){
                    cmd_vel.angular.z = 0.0;
                    is_reach_goal = true;
                }
                else{
                    
                    // cmd_vel.angular.z = angle_kp * (yaw - yaw_odom);
                    cmd_vel.angular.z = angle_kp * NormalizeAngle(yaw - yaw_odom);
                }
                // auto end_time = std::chrono::steady_clock::now();
                // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
                // std::cout << "test_Duration: " << duration << " microseconds" << std::endl;

                // pub_cmd_vel_->publish(cmd_vel);
                return;
            }
        auto start_time = std::chrono::steady_clock::now();
        tf2::Quaternion odom_quat(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        double roll, pitch, yaw;//定义存储r\p\y的容器
        tf2::Matrix3x3 m(odom_quat);
        m.getRPY(roll, pitch, yaw);//进行转换

        Mpc_diff_car::MPCState mpc_state;
        mpc_state.theta = yaw;
        // double v_limit = 0.001, w_limit = 0.001;
        // if(msg->twist.twist.linear.x>0&&msg->twist.twist.linear.x<v_limit){
        //     mpc_state.v = v_limit;
        // }
        // else if(msg->twist.twist.linear.x<0&&msg->twist.twist.linear.x>-v_limit){
        //     mpc_state.v = -v_limit;
        // }
        // else{
        // mpc_state.v = msg->twist.twist.linear.x;
        // }
        // if(msg->twist.twist.angular.z>0&&msg->twist.twist.angular.z<w_limit){
        //     mpc_state.w = w_limit;
        // }
        // else if(msg->twist.twist.angular.z<0&&msg->twist.twist.angular.z>-w_limit){
        //     mpc_state.w = -w_limit;
        // }
        // else{
        // mpc_state.w = msg->twist.twist.angular.z;
        // }
        mpc_state.v = msg->twist.twist.linear.x;
        mpc_state.w = msg->twist.twist.angular.z;
        mpc_state.x = msg->pose.pose.position.x;
        mpc_state.y = msg->pose.pose.position.y;
        RCLCPP_INFO(this->get_logger(), "MPCState: %f %f %f %f %f %f", mpc_state.x, mpc_state.y, mpc_state.theta, mpc_state.v, mpc_state.w, mpc_state.theta);
        
        // if(opt_traj.poses.size() <= 1){
        //     cmd_vel.linear.x = 0.0;
        //     cmd_vel.angular.z = 0.0;
        //     pub_cmd_vel_->publish(cmd_vel);
        //     std::cout << "路径点数小于等于1,退出MPC" << std::endl;
        //     return;
        // }
        RCLCPP_INFO(this->get_logger(), "传入mpc的轨迹起始时间戳: %f", opt_traj.header.stamp.sec + opt_traj.header.stamp.nanosec * 1e-9);
        Mpc_diff_car::MPC_Controller mpc_controller(mpc_state, opt_traj);
        // 参数设置
        mpc_controller.N = 15;
        mpc_controller.a_max = 0.5; // 0.5  0.5
        mpc_controller.v_max = 1.0; // 2.0  1.0
        mpc_controller.w_max = 1.0; // 1.0  1.0
        mpc_controller.j_max = 1.0; // 1.0  1.0
        // 开始优化
        mpc_controller.init_controller();
        mpc_controller.set_Q_R();
        mpc_controller.get_Ak_Bk_Ok(mpc_state);
        mpc_controller.get_Abar_Bbar_Cbar_O();
        mpc_controller.get_X_r_X_k();
        mpc_controller.get_H_q_A_l_u();
        solution = mpc_controller.OSQP_solve();

        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        auto duration_odom = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_odom_time).count();
        std::cout << "Duration: " << duration << " microseconds" << std::endl;
        std::cout << "odom_Duration: "<< duration_odom << " microseconds" << std::endl;
        cmd_vel.linear.x = solution[0];
        cmd_vel.angular.z = solution[1];

        // pub_cmd_vel_->publish(cmd_vel); 
    }

};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MPC_test>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}