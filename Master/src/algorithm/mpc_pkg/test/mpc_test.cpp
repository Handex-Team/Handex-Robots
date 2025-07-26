#include "mpc_pkg/mpc_diff_car.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"
class MPC_test : public rclcpp::Node {
public:
    MPC_test() : Node("mpc_test") {
        path_test = path_circle_test_generate();
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 1, std::bind(&MPC_test::OdomCallback, this, std::placeholders::_1));
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        pub_path_test_ = this->create_publisher<nav_msgs::msg::Path>("/path_test", 1);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/path_test_marker", 1);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MPC_test::timer_callback, this));
    }

    void Test(){
        auto start_time = std::chrono::steady_clock::now();
        std::cout << "Hello World!" << std::endl;
        Mpc_diff_car::MPCState mpc_state;
        mpc_state.theta = 0;
        mpc_state.v = 0.0;
        mpc_state.w = 0.0;
        mpc_state.x = -1.0;
        mpc_state.y = -2.0;

        auto path_test = path_circle_test_generate();
        Mpc_diff_car::MPC_Controller mpc_controller(mpc_state, path_test);

        mpc_controller.get_Ak_Bk_Ok(mpc_state);
        std::cout << "Ak: " << mpc_controller.Ak << std::endl;
        std::cout << "Bk: " << mpc_controller.Bk << std::endl;
        std::cout << "Ok: " << mpc_controller.Ok << std::endl;

        std::cout << "A_hat: " << mpc_controller.A_hat << std::endl;
        std::cout << "B_hat: " << mpc_controller.B_hat << std::endl;
        std::cout << "O_hat: " << mpc_controller.O_hat << std::endl;
        

        mpc_controller.get_Abar_Bbar_Cbar_O();
        mpc_controller.get_X_r_X_k();
        mpc_controller.get_H_q_A_l_u();
        mpc_controller.OSQP_solve();

        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        std::cout << "Duration: " << duration << " microseconds" << std::endl;
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_test_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Path path_test;
    double path_generate_time;// 轨迹生成的起始时间
    // 控制信号
    geometry_msgs::msg::Twist cmd_vel;

    void timer_callback(){
        path_generate_time += 0.1;
        int path_generate_time_index = path_generate_time/0.1 - 1;
        // 发布控制信号
        // pub_cmd_vel_->publish(cmd_vel); // 这里控制频率比较低，后面要调整
        // 发布轨迹
        pub_path_test_->publish(path_test);
        // path_test = path_test_generate();
        double x = path_test.poses[path_generate_time_index].pose.position.x;
        double y = path_test.poses[path_generate_time_index].pose.position.y;
        double theta = path_test.poses[path_generate_time_index].pose.orientation.x;
        publish_moving_marker(x,y,theta);
        
        RCLCPP_INFO(this->get_logger(), "轨迹发布成功");
        
    }
    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
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
        mpc_state.v = msg->twist.twist.linear.x;
        mpc_state.w = msg->twist.twist.angular.z;
        mpc_state.x = msg->pose.pose.position.x;
        mpc_state.y = msg->pose.pose.position.y;
        RCLCPP_INFO(this->get_logger(), "MPCState: %f %f %f %f %f %f", mpc_state.x, mpc_state.y, mpc_state.theta, mpc_state.v, mpc_state.w, mpc_state.theta);
        
        if(path_test.poses.size() <= 1){
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            pub_cmd_vel_->publish(cmd_vel);
            std::cout << "路径点数小于等于1,退出MPC" << std::endl;
            return;
        }
        Mpc_diff_car::MPC_Controller mpc_controller(mpc_state, path_test);
        mpc_controller.N = 15;
        mpc_controller.init_controller();
        mpc_controller.set_Q_R();
        mpc_controller.get_Ak_Bk_Ok(mpc_state);
        mpc_controller.get_Abar_Bbar_Cbar_O();
        mpc_controller.get_X_r_X_k();
        mpc_controller.get_H_q_A_l_u();
        auto solution = mpc_controller.OSQP_solve();
        
        // auto X_error = mpc_controller.A_bar * mpc_controller.X_k - mpc_controller.B_bar * solution 
        //     +mpc_controller.C_bar * mpc_controller.O - mpc_controller.X_r;


        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        std::cout << "Duration: " << duration << " microseconds" << std::endl;
        cmd_vel.linear.x = solution[0];
        cmd_vel.angular.z = solution[1];

        pub_cmd_vel_->publish(cmd_vel); 
    }



    nav_msgs::msg::Path path_circle_test_generate() {
        path_generate_time = 0.0;
        rclcpp::Clock clock(RCL_SYSTEM_TIME);
        rclcpp::Time current_time = clock.now();
        nav_msgs::msg::Path path;
        path.header.frame_id = "odom";
        path.header.stamp = current_time;
        
        double dt = 0.1;  // 控制周期
        double R = 3.0;   // 圆的半径
        double omega = 0.1; // 角速度

        for (size_t i = 0; i < 1000; i++) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = "odom";
            pose_stamped.header.stamp = current_time;

            // 当前角度
            double t = i * dt;
            double theta = omega * t; // 角度随时间变化

            // 圆轨迹计算
            pose_stamped.pose.position.x = R * std::cos(theta);
            pose_stamped.pose.position.y = R * std::sin(theta);

            // 计算线速度 v = R * omega
            double v = R * omega;

            // 角速度 w = omega
            double w = omega;

            // 设置朝向 (航向角与圆的切线方向一致)
            pose_stamped.pose.orientation.x = theta; // 航向角
            pose_stamped.pose.orientation.z = v;     // 线速度
            pose_stamped.pose.orientation.w = w;     // 角速度
            
            path.poses.push_back(pose_stamped);

            // 更新时间
            current_time += rclcpp::Duration(0, dt * 1e9);
        }

        return path;
    }

    nav_msgs::msg::Path path_point_test_generate() {
        path_generate_time = 0.0;
        rclcpp::Clock clock(RCL_SYSTEM_TIME);
        rclcpp::Time current_time = clock.now();
        nav_msgs::msg::Path path;
        path.header.frame_id = "odom";
        path.header.stamp = current_time;
        
        double dt = 0.1;  // 控制周期

        for (size_t i = 0; i < 1000; i++) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = "odom";
            pose_stamped.header.stamp = current_time;

            // 圆轨迹计算
            pose_stamped.pose.position.x = 3.0;
            pose_stamped.pose.position.y = 3.0;

            // 当前角度
            double t = i * dt;
            // double theta = atan2(pose_stamped.pose.position.y, pose_stamped.pose.position.x); // 角度随时间变化
            double theta = M_PI/4;
            

            // 计算线速度 v = R * omega
            double v = 0.5;

            // 角速度 w = omega
            double w = 0.5;

            // 设置朝向 (航向角与圆的切线方向一致)
            pose_stamped.pose.orientation.x =theta; // 航向角
            pose_stamped.pose.orientation.z = v;     // 线速度
            pose_stamped.pose.orientation.w = w;     // 角速度
            
            path.poses.push_back(pose_stamped);

            // 更新时间
            current_time += rclcpp::Duration(0, dt * 1e9);
        }

        return path;
    }

    void publish_moving_marker(double x, double y, double theta) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = this->now();
        marker.ns = "trajectory";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        
        marker_pub_->publish(marker);
    }
};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MPC_test>();
    // node->Test();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}