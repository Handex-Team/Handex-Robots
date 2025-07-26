// 最终版本的MPC控制代码，使用OSQP求解器优化轨迹
#include "mpc_pkg/mpc_diff_car.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"
class MPC_test : public rclcpp::Node {
public:
    MPC_test() : Node("mpc_test") {
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 1, std::bind(&MPC_test::OdomCallback, this, std::placeholders::_1));

        opt_traj_sub_ = this->create_subscription<nav_msgs::msg::Path>("/optimized_path", 1,
                     std::bind(&MPC_test::OptTrajCallback, this, std::placeholders::_1));
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    }


private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr opt_traj_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    // rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_test_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    nav_msgs::msg::Path opt_traj;
    double path_generate_time;// 轨迹生成的起始时间
    // 控制信号
    geometry_msgs::msg::Twist cmd_vel;
    void OptTrajCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        if(msg->poses.size() >= 3){
            // 检测轨迹有没有异常，如果第一个点和最后一个点的坐标位置在地图范围之内，认为轨迹正常
            if(msg->poses[0].pose.position.x > -15.0 && msg->poses[0].pose.position.x < 15.0 &&
                msg->poses[0].pose.position.y > -15.0 && msg->poses[0].pose.position.y < 15.0 )
                {
                    opt_traj = *msg;
                    std::cout << "第一个路径点:\n";
                    std::cout << opt_traj.poses[0].pose.position.x << std::endl;
                    std::cout << opt_traj.poses[0].pose.position.y << std::endl;
                    std::cout << opt_traj.poses[0].pose.orientation.x << std::endl;

                    std::cout << "最后一个路径点:\n";
                    std::cout << opt_traj.poses[opt_traj.poses.size()-1].pose.position.x << std::endl;
                    std::cout << opt_traj.poses[opt_traj.poses.size()-1].pose.position.y << std::endl;
                    std::cout << opt_traj.poses[opt_traj.poses.size()-1].pose.orientation.x << std::endl;
                }
            
        }
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
        
        if(opt_traj.poses.size() <= 1){
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            pub_cmd_vel_->publish(cmd_vel);
            std::cout << "路径点数小于等于1,退出MPC" << std::endl;
            return;
        }
        Mpc_diff_car::MPC_Controller mpc_controller(mpc_state, opt_traj);
        // 参数设置
        mpc_controller.N = 15;
        mpc_controller.a_max = 0.5;
        mpc_controller.v_max = 2.0;
        mpc_controller.w_max = 1.0;
        mpc_controller.j_max = 1.0;
        // 开始优化
        mpc_controller.init_controller();
        mpc_controller.set_Q_R();
        mpc_controller.get_Ak_Bk_Ok(mpc_state);
        mpc_controller.get_Abar_Bbar_Cbar_O();
        mpc_controller.get_X_r_X_k();
        mpc_controller.get_H_q_A_l_u();
        auto solution = mpc_controller.OSQP_solve();

        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        std::cout << "Duration: " << duration << " microseconds" << std::endl;
        cmd_vel.linear.x = solution[0];
        cmd_vel.angular.z = solution[1];

        pub_cmd_vel_->publish(cmd_vel); 
    }

};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MPC_test>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}