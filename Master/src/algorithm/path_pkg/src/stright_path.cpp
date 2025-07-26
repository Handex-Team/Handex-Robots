#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

class StraightPathPublisher : public rclcpp::Node
{
public:
    StraightPathPublisher()
    : Node("straight_path_publisher"),clock(RCL_SYSTEM_TIME)
    {
        using std::placeholders::_1;

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&StraightPathPublisher::odom_callback, this, _1));

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&StraightPathPublisher::goal_callback, this, _1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/straight_plan", 10);

        RCLCPP_INFO(this->get_logger(), "Straight path publisher node (replanning) has been started.");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::PoseStamped goal_pose_;

    rclcpp::Clock clock;// 时钟

    bool has_current_pose_ = false;
    bool has_goal_pose_ = false;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_.header = msg->header;
        current_pose_.pose = msg->pose.pose;
        has_current_pose_ = true;

        // 当有目标点时，实时重规划
        if (has_goal_pose_)
        {
            publish_straight_path();
        }
    }

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "开始path find");
        goal_pose_ = *msg;
        has_goal_pose_ = true;

        if (has_current_pose_) {
            publish_straight_path();
        }
    }

    void publish_straight_path()
    {
        nav_msgs::msg::Path path;
        path.header.stamp = clock.now();
        std::cout << "当前轨迹时间戳（秒）: " << path.header.stamp.sec << std::endl;
        std::cout << "当前轨迹时间戳（纳秒）: " << path.header.stamp.nanosec << std::endl;
 
        path.header.frame_id = "odom";  // 或 "map"，根据TF树调整

        const int N = 20;  // 点数量
        for (int i = 0; i <= N; ++i)
        {
            double ratio = static_cast<double>(i) / N;

            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;

            pose.pose.position.x = current_pose_.pose.position.x + ratio * (goal_pose_.pose.position.x - current_pose_.pose.position.x);
            pose.pose.position.y = current_pose_.pose.position.y + ratio * (goal_pose_.pose.position.y - current_pose_.pose.position.y);
            pose.pose.position.z = 0.0;

            // 可选：保持起点朝向，也可以插值
            pose.pose.orientation = current_pose_.pose.orientation;

            path.poses.push_back(pose);
        }
        // path.poses.push_back(goal_pose_);

        path_pub_->publish(path);
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Re-published straight path.");
        RCLCPP_INFO(this->get_logger(), "路径已更新");

    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StraightPathPublisher>());
    rclcpp::shutdown();
    return 0;
}
