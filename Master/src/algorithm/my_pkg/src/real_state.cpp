#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;
class SimGroundTruthPublisher : public rclcpp::Node
{
public:
    SimGroundTruthPublisher() : Node("sim_ground_truth_publisher")
    {
        // 创建发布器，发布 Odometry 消息到 /sim_ground_truth_odom 话题
        
        pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/my_sim_ground_truth_pose", 1);

        pub_map_pos = this->create_publisher<nav_msgs::msg::Odometry>("/base_link_to_map_pose", 1);
        
        
        // 创建 tf2 的缓冲区和监听器
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 定时器，周期性发布变换
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SimGroundTruthPublisher::timer_callback, this)
        );
    }

private:
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "real_state_publisher: Publishing simulated ground truth odometry");
        try {

            // 获取 laser_link 到 odom 的坐标变换
            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
                "odom",     // 目标坐标系
                "base_link", // 源坐标系
                tf2::TimePointZero); // 获取当前变换
            // 创建 Odometry 消息，并填充变换数据
            nav_msgs::msg::Odometry odom;
            // odom.header.stamp = this->get_clock()->now();
            odom.header.frame_id = "odom"; // 坐标系为 odom
            odom.child_frame_id = "base_link"; // child_frame_id 为 base_link

            // 填充位置数据
            odom.pose.pose.position.x = transform_stamped.transform.translation.x;
            odom.pose.pose.position.y = transform_stamped.transform.translation.y;
            odom.pose.pose.position.z = transform_stamped.transform.translation.z;

            odom.pose.pose.orientation = transform_stamped.transform.rotation;

            // 由于我们没有速度信息，这里将速度信息设为零
            odom.twist.twist.linear.x = 0.0;
            odom.twist.twist.linear.y = 0.0;
            odom.twist.twist.linear.z = 0.0;

            odom.twist.twist.angular.x = 0.0;
            odom.twist.twist.angular.y = 0.0;
            odom.twist.twist.angular.z = 0.0;

            // 发布消息
            pub_->publish(odom);
            
            RCLCPP_INFO(this->get_logger(), "real_state_publisher: 发布成功");
        }
        catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }

        try{
        geometry_msgs::msg::TransformStamped transform_stamped_map = tf_buffer_->lookupTransform(
                "map",     // 目标坐标系
                "base_link", // 源坐标系
               tf2::TimePointZero,1s);
            // 发布 base_link 到 map 的坐标
            nav_msgs::msg::Odometry odom_map_pose;
            odom_map_pose.header.stamp = this->get_clock()->now();
            RCLCPP_INFO(this->get_logger(), "系统时间：%f",this->get_clock()->now().seconds());
            odom_map_pose.header.frame_id = "map"; // 坐标系为 map
            odom_map_pose.child_frame_id = "base_link"; // child_frame_id 为 base_link

            odom_map_pose.pose.pose.position.x = transform_stamped_map.transform.translation.x;
            odom_map_pose.pose.pose.position.y = transform_stamped_map.transform.translation.y;
            odom_map_pose.pose.pose.position.z = transform_stamped_map.transform.translation.z;

            odom_map_pose.pose.pose.orientation = transform_stamped_map.transform.rotation;

            // 由于我们没有速度信息，这里将速度信息设为零
            odom_map_pose.twist.twist.linear.x = 0.0;
            odom_map_pose.twist.twist.linear.y = 0.0;
            odom_map_pose.twist.twist.linear.z = 0.0;

            odom_map_pose.twist.twist.angular.x = 0.0;
            odom_map_pose.twist.twist.angular.y = 0.0;
            odom_map_pose.twist.twist.angular.z = 0.0;
            pub_map_pos->publish(odom_map_pose);
            
            RCLCPP_INFO(this->get_logger(), "(map)real_state_publisher: 发布成功");
        }
        catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "(base_link_to_map_pose)Could not transform: %s", ex.what());
        }
            
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_,pub_map_pos;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimGroundTruthPublisher>());
    rclcpp::shutdown();
    return 0;
}
