#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
using namespace std::chrono_literals;
class Vis : public rclcpp::Node
{
public:
    Vis() : Node("vis")
    {
        RCLCPP_INFO(this->get_logger(), "可视化节点初始化成功");

        // 订阅 /obs_fcous 消息
        obs_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/obs_fcous", 10, std::bind(&Vis::obs_callback, this, std::placeholders::_1));
        
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 发布 Marker 消息
        pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("/obstacle_marker", 10);

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 1,
            std::bind(&Vis::odom_callback, this, std::placeholders::_1));
        
        timer_ = this->create_wall_timer(100ms, std::bind(&Vis::timer_callback, this));



    }

private:
    
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obs_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

    
    
    void obs_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // 创建 Marker 消息
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";  // 设置参考坐标系
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "obstacles";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;  // 点的大小
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;  // 透明度
        marker.color.r = 1.0;  // 红色
        marker.color.g = 0.5;  // 绿色
        marker.color.b = 0.3;  // 蓝色

        // 从消息中提取坐标点并填充到 Marker 中
        for (size_t i = 0; i < msg->data.size(); i += 2)  // 每个障碍物有 2 个数据（x, y）
        {
            geometry_msgs::msg::Point p;
            p.x = msg->data[i];
            p.y = msg->data[i + 1];
            p.z = 0.0;  // Z轴为 0
            marker.points.push_back(p);
        }

        // 发布 Marker 消息
        pub_marker_->publish(marker);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        // 订阅当前坐标并发布到marker
        // visualization_msgs::msg::Marker marker;
        // marker.header.frame_id = "odom";  // 设置参考坐标系
        // marker.header.stamp = this->get_clock()->now();
        // marker.ns = "robot";
        // marker.id = 0;
        // marker.type = visualization_msgs::msg::Marker::SPHERE;
        // marker.action = visualization_msgs::msg::Marker::ADD;
        // marker.pose.position.x = msg->pose.pose.position.x;
        // marker.pose.position.y = msg->pose.pose.position.y;
        // marker.pose.position.z = 0.0;  // Z轴为 0
        // marker.pose.orientation.w = 1.0;
        // marker.scale.x = 0.2;  // 点的大小
        // marker.scale.y = 0.2;
        // marker.scale.z = 0.2;
        // marker.color.a = 1.0;  // 透明度
        // marker.color.r = 0.7;  // 红色
        // marker.color.g = 0.3;  // 绿色
        // marker.color.b = 1.0;  // 蓝色
        // // 发布 Marker 消息
        // pub_marker_->publish(marker);
    }




    void timer_callback()
    {
        try
        {
            // 查询 base_link 到 odom 的坐标变换
            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
                "odom", "base_link", tf2::TimePointZero);

            // 创建 Marker 消息
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";  // 设置参考坐标系
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "robot";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // 从坐标变换中提取位置信息
            marker.pose.position.x = transform_stamped.transform.translation.x;
            marker.pose.position.y = transform_stamped.transform.translation.y;
            marker.pose.position.z = transform_stamped.transform.translation.z;
            marker.pose.orientation = transform_stamped.transform.rotation;

            // 设置 Marker 外观
            marker.scale.x = 0.2;  // Marker 的大小
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.a = 1.0;  // 透明度
            marker.color.r = 1.0;  // 红色
            marker.color.g = 1.0;  // 绿色
            marker.color.b = 1.0;  // 蓝色

            // 发布 Marker 消息
            pub_marker_->publish(marker);

            RCLCPP_INFO(this->get_logger(), "Marker发布成功，位置:x=%.2f, y=%.2f, z=%.2f",
                        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "无法查询坐标变换：%s", ex.what());
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Vis>());
    rclcpp::shutdown();
    return 0;
}
