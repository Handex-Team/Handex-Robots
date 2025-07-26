
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "message_filters/subscriber.h"
#include "tf2_ros/message_filter.h"

using namespace std::chrono_literals;
class laser_node : public rclcpp::Node
{
public:
    laser_node() : Node("laser_node")
    {   

        RCLCPP_INFO(this->get_logger(), "雷达节点初始化成功");

        this->tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        this->tf_timer = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(),
            this->get_node_timers_interface());
        this->tf_buffer->setCreateTimerInterface(this->tf_timer);  // 绑定定时器接口

        this->tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        laser_sub.subscribe(this, "/scan");
        laser_filter=std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(laser_sub,*tf_buffer,"base_link",10, this->get_node_logging_interface(),this->get_node_clock_interface(),1s);
        laser_filter->registerCallback(std::bind(&laser_node::laser_callback, this, std::placeholders::_1));
        pub_laser_fcous = this->create_publisher<std_msgs::msg::Float32MultiArray>("/obs_fcous", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 1,
            std::bind(& laser_node::odomCallback, this, std::placeholders::_1));
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_laser_fcous;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::shared_ptr<tf2_ros::CreateTimerROS> tf_timer;
    message_filters::Subscriber<sensor_msgs::msg::LaserScan> laser_sub;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> laser_filter;

    std::vector<std::vector<double>> obs_list;
    double safe_distance = 10.0;
    nav_msgs::msg::Odometry now_pose;

    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        now_pose = *msg;
    }
    

   
    void laser_callback(const std::shared_ptr<const sensor_msgs::msg::LaserScan>& msg)
    {
        obs_list.clear();

        for (int i = 0; i < msg->ranges.size(); i++)
        {
            if (!std::isinf(msg->ranges[i]) && !std::isnan(msg->ranges[i]) && msg->ranges[i] < safe_distance)
            {
                double angle = (double) msg->angle_min + i * msg->angle_increment;

                // 障碍点到雷达的坐标
                double x = (double) msg->ranges[i] * cos(angle);
                double y = (double) msg->ranges[i] * sin(angle);

                geometry_msgs::msg::Point point;
                point.x = x;
                point.y = y;
                point.z = 0;

                // 障碍点在地图坐标系下的坐标
                geometry_msgs::msg::PointStamped point_in_laser, point_in_odom;
                point_in_laser.header.frame_id = "base_link";  // 使用base_link作为参考坐标系
                point_in_laser.header.stamp = msg->header.stamp;  // 使用激光数据的时间戳
                point_in_laser.point = point;

                try
                {
                    // 使用1秒的超时来获取变换，确保足够的时间同步
                    // tf_buffer->waitForTransform("odom", "base_link", msg->header.stamp, tf2::durationFromSec(0.5),std::bind(&laser_node::time_callback, this));
                    point_in_odom = tf_buffer->transform(point_in_laser, "odom", tf2::durationFromSec(1.0));  
                }
                catch (const std::exception& e)
                {
                    RCLCPP_WARN(this->get_logger(), "TF变换失败: %s", e.what());  // 日志记录警告
                    continue;  // 如果变换失败，跳过当前障碍点
                }

                // 使用从激光坐标系转换到odom坐标系后的坐标
                std::vector<double> obs_state = {point_in_odom.point.x, point_in_odom.point.y};
                obs_list.push_back(obs_state);
            }
        }

        if (!obs_list.empty())  // 只有在obs_list非空时才进行发布
        {
            RCLCPP_INFO(this->get_logger(), "雷达数据更新成功,第一个坐标点为(%f,%f)", obs_list[0][0], obs_list[0][1]);

            auto message = std_msgs::msg::Float32MultiArray();
            
            // 设置消息的布局：行数和列数
            std_msgs::msg::MultiArrayLayout layout;
            layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
            layout.dim[0].label = "rows";
            layout.dim[0].size = obs_list.size();   // 行数
            layout.dim[0].stride = obs_list[0].size() * obs_list.size();  // 每一行的元素数
            layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
            layout.dim[1].label = "cols";
            layout.dim[1].size = obs_list[0].size();  // 列数
            layout.dim[1].stride = obs_list[0].size();  // 列的步长
            message.layout = layout;

            for (const auto& row : obs_list)
            {
                message.data.insert(message.data.end(), row.begin(), row.end());
            }

            pub_laser_fcous->publish(message);  // 发布消息
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "没有检测到障碍物");
            auto message = std_msgs::msg::Float32MultiArray();
            pub_laser_fcous->publish(message);  // 发布消息
        }
    }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<laser_node>());
    rclcpp::shutdown();
    return 0;
}
