#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Matrix3x3.h>
class DispatchCar : public rclcpp::Node
{
public:
    DispatchCar() : Node("dispatch_car")
    {
        RCLCPP_INFO(this->get_logger(), "DispatchCar 节点初始化成功");

        // 创建发布器，发布 odom 话题
        car_target_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("car_target_odom", 10);

        // 创建定时器，每隔 1 秒发布一次目标点
        car_target_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&DispatchCar::CarTargetTimerCallback, this));

        car_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&DispatchCar::CarOdomCallback, this, std::placeholders::_1));
        car_state_sub_ = this->create_subscription<std_msgs::msg::Int8>("/car_state", 10, std::bind(&DispatchCar::CarStateCallback, this, std::placeholders::_1));
        task_sub_   = this->create_subscription<std_msgs::msg::Int8>("/task", 10, std::bind(&DispatchCar::TaskCallback, this, std::placeholders::_1));
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        
        target_odom_msg_ = std::make_shared<nav_msgs::msg::Odometry>();
        car_odom_msg_ = std::make_shared<nav_msgs::msg::Odometry>();
        car_state = std_msgs::msg::Int8();
        car_state.data = 0;
        task = std_msgs::msg::Int8();
        task.data = 10; // 未订阅到任务
        yaw = 0;
    }
    

private:
    
    rclcpp::TimerBase::SharedPtr car_target_timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr car_target_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr car_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr task_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr car_odom_sub_;


    nav_msgs::msg::Odometry::SharedPtr target_odom_msg_;
    nav_msgs::msg::Odometry::SharedPtr car_odom_msg_;
    std_msgs::msg::Int8 car_state;
    std_msgs::msg::Int8 task;
    double yaw;

    void CarOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
        *car_odom_msg_ = *msg;
        yaw = getYawFromOdom(car_odom_msg_);
    }
    
    void CarStateCallback(const std_msgs::msg::Int8::SharedPtr msg){
        car_state.data = msg->data;
    }

    void TaskCallback(const std_msgs::msg::Int8::SharedPtr msg){
        task.data = msg->data;
        deal_task(task);
    }

    void CarTargetTimerCallback()
    {
        switch(car_state.data){
            case 1:
                RCLCPP_WARN(this->get_logger(), "当前状态忙碌，不能发布目标点");
                break;
            case 2:{
                if(task.data == 10){
                    RCLCPP_WARN(this->get_logger(), "未订阅到任务");
                }
                car_target_publisher_->publish(*target_odom_msg_);
                geometry_msgs::msg::PoseStamped goal_pose;
                goal_pose.header = target_odom_msg_->header;
                goal_pose.pose = target_odom_msg_->pose.pose;
                goal_pub_->publish(goal_pose);
                RCLCPP_INFO(this->get_logger(), "已发布目标点 (%.2f, %.2f)", 
                    target_odom_msg_->pose.pose.position.x, target_odom_msg_->pose.pose.position.y);
                break;
            }
                
            case 3:
                RCLCPP_ERROR(this->get_logger(), "当前状态危险，不能发布目标点");
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "当前状态未知，不能发布目标点");
                break;
        }
        
    }

    void deal_task(std_msgs::msg::Int8 Task){
        switch (Task.data)
        {
        case 0:
            *target_odom_msg_ = set_target(0.0, 0.0, 0.0);
            std::cout << "任务0,回到原点,方向朝向 0.0" << std::endl;
            break;
        case 1:
            *target_odom_msg_ = set_target(1.0, 0.0, 0.0);
            std::cout << "任务1,前进到 (0.2, 0.0),方向朝向 0.0" << std::endl;
            break;
        case 2:
            *target_odom_msg_ = set_target(0.0, 0.0, 0.0);
            std::cout << "任务2,前进到 (-0.2, 0.0),方向朝向 0.0" << std::endl;
            break;
        case 3:
            *target_odom_msg_ = set_target(2.0, 0.0, 1.57);
            std::cout << "任务3,前进到 (0.0, 0.0),方向朝向 1.57" << std::endl;
            break;
        case 4:
            *target_odom_msg_ = set_target(0.0, -2.0, -1.57);
            std::cout << "任务4,前进到 (0.0, 0.0),方向朝向 -1.57" << std::endl;
            break;
        case 5:
            *target_odom_msg_ = set_target(2.0, 2.0, 0.0);
            std::cout << "任务5,回到原点,方向朝向 0.0" << std::endl;
            break;

        
        }
    }
   
    nav_msgs::msg::Odometry set_target(double x, double y, double theta){
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);  // 只设置 Z 轴方向的旋转（Yaw）
        // 设置 orientation
        nav_msgs::msg::Odometry target_odom;
        target_odom.pose.pose.orientation.x = q.x();
        target_odom.pose.pose.orientation.y = q.y();
        target_odom.pose.pose.orientation.z = q.z();
        target_odom.pose.pose.orientation.w = q.w();
        target_odom.header.stamp = this->get_clock()->now();
        target_odom.header.frame_id = "odom"; 

        // 示例：目标位置为 (1.0, 2.0)，方向朝向默认不变
        target_odom.pose.pose.position.x = x;
        target_odom.pose.pose.position.y = y;
        target_odom.pose.pose.position.z = 0.0;
        return target_odom;
    }
    
    double getYawFromOdom(const nav_msgs::msg::Odometry::SharedPtr& msg)
    {
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        return yaw;  // 返回的是弧度，单位为 rad
    }
   
    
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DispatchCar>());
    rclcpp::shutdown();
    return 0;
}
