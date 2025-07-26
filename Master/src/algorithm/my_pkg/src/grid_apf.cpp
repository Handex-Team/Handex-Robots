#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include <vector>
#include <cmath>
#include "nav_msgs/msg/odometry.hpp"


#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_ros/buffer.h"

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>


using namespace std::placeholders;
// 设定步长
double step_size = 0.01;
int max_iterations = 1000; // 最大迭代次数
double distance_min = 0.3; // 最小距离
double k_rep = 0.02;  // 排斥力系数
double influence_distance = 5.0; // 设置影响距离
double k_att = 1.0; // 吸引力系数
double slope_threshold_max=100.0,slope_threshold_min=1.0;
double influence_distance_dynamic = 20.0; // 设置动态障碍物影响距离


using namespace std::chrono_literals;
class APFPathPlanner : public rclcpp::Node
{
public:
    APFPathPlanner() : Node("apf_path_planner"){
        // 坐标变换
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        // 创建地图订阅者
        map_sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
            "grid_map_msgs", 10, std::bind(&APFPathPlanner::mapCallback, this, std::placeholders::_1));
        
        // 创建路径发布者
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 1);

        odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/my_sim_ground_truth_pose", 1,
        std::bind(&APFPathPlanner::odometry_callback, this, _1));

       
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&APFPathPlanner::goalCallback, this, std::placeholders::_1));

        map_msg_ = std::make_shared<grid_map_msgs::msg::GridMap>();
        timer_path_planning_ = this->create_wall_timer(0.01s, std::bind(&APFPathPlanner::pathPlanning, this));


        // 动态障碍物位置
        dynamic_obstacle_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/model_pose/dynamic_obstacle", 10, std::bind(&APFPathPlanner::dynamicObstacleCallback, this, std::placeholders::_1));
        
        goal_pose_.pose.position.x = 0.0;
        goal_pose_.pose.position.y = 0.0;
        start_pose_.pose.position.x = 0.0;
        start_pose_.pose.position.y = 0.0;
    }

private:

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr dynamic_obstacle_sub_;
    // 地图尺寸和数据
    grid_map_msgs::msg::GridMap::SharedPtr map_msg_;
    grid_map::GridMap grid_map;

    // 坐标变换
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    

    // 起始位置和目标位置
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    geometry_msgs::msg::PoseStamped start_pose_;
    geometry_msgs::msg::PoseStamped goal_pose_;
    nav_msgs::msg::Odometry real_pose_;
    rclcpp::TimerBase::SharedPtr timer_path_planning_;
    geometry_msgs::msg::Pose dynamic_obstacle_pose_;// 动态障碍物位置
    

    void dynamicObstacleCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        dynamic_obstacle_pose_ = *msg;
    }

    void pathPlanning()
    {
        const geometry_msgs::msg::Point::SharedPtr start = std::make_shared<geometry_msgs::msg::Point>();
        start->x = start_pose_.pose.position.x;
        start->y = start_pose_.pose.position.y;

        const geometry_msgs::msg::Point::SharedPtr goal = std::make_shared<geometry_msgs::msg::Point>();
        goal->x = goal_pose_.pose.position.x;
        goal->y = goal_pose_.pose.position.y;

        RCLCPP_INFO(this->get_logger(), "起点位置 (map): [%.2f, %.2f],障碍物位置 (map): [%.2f, %.2f]", start_pose_.pose.position.x, start_pose_.pose.position.y,
                        dynamic_obstacle_pose_.position.x, dynamic_obstacle_pose_.position.y);

        // 计算路径
        auto start_time = std::chrono::high_resolution_clock::now();
        std::vector<geometry_msgs::msg::Point> path = calculatePath(*start, *goal);
        auto end_time = std::chrono::high_resolution_clock::now();
        RCLCPP_INFO(this->get_logger(), "规划耗时为: %f ms", std::chrono::duration<double, std::milli>(end_time - start_time).count());


        // 创建并发布路径
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->get_clock()->now();
        path_msg.header.frame_id = "map";  // 设置路径的坐标系为 map

        // 遍历路径并将每个点转换为 PoseStamped
        for (const auto& point : path)
        {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = path_msg.header.stamp;
            pose_stamped.header.frame_id = "map";  // 保证每个 PoseStamped 均在 map 坐标系中

            pose_stamped.pose.position = point;
            pose_stamped.pose.orientation.w = 1.0;  // 假设路径点是平面上的点，旋转角度设置为 0（默认为单位四元数）

            path_msg.poses.push_back(pose_stamped);
        }

        // 发布路径
        path_pub_->publish(path_msg);

        // try
        // {
        //     // 获取 map -> odom 变换
        //     geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform("odom", "map", tf2::TimePointZero);

        //     for (const auto &point : path)
        //     {
        //         geometry_msgs::msg::PoseStamped map_pose, odom_pose;
        //         map_pose.header.frame_id = "map";
        //         map_pose.pose.position = point;

        //         // 将路径点从 map 坐标转换到 odom 坐标
        //         tf2::doTransform(map_pose, odom_pose, transform_stamped);
        //         odom_pose.header.stamp = path_msg.header.stamp;
        //         odom_pose.header.frame_id = "odom";

        //         path_msg.poses.push_back(odom_pose);
        //     }

        //     path_pub_->publish(path_msg);  // 发布路径
        // }
        // catch (tf2::TransformException &ex)
        // {
        //     RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
        // }
    }


    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        real_pose_ = *msg;  // 更新机器人的当前位置信息
        try
        {
            // 获取 odom 到 map 的变换
            geometry_msgs::msg::TransformStamped transform_stamped =
                tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);

            // 将 odom 坐标转换到 map 坐标
            geometry_msgs::msg::PoseStamped odom_pose;
            odom_pose.header = msg->header;
            odom_pose.pose = msg->pose.pose;

            geometry_msgs::msg::PoseStamped map_pose;
            tf2::doTransform(odom_pose, map_pose, transform_stamped);

            // 更新起点位置为 map 坐标下的位置
            start_pose_.pose.position.x = map_pose.pose.position.x;
            start_pose_.pose.position.y = map_pose.pose.position.y;

            RCLCPP_INFO(this->get_logger(), "起点位置 (map): [%.2f, %.2f]",
                        start_pose_.pose.position.x, start_pose_.pose.position.y);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
        }

    }


    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal_pose_ = *msg;
        
        // try
        // {
        //     // 获取 odom 到 map 的变换
        //     geometry_msgs::msg::TransformStamped transform_stamped =
        //         tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);

        //     // 将 goal_pose 的坐标从 odom 转换到 map
        //     geometry_msgs::msg::PoseStamped odom_goal_pose;
        //     odom_goal_pose.header = msg->header;
        //     odom_goal_pose.pose = msg->pose;

        //     geometry_msgs::msg::PoseStamped map_goal_pose;
        //     tf2::doTransform(odom_goal_pose, map_goal_pose, transform_stamped);

        //     // 更新目标位置为 map 坐标下的位置
        //     goal_pose_.pose.position.x = map_goal_pose.pose.position.x;
        //     goal_pose_.pose.position.y = map_goal_pose.pose.position.y;

        //     RCLCPP_INFO(this->get_logger(), "目标位置 (map): [%.2f, %.2f]",
        //                 goal_pose_.pose.position.x, goal_pose_.pose.position.y);
        // }
        // catch (tf2::TransformException &ex)
        // {
        //     RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
        // }
    }


    void mapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
    {
        // 获取栅格地图的尺寸和数据
        *map_msg_ = *msg;
        // 转换成后面要用的栅格地图
        grid_map::GridMapRosConverter::fromMessage(*msg, grid_map);
    }

    // 计算路径的函数
    std::vector<geometry_msgs::msg::Point> calculatePath(const geometry_msgs::msg::Point &start, const geometry_msgs::msg::Point &goal)
    {
        std::vector<geometry_msgs::msg::Point> path;
        path.push_back(start);
        // 初始位置
        geometry_msgs::msg::Point current_position = start;

        
        
        int iter=0; // 迭代次数
        // 循环直到机器人到达目标
        while (std::sqrt(std::pow(current_position.x - goal.x, 2) + std::pow(current_position.y - goal.y, 2)) > 0.1)
        {   
            if(iter>max_iterations)
            {
                RCLCPP_ERROR(this->get_logger(), "规划失败，超出最大迭代次数");
                iter=0;
                return path;
            }
            iter++;
            // 计算总的势场（目标的吸引力 + 障碍物的排斥力）
            geometry_msgs::msg::Point force = calculateTotalForce(current_position, goal);

            // 更新位置
            current_position.x += force.x * step_size;
            current_position.y += force.y * step_size;

            // 保存路径点
            path.push_back(current_position);
        }

        return path;
    }

    // 计算目标的吸引力
    geometry_msgs::msg::Point calculateAttractiveForce(const geometry_msgs::msg::Point &current_position, const geometry_msgs::msg::Point &goal)
    {
        geometry_msgs::msg::Point force;
        
        force.x = k_att * (goal.x - current_position.x);
        force.y = k_att * (goal.y - current_position.y);
        return force;
    }

    // 计算障碍物的排斥力
    geometry_msgs::msg::Point calculateRepulsiveForce(const geometry_msgs::msg::Point &current_position)
    {
        geometry_msgs::msg::Point force;
        // 初始化排斥力
        force.x = 0.0;
        force.y = 0.0;

        // 计算动态障碍物的斥力影响
        double dynamic_obstacle_dist_x = dynamic_obstacle_pose_.position.x - current_position.x;
        double dynamic_obstacle_dist_y = dynamic_obstacle_pose_.position.y - current_position.y;
        double dynamic_obstacle_distance = std::hypot(dynamic_obstacle_dist_x, dynamic_obstacle_dist_y);
        dynamic_obstacle_distance = std::max(dynamic_obstacle_distance, distance_min); // 防止除零错误
        
        // RCLCPP_INFO(this->get_logger(), "动态障碍物距离: %.2f,机器人横坐标: %.2f,障碍物横坐标: %.2f", dynamic_obstacle_distance,
        //  current_position.x, dynamic_obstacle_pose_.position.x);
        // 如果距离小于影响范围，计算排斥力
        if (dynamic_obstacle_distance < influence_distance_dynamic) {
            double force_magnitude = 100.0*k_rep * std::pow((1.0 / dynamic_obstacle_distance),2);
            force.x += force_magnitude * (dynamic_obstacle_dist_x / dynamic_obstacle_distance);
            force.y += force_magnitude * (dynamic_obstacle_dist_y / dynamic_obstacle_distance);
            // RCLCPP_INFO(this->get_logger(), "动态障碍物斥力影响: [%.2f, %.2f],force_magnitude: %.2f", force.x, force.y,force_magnitude);
        }

        // 正确计算栅格数
        const double resolution = grid_map.getResolution();
        const int map_width_ = grid_map.getSize()(0); 
        const int map_height_ = grid_map.getSize()(1);
        // 获取当前位置在栅格地图中的位置
        // int x_idx = static_cast<int>((current_position.x - map_msg_->info.pose.position.x) / map_msg_->info.resolution);
        // int y_idx = static_cast<int>((current_position.y - map_msg_->info.pose.position.x) / map_msg_->info.resolution);
        grid_map::Position world_position(current_position.x, current_position.y);
        grid_map::Index grid_index;
         if (grid_map.getIndex(world_position, grid_index)) {
            // std::cout << "栅格索引: (" << grid_index(0) << ", " << grid_index(1) << ")" << std::endl;
            // std::cout << "当前实际位置" << world_position<< std::endl;
        } else {
            std::cout << "路径规划失败，起始位置超出地图范围" << std::endl;
        }


        
        
        // 获取当前位置的高度值
        double current_height = grid_map.atPosition("elevation", world_position);

        // 获取当前位置的影响范围
        auto count_influence_grid = static_cast<int>(influence_distance / map_msg_->info.resolution);
        int x_min = std::max(0, grid_index(0) - count_influence_grid);
        int x_max = std::min(map_width_ - 1, grid_index(0) + count_influence_grid);
        int y_min = std::max(0, grid_index(1) - count_influence_grid);
        int y_max = std::min(map_height_ - 1, grid_index(1) + count_influence_grid);

        double dist_x, dist_y, distance, force_magnitude;

        // 遍历影响范围内的栅格
        for (int i = y_min; i <= y_max; ++i)
        {
            for (int j = x_min; j <= x_max; ++j)
            {
                // 获取当前栅格的高度值
                // auto height = map_msg_->data[i * map_width_ + j];
                auto height = grid_map.at("elevation", grid_map::Index(j, i));
                // 计算当前位置和当前栅格之间的水平距离
                grid_map::Index focus_index(j, i);
                grid_map::Position focus_position;
                grid_map.getPosition(focus_index, focus_position);
                dist_x = focus_position.x() - current_position.x;
                dist_y = focus_position.y() - current_position.y;
                distance = std::sqrt(dist_x * dist_x + dist_y * dist_y);
                distance = std::max(distance, distance_min); // 防止除零错误
                auto dealt_height=std::abs(height-current_height);
                auto abs_slope=std::abs(dealt_height/distance);
                if (abs_slope>slope_threshold_min )
                {
                    // 根据高度值计算排斥力大小
                    force_magnitude = 100.0 * k_rep * std::pow((1.0 / distance), 3) * (abs_slope-slope_threshold_min)/slope_threshold_max;
                    // 将排斥力分量加到总力上
                    force.x += force_magnitude * (dist_x / distance);
                    force.y += force_magnitude * (dist_y / distance);
                }
            }
        }

        return force;
    }




    // 计算总的势场（吸引力 + 排斥力）
    geometry_msgs::msg::Point calculateTotalForce(const geometry_msgs::msg::Point &current_position, const geometry_msgs::msg::Point &goal)
    {
        geometry_msgs::msg::Point attractive_force = calculateAttractiveForce(current_position, goal);
        geometry_msgs::msg::Point repulsive_force = calculateRepulsiveForce(current_position);
        
        geometry_msgs::msg::Point total_force;
        total_force.x = attractive_force.x - repulsive_force.x;
        total_force.y = attractive_force.y - repulsive_force.y;
        
        return total_force;
    }

    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<APFPathPlanner>());
    rclcpp::shutdown();
    return 0;
}
