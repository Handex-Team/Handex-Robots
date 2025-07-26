#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>  // RRT* 规划器头文件
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace std::chrono_literals;

class ObstacleChecker : public ob::StateValidityChecker
{
public:
    ObstacleChecker(const ob::SpaceInformationPtr &si, const std::vector<std::pair<double, double>> &obstacles, double safety_threshold)
        : ob::StateValidityChecker(si), obstacles_(obstacles), safety_threshold_(safety_threshold) {}

    bool isValid(const ob::State *state) const override
    {
        const auto *realState = state->as<ob::RealVectorStateSpace::StateType>();
        double x = realState->values[0];
        double y = realState->values[1];

        // 判断当前点到最近障碍物的距离
        for (const auto &obstacle : obstacles_)
        {
            double obs_x = obstacle.first, obs_y = obstacle.second;
            
            // 计算点到障碍物的欧氏距离
            double distance = std::sqrt((obs_x - x) * (obs_x - x) + (obs_y - y) * (obs_y - y));

            // 如果距离小于安全阈值，认为碰撞
            if (distance < safety_threshold_)
            {
                return false;
            }
        }
        return true;
    }

private:
    std::vector<std::pair<double, double>> obstacles_;  // 存储障碍物的坐标
    double safety_threshold_;  // 安全阈值
};

// 自定义的障碍物处理类，用于根据地图数据生成障碍物列表
class MapObstacleGenerator
{
public:
    MapObstacleGenerator(double height_threshold, double safety_threshold)
        : height_threshold_(height_threshold), safety_threshold_(safety_threshold) {}

    void generateObstacles(const grid_map_msgs::msg::GridMap::SharedPtr grid_map)
    {
        obstacles_.clear();
        double resolution = grid_map->info.resolution;  // 栅格的分辨率
        double origin_x = grid_map->info.pose.position.x;  // 地图原点的x坐标
        double origin_y = grid_map->info.pose.position.y;  // 地图原点的y坐标

        grid_map::GridMap new_grid_map;
        grid_map::GridMapRosConverter::fromMessage(*grid_map, new_grid_map); 

        for (grid_map::GridMapIterator iterator(new_grid_map); !iterator.isPastEnd(); ++iterator)
    {
        const grid_map::Index index = *iterator;  // 当前栅格的索引
        if (new_grid_map.at("elevation", index) > height_threshold_)  // 判断是否为障碍物
        {
            grid_map::Position position;
            new_grid_map.getPosition(index, position);  // 获取实际坐标

            // 将障碍物坐标存入列表
            obstacles_.push_back({position.x(), position.y()});
        }
    }
    }


    const std::vector<std::pair<double, double>> &getObstacles() const { return obstacles_; }

private:
    double height_threshold_;
    double safety_threshold_;
    std::vector<std::pair<double, double>> obstacles_;
};

class PathPlannerNode : public rclcpp::Node
{
public:
    PathPlannerNode() : Node("path_planner")
    {
        // 初始化订阅地图话题
        map_subscription_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
            "/grid_map_msgs", 10, std::bind(&PathPlannerNode::mapCallback, this, std::placeholders::_1));

        map_pose_subscription_=this->create_subscription<nav_msgs::msg::Odometry>("/base_link_to_map_pose", 1, 
            std::bind(&PathPlannerNode::mapPoseCallback, this, std::placeholders::_1));

        // 设置高度阈值和安全阈值
        map_obstacle_generator_ = std::make_shared<MapObstacleGenerator>(0.5, 0.5); // 高度阈值30，安全阈值1

        // 初始化路径发布器
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/rrt_star_path", 10);

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&PathPlannerNode::goalCallback, this, std::placeholders::_1));

        map_=std::make_shared<grid_map_msgs::msg::GridMap>();
        global_start[0] = -12.0;
        global_start[1] = -12.0;
        global_goal[0] = 0.0;
        global_goal[1] = 0.0;
    }

private:

    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr map_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr map_pose_subscription_;
    std::shared_ptr<MapObstacleGenerator> map_obstacle_generator_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

    grid_map_msgs::msg::GridMap::SharedPtr map_;
    

    double global_start[2], global_goal[2];



    void mapPoseCallback(const nav_msgs::msg::Odometry::SharedPtr map_pose){
        global_start[0] = map_pose->pose.pose.position.x;
        global_start[1] = map_pose->pose.pose.position.y ;
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        global_goal[0] = msg->pose.position.x;
        global_goal[1] = msg->pose.position.y;
    }

    void mapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
    {
        // 根据接收到的地图生成障碍物
        map_obstacle_generator_->generateObstacles(msg);
        auto obstacles = map_obstacle_generator_->getObstacles();
        RCLCPP_INFO(this->get_logger(), "有 %d个障碍物 ", obstacles.size());

        // 获取地图的实际尺寸
        double map_width = msg->info.length_x;   // 地图宽度（实际尺寸，单位：米）
        double map_height = msg->info.length_y; // 地图高度（实际尺寸，单位：米）

        // 初始化OMPL空间和规划器
        auto space = std::make_shared<ob::RealVectorStateSpace>(2);
        ob::RealVectorBounds bounds(2);
        
        // 设置边界为地图的真实尺寸
        bounds.setLow(0, msg->info.pose.position.x-map_width/2);                // 设置x方向的最小值（地图原点x坐标）
        bounds.setHigh(0, msg->info.pose.position.x + map_width/2);    // 设置x方向的最大值
        bounds.setLow(1, msg->info.pose.position.y - map_height/2);                // 设置y方向的最小值（地图原点y坐标）
        bounds.setHigh(1, msg->info.pose.position.y + map_height/2);   // 设置y方向的最大值

        space->setBounds(bounds);

        og::SimpleSetup ss(space);

        // 设置障碍物检测
        auto obstacleChecker = std::make_shared<ObstacleChecker>(ss.getSpaceInformation(), obstacles, 0.2);  // 安全阈值设为0.2
        ss.setStateValidityChecker(obstacleChecker);

        // 设置起点和目标点
        ob::ScopedState<> start(space);
        start[0] = global_start[0];
        start[1] = global_start[1];

        ob::ScopedState<> goal(space);
        goal[0] = global_goal[0];
        goal[1] = global_goal[1];

        ss.setStartAndGoalStates(start, goal);

        // 设置RRT*规划器
        ss.setPlanner(std::make_shared<og::RRTstar>(ss.getSpaceInformation()));

        // 执行路径规划
        rclcpp::Clock clock(RCL_SYSTEM_TIME);
        auto now_time = clock.now(); // 当前时间
        if (ss.solve(0.1))
        {
            std::cout << "Found solution:" << std::endl;
            
            
            ss.simplifySolution();
            ss.getSolutionPath().print(std::cout);

            // 发布路径
            nav_msgs::msg::Path rrt_star_path;
            rrt_star_path.header.frame_id = "map";
            rrt_star_path.header.stamp = now_time;
            for (const auto &state : ss.getSolutionPath().getStates())
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
                pose.pose.position.y = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
                rrt_star_path.poses.push_back(pose);
            }
            path_publisher_->publish(rrt_star_path);
        }
        else
        {
            std::cout << "No solution found." << std::endl;
        }
    }


    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}