#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vector>
#include <cmath>
#include "AStar.hpp"

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>


double obstacle_limit = 0.1;  // 障碍物高度阈值
using namespace std::chrono_literals;
class AStarPlannerNode : public rclcpp::Node
{
public:
    AStarPlannerNode() : Node("astar_planner_node")
    {
        // 初始化地图订阅者
        map_subscription_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
            "grid_map_msgs", 10, std::bind(&AStarPlannerNode::mapCallback, this, std::placeholders::_1));
        

        // 发布路径的订阅者
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("A_star_planned_path", 10);

        map_pose_subscription_=this->create_subscription<nav_msgs::msg::Odometry>("/base_link_to_map_pose", 1, 
            std::bind(&AStarPlannerNode::mapPoseCallback, this, std::placeholders::_1));

        
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&AStarPlannerNode::goalCallback, this, std::placeholders::_1));
        
        timer_= this->create_wall_timer(0.1s, std::bind(&AStarPlannerNode::TimePlan, this));
        RCLCPP_INFO(this->get_logger(), "A* Planner Node Started");

        // 初始化A*算法
        generator.setWorldSize({200, 200});
        generator.setHeuristic(AStar::Heuristic::euclidean);
        generator.setDiagonalMovement(true);
        
        
        map_=std::make_shared<grid_map_msgs::msg::GridMap>();
    }

private:

    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr map_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr map_pose_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    AStar::Generator generator;
    grid_map_msgs::msg::GridMap::SharedPtr map_;
    // 设置起点和终点坐标 (map坐标系中的起点和终点)
    AStar::Vec2i start{0, 0};  // 起点坐标，示例值，需根据实际情况设定
    AStar::Vec2i goal{50, 120};  // 终点坐标，示例值，需根据实际情况设定

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal.x = (msg->pose.position.x - map_->info.pose.position.x) / map_->info.resolution;
        goal.y = (msg->pose.position.y - map_->info.pose.position.y) / map_->info.resolution;
        RCLCPP_INFO(this->get_logger(), "终点已设定");
        RCLCPP_INFO(this->get_logger(), "终点坐标: x: %d, y: %d", goal.x, goal.y);
    }

    void mapPoseCallback(const nav_msgs::msg::Odometry::SharedPtr map_pose)
    {
        start.x = (map_pose->pose.pose.position.x - map_->info.pose.position.x) / map_->info.resolution;
        start.y = (map_pose->pose.pose.position.y - map_->info.pose.position.y) / map_->info.resolution;
        // // 终点坐标转换到地图坐标系
        // goal.x = (map_pose->pose.pose.position.x - map_->info.origin.position.x) / map_->info.resolution;
        // goal.y = (map_pose->pose.pose.position.y - map_->info.origin.position.y) / map_->info.resolution;
        RCLCPP_INFO(this->get_logger(), "起点坐标: x: %d, y: %d", start.x, start.y);
    }
    void TimePlan(){

        // 地图信息
        int map_width = map_->info.length_x/map_->info.resolution;
        int map_height = map_->info.length_y/map_->info.resolution;
        float resolution = map_->info.resolution;
        auto origin = map_->info.pose.position;

        // 设置新的地图大小
        generator.setWorldSize({map_width, map_height});
        generator.setHeuristic(AStar::Heuristic::euclidean);  // 重新设置启发式方法
        generator.setDiagonalMovement(true);  // 重新设置对角线移动
        // 将地图中的障碍物坐标添加到A*算法中
        generator.setWorldSize({map_width, map_height});
        

        // 寻找路径
        auto start_time = std::chrono::high_resolution_clock::now();

        // 打印起点和终点坐标
        
        auto api_start=start,api_goal=goal;
        api_start.x=start.x+(map_->info.length_x/map_->info.resolution)/2;
        api_start.y=start.y+(map_->info.length_y/map_->info.resolution)/2;
        api_goal.x=goal.x+(map_->info.length_x/map_->info.resolution)/2;
        api_goal.y=goal.y+(map_->info.length_y/map_->info.resolution)/2;
        // RCLCPP_INFO(this->get_logger(), "符合A*API的起点坐标: x: %d, y: %d", api_start.x, api_start.y);
        // RCLCPP_INFO(this->get_logger(), "符合A*API的终点坐标: x: %d, y: %d", api_goal.x, api_goal.y);
        auto path = generator.findPath(api_start, api_goal);
        AStar::CoordinateList api_path;
        for (auto& coordinate : path){
            api_path.push_back(AStar::Vec2i{coordinate.x-int(map_->info.length_x/map_->info.resolution)/2,
             coordinate.y-int(map_->info.length_y/map_->info.resolution)/2});
        }
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        if(api_path.size() == 1){
            RCLCPP_INFO(this->get_logger(), "A* 搜索路径失败");
        }
        else RCLCPP_INFO(this->get_logger(), "A* 路径搜索成功, 耗时: %d ms", duration);
        

        // 将路径从栅格坐标系映射到真实坐标系
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";  // 发布到map坐标系下
        path_msg.header.stamp = this->get_clock()->now();  // 设置时间戳
        for (const auto& coordinate : api_path) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.pose.position.x = origin.x + coordinate.x * resolution;  // 映射为实际坐标
            pose_stamped.pose.position.y = origin.y + coordinate.y * resolution;
            pose_stamped.pose.position.z = 0.0;  // 假设地图是二维的，z坐标为0
            pose_stamped.header.frame_id = "map";  // 坐标系
            pose_stamped.header.stamp = this->get_clock()->now();  // 设置时间戳
            path_msg.poses.push_back(pose_stamped);
        }
 
        // 发布路径
        path_publisher_->publish(path_msg);

        // 输出路径坐标
        // for (auto& coordinate : path) {
        //     RCLCPP_INFO(this->get_logger(), "Path: x: %d, y: %d", coordinate.x, coordinate.y);
        // }

        // 输出路径长度
        RCLCPP_INFO(this->get_logger(), "Path Length: %d", path.size());

    }


    void mapCallback(const grid_map_msgs::msg::GridMap::SharedPtr map)
    {   
        // 更新地图信息
        *map_ = *map;
        // 每次回调时，重新初始化 A* 生成器
        generator = AStar::Generator();  // 清除旧的生成器状态
        
        // 将消息类型的 GridMap 转换为 grid_map::GridMap
        grid_map::GridMap grid_map;
        grid_map::GridMapRosConverter::fromMessage(*map, grid_map);  // 使用 grid_map_ros 的转换函数

        // 旋转栅格地图再使用
        for (const auto& layer : grid_map.getLayers()) {
            grid_map[layer] = grid_map[layer].colwise().reverse().rowwise().reverse();
        }

        // 获取地图尺寸和分辨率
        // int map_width = map->info.length_x;
        // int map_height = map->info.length_y;
        // float resolution = map->info.resolution;
        // auto origin = map->info.pose.position;
        int count_obs = 0;  // 障碍物个数

        // 遍历地图并添加障碍物（高度超过50视为障碍物）
        for (grid_map::GridMapIterator it(grid_map); !it.isPastEnd(); ++it) {
            // 获取当前栅格的索引
            grid_map::Index index(*it);

            // 获取当前 (x, y) 位置的高度值
            double map_point = grid_map.at("elevation", index);  // 获取 "elevation" 图层的值
            
            if (map_point > obstacle_limit) {  // 设定为超过阈值的高度为障碍物
                generator.addCollision(AStar::Vec2i{index(0), index(1)});
                // RCLCPP_INFO(this->get_logger(), "障碍物: x: %d, y: %d", index(0), index(1));
                // std::cout << "map_point: " << map_point << std::endl;
                count_obs++;
            }
        }

        RCLCPP_INFO(this->get_logger(), "A*障碍物个数: %d", count_obs);
    }


   
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AStarPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
