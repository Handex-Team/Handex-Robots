// src/occupancy_to_gridmap.cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

class OccupancyToGridMapNode : public rclcpp::Node {
public:
  OccupancyToGridMapNode() : Node("occupancy_to_gridmap") {
    // 声明订阅QoS
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.transient_local();  // 设置为transient_local

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", qos, std::bind(&OccupancyToGridMapNode::mapCallback, this, std::placeholders::_1));

    // rclcpp::QoS qos(rclcpp::KeepLast(10));
    // qos.transient_local();  // 关键！加上这一句
    gridmap_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("/grid_map", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&OccupancyToGridMapNode::timerCallback, this));
  }

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<grid_map_msgs::msg::GridMap, std::default_delete<grid_map_msgs::msg::GridMap>> message;

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    const auto& info = msg->info;

    grid_map::GridMap map;
    map.setFrameId(msg->header.frame_id);
    map.setGeometry(grid_map::Length(info.width * info.resolution,
                                     info.height * info.resolution),
                    info.resolution,
                    grid_map::Position(
                      info.origin.position.x + 0.5 * info.width * info.resolution,
                      info.origin.position.y + 0.5 * info.height * info.resolution));

    map.add("occupancy");

    for (size_t y = 0; y < info.height; ++y) {
      for (size_t x = 0; x < info.width; ++x) {
        int index = y * info.width + x;
        float value = msg->data[index];
        
        // if (value < 0) value = NAN;
        // else value = value / 100.0f;  // Normalize to 0~1
        // std::cout << value << std::endl;
        // value = 50;
        grid_map::Index idx(info.height - 1 - y, x); // Flip Y for grid_map
        if (map.isValid(idx)) {
          map.at("occupancy", idx) = value;
          std::cout << "Occupancy: " << value << std::endl;
        }
      }
    }

    message = grid_map::GridMapRosConverter::toMessage(map);
    // gridmap_pub_->publish(*message);
    std::cout << map.getSize() << std::endl;
    RCLCPP_INFO(this->get_logger(), "OccupancyGrid converted to GridMap");
  }

  void timerCallback() {
    gridmap_pub_->publish(*message);
    RCLCPP_INFO(this->get_logger(), "GridMap published");
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyToGridMapNode>());
  rclcpp::shutdown();
  return 0;
}
