cmake_minimum_required(VERSION 3.8)
project(rrt_star)
set(CMAKE_BUILD_TYPE Debug)
# 找到必要的包
find_package(ament_cmake REQUIRED)
find_package(ompl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)


# 添加可执行文件

add_executable(rrt_star src/rrt_star.cpp)

set(dependencies
  rclcpp
  nav_msgs
  octomap
  geometry_msgs
  grid_map_ros
  grid_map_msgs
)

# 链接 OMPL 库

target_link_libraries(rrt_star ompl)

ament_target_dependencies(rrt_star  ${dependencies})

# 添加包含目录


target_include_directories(rrt_star PRIVATE
  ${OMPL_INCLUDE_DIRS}
)
# 安装可执行文件
install(TARGETS
rrt_star
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
