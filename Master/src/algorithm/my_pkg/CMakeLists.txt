cmake_minimum_required(VERSION 3.5)

project(my_pkg)
set(CMAKE_BUILD_TYPE Debug)
# 默认使用 C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 查找 ROS2 的必要包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)

find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)




find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(model_push SHARED plugins/model_push.cc)

ament_target_dependencies(model_push
  rclcpp
  geometry_msgs
  std_msgs
)
target_link_libraries(model_push ${GAZEBO_LIBRARIES})



set(nlopt_DIR "${CMAKE_SOURCE_DIR}/nlopt/build")


find_package(nlopt REQUIRED)


#### 设置依赖库
set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  sensor_msgs
  tf2_geometry_msgs
  grid_map_ros
  grid_map_msgs
)
set(dependencies_laser
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  message_filters
)

#### 添加源文件并创建目标
add_executable(vis src/vis.cpp)
add_executable(mpc src/mpc.cpp)
add_executable(laser src/laser.cpp)
add_executable(real_state src/real_state.cpp)
add_executable(grid_apf src/grid_apf.cpp)
add_executable(A_star src/A_star.cpp include/my_pkg/AStar.cpp)
add_executable(cpp_test src/cpp_test.cpp)




# 设置包含头文件的路径



target_include_directories(A_star PUBLIC
  /home/humble2204/work/APF_MPC_ws/src/my_pkg/include/my_pkg
)

# 链接到所需的 ROS 2 包

ament_target_dependencies(vis  ${dependencies})
ament_target_dependencies(mpc  ${dependencies})
ament_target_dependencies(laser  ${dependencies_laser})
ament_target_dependencies(real_state  ${dependencies})
ament_target_dependencies(grid_apf  ${dependencies})
ament_target_dependencies(A_star  ${dependencies})


# 链接 nlopt 库
target_link_libraries(mpc nlopt)

#### 安装目标
install(TARGETS vis mpc laser real_state grid_apf A_star cpp_test
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件目录
install(DIRECTORY include/my_pkg/
  DESTINATION include/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY launch config rviz 
  DESTINATION share/${PROJECT_NAME}
)

# 安装 CMake 配置
ament_export_include_directories(include)

ament_export_dependencies(${dependencies})

# 处理测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
