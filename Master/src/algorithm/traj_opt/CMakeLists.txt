cmake_minimum_required(VERSION 3.8)
project(traj_opt)
# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE "Release")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Python3 COMPONENTS Development NumPy REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(
    ${OsqpEigen_INCLUDE_DIRS}
)
include_directories(include)
include_directories(
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
)


add_library(mars_traj_opt src/mars_traj_opt.cpp)

add_executable(main_test test/main_test.cpp)
add_executable(traj_opt src/traj_opt.cpp)
add_executable(traj_test src/traj_test.cpp src/mars_traj_opt.cpp)

target_link_libraries(main_test
    OsqpEigen::OsqpEigen
)

target_link_libraries(traj_opt
    OsqpEigen::OsqpEigen
)

ament_target_dependencies(main_test 
rclcpp
std_msgs
nav_msgs
)

ament_target_dependencies(traj_opt 
rclcpp
std_msgs
nav_msgs
tf2
tf2_geometry_msgs
visualization_msgs
)


ament_target_dependencies(mars_traj_opt
 rclcpp
 visualization_msgs
)

ament_target_dependencies(traj_test 
  rclcpp
  std_msgs
  nav_msgs
  visualization_msgs
)

install(
  TARGETS main_test traj_opt traj_test
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
