cmake_minimum_required(VERSION 3.8)
project(mpc_pkg)
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
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(
    ${OsqpEigen_INCLUDE_DIRS}
)
include_directories(include)

add_executable(mpc_test  test/mpc_test.cpp src/mpc_diff_car.cpp) 
add_executable(opt_traj_mpc  test/opt_traj_mpc.cpp src/mpc_diff_car.cpp) 

target_link_libraries(mpc_test
    OsqpEigen::OsqpEigen
)
target_link_libraries(opt_traj_mpc
    OsqpEigen::OsqpEigen
)

ament_target_dependencies(mpc_test 
rclcpp
std_msgs
nav_msgs
geometry_msgs
tf2_msgs
tf2
tf2_geometry_msgs
visualization_msgs
)
ament_target_dependencies(opt_traj_mpc 
rclcpp
std_msgs
nav_msgs
geometry_msgs
tf2_msgs
tf2
tf2_geometry_msgs
visualization_msgs
)

install(
  TARGETS mpc_test
  DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS opt_traj_mpc
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
