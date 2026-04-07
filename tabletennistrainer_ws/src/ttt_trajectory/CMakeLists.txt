cmake_minimum_required(VERSION 3.8)
project(ttt_trajectory)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(trajectory_node src/trajectory_node.cpp)
ament_target_dependencies(trajectory_node rclcpp geometry_msgs std_msgs)

install(TARGETS trajectory_node DESTINATION lib/${PROJECT_NAME})

ament_package()