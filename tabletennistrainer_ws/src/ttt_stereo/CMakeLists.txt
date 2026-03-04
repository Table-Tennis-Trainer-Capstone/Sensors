cmake_minimum_required(VERSION 3.8)
project(ttt_stereo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ttt_msgs REQUIRED)
find_package(message_filters REQUIRED)

add_executable(stereo_node src/stereo_node.cpp)
ament_target_dependencies(stereo_node
  rclcpp geometry_msgs ttt_msgs message_filters)

install(TARGETS stereo_node DESTINATION lib/${PROJECT_NAME})

ament_package()