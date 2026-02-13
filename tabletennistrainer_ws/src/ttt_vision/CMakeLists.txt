cmake_minimum_required(VERSION 3.8)
project(ttt_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ttt_msgs REQUIRED)
find_package(message_filters REQUIRED)

add_executable(ball_tracker_node src/ball_tracker_node.cpp)

ament_target_dependencies(ball_tracker_node
  rclcpp
  sensor_msgs
  cv_bridge
  ttt_msgs
  message_filters
)

install(TARGETS
  ball_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()