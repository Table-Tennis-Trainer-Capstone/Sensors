cmake_minimum_required(VERSION 3.22)
project(ttt_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(control_node src/control_node.cpp)
ament_target_dependencies(control_node
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf    DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes  DESTINATION share/${PROJECT_NAME})

ament_package()
