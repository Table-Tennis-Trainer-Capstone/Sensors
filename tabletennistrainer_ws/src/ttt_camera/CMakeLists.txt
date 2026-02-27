cmake_minimum_required(VERSION 3.8)
project(ttt_camera)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# OpenCV for GStreamer handling
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio)

add_executable(camera_node src/camera_node.cpp)

ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(camera_node ${OpenCV_LIBRARIES})

install(TARGETS
  camera_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()