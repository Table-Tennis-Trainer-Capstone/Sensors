cmake_minimum_required(VERSION 3.8)
project(ttt_camera)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(camera_node src/camera_node.cpp)
target_link_libraries(camera_node ${OpenCV_LIBS})
ament_target_dependencies(camera_node 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  image_transport
)

install(TARGETS camera_node DESTINATION lib/${PROJECT_NAME})
ament_package()