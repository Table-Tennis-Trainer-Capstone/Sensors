cmake_minimum_required(VERSION 3.8)
project(ttt_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ttt_msgs REQUIRED)

# Find OpenCV with CUDA components
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio cudafilters cudaimgproc cudawarping cudaarithm)

add_executable(vision_node src/vision_node.cpp)

# Target dependencies
ament_target_dependencies(vision_node
  rclcpp
  sensor_msgs
  cv_bridge
  ttt_msgs
)

# Link OpenCV (including CUDA libs)
target_link_libraries(vision_node ${OpenCV_LIBRARIES})

install(TARGETS
  vision_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()