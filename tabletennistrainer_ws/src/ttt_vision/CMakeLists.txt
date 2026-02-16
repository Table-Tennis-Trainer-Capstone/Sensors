cmake_minimum_required(VERSION 3.8)
project(ttt_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ttt_msgs REQUIRED)

# Find standard OpenCV
find_package(OpenCV REQUIRED)

# -------------------------------------------------------------
# 1. Ball Detector Node (Pure OpenCV)
# -------------------------------------------------------------
add_executable(ball_detector_node src/ball_detector_node.cpp)

# Link standard OpenCV include directories and libraries
target_include_directories(ball_detector_node PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ball_detector_node ${OpenCV_LIBS})

# Link ROS 2 packages
ament_target_dependencies(ball_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
  ttt_msgs
)

# -------------------------------------------------------------
# Install Executables
# -------------------------------------------------------------
install(TARGETS
  ball_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files/directories if you add them later
# install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
