cmake_minimum_required(VERSION 3.20)
project(camera_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)

# =========================================================================
# 1. Target: video_processor
# =========================================================================
add_executable(video_processor src/video_processor.cpp)
target_link_libraries(video_processor PRIVATE
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  cv_bridge::cv_bridge
  ament_index_cpp::ament_index_cpp
  ${OpenCV_LIBS}
  ${OpenCV_LIBRARIES}
)
target_include_directories(video_processor PRIVATE ${OpenCV_INCLUDE_DIRS})

# =========================================================================
# 2. Target: dashboard
# =========================================================================
add_executable(dashboard src/dashboard.cpp)
target_link_libraries(dashboard PRIVATE
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  cv_bridge::cv_bridge
  ament_index_cpp::ament_index_cpp
  ${OpenCV_LIBS}
  ${OpenCV_LIBRARIES}
)
target_include_directories(dashboard PRIVATE ${OpenCV_INCLUDE_DIRS})

# =========================================================================
# Install Rules
# =========================================================================
# Model directory
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

# ROS2 nodes
install(TARGETS
  video_processor
  dashboard
  DESTINATION lib/${PROJECT_NAME}
)

# =========================================================================
# Testing
# =========================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
