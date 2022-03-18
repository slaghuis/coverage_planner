cmake_minimum_required(VERSION 3.5)
project(coverage_planner)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(coverage_planner_interfaces REQUIRED)
find_package(navigation_interfaces REQUIRED)
find_package(camera_lite_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(coverage_planner_node 
  src/coverage_planner_node.cpp
  src/edge.cpp)
ament_target_dependencies(coverage_planner_node
  "rclcpp"
  "rclcpp_action"
  "geometry_msgs"
  "coverage_planner_interfaces"
  "tf2"
  "tf2_ros")
target_include_directories(coverage_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(photogrammetry_node 
  src/photogrammetry_node.cpp)
ament_target_dependencies(photogrammetry_node 
  "rclcpp"
  "rclcpp_action"
  "geometry_msgs"
  "coverage_planner_interfaces"
  "navigation_interfaces"
  "camera_lite_interfaces")

add_executable(camera_model_node 
  src/camera_model_node.cpp)
ament_target_dependencies(camera_model_node 
  "rclcpp"
  "coverage_planner_interfaces" )


install(TARGETS 
  coverage_planner_node
  photogrammetry_node
  camera_model_node
  DESTINATION lib/${PROJECT_NAME})
  
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)   

ament_package()
