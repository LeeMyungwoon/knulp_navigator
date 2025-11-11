cmake_minimum_required(VERSION 3.8)
project(global_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_executable(global_planner_node
  src/global_planner_node.cpp
  src/global_planner.cpp
)

ament_target_dependencies(global_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS global_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

