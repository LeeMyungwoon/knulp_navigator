cmake_minimum_required(VERSION 3.8)
project(map_builder)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

add_executable(map_builder src/map_builder_node.cpp)
ament_target_dependencies(map_builder rclcpp sensor_msgs nav_msgs tf2_ros tf2)

install(TARGETS
  map_builder
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

