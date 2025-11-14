cmake_minimum_required(VERSION 3.8)
project(local_planner)

# C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)

# OSQP 사용 여부 (선택)
option(USE_OSQP "Enable OSQP-based QP solver for MPC" ON)

# ------------------------------------------------------------------------------
# Executable
# ------------------------------------------------------------------------------

# 실제 경로 기준:
#   src/local_planner_node.cpp
#   src/trajectory_utils.cpp
#   src/costmap_handler.cpp
#   src/constraint_builder.cpp
#   src/local_mpc.cpp
add_executable(local_planner_node
  src/local_planner_node.cpp
  src/trajectory_utils.cpp
  src/costmap_handler.cpp
  src/constraint_builder.cpp
  src/local_mpc.cpp
)

# Test
add_executable(test_local_planner_feeder
  src/test_local_planner_feeder.cpp
)

# include 디렉토리
target_include_directories(local_planner_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ROS 의존성 연결
ament_target_dependencies(local_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
)

# Test
ament_target_dependencies(test_local_planner_feeder
  rclcpp
  nav_msgs
  geometry_msgs
)

# Eigen3 (헤더 전용이지만 target으로 걸어두면 깔끔)
target_link_libraries(local_planner_node
  Eigen3::Eigen
)

# ------------------------------------------------------------------------------
# OSQP 연동 (선택)
# ------------------------------------------------------------------------------
if(USE_OSQP)
  target_compile_definitions(local_planner_node PRIVATE USE_OSQP)

  # libosqp.so 찾기 (보통 /usr/local/lib 에 설치됨)
  find_library(OSQP_LIB osqp)

  if(OSQP_LIB)
    target_link_libraries(local_planner_node ${OSQP_LIB})
  else()
    message(WARNING "USE_OSQP is ON but libosqp not found. "
                    "osqp.h 는 컴파일 시 필요하고, 링크는 실패할 수 있습니다.")
  endif()
endif()

# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------
install(
  TARGETS local_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Test
install(TARGETS
  test_local_planner_feeder
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()

