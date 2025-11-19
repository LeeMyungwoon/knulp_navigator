cmake_minimum_required(VERSION 3.8)
project(keyboard_controller)

# 기본 설정
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 17)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 필요한 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)

# 실행파일 추가
add_executable(keyboard_ackermann src/keyboard_controller_node.cpp)
ament_target_dependencies(keyboard_ackermann rclcpp ackermann_msgs)

# 설치 설정
install(TARGETS
  keyboard_ackermann
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

