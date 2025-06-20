cmake_minimum_required(VERSION 3.16)
project(can_gateway)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(can_msgs REQUIRED)

add_executable(can_gateway_node
  src/can_gateway_node.cpp
  src/main.cpp 
)

target_include_directories(can_gateway_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(can_gateway_node
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  can_msgs
)

install(
  TARGETS can_gateway_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()