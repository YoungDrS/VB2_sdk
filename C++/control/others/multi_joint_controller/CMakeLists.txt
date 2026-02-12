cmake_minimum_required(VERSION 3.8)
project(multi_joint_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lx_motor_interfaces REQUIRED)

add_executable(multi_joint_controller_node src/multi_joint_controller_node.cpp)
ament_target_dependencies(multi_joint_controller_node rclcpp lx_motor_interfaces)

install(TARGETS
  multi_joint_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
