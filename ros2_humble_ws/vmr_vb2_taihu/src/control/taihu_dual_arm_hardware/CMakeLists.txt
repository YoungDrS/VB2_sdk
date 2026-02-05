cmake_minimum_required(VERSION 3.8)
project(taihu_dual_arm_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(SRC_DIR EXP_SRC_DIR)

#compile
add_library(
  taihu_dual_arm_hardware 
  SHARED 
  ./src/taihu_dual_arm_hardware_interface.cpp
  # ./src/taihu_double_CAN_hardware_interface.cpp
)

# 检测平台并设置 libcontrolcan.so 路径
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "i686")
  set(CONTROLCAN_LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/x86/libcontrolcan.so)
  set(CONTROLCAN_LIB_INSTALL_PATH lib/x86/libcontrolcan.so)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(CONTROLCAN_LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm/libcontrolcan.so)
  set(CONTROLCAN_LIB_INSTALL_PATH lib/arm/libcontrolcan.so)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

#add headfile source
target_include_directories(taihu_dual_arm_hardware PUBLIC 
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/taihu_dual_arm_hardware>
)
target_link_libraries(taihu_dual_arm_hardware  
  ${CONTROLCAN_LIB_PATH}
)

#add dependencies
ament_target_dependencies(taihu_dual_arm_hardware 
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle 
)


#export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface taihu_dual_arm_hardware.xml)


#ros2_control hardware
install(TARGETS taihu_dual_arm_hardware
  EXPORT taihu_dual_arm_hardware
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# 安装动态库
install(FILES
  ${CONTROLCAN_LIB_INSTALL_PATH}
  DESTINATION lib
)


add_executable(taihu_motor_disable src/taihu_motor_disable.cpp)
ament_target_dependencies(taihu_motor_disable
  rclcpp
)
target_include_directories(taihu_motor_disable PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/)
target_link_libraries(taihu_motor_disable  
  ${CONTROLCAN_LIB_PATH}
)
install(TARGETS taihu_motor_disable
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(hardware_interface REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#export
ament_export_targets(taihu_dual_arm_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake rclcpp hardware_interface pluginlib rclcpp_lifecycle)

ament_package()
