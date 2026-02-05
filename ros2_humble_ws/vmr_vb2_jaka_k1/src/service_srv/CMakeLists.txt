cmake_minimum_required(VERSION 3.16)
project(service_srv)

# 查找必要的依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 声明服务文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ArmCommandService.srv"
  "srv/RunAction.srv"
)

# 安装 srv 文件到 share 目录
install(DIRECTORY
    include
    srv
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
