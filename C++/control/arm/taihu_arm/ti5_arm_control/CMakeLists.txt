cmake_minimum_required(VERSION 3.10)
project(HumanoidDualArmSolver)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 头文件路径
include_directories(${CMAKE_SOURCE_DIR}/include)

# 源文件
set(SRC_FILES
    ${CMAKE_SOURCE_DIR}/src/arm_control.cpp
)

# 库文件路径
link_directories(${CMAKE_SOURCE_DIR}/lib)
link_directories(${CMAKE_SOURCE_DIR}/include/can)

# 可执行文件
add_executable(ti5_arm_control ${SRC_FILES})

# 链接动态库
target_link_libraries(ti5_arm_control
    mylibti5_arm_2004
    controlcan
    fmt
    udev
    pthread
)
