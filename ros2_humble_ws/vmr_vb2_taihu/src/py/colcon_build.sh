#!/bin/bash

# 定义工作空间路径
WORKSPACE_DIR="/home/jing/lanxin/ros2_test/ws_vb2_taihu"
# WORKSPACE_DIR="/home/aaeon/ros2_ws/ws_taihu_dual_arm"

# 检查并进入工作空间目录
if [ -d "$WORKSPACE_DIR" ]; then
    cd "$WORKSPACE_DIR" || exit
else
    echo "Error: Workspace directory $WORKSPACE_DIR not found."
    echo "Please run this script from the workspace root or update the WORKSPACE_DIR variable."
    exit 1
fi

echo "Current directory: $(pwd)"
echo "Starting colcon build..."
echo "Ignored packages: brainco_control"

cd "$WORKSPACE_DIR" 

# 先编译接口包
echo "Step 1: Building interfaces (lx_motor_interfaces, service_srv, service_interfaces)..."
colcon build --packages-select service_srv
colcon build --packages-select lx_motor_interfaces
colcon build --packages-select service_interfaces

source install/setup.bash

# 执行编译
# --packages-ignore: 指定不编译的包
colcon build --packages-ignore brainco_control

# 检查编译结果
if [ $? -eq 0 ]; then
    echo -e "\033[1;32mBuild success!\033[0m"
    
    # 尝试加载环境（注意：这只在当前脚本执行期间有效，退出脚本后失效）
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
    fi
else
    echo -e "\033[1;31mBuild failed!\033[0m"
    exit 1
fi
