#!/bin/bash

cd /home/aaeon/ros2_ws/ws_taihu_dual_arm/
source install/setup.bash
source /opt/ros/humble/setup.bash

ros2 run taihu_dual_arm_hardware taihu_motor_zero
