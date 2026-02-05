#!/bin/bash

cd /home/aaeon/ros2_ws/ws_taihu_dual_arm/
source install/setup.bash
source /opt/ros/humble/setup.bash

ros2 run dual_arm_move pinocchio_expo_test
