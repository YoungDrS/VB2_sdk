#!/bin/bash

cd /home/aaeon/ros2_ws/ws_taihu_dual_arm/
source install/setup.bash
source /opt/ros/humble/setup.bash

ros2 launch vb2_taihu_inspire_gripper_config demo_test.launch.py

