#!/bin/bash

stopProcess(){
	ret=`ps aux | grep $1 | grep -v grep | awk '{print $2}'`
	if [ -n "$ret" ];then
		kill -9 $ret >/dev/null
		if [[ $? == 0 ]]; then
			printf "Process  \033[1;36m%-15s\033[0m %s\n"  "$1" "Kill success!"
		else
			printf "Process  \033[1;33m%-15s\033[0m %s\n"  "$1" "Kill Failed!"
		fi
	else
		printf "Process  \033[1;34m%-15s\033[0m %s\n"  "$1" "Not exsit!"
	fi
}

stopProcess taihu
stopProcess yunxiang_taihu_revo2_config
stopProcess pinocchio_yunxiang_test
stopProcess demo_test

cd /home/aaeon/ros2_ws/ws_taihu_dual_arm/
source /opt/ros/humble/setup.bash
source install/setup.bash

# 机械臂下使能
ros2 run taihu_dual_arm_hardware taihu_motor_disable  


