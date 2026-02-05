#!/bin/bash
set -euo pipefail

set +u
cd /home/aaeon/ros2_ws/ws_taihu_dual_arm/
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

LOGDIR=/home/aaeon/ros2_ws/ws_taihu_dual_arm/taihu_arm_start_logs
mkdir -p "$LOGDIR"

echo "[info] starting vb2_taihu_inspire_gripper_config demo_test, log: $LOGDIR/demo_test.log"
nohup ros2 launch vb2_taihu_inspire_gripper_config demo_test.launch.py >"$LOGDIR/demo_test.log" 2>&1 &
PID1=$!

sleep 50

echo "[info] starting dual_arm_move pinocchio_expo_test, log: $LOGDIR/arm_move_test.log"
nohup ros2 run dual_arm_move pinocchio_expo_test >"$LOGDIR/arm_move_test.log" 2>&1 &
PID2=$!

if ! kill -0 "$PID1" 2>/dev/null; then
  echo "[error] vb2_taihu_inspire_gripper_config exited early. check $LOGDIR/demo_test.log"
  exit 1
fi
if ! kill -0 "$PID2" 2>/dev/null; then
  echo "[error] dual_arm_move exited early. check $LOGDIR/arm_move_test.log"
  exit 1
fi

echo "PIDs: $PID1 $PID2"


