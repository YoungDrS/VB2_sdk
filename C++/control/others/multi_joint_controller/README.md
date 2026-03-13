# Multi-Joint Control (ROS2 C++)

A minimal ROS2 C++ control package: `multi_joint_controller`.

> **⚠️ Architecture Note**
> All hardware devices within the `others` directory are directly managed by the chassis controller. Consequently, the main control unit (Orin) cannot access their low-level hardware interfaces directly and must control these devices via ROS 2 communication.

## Control Targets (`control_target`)

* **`waistd` (or `waist`)**
  * Control Topic: `/vmr/waist_joint_control`
  * State Topic: `/vmr/waist_joint_state`
  * Control Strategy: Stepping control + soft start + deceleration upon approaching the target + exit upon reaching the position.
* **`rotatehead`**
  * Control Topic: `/vmr/head_joint_control_1`
  * State Topic: `/vmr/head_joint_state_1`
  * Control Strategy: Stepping control + soft start + deceleration upon approaching the target + exit upon reaching the position.
* **`nodhead`**
  * Control Topic: `/vmr/head_joint_control_2`
  * State Topic: `/vmr/head_joint_state_2`
  * Control Strategy: Stepping control + soft start + deceleration upon approaching the target + exit upon reaching the position.
* **`raise`**
  * Control Topic: `/vmr/raise_control`
  * State Topic: `/vmr/raise_state`
  * Control Strategy: Directly sends the target point command without stepping control.
  * Constraint: The target value only uses `target_position[0]` and is strictly clamped to the range `[0.0, 0.2]`.

## Parameters

* `control_target` (string): The target to control. Options are `waistd` / `rotatehead` / `nodhead` / `raise`. Default: `waistd`.
* `target_position` (double[]): Target angle(s) in radians. Default: `[0.0]`.
* `threshold` (double): Position reached threshold in radians. Default: `0.01` (Only active in stepping mode).
* `period` (double): Control loop period in seconds. Default: `0.02`.
* `step_deg` (double): Maximum step size per period in degrees. Default: `0.2` (Only active in stepping mode).

## Build Instructions

```bash
colcon build --packages-select multi_joint_controller
```

## Running Examples
```bash
source install/setup.bash

# Control waistd
ros2 run multi_joint_controller multi_joint_controller_node --ros-args -p control_target:=waistd -p target_position:="[0.5]"

# Control rotatehead
ros2 run multi_joint_controller multi_joint_controller_node --ros-args -p control_target:=rotatehead -p target_position:="[0.3]"

# Control nodhead
ros2 run multi_joint_controller multi_joint_controller_node --ros-args -p control_target:=nodhead -p target_position:="[-0.2]"

# Control raise (automatically clamped to [0.0, 0.2])
ros2 run multi_joint_controller multi_joint_controller_node --ros-args -p control_target:=raise -p target_position:="[0.15]"
```