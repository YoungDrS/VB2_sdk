# 多关节控制（ROS2 C++）

最简 ROS2 C++ 包：`multi_joint_controller`

## 控制模式（`control_target`）
- `waistd`（或 `waist`）
  - control: `/vmr/waist_joint_control`
  - state: `/vmr/waist_joint_state`
  - 控制策略：步进+缓启动+接近目标减速+到位退出
- `rotatehead`
  - control: `/vmr/head_joint_control_1`
  - state: `/vmr/head_joint_state_1`
  - 控制策略：步进+缓启动+接近目标减速+到位退出
- `nodhead`
  - control: `/vmr/head_joint_control_2`
  - state: `/vmr/head_joint_state_2`
  - 控制策略：步进+缓启动+接近目标减速+到位退出
- `raise`
  - control: `/vmr/raise_control`
  - state: `/vmr/raise_state`
  - 控制策略：只发目标点，不做步进
  - 限制：目标值只使用 `target_position[0]`，并强制限制在 `[0.0, 0.2]`

## 参数
- `control_target` (string)：`waistd` / `rotatehead` / `nodhead` / `raise`，默认 `waistd`
- `target_position` (double[])：目标角度（弧度），默认 `[0.0]`
- `threshold` (double)：到位阈值（弧度），默认 `0.01`（仅步进模式使用）
- `period` (double)：控制周期（秒），默认 `0.02`
- `step_deg` (double)：每周期最大步进（角度），默认 `0.2`（仅步进模式使用）

## 编译
```bash
colcon build --packages-select multi_joint_controller
```

## 运行示例
```bash
source install/setup.bash

# waistd
ros2 run multi_joint_controller multi_joint_controller_node --ros-args -p control_target:=waistd -p target_position:="[0.5]"

# rotatehead
ros2 run multi_joint_controller multi_joint_controller_node --ros-args -p control_target:=rotatehead -p target_position:="[0.3]"

# nodhead
ros2 run multi_joint_controller multi_joint_controller_node --ros-args -p control_target:=nodhead -p target_position:="[-0.2]"

# raise（会自动限制到 [0.0, 0.2]）
ros2 run multi_joint_controller multi_joint_controller_node --ros-args -p control_target:=raise -p target_position:="[0.15]"
```
