ros2 service call /lanxin_robot/run_action service_srv/srv/RunAction "{action_num: 1}"

ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\": 1, \"left_arm_pose\": {\"x\": 0.6, \"y\": 0.2025, \"z\": 0.6, \"qx\": 0.0, \"qy\": 0.0, \"qz\": 0.0, \"qw\": 1.0}, \"right_arm_pose\": {\"x\": 0.6, \"y\": -0.2025, \"z\": 0.6, \"qx\": 0.0, \"qy\": 0.0, \"qz\": 0.0, \"qw\": 0.1}}'}"

# 车上搬箱子偏移0度
ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\": 2, \"left_arm_pose\": {\"x\": 0.6, \"y\": 0.2025, \"z\": 0.75, \"qx\": 0.0, \"qy\": 0.0, \"qz\": 0.0, \"qw\": 1.0}, \"right_arm_pose\": {\"x\": 0.6, \"y\": -0.2025, \"z\": 0.75, \"qx\": 0.0, \"qy\": 0.0, \"qz\": 0.0, \"qw\": 0.1}}'}"

ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\": 2, \"left_arm_pose\": {\"x\": 0.6304, \"y\": 0.199422, \"z\": 0.75, \"qx\": 0.0, \"qy\": 0.0, \"qz\": 0.0, \"qw\": 1.0}, \"right_arm_pose\": {\"x\": 0.5696, \"y\": -0.199422, \"z\": 0.75, \"qx\": 0.0, \"qy\": 0.0, \"qz\": 0.0, \"qw\": 0.1}}'}"

# 狗上搬箱子偏移10度
ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\": 3, \"left_arm_pose\": {\"x\": 0.6304, \"y\": 0.199422, \"z\": 0.6, \"qx\": 0.0, \"qy\": 0.0, \"qz\": 0.0, \"qw\": 1.0}, \"right_arm_pose\": {\"x\": 0.5696, \"y\": -0.199422, \"z\": 0.6, \"qx\": 0.0, \"qy\": 0.0, \"qz\": 0.0, \"qw\": 0.1}}'}"

# 箱子放车上偏移10度
ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\": 4, \"left_arm_pose\": {\"x\": 0.6304, \"y\": 0.199422, \"z\": 0.6, \"qx\": 0.0, \"qy\": 0.0, \"qz\": 0.0, \"qw\": 1.0}, \"right_arm_pose\": {\"x\": 0.5696, \"y\": -0.199422, \"z\": 0.6, \"qx\": 0.0, \"qy\": 0.0, \"qz\": 0.0, \"qw\": 0.1}}'}"

# 箱子放狗上偏移10度
ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\": 5, \"left_arm_pose\": {\"x\": 0.6304, \"y\": 0.199422, \"z\": 0.6, \"qx\": 0.0, \"qy\": 0.0, \"qz\": 0.0, \"qw\": 1.0}, \"right_arm_pose\": {\"x\": 0.5696, \"y\": -0.199422, \"z\": 0.6, \"qx\": 0.0, \"qy\": 0.0, \"qz\": 0.0, \"qw\": 0.1}}'}"

# 从货架上搬箱子
ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\": 6, \"left_arm_pose\": {\"x\": 0.7, \"y\": 0.2025, \"z\": 0.9, \"qx\": 0.0, \"qy\": 0.0, \"qz\": 0.0, \"qw\": 1.0}, \"right_arm_pose\": {\"x\": 0.7, \"y\": -0.2025, \"z\": 0.9, \"qx\": 0.0, \"qy\": 0.0, \"qz\": 0.0, \"qw\": 0.1}}'}"

# 搬箱子放货架
ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\": 7, \"left_arm_pose\": {\"x\": 0.7, \"y\": 0.2025, \"z\": 0.9, \"qx\": 0.0, \"qy\": 0.0, \"qz\": 0.0, \"qw\": 1.0}, \"right_arm_pose\": {\"x\": 0.7, \"y\": -0.2025, \"z\": 0.9, \"qx\": 0.0, \"qy\": 0.0, \"qz\": 0.0, \"qw\": 0.1}}'}"



搬箱子相机观察位姿
[INFO] [1761208579.388150974] [end_effector_pose_node]: === End Link Pose ===
[INFO] [1761208579.388226626] [end_effector_pose_node]: left_end_Link Position: [0.631, 0.345, 0.840]
[INFO] [1761208579.388259588] [end_effector_pose_node]: left_end_Link Orientation (xyzw): [0.700, 0.627, 0.176, 0.295]
[INFO] [1761208579.388289926] [end_effector_pose_node]: left_end_Link RPY(度): [140.354, 7.150, 81.119]
[INFO] [1761208579.388316520] [end_effector_pose_node]: right_end_Link Position: [0.650, -0.300, 0.859]
[INFO] [1761208579.388336969] [end_effector_pose_node]: right_end_Link Orientation (xyzw): [0.637, 0.693, 0.285, 0.181]
[INFO] [1761208579.388360811] [end_effector_pose_node]: right_end_Link RPY(度): [141.005, -6.454, 97.173]
[INFO] [1761208579.388383980] [end_effector_pose_node]: === Joint Values ===
[INFO] [1761208579.388400877] [end_effector_pose_node]: left Joint Values: [-2.139, -0.394, 0.601, -2.084, -0.632, 0.651, 0.256]
[INFO] [1761208579.388425743] [end_effector_pose_node]: right Joint Values: [2.128, -0.559, -0.506, -2.088, 0.671, -0.788, -0.160]
[INFO] [1761208579.388450320] [end_effector_pose_node]: === =============================== ===

无箱子状态
[INFO] [1761212621.497903482] [end_effector_pose_node]: left Joint Values: [-1.500, -0.837, 0.493, -1.349, -0.616, 0.379, 0.200]
[INFO] [1761212621.497925050] [end_effector_pose_node]: right Joint Values: [1.771, -0.936, -0.549, -2.027, 0.799, -0.709, -0.117]




箱子目标位姿, tool_link
ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\": 2, \"left_arm_pose\": {\"x\": 0.63, \"y\": 0.28, \"z\": 0.72, \"qx\": 0.0, \"qy\": 0.0, \"qz\": 0.0, \"qw\": 1.0}, \"right_arm_pose\": {\"x\": 0.63, \"y\": -0.25, \"z\": 0.72, \"qx\": 0.0, \"qy\": 0.0, \"qz\": 0.0, \"qw\": 0.1}}'}"
[INFO] [1761185989.837547943] [end_effector_pose_node]: === End Link Pose ===
[INFO] [1761185989.837618951] [end_effector_pose_node]: left_end_Link Position: [0.630, 0.280, 0.720]
[INFO] [1761185989.837712614] [end_effector_pose_node]: right_end_Link Position: [0.630, -0.250, 0.720]
[INFO] [1761185989.837782373] [end_effector_pose_node]: === Joint Values ===
[INFO] [1761185989.837799397] [end_effector_pose_node]: left Joint Values: [-1.603, -0.661, 0.411, -1.938, -0.103, 0.778, -0.150]
[INFO] [1761185989.837825157] [end_effector_pose_node]: right Joint Values: [1.587, -0.770, -0.378, -1.919, 0.138, -0.865, 0.154]
[INFO] [1761185989.837850821] [end_effector_pose_node]: === =============================== ===
相机识别出的箱子目标点
point_left: [0.732984 0.194419 0.616604 ]
point_right: [ 0.7393   -0.188881  0.611899 ]

left 
x  0.630-0.732984=-0.102984
y  0.280-0.194419=0.085581
z  0.720-0.616604=0.103396

right
x  0.630-0.7393=−0.1093
y  -0.250-(-0.188881)=−0.061119
z  0.720-0.611899=0.108101
ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\": 2, \"left_arm_pose\": {\"x\": 0.732984, \"y\": 0.194419, \"z\": 0.616604, \"qx\": 0.0, \"qy\": 0.0, \"qz\": 0.0, \"qw\": 1.0}, \"right_arm_pose\": {\"x\": 0.7393, \"y\": -0.188881, \"z\": 0.611899, \"qx\": 0.0, \"qy\": 0.0, \"qz\": 0.0, \"qw\": 0.1}}'}"

# 车搬箱子
ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\": 3, \"left_arm_pose\": {\"x\": 0.732984, \"y\": 0.194419, \"z\": 0.616604, \"qx\": 0.0, \"qy\": 0.0, \"qz\": 0.0, \"qw\": 1.0}, \"right_arm_pose\": {\"x\": 0.7393, \"y\": -0.188881, \"z\": 0.611899, \"qx\": 0.0, \"qy\": 0.0, \"qz\": 0.0, \"qw\": 0.1}}'}"
# 车放箱子
ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\": 4, \"left_arm_pose\": {\"x\": 0.732984, \"y\": 0.194419, \"z\": 0.616604, \"qx\": 0.0, \"qy\": 0.0, \"qz\": 0.0, \"qw\": 1.0}, \"right_arm_pose\": {\"x\": 0.7393, \"y\": -0.188881, \"z\": 0.611899, \"qx\": 0.0, \"qy\": 0.0, \"qz\": 0.0, \"qw\": 0.1}}'}"


left  0.7548 0.1724 0.6247
right 0.7702 -0.2322 0.62
# 车搬箱子
ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\": 3, \"left_arm_pose\": {\"x\": 0.7548, \"y\": 0.1724, \"z\": 0.6247, \"qx\": 0.0, \"qy\": 0.0, \"qz\": 0.0, \"qw\": 1.0}, \"right_arm_pose\": {\"x\": 0.7702, \"y\": -0.2322, \"z\": 0.62, \"qx\": 0.0, \"qy\": 0.0, \"qz\": 0.0, \"qw\": 0.1}}'}"

左臂目标点 (x, y, z, rx, ry, rz):
 (0.7604327356099887, 0.198097492691638, 0.6318512732497171, -1.542062257964094, -0.01844970602417173, -1.5416025218306213)
右臂目标点 (x, y, z, rx, ry, rz):
 (0.7722526387791131, -0.20666103856120105, 0.6393229970674272, -1.542062257964094, -0.01844970602417173, -1.5416025218306213)
# 货架搬箱子
ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\": 3, \"left_arm_pose\": {\"x\": 0.7604327356099887, \"y\": 0.198097492691638, \"z\": 0.6318512732497171, \"qx\": 0.0, \"qy\": 0.0, \"qz\": 0.0, \"qw\": 1.0}, \"right_arm_pose\": {\"x\": 0.7722526387791131, \"y\": -0.20666103856120105, \"z\": 0.6393229970674272, \"qx\": 0.0, \"qy\": 0.0, \"qz\": 0.0, \"qw\": 0.1}}'}"

机械臂启动方法：
1.
ssh aaeon@192.168.31.105 -p 1022
root

2.
cd ros2_ws/

3.
bash vb2_jaka_start.sh

# ------------------------------------------
ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\":\"calculate_current_joint_and_pose\",\"left_end_link_name\":\"left_tool_link\",\"right_end_link_name\":\"right_tool_link\"}'}"

ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\":\"arm_joint_planning\",\"left_arm_joint_values\":[-0.871979,-1.039040,0.526925,-1.223019,0.000090,-0.000009,-0.000022],\"right_arm_joint_values\":[0.871910,1.039080,-0.527049,1.223078,-0.000065,0.000073,-0.000000]}'}"

ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\":\"hand_joint_planning\",\"left_hand_joint_values\":[0.0,0.0,0.0,0.0,0.436,0.436],\"right_hand_joint_values\":[0.0,0.0,0.0,0.0,0.436,0.436]}'}"

ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\":\"waist_arm_joint_planning\",\"left_arm_joint_values\":[-0.480, -0.720, 0.091, -1.115, 0.274, -0.251, -0.274],\"right_arm_joint_values\":[0.753, -1.455, 1.668, -1.793, -2.651, -0.380, 0.251],\"waist_joint_values\":[0.0,0.0]}'}"

ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\":\"waist_arm_hand_joint_planning\",\"left_arm_joint_values\":[-0.17,-1.215,0.492,-0.602,0.047,0.0,0.0],\"right_arm_joint_values\":[0.959,1.160,-0.479,1.546,-1.005,0.284,-0.564],\"waist_joint_values\":[0.1,-0.215],\"left_hand_joint_values\":[0.0,0.0,0.0,0.0,0.436,0.436],\"right_hand_joint_values\":[0.0,0.0,1.395,1.395,1.57,0.697]}'}"




# 比心
ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\":\"arm_joint_planning\",\"left_arm_joint_values\":[-2.977493,-0.478691,0.046960,-1.681645,-0.046863,-0.650467,0.234353],\"right_arm_joint_values\":[2.883787,-0.471722,0.070398,-1.664664,-0.000012,0.738391,0.000034]}'}"

# 初始姿态
ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\":\"arm_joint_planning\",\"left_arm_joint_values\":[-2.977493,-0.478691,0.046960,-1.681645,-0.046863,-0.650467,0.234353],\"right_arm_joint_values\":[2.883787,-0.471722,0.070398,-1.664664,-0.000012,0.738391,0.000034]}'}"

