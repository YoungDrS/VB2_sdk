arm_joint_planning 双臂全关节规划
hand_joint_planning 灵巧手全关节规划
arm_hand_joint_planning 双臂+灵巧手全关节规划
waist_arm_joint_planning 双臂+腰全关节规划
waist_arm_hand_joint_planning 双臂+腰+灵巧手全关节规划
dual_arm_cartesian_line_planning 双臂直线笛卡尔规划
dual_arm_waist_cartesian_line_planning 双臂+腰直线笛卡尔规划
one_arm_cartesian_line_planning 单臂直线笛卡尔规划
one_arm_waist_cartesian_line_planning 单臂+腰直线笛卡尔规划
calculate_current_joint_and_pose 获取当前机械臂关节数据和末端位姿
one_arm_cartesian_bezier_planning 单臂贝塞尔轨迹规划
one_arm_waist_cartesian_bezier_planning 单臂+腰贝塞尔轨迹规划
arm_dray_switch 机械臂拖拽模式开关


// 拖拽模式切换
ros2 service call /drag_swich_service std_srvs/srv/SetBool "{data: 1}"

ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\":98}'}"

ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\":\"arm_joint_planning\",\"left_arm_joint_values\":[-0.871979,-1.039040,0.526925,-1.223019,0.000090,-0.000009,-0.000022],\"right_arm_joint_values\":[0.871910,1.039080,-0.527049,1.223078,-0.000065,0.000073,-0.000000]}'}"

ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\":\"hand_joint_planning\",\"left_hand_joint_values\":[0.0,0.0,0.0,0.0,0.436,0.436],\"right_hand_joint_values\":[0.0,0.0,1.395,1.395,1.57,0.697]}'}"

ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\":\"arm_hand_joint_planning\",\"left_hand_joint_values\":[0.0,0.0,0.0,0.0,0.436,0.436],\"right_hand_joint_values\":[0.0,0.0,1.395,1.395,1.57,0.697],\"left_arm_joint_values\":[-0.07,-1.215,0.492,-0.602,0.047,0.0,0.0],\"right_arm_joint_values\":[0.959,1.160,-0.479,1.546,-1.005,0.284,-0.564]}'}"

ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\":\"waist_arm_joint_planning\",\"left_arm_joint_values\":[-0.871979,-1.039040,0.526925,-1.223019,0.000090,-0.000009,-0.000022],\"right_arm_joint_values\":[0.871910,1.039080,-0.527049,1.223078,-0.000065,0.000073,-0.000000],\"waist_joint_values\":[0.0,-0.215]}'}"

ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\":\"waist_arm_hand_joint_planning\",\"left_arm_joint_values\":[-0.17,-1.215,0.492,-0.602,0.047,0.0,0.0],\"right_arm_joint_values\":[0.959,1.160,-0.479,1.546,-1.005,0.284,-0.564],\"waist_joint_values\":[0.1,-0.215],\"left_hand_joint_values\":[0.0,0.0,0.0,0.0,0.436,0.436],\"right_hand_joint_values\":[0.0,0.0,1.395,1.395,1.57,0.697]}'}"


ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\":99}'}"

ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\":98}'}"

# ------------------------------------------------------

ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\":\"dual_arm_cartesian_line_planning\",\"left_arm_end_values\":[0.727, 0.205, 1.126, 0.590, 0.563, -0.035, 0.578],\"right_arm_end_values\":[0.717, -0.201, 1.147, -0.589, 0.550, 0.025, 0.591],\"cartesian_line_step\":10}'}"

ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\":\"dual_arm_waist_cartesian_line_planning\",\"left_arm_end_values\":[0.727, 0.205, 1.126, 0.590, 0.563, -0.035, 0.578],\"right_arm_end_values\":[0.717, -0.201, 1.147, -0.589, 0.550, 0.025, 0.591],\"cartesian_line_step\":10}'}"

ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\":\"one_arm_cartesian_line_planning\",\"end_link_name\":\"left_arm_link\",\"arm_end_values\":[0.680061,0.052908,0.849062,0.229669,0.753951,-0.556331,0.263260],\"cartesian_line_step\":30}'}"

ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\":\"one_arm_cartesian_line_planning\",\"end_link_name\":\"right_arm_link\",\"arm_end_values\":[0.619957,-0.383315,1.170261,-0.280345,-0.357588,0.576469,0.679132],\"cartesian_line_step\":30}'}"

ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\":\"one_arm_waist_cartesian_line_planning\",\"end_link_name\":\"left_arm_link\",\"arm_end_values\":[0.828750,0.230299,0.969448,-0.140582,-0.188488,-0.719082,0.653934],\"cartesian_line_step\":30}'}"

ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\":\"one_arm_waist_cartesian_line_planning\",\"end_link_name\":\"right_arm_link\",\"arm_end_values\":[0.780257,-0.193184,1.323448,0.006108,0.600539,0.203274,0.773302],\"cartesian_line_step\":30}'}"


ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\":\"right_servo_planning\",\"right_arm_end_values\":[-0.00005,0,0,0,0,0]}'}"


ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\":\"calculate_current_joint_and_pose\",\"left_end_link_name\":\"left_tool_link\",\"right_end_link_name\":\"right_tool_link\"}'}"

ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\":\"get_current_joint_and_pose\"}'}"



ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService \
"{arm_command: '{\"dual_arm_action\":\"one_arm_cartesian_bezier_planning\",\"grasp_poses_a\":[{\"x\":0.577283,\"y\":0.0597073,\"z\":0.967047,\"qx\":0.638833,\"qy\":-0.443179,\"qz\":-0.482387,\"qw\":-0.403469}],\"grasp_poses_b\":[{\"x\":0.661641,\"y\":-0.133207,\"z\":1.23442,\"qx\":-0.589399,\"qy\":0.549913,\"qz\":0.0253193,\"qw\":0.591239}],\"cartesian_line_step\":50,\"arm_name\":\"right_arm\"}'}"


ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService \
"{arm_command: '{\"dual_arm_action\":\"one_arm_waist_cartesian_bezier_planning\",\"grasp_poses_a\":[{\"x\":0.577283,\"y\":0.0597073,\"z\":0.967047,\"qx\":0.638833,\"qy\":-0.443179,\"qz\":-0.482387,\"qw\":-0.403469}],\"grasp_poses_b\":[{\"x\":0.661641,\"y\":-0.133207,\"z\":1.23442,\"qx\":-0.589399,\"qy\":0.549913,\"qz\":0.0253193,\"qw\":0.591239}],\"cartesian_line_step\":50,\"arm_name\":\"right_arm\"}'}"


ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\":\"inspire_gripper_switch\",\"gripper_name\":\"left_gripper\",\"gripper_switch\":\"close\",\"gripper_speed\":200,\"gripper_power\":200}'}"

ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService \
"{arm_command: '{\"dual_arm_action\":\"one_arm_waist_cartesian_line_planning_array\",\"grasp_poses_a\":[{\"x\":0.744849,\"y\":0.120939,\"z\":0.789960,\"qx\":0.011064,\"qy\":0.707501,\"qz\":0.027372,\"qw\":0.706096}],\"cartesian_line_step\":30,\"arm_name\":\"left_arm\"}'}"

ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\":\"one_arm_servo_planning\",\"end_link_name\":\"right_arm\",\"currents_data\":500,\"joint_num\":7,\"arm_end_values\":[0.00001,0,0,0,0,0]}'}"
# ------------------------------------------------------




ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService \
"{arm_command: '{\"dual_arm_action\":\"one_arm_waist_cartesian_bezier_planning\",\"grasp_poses_a\":[{\"x\":0.744849,\"y\":0.120939,\"z\":0.789960,\"qx\":0.011064,\"qy\":0.707501,\"qz\":0.027372,\"qw\":0.706096}],\"grasp_poses_b\":[{\"x\":0.638397,\"y\":0.118386,\"z\":0.790348,\"qx\":0.011008,\"qy\":0.707359,\"qz\":0.027252,\"qw\":0.706243}],\"cartesian_line_step\":30,\"arm_name\":\"left_arm\"}'}"

ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService \
"{arm_command: '{\"dual_arm_action\":\"one_arm_waist_cartesian_line_planning_array\",\"grasp_poses_a\":[{\"x\":0.744849,\"y\":0.120939,\"z\":0.789960,\"qx\":0.011064,\"qy\":0.707501,\"qz\":0.027372,\"qw\":0.706096}],\"cartesian_line_step\":30,\"arm_name\":\"left_arm\"}'}"



ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\":\"waist_arm_joint_planning\",\"left_arm_joint_values\":[-0.035166,-0.843656,0.791359,-1.677668,0.092513,-0.596136,-0.248065],\"right_arm_joint_values\":[0.035165,0.843678,-0.791344,1.677644,-0.092364,0.596238,0.248160],\"waist_joint_values\":[0.2,0.0]}'}"

ros2 service call /lanxin_robot/arm_command_service service_srv/srv/ArmCommandService "{arm_command: '{\"dual_arm_action\":\"arm_joint_planning\",\"left_arm_joint_values\":[0,-1.57,0,0,0,0,0],\"right_arm_joint_values\":[0,1.57,0,0,0,0,0]}'}"



ros2 topic pub /vmr/vmr_stop std_msgs/msg/Int32 "{data: 1}"
