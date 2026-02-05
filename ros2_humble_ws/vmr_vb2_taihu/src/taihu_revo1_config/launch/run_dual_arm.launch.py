import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from pathlib import Path
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 构建 MoveIt 的配置对象
    # MoveItConfigsBuilder 用于生成 MoveIt 配置，并通过 to_moveit_configs() 方法转换成 MoveItConfigs 对象
    moveit_config = MoveItConfigsBuilder("taihu_revo1_description", package_name="taihu_revo1_config").to_moveit_configs()
    # 获取 MoveIt 配置包所在的路径
    moveit_config_package_path = Path(moveit_config.package_path)

    # 创建 LaunchDescription 对象，后续所有的启动动作都将添加到此对象中
    ld = LaunchDescription()

    # 声明启动参数：是否启动数据库（默认 false，因为数据库可能占用较多资源）
    ld.add_action(
        DeclareLaunchArgument(
            "db",
            default_value="false",
            description="默认不启动数据库（可能占用较大资源）",
        )
    )

    # 声明启动参数：是否启动调试模式（默认 false）
    ld.add_action(
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            description="默认不开启调试模式",
        )
    )
    
    # 声明启动参数：是否启动 RViz，可视化工具（默认 true）
    ld.add_action(DeclareLaunchArgument("use_rviz", default_value="false"))

    # 包含静态虚拟关节 TF 的启动文件
    # 根据 MoveIt 配置包路径找到静态虚拟关节 TF 的启动文件，如果该文件存在则包含它
    virtual_joints_launch = moveit_config_package_path / "launch/static_virtual_joint_tfs.launch.py"
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # 包含机器人状态发布器的启动文件，用于发布机器人的状态信息
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config_package_path / "launch/rsp.launch.py")
            ),
        )
    )

    # 包含 move_group 启动文件，用于启动 MoveIt 的 move_group 节点，控制运动规划
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config_package_path / "launch/move_group.launch.py")
            ),
        )
    )

    # 包含 MoveIt RViz 启动文件，如果 use_rviz 参数为 true，则启动 RViz 可视化工具
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config_package_path / "launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    # 包含仓库数据库启动文件，如果 db 参数为 true，则启动数据库服务
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config_package_path / "launch/warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    # 添加虚拟关节驱动节点（使用 ros2_control_node）
    # ros2_control_node 用于仿真或驱动机器人的控制器
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                # 传入控制器的配置文件路径
                str(moveit_config_package_path / "config/ros2_controllers.yaml"),
            ],
            remappings=[
                # 重新映射话题，将 /controller_manager/robot_description 映射到 /robot_description
                ("/controller_manager/robot_description", "/robot_description"),
            ],
        )
    )

    # 启动控制器
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[ 
                "joint_state_broadcaster",
                "left_arm_controller",
                "right_arm_controller"
            ],
            output="screen",
        )
    )
    
    # 等待15秒以确保所有节点都已启动，再启动 ros2 run dual_arm_move run_test
    ld.add_action(
        Node(
            package="dual_arm_move",
            executable="run_test",
            output="screen",
            # 延时15秒启动，确保前面的节点都已启动
            prefix="bash -c 'sleep 20; $0 $@'",
        )
    )

    return ld
