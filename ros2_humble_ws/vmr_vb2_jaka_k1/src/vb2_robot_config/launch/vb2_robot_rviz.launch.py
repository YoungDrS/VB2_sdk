from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    rviz_path = get_package_share_path('vb2_robot_config')
    default_rviz_config_path = rviz_path / 'config/vb2_robot.rviz'

    ld = LaunchDescription()

    # 启动 RViz 文件，如果 use_rviz 参数为 true，则启动 RViz 可视化工具
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(default_rviz_config_path),
        )
    )
    ld.add_action(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
        )
    )


    return ld
