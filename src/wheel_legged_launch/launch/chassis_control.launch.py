"""
chassis_control.launch.py

根据 robot_type 加载对应的 YAML 配置文件:
  wheel_legged_launch/config/<robot_type>/chassis_control_config.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_type_arg = DeclareLaunchArgument(
        'robot_type', default_value='cyclBot',
        description='机器人类型: cyclBot / seriBot'
    )

    env_mode_arg = DeclareLaunchArgument(
        'env_mode', default_value='sim',
        description='运行环境: sim / real'
    )

    robot_type = LaunchConfiguration('robot_type')

    chassis_yaml = PathJoinSubstitution([
        FindPackageShare('wheel_legged_launch'),
        'config', robot_type, 'chassis_control_config.yaml'
    ])

    chassis_node = Node(
        package='wheel_legged_control',
        executable='chassis_control',
        name='chassis_control_node',
        output='screen',
        parameters=[chassis_yaml],
    )

    return LaunchDescription([
        robot_type_arg,
        env_mode_arg,
        chassis_node,
    ])