"""
gimbal_control.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_type_arg = DeclareLaunchArgument('robot_type', default_value='cyclBot')
    env_mode_arg   = DeclareLaunchArgument('env_mode',   default_value='sim')
    robot_type     = LaunchConfiguration('robot_type')

    gimbal_yaml = PathJoinSubstitution([
        FindPackageShare('wheel_legged_launch'),
        'config', robot_type, 'gimbal_control_config.yaml'
    ])

    return LaunchDescription([
        robot_type_arg,
        env_mode_arg,
        Node(
            package='wheel_legged_control',
            executable='gimbal_control',
            name='gimbal_control_node',
            output='screen',
            parameters=[gimbal_yaml],
        ),
    ])