"""
user_control.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='cyclBot'),
        DeclareLaunchArgument('env_mode',   default_value='sim'),

        # joystick
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),

        # user control
        Node(
            package='wheel_legged_control',
            executable='user_control',
            name='user_control_node',
            output='screen',
        ),
    ])