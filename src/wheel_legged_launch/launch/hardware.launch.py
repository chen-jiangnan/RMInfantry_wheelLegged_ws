"""
hardware.launch.py

启动硬件驱动节点。是否启动由 bringup.launch.py 的 IfCondition 控制。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='cyclBot'),

        Node(
            package='wheel_legged_hw',
            executable='hardware_bridge_node',
            name='hw_node',
            output='screen',
        ),
    ])