"""
display.launch.py - RViz + PlotJuggler
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_launch = FindPackageShare('wheel_legged_launch')

    rviz_config = PathJoinSubstitution([pkg_launch, 'rviz', 'display.rviz'])
    plotjuggler_config = PathJoinSubstitution([pkg_launch, 'plotjuggler', 'plotjuggler.xml'])

    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='cyclBot'),
        DeclareLaunchArgument('env_mode',   default_value='sim'),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config],
        #     output='screen',
        # ),

        Node(
            package='plotjuggler',
            executable='plotjuggler',
            name='plotjuggler',
            output='screen',
        ),
    ])