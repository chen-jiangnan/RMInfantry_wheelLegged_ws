"""
simulation.launch.py

根据 robot_type 从 wheel_legged_description 中选择对应的 MuJoCo scene.xml。
是否启动由 bringup.launch.py 的 IfCondition 控制，本文件只负责启动仿真节点。

描述文件路径规则:
  wheel_legged_description/<robot_type>/mjcf/scene.xml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='cyclBot',
        description='机器人类型: cyclBot / seriBot'
    )

    robot_type = LaunchConfiguration('robot_type')

    robot_scene = PathJoinSubstitution([
        FindPackageShare('wheel_legged_description'),
        robot_type,   # cyclBot 或 seriBot
        'mjcf',
        'scene_terrain.xml'
    ])

    sim_node = Node(
        package='wheel_legged_sim',
        executable='mujoco_sim',
        name='mujoco_sim_node',
        output='screen',
        parameters=[{
            'robot':                   robot_type,
            'robot_scene':             robot_scene,
            'enable_elastic_band':     False,
            'print_scene_information': False,
            'band_attached_link':      0,
        }],
    )

    return LaunchDescription([
        robot_type_arg,
        sim_node,
    ])