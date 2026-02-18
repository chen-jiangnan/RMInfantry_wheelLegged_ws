"""
shoot_control.launch.py - 修正版

支持通过robot_type参数选择不同的配置文件
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ==================== 声明Launch参数 ====================
    
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='cyclBot',  
        description='机器人类型: cyclBot/serialBot',
    )
    
    env_mode_arg = DeclareLaunchArgument(
        'env_mode',
        default_value='sim',
        description='运行环境: sim/real'
    )
    
    # ==================== 获取配置 ====================
    
    # robot_type = LaunchConfiguration('robot_type')
    # env_mode = LaunchConfiguration('env_mode')

    # pkg_share = FindPackageShare('wheel_legged_launch')
    
    # chassis_yaml = PathJoinSubstitution([
    #     pkg_share,
    #     'config',
    #     robot_type,  # ← 直接使用LaunchConfiguration
    #     'chassis_control_config.yaml'
    # ])
    
    # ==================== 节点定义 ====================
    
    shoot_node = Node(
        package='wheel_legged_control',
        executable='shoot_control',
        name='shoot_control_node',
        # parameters=[chassis_yaml],
        output='screen',
        # 可选：添加日志配置
        arguments=['--ros-args', '--log-level', 'INFO']
    )
    
    return LaunchDescription([
        robot_type_arg,
        env_mode_arg,
        shoot_node,
    ])
