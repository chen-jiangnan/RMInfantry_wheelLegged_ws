from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.actions import IncludeLaunchDescription
import os

def generate_launch_description():
    # Launch arguments
    robot_type = LaunchConfiguration('robot_type', default='cyclBot')
    env_mode   = LaunchConfiguration('env_mode', default='sim')

    ld = LaunchDescription([
        # 可通过命令行覆盖
        DeclareLaunchArgument(
            'robot_type', 
            default_value='cyclBot',
            description='机器人类型: cyclBot/serialBot'),
        DeclareLaunchArgument(
            'env_mode',
            default_value='sim',
            description='运行环境: sim/real')
    ])

    launch_dir = os.path.dirname(__file__)

    # ==== 环境节点 ====
    sim_node = Node(
        package='wheel_legged_sim',
        executable='mujoco_sim',
        name='sim_node',
        output='screen',
        condition=IfCondition(PythonExpression(["'", env_mode, "' == 'sim'"]))
    )
    ld.add_action(sim_node)

    hw_node = Node(
        package='wheel_legged_hw',
        executable='hardware_bridge',
        name='hw_node',
        output='screen',
        condition=IfCondition(PythonExpression(["'", env_mode, "' == 'real'"]))
    )
    ld.add_action(hw_node)


    # 控制节点
    control_launches = ['chassis_control', 'gimbal_control', 'shoot_control', 'user_control']
    for node_name in control_launches:
        launch_file = os.path.join(launch_dir, f'{node_name}.launch.py')
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file),
                launch_arguments={'robot_type': robot_type, 'run_mode': env_mode}.items()
            )
        )
    
    # joystick 节点
    ld.add_action(
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        )
    )

    # plotjuggler节点
    plotjuggler_config_file = os.path.join(launch_dir, '..', 'plotjuggler', 'plotjuggler.xml')
    ld.add_action(
        Node(
            package='plotjuggler',
            executable='plotjuggler',
            name='plotjuggler',
            arguments=['-d', plotjuggler_config_file],
            output='screen'
        )
    )

    # RViz 节点
    rviz_config_file = os.path.join(launch_dir, '..', 'rviz', 'display.rviz')
    ld.add_action(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    )

    return ld


