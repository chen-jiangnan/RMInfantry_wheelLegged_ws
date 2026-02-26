"""
bringup.launch.py - 顶层启动文件

用法:
  ros2 launch wheel_legged_launch bringup.launch.py robot_type:=seriBot env_mode:=sim
  ros2 launch wheel_legged_launch bringup.launch.py robot_type:=cyclBot env_mode:=real
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ==================== 参数声明 ====================

    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='cyclBot',
        description='机器人类型: cyclBot / seriBot'
    )
    env_mode_arg = DeclareLaunchArgument(
        'env_mode',
        default_value='sim',
        description='运行环境: sim / real'
    )

    robot_type = LaunchConfiguration('robot_type')
    env_mode   = LaunchConfiguration('env_mode')

    pkg_launch = FindPackageShare('wheel_legged_launch')

    def sub_launch(filename):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_launch, 'launch', filename])
            ),
            launch_arguments={
                'robot_type': robot_type,
                'env_mode':   env_mode,
            }.items()
        )

    # ==================== 环境节点：顶层根据 env_mode 切换 ====================

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_launch, 'launch', 'simulation.launch.py'])
        ),
        launch_arguments={'robot_type': robot_type}.items(),
        condition=IfCondition(PythonExpression(["'", env_mode, "' == 'sim'"]))
    )

    hw_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_launch, 'launch', 'hardware.launch.py'])
        ),
        condition=IfCondition(PythonExpression(["'", env_mode, "' == 'real'"]))
    )

    # ==================== 组装 ====================

    return LaunchDescription([
        robot_type_arg,
        env_mode_arg,

        # 根据 env_mode 只启动其中一个
        sim_launch,
        hw_launch,

        # 控制节点（sim/real 下都运行）
        sub_launch('chassis_control.launch.py'),
        # sub_launch('gimbal_control.launch.py'),
        # sub_launch('shoot_control.launch.py'),
        sub_launch('user_control.launch.py'),

        # 工具
        sub_launch('display.launch.py'),
    ])