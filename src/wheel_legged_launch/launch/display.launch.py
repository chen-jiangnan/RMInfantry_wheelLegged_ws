import os
import re
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    description_pkg_path = get_package_share_directory("wheel_legged_description")
    urdf_path = os.path.join(
        description_pkg_path,
        "Infantry_wheelLegged_cyclBot",
        "urdf",
        "zt_2025_Infantry_wheelLeggedP_withShoot_s.urdf"
    )

    # 读取 URDF
    with open(urdf_path, 'r') as f:
        urdf_text = f.read()

    # mesh 文件的绝对路径
    mesh_dir = os.path.join(description_pkg_path, "Infantry_wheelLegged_cyclBot", "meshes")

    # 替换 ../meshes/ 为绝对路径（file:// 协议）
    def replace_mesh(match):
        mesh_file = match.group(1)  # 提取文件名
        abs_path = os.path.join(mesh_dir, mesh_file)
        return f'filename="file://{abs_path}"'

    # 匹配相对路径格式
    urdf_text = re.sub(r'filename="\.\.\/meshes\/([^"]+)"', replace_mesh, urdf_text)
    
    robot_description = {"robot_description": urdf_text}

    launch_pkg_path = get_package_share_directory("wheel_legged_launch")
    rviz_config = os.path.join(launch_pkg_path, "rviz", "display.rviz")

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen"
        )
    ])