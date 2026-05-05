import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Update these variables to your new names
    package_name = 'cpp_WCL'
    urdf_file = 'WCR.urdf'  # Your updated URDF name
    rviz_config_file = 'config.rviz'

    # 2. Get the paths
    pkg_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file)
    rviz_config_path = os.path.join(pkg_share, 'config', rviz_config_file)

    # 3. Read the URDF content
    with open(urdf_path, 'r') as infp:
        robot_description_content = infp.read()

    # 4. Define Nodes
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])