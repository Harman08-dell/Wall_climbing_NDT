import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Set package and file names
    package_name = 'cpp_WCL'
    urdf_file = 'WCR2.urdf' 
    rviz_config_file = 'config.rviz' # Optional: pre-saved RViz layout

    # 2. Path resolution
    pkg_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file)
    rviz_config_path = os.path.join(pkg_share, 'config', rviz_config_file)

    # 3. Read URDF content for robot_state_publisher
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 4. Define Nodes
    
    # Robot State Publisher: Publishes the 3D transforms (TF) of the crawlers
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Joint State Publisher GUI: Provides sliders for the tracks and magnets
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2: Visualizer
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        rsp_node,
        jsp_gui_node,
        rviz_node
    ])