import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node           
import xacro

def generate_launch_description():
    # This name has to match the robot name in the Xacro File 
    
    
    # Name of your package
    namePackage = 'cpp_wcl'

    # Fixed paths based on standard ROS 2 structures
    # Note: earlier we discussed putting xacro in 'urdf' folder, 
    # but I will use 'model' to match your snippet.
    modelFileRelativePath = 'model/model.xacro'

    rvizRelativePath='config/config.rviz'
    ros2controlRelativePath='config/robot_control.yaml'

    # Corrected os.path.join (not joint)
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)
   
    pathRviz = os.path.join(get_package_share_directory(namePackage),rvizRelativePath )
    pathRos2control = os.path.join(get_package_share_directory(namePackage),ros2controlRelativePath )

    # Process the xacro file to get the raw URDF string
    robotDescription = xacro.process_file(pathModelFile).toxml()

    # 1. Include the Gazebo launch file from the gazebo_ros package
    
   

    # 2. rviz node
    rviz_node = Node(
        package='rviz2', 
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', pathRviz],
    )

    # 3. Robot State Publisher Node
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': robotDescription,
            'use_sim_time': True
        }]
    )
    #4. Ros2_control.Node
    control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[
        {"robot_description": robotDescription},
        pathRos2control,
    ],
    remappings=[
        ("~/robot_description", "/robot_description"),
    ],
    output="screen",
)


    #joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    #foreward postion controller 
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--param-file",pathRos2control ],


    )
    joint_state_publisher_gui =Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    name='joint_state_publisher_gui'
)



    # Create the launch description and add actions
    launchDescriptionObject = LaunchDescription()
    #launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(control_node)
    #launchDescriptionObject.add_action(nodeRobotStatePublisher)
    #launchDescriptionObject.add_action(joint_state_broadcaster_spawner)
    #launchDescriptionObject.add_action(robot_controller_spawner)
    #launchDescriptionObject.add_action(joint_state_publisher_gui)


    return launchDescriptionObject



