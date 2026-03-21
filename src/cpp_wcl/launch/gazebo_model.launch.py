import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node           
import xacro

def generate_launch_description():
    # This name has to match the robot name in the Xacro File 
    robotXacroName = 'differential_drive_robot'
    
    # Name of your package
    namePackage = 'cpp_wcl'

    # Fixed paths based on standard ROS 2 structures
    # Note: earlier we discussed putting xacro in 'urdf' folder, 
    # but I will use 'model' to match your snippet.
    modelFileRelativePath = 'model/robot_model.xacro'
    worldFileRelativePath = 'model/tank_world.world'

    # Corrected os.path.join (not joint)
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)
    pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)
    
    # Process the xacro file to get the raw URDF string
    robotDescription = xacro.process_file(pathModelFile).toxml()

    # 1. Include the Gazebo launch file from the gazebo_ros package
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'))
    
    gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'world': pathWorldFile}.items())


    # 2. Spawn the robot entity in Gazebo
    spawnModelNode = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic','robot_description', '-entity', 'differential_drive_robot'],
        output='screen'
    )

    # 3. Robot State Publisher Node
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robotDescription,
            'use_sim_time': True
        }]
    )
    nodeJointStateBroadcaster = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    output='screen'
)

    # Create the launch description and add actions
    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(gazeboLaunch)
    launchDescriptionObject.add_action(spawnModelNode)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(nodeJointStateBroadcaster)

    return launchDescriptionObject



