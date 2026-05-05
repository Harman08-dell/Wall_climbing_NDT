import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_name = 'cpp_wcl'
    pkg_share = get_package_share_directory(pkg_name)

    #paths

    urdf_file   = os.path.join(pkg_share, 'model',  'robot_model.xacro')
    world_file  = os.path.join(pkg_share, 'model', 'Slam_world.world')
    slam_config = os.path.join(pkg_share, 'config', 'localization_slam_toolbox.yaml')
    map_file    = '/home/harman/WCL_ws/maps/slam_map2.yaml'
    joy_config = os.path.join(pkg_share, 'config', 'ps4_teleop.yaml')
    #rviz_config = os.path.join(pkg_share, 'rviz',   'slam.rviz')
    


    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = ParameterValue(Command('xacro ' + urdf_file), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }]
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
        }.items()
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'wcl_robot',
            '-x', '-3.0',   # spawn in SW corner, away from obstacles
            '-y', '-3.0',
            '-z', '0.21',   # slightly above ground (wheel_radius = 0.2)
            '-Y', '0.0',
        ]
    )


    # map_server = TimerAction(
    # period=3.0,
    # actions=[
    #     Node(
    #         package='nav2_map_server',
    #         executable='map_server',
    #         name='map_server',
    #         output='screen',
    #         parameters=[{
    #            'use_sim_time': use_sim_time, 'yaml_filename' : map_file
    #         }]
    #     )
    # ]
    # )
    # Lifecycle manager — map_server needs this to activate
    # lifecycle_manager = TimerAction(
    #     period=4.0,
    #     actions=[
    #         Node(
    #             package='nav2_lifecycle_manager',
    #             executable='lifecycle_manager',
    #             name='lifecycle_manager',
    #             output='screen',
    #             parameters=[{
    #                 'use_sim_time': use_sim_time,
    #                 'autostart': True,
    #                 'node_names': ['map_server'],
    #             }]
    #         )
    #     ]
    # )
    slam_toolbox = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='localization_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_config,
                    {'use_sim_time': use_sim_time}
                ],
            )
        ]
    )

    
    rviz2 = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ]
    )
    joy_node = Node(
    package='joy',
    executable='joy_node',
    name='joy_node',
    output='screen',
    parameters=[{'device': '/dev/input/js0'}],
   )
    teleop_joy = Node(
    package='teleop_twist_joy',
    executable='teleop_node',
    name='teleop_twist_joy',
    output='screen',
    parameters=[joy_config],
    remappings=[('cmd_vel', '/cmd_vel')],
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher,
        gazebo_launch,
        spawn_robot,
        # map_server,
        # lifecycle_manager,
        slam_toolbox,
        rviz2,
        joy_node,
        teleop_joy,
       
        
    ]) 