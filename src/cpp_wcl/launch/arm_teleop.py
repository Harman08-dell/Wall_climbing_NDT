from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # micro-ROS Agent
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyUSB0', '-b', '115200'],
            output='screen'
        ),

        # Joystick Node
        Node(
            package='joy',
            executable='joy_node',
            output='screen'
        ),

        # Robotic Arm Node
        Node(
            package='cpp_wcl',
            executable='robotic_arm',
            output='screen'
        ),

    ])