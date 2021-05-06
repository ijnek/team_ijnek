from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='walk_generator',
            executable='walk_generator'
        ),
        Node(
            package='walk_generator',
            executable='inverse_kinematics'
        ),
    ])
