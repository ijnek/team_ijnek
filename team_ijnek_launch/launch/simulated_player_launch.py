from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('team', default_value='ijnek'),
        DeclareLaunchArgument('player_number', default_value='2'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_theta', default_value='0.0'),

        # player_launch (naosoccer_sim)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('naosoccer_sim'),
                '/launch', '/player_launch.py']),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'team': LaunchConfiguration('team'),
                'player_number': LaunchConfiguration('player_number'),
                'initial_pose_x': LaunchConfiguration('initial_pose_x'),
                'initial_pose_y': LaunchConfiguration('initial_pose_y'),
                'initial_pose_theta': LaunchConfiguration('initial_pose_theta')
            }.items(),
        ),

        # static_pose_publisher
        Node(
            package='static_pose_publisher',
            executable='static_pose_publisher',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'initial_pose_x': LaunchConfiguration('initial_pose_x'),
                'initial_pose_y': LaunchConfiguration('initial_pose_y'),
                'initial_pose_theta': LaunchConfiguration('initial_pose_theta')
            }]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('walk_generator'),
                '/launch', '/walk_launch.py'])
        ),
    ])
