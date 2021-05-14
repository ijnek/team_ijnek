# Copyright 2021 Kenji Brameld
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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

        # player_launch (rcss3d_agent)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('rcss3d_agent'),
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
