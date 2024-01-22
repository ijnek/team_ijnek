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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('team', default_value='ijnek'),
        DeclareLaunchArgument('unum', default_value='2'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('theta', default_value='0.0'),
        Node(
            package='rcss3d_nao',
            executable='rcss3d_nao',
            parameters=[{
                'team': LaunchConfiguration('team'),
                'unum': LaunchConfiguration('unum'),
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'theta': LaunchConfiguration('theta')
            }]),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('urdf_launch'), 'launch', 'description.launch.py']),
            launch_arguments={
                'urdf_package': 'nao_description',
                'urdf_package_path': PathJoinSubstitution(['urdf', 'nao.urdf'])}.items()),
        Node(
            package='state_estimation',
            executable='static_pose_publisher',
            parameters=[{
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'theta': LaunchConfiguration('theta')
            }]),
        Node(package='nao_ik', executable='nao_ik'),
        Node(package='crouch', executable='crouch_node'),
        Node(package='kick', executable='kick_node'),
    ])
