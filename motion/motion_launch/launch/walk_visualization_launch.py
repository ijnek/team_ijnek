# Copyright 2023 Kenji Brameld
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('motion_launch'), 'rviz', 'walk.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
    )

    plotjuggler_config_launch_arg = DeclareLaunchArgument(
        'plotjuggler_config_path',
        default_value=PathJoinSubstitution(
          [FindPackageShare('motion_launch'), 'plotjuggler', 'walk.xml']))
    plotjuggler_node = Node(
        package='plotjuggler',
        executable='plotjuggler',
        arguments=[
          '--window_title', 'Walk',
          '--layout', LaunchConfiguration('plotjuggler_config_path'),
          '--nosplash',
        ],
    )

    return LaunchDescription([
        rviz_node,
        plotjuggler_config_launch_arg,
        plotjuggler_node,
    ])
