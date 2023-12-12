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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ijnek_launch'), 'launch', 'simulation_launch.py'])))

    nao_interfaces_bridge_node = Node(
        package='nao_interfaces_bridge', executable='nao_interfaces_bridge')

    nao_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('nao_state_publisher'),
                    'launch', 'nao_state_publisher_launch.py'])))

    walk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ijnek_launch'), 'launch', 'walk_launch.py'])))

    walk_visualization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ijnek_launch'), 'launch', 'walk_visualization_launch.py'])))

    return LaunchDescription([
        simulation_launch,
        nao_interfaces_bridge_node,
        nao_state_publisher_launch,
        walk_launch,
        walk_visualization_launch,
    ])
