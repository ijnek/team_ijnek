# Copyright 2024 Kenji Brameld
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
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('visualization'), 'rviz', 'default.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
    )

    soccer_vision_3d_rviz_markers_config_path = PathJoinSubstitution(
        [FindPackageShare('soccer_vision_3d_rviz_markers'), 'config', 'spl.yaml'])
    soccer_vision_3d_rviz_markers_node = Node(
        package='soccer_vision_3d_rviz_markers',
        executable='visualizer',
        parameters=[soccer_vision_3d_rviz_markers_config_path],
    )

    return LaunchDescription([
        rviz_node,
        soccer_vision_3d_rviz_markers_node,
    ])
