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
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

stand_height = 0.04519 + 0.10290 + 0.100 + 0.085
hip_offset_y = 0.050


def generate_launch_description():

    urdf_file = PathJoinSubstitution(
        [FindPackageShare('nao_description'), 'urdf', 'nao.urdf'])
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_file]
    )

    nao_ik = Node(package='nao_ik', executable='nao_ik')
    nao_loopback = Node(package='nao_loopback', executable='nao_loopback')
    sole_poses_ims = Node(
      package='sole_poses_ims',
      executable='sole_poses_ims',
      parameters=[{
        'l_sole_default_y': hip_offset_y,
        'l_sole_default_z': -stand_height,
        'r_sole_default_y': -hip_offset_y,
        'r_sole_default_z': -stand_height,
      }],
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('ijnek_launch'), 'rviz', 'ik.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        sole_poses_ims,
        nao_ik,
        nao_loopback,
        rviz_node,
    ])
