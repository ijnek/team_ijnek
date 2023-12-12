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
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    nao_ik = Node(package='nao_ik', executable='nao_ik')

    nao_phase_provider_node = Node(package='nao_phase_provider', executable='nao_phase_provider',
                                   remappings=[('fsr', '/sensors/fsr')])

    walk_node = Node(package='walk', executable='walk', remappings=[('imu', '/sensors/imu')])

    lower_arms = ExecuteProcess(
        cmd=['ros2 topic pub --once /effectors/joint_positions '
             'nao_lola_command_msgs/msg/JointPositions '
             '"{indexes: [2, 18], positions: [1.517, 1.517]}"'],
        shell=True)

    return LaunchDescription([
        nao_ik,
        nao_phase_provider_node,
        walk_node,
        lower_arms,
    ])
