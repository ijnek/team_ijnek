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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Launch arguments
    launch_arguments = [
        DeclareLaunchArgument('team_num', default_value='101'),
        DeclareLaunchArgument('player_num', default_value='2'),
    ]

    # Game Controller communication
    game_controller_spl_node = Node(
        package='game_controller_spl',
        executable='game_controller_spl',
        parameters=[{
            'rcgcd_version': 15,
            'rcgcrd_version': 4,
        }])

    # Robot to Robot Communication
    r2r_spl_node = Node(
        package='r2r_spl',
        executable='r2r_spl',
        parameters=[{
            'team_num': LaunchConfiguration('team_num'),
            'player_num': LaunchConfiguration('player_num'),
            'msg_type': 'communication_interfaces.msg.R2R',
            'filter_own': True,
        }]
    )

    return LaunchDescription([
        *launch_arguments,
        game_controller_spl_node,
        r2r_spl_node,
    ])
