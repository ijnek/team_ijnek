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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Declare launch arguments
    simulator_arg = DeclareLaunchArgument(
        name='simulator',
        description='The simulator to use',
        default_value='simspark',
        choices=['simspark', 'webots'])

    simspark_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('simulation'), 'launch', 'simspark_launch.py'])),
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('simulator'), 'simspark')))

    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('simulation'), 'launch', 'webots_launch.py'])),
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('simulator'), 'webots')))

    return LaunchDescription([
        simulator_arg,
        simspark_launch,
        webots_launch,
    ])
