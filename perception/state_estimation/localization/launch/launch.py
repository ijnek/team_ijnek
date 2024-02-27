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

import launch
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Launch arguments
    launch_arguments = [
        DeclareLaunchArgument(name='player_number', default_value='2', description='The player number of the robot.'),
        DeclareLaunchArgument(name='field_length', default_value='9000.0', description='Full length of field (in metres)'),
        DeclareLaunchArgument(name='field_width', default_value='6000.0', description='Full width of field (in metres)'),
    ]

    # Parameter files
    pose_resetter_parameters = ParameterFile(
        param_file=PathJoinSubstitution([FindPackageShare('localization'), 'config', 'pose_resetter.yaml']),
        allow_substs=True)

    # Create composable node container
    container = ComposableNodeContainer(
        name='localization',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='localization',
                plugin='localization::TransitionDetector',
                remappings=[
                    ('rcgcd', 'gc/data'),
                    ('transition', 'localization/transition')],
                extra_arguments=[{'use_intra_process_comms': True}]),
            ComposableNode(
                package='localization',
                plugin='localization::PoseResetter',
                remappings=[
                    ('transition', 'localization/transition'),
                    ('set_poses', 'localization/set_poses')],
                parameters=[
                    {'player_number': LaunchConfiguration('player_number')},
                    pose_resetter_parameters],
                extra_arguments=[{'use_intra_process_comms': True}]),
            ComposableNode(
                package='localization',
                plugin='localization::MMCMKF',
                remappings=[
                    ('marking_array', 'soccer_vision_3d/markings'),
                    ('set_poses', 'localization/set_poses'),],
                extra_arguments=[{'use_intra_process_comms': True}])
        ]
    )

    return launch.LaunchDescription([*launch_arguments, container])
