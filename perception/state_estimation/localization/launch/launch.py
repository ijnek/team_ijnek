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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
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

    return launch.LaunchDescription([container])
