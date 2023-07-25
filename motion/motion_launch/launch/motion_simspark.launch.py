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
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Declare launch arguments
    perspective_arg = DeclareLaunchArgument(
        name='perspective',
        description='Path to the perspective file to open.',
        default_value=PathJoinSubstitution(
            [FindPackageShare('walk'), 'perspective', 'walk', 'walk.perspective']))

    kill_rcssserver3d = ExecuteProcess(cmd=['pkill -9 rcssserver3d || true'], shell=True)
    run_rcsoccersim3d = ExecuteProcess(cmd=['rcsoccersim3d'])

    rcss3d_nao_node = Node(package='rcss3d_nao', executable='rcss3d_nao')
    ik_node = Node(package='nao_ik', executable='ik_node')
    nao_phase_provider_node = Node(package='nao_phase_provider', executable='nao_phase_provider',
                                   remappings=[('fsr', '/sensors/fsr')])
    walk_node = Node(package='walk', executable='walk', remappings=[('imu', '/sensors/imu')])

    nao_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('nao_state_publisher'),
                    'launch',
                    'nao_state_publisher_launch.py'])))

    nao_interfaces_bridge_node = Node(
        package='nao_interfaces_bridge', executable='nao_interfaces_bridge')

    lower_arms = ExecuteProcess(
        cmd=['ros2 topic pub --once /effectors/joint_positions nao_command_msgs/msg/JointPositions "{indexes: [2, 18], positions: [1.517, 1.517]}"'],
        shell=True)

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('motion_launch'), 'rviz', 'motion_webots.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
    )

    plotjuggler_config_launch_arg = DeclareLaunchArgument('plotjuggler_config_path',
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
        perspective_arg,
        kill_rcssserver3d,
        RegisterEventHandler(
            OnProcessExit(
                target_action=kill_rcssserver3d,
                on_exit=[
                    LogInfo(msg='killed any lingering rcssserver3d processes'),
                    run_rcsoccersim3d,
                    TimerAction(
                        period=3.0,
                        actions=[rcss3d_nao_node],
                    )
                ]
            )
        ),
        ik_node,
        nao_phase_provider_node,
        walk_node,
        nao_state_publisher_launch,
        nao_interfaces_bridge_node,
        lower_arms,
        rviz_node,
        plotjuggler_config_launch_arg,
        plotjuggler_node,
    ])
