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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, RegisterEventHandler, \
                           TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    team_arg = DeclareLaunchArgument('team', default_value='ijnek', description='Team name')
    unum_arg = DeclareLaunchArgument('unum', default_value='2', description='Player number')
    x_arg = DeclareLaunchArgument('x', default_value='0.0',
                                  description='Initial x position of robot in meters')
    y_arg = DeclareLaunchArgument('y', default_value='0.0',
                                  description='Initial y position of robot in meters')
    theta_arg = DeclareLaunchArgument('theta', default_value='0.0',
                                      description='Initial heading of robot in degrees')

    kill_rcssserver3d = ExecuteProcess(cmd=['pkill -9 rcssserver3d || true'], shell=True)
    run_rcsoccersim3d = ExecuteProcess(cmd=['rcsoccersim3d'])
    rcss3d_nao_node = Node(
        package='rcss3d_nao',
        executable='rcss3d_nao',
        parameters=[{
            'team': LaunchConfiguration('team'),
            'unum': LaunchConfiguration('unum'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'theta': LaunchConfiguration('theta')
        }])

    return LaunchDescription([
        team_arg,
        unum_arg,
        x_arg,
        y_arg,
        theta_arg,
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
    ])
