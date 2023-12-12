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
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

    kill_webots = ExecuteProcess(cmd=['pkill -9 webots|| true'], shell=True)
    run_webots = ExecuteProcess(cmd=['webots ~/WebotsLoLaController/worlds/nao_robocup.wbt'],
                                shell=True)
    nao_lola_client_node = Node(package='nao_lola_client', executable='nao_lola_client')

    return LaunchDescription([
        kill_webots,
        RegisterEventHandler(
            OnProcessExit(
                target_action=kill_webots,
                on_exit=[
                    LogInfo(msg='killed any lingering webots processes'),
                    run_webots,
                    TimerAction(
                        period=5.0,
                        actions=[nao_lola_client_node],
                    )
                ]
            )
        ),
    ])
