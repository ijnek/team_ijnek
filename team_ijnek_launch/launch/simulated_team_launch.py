# Copyright 2021 Kenji Brameld
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

import yaml


def generate_launch_description():

    yaml_file_name = 'teams_params.yaml'
    yaml_file = os.path.join(
        get_package_share_directory('team_ijnek_launch'),
        'config',
        yaml_file_name)
    with open(yaml_file, 'r') as infp:
        teams_params = yaml.load(infp.read(), Loader=yaml.FullLoader)

    ld = LaunchDescription()

    # We must launch the agents one after another, or the server crashes
    # Try and have a one second delay between the agent launches
    delay_seconds = 0

    for team, team_params in teams_params.items():
        print('Found team: ' + str(team))
        print(team_params)
        for player, player_params in team_params.items():
            print('Found player: ' + str(player))
            print(player_params)
            ld.add_action(
                TimerAction(period=str(delay_seconds), actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([
                            get_package_share_directory('team_ijnek_launch'),
                            '/launch', '/simulated_player_launch.py']),
                        launch_arguments={
                            'namespace': str(team) + '/player' + str(player),
                            'team': str(team),
                            'number': str(player),
                            'x': str(player_params['x']),
                            'y': str(player_params['y']),
                            'theta': str(player_params['theta']),
                        }.items(),
                    ),
                ]))

            delay_seconds = delay_seconds + 1

    return ld
