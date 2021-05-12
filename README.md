## NOTE: THIS PACKAGE IS UNDER HEAVY DEVELOPMENT AND NOT READY FOR USE


# team_ijnek

[![CI](https://github.com/ijnek/team_ijnek/actions/workflows/main.yml/badge.svg)](https://github.com/ijnek/team_ijnek/actions/workflows/main.yml)

Sample "team" package

These packages show how a team would use other packages to get a robot up and running.
This allows teams to "use" the packages rather than be "used" by the packages :D

Launch the robot for simulation using 

`ros2 launch team_ijnek_launch simulated_player_launch.py`

To see launch arguments, use

`ros2 launch team_ijnek_launch simulated_player_launch.py --show-args`

Currently the supported arguments are:

* **namespace** - string (default: '')
* **team** - string (default: 'ijnek')
* **player_number** - int (default: '2')
* **initial_pose_x** - double (default: '0.0')
* **initial_pose_y** - double (default: '0.0')
* **initial_pose_theta** - double (default: '0.0')
