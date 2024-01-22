# Message to use for the robot-to-robot communication

# position and orientation of robot
# coordinates in millimeters
# 0,0 is in center of field
# +ve x-axis points towards the goal we are attempting to score on
# +ve y-axis is 90 degrees counter clockwise from the +ve x-axis
# angle in radians, 0 along the +x axis, increasing counter clockwise
float32[3] pose         # x,y,theta

# position of ball relative to the robot
# coordinates in millimeters
# 0,0 is in center of the robot
# +ve x-axis points forward from the robot
# +ve y-axis is 90 degrees counter clockwise from the +ve x-axis
float32[2] ball

# Say konnichiwa to your teammates!
string message
