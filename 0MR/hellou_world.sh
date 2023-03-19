#!/bin/bash

# Hello World
# Keyboard Control
rosrun turtlesim turtle_teleop_key

# Info Part
# 1.
rosnode list
# 1.1
rosnode info /turtlesim

# 2
rostopic list
# 2.1
rostopic info /turtle1/pose
# 2.2
rostopic type /turtle1/pose
# 2.3
rostopic echo /turtle1/pose
# 2.4
rosmsg show turtlesim/Pose

# Control via ROS Services
# 3.                                      [x y theta]
rosservice call /turtle1/teleport_absolute 0 1 0
rosservice call /turtle1/teleport_absolute 1 1 1.57

# 4. 
rosservice call /turtle1/teleport_relative 1 0
rosservice call /turtle1/teleport_relative 2 1.57

# Control via ROS Topics
# 5.
rostopic type /turtle1/cmd_vel
# 5.1
rosmsg show geometry_msgs/Twist
# 5.2 once vs rate
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
rostopic pub -r 10 /turtle1/cmd_vel geometry_msgs/Twist "[1.0, 0.0, 0.0]" "[0.0, 0.0, 1.8]"
# Rate : 1/T
# 5.3. 
rostopic hz /turtle1/pose
# Check via rqt plot
# 5.4 
rqt_plot /turtle1/pose/x:y

# Reset
# 6.
rosservice call /reset

# 7. Graph level
rqt_graph
