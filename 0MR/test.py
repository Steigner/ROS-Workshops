#!/usr/bin/env python3

"""
## =========================================================================== 
Author   : Roman Parak
Email    : Roman.Parak@outlook.com
Github   : https://github.com/rparak

File Name: test.py

Forked   : Martin Juricek
Github   : https://github.com/Steigner
## ===========================================================================
"""

# System (Default Lib.)
import sys

# Python client library for ROS
import rospy

# Data types (messages, services)
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty 
from turtlesim.srv import SetPen, TeleportAbsolute

def move(lin_vel_x, ang_vel_th):
    """
    Description:
        TurtleSim robot velocity control function. A simple demonstration of working with parameters, services, etc.

        More information about Turtlesim at:
        http://wiki.ros.org/turtlesim
    Args:
        (1) lin_vel_x [Float]: Linear Velocity (x).
        (2) ang_vel_th [Float]: Angular Velocity (theta).
    """
  
    # Turtlesim Node initialization (new)
    rospy.init_node('turtle_init', anonymous=True)

    # Publisher initialization
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    rospy.loginfo('Hellou from control script!')

    # Reset Environment
    rospy.wait_for_service('/reset')
    reset_simulator   = rospy.ServiceProxy('/reset', Empty)
    reset_simulator()

    # Clear Environment
    rospy.wait_for_service('/clear')
    clear_simulator   = rospy.ServiceProxy('/clear', Empty)

    # Turtle Teleport (Absolute)
    rospy.wait_for_service('/turtle1/teleport_absolute')
    turtle_teleport = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)

    # Pen Color
    rospy.wait_for_service('/turtle1/set_pen')
    set_pen_simulator = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
    
    # Call Functions
    turtle_teleport(5, 5, 0)
    set_pen_simulator(250,0,0,5,0)
    clear_simulator()

    # Set the Rate (10hz)
    rate = rospy.Rate(10)

    vel_msg = Twist()

    # Set the parameters of the turtle movement
    vel_msg.linear.x  = lin_vel_x
    vel_msg.linear.y  = 0
    vel_msg.linear.z  = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = ang_vel_th

    while not rospy.is_shutdown():
        # Move (Publish parameters)
        velocity_publisher.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        # Read Parameters from Argument: rosrun t_ctrl test.py 2.0 1.0
        move(float(sys.argv[1]), float(sys.argv[2]))
    except rospy.ROSInterruptException: 
        pass