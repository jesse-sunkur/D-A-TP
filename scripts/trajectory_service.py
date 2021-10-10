#!/usr/bin/env python
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import numpy as np
import math
import sys

def trajectory():
    # Starts a new node
    rospy.init_node('trajectory_control_service')
    velocity_publisher = rospy.Publisher('/trajectory_control_service', Twist, queue_size=10)
    traj = Twist()
    
    traj.linear.x = input("X: ")
    traj.linear.y = input("Y: ")
    traj.linear.z = 23.0
    traj.angular.x = 0
    traj.angular.y = 0
    traj.angular.z = 0

    while not rospy.is_shutdown():

        velocity_publisher.publish(traj)       

if __name__ == '__main__':
    try:
        #Calling the Trajectory Function
        trajectory()
    except rospy.ROSInterruptException: pass


