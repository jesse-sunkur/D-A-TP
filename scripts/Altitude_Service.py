#!/usr/bin/env python
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import numpy as np
import math
import sys

def move():
    # Starts a new node
    rospy.init_node('altitude_control_service')
    velocity_publisher = rospy.Publisher('/altitude_control_service', Twist, queue_size=10)
    alt = Twist()
    
    alt.linear.x = 0
    alt.linear.y = 0
    alt.linear.z = input("Z: ")
    alt.angular.x = 0
    alt.angular.y = 0
    alt.angular.z = 0

    while not rospy.is_shutdown():

        velocity_publisher.publish(alt)       

if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass


