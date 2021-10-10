#!/usr/bin/env python
# Created by: Jesse Sunkur
# TP: Drones & Applications

import rospy
import tf.transformations
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import numpy as np
import math
import sys


if __name__ == '__main__':
    while True:

        rospy.init_node('test_control_service')

        velocity_publisher = rospy.Publisher('/waypoint_control_service', Twist, queue_size=10)
        waypoint = Twist()

        waypoint.linear.x = input("X1: ")
        waypoint.linear.y = input("Y1: ")
        waypoint.linear.z = input("Z1: ")
        waypoint.angular.x = 0
        waypoint.angular.y = 0
        waypoint.angular.z = 0

        while not rospy.is_shutdown():

            velocity_publisher.publish(waypoint)
