#!/usr/bin/env python
# Created by: Jesse Sunkur
# TP: Drones & Applications

import rospy
import message_filters
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
import sys

def callback(alt):
    global pose_x
    global pose_y
    global pose_z
    global heading_x
    global heading_y
    global heading_z

    pose_x = (alt.pose.position.z)
    pose_y = (alt.pose.position.z)
    pose_z = (alt.pose.position.z)
    rotation_x = (alt.pose.orientation.x)
    rotation_y = (alt.pose.orientation.y)
    rotation_z = (alt.pose.orientation.z)

def service(alt1):
    global desired_z
    desired_z = alt1.linear.z

if __name__=="__main__":
    rospy.init_node("altitude_control",anonymous=True)

    #Desired Drone Waypoints
    desired_z = 0

    rospy.Subscriber("altitude_control_service", Twist, service)

    #Initial Drone Position and Heading
    pose_x = 0
    pose_y = 0
    pose_z = 0
    rotation_x = 0
    rotation_y = 0
    rotation_z = 0

    hector = message_filters.Subscriber("/ground_truth_to_tf/pose", PoseStamped)
    ts = message_filters.ApproximateTimeSynchronizer([hector], queue_size=5, slop=0.1, allow_headerless=True)
    ts.registerCallback(callback)

    hector_namespace = "/cmd_vel"
    alt = Twist()
    kp_z = 0.95
    pub = rospy.Publisher(hector_namespace,Twist,queue_size=10)

    while not rospy.is_shutdown():
        error_z = desired_z - pose_z

        print("Error Z: ", round(error_z, 5))
        alt.linear.z = kp_z * error_z
        pub.publish(alt)

    #rospy.spin()
