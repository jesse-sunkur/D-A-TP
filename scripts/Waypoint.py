#!/usr/bin/env python

import rospy
import message_filters
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
import sys
import time
from sys import exit

def callback(vel):
    global pose_x
    global pose_y
    global pose_z
    global heading_x
    global heading_y
    global heading_z

    pose_x = (vel.pose.position.x)
    pose_y = (vel.pose.position.y)
    pose_z = (vel.pose.position.z)
    rotation_x = (vel.pose.orientation.x)
    rotation_y = (vel.pose.orientation.y)
    rotation_z = (vel.pose.orientation.z)

def service(waypoint):
    global desired_x1
    global desired_y1
    global desired_z1
    desired_x1 = waypoint.linear.x
    desired_y1 = waypoint.linear.y
    desired_z1 = waypoint.linear.z
    print(desired_x1)

if __name__=="__main__":
    rospy.init_node("waypoint_control", anonymous=True)

    #Desired Drone Waypoints
    desired_x1 = 0
    desired_y1 = 0
    desired_z1 = 0
    desired_x2 = 0
    desired_y2 = 0
    desired_z2 = 0

    rospy.Subscriber("waypoint_control_service", Twist, service)
    #rospy.Subscriber("waypoint_control_service", Twist, service1)

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

    hector_cmdvel = "/cmd_vel"
    traj = Twist()
    kp_x1 = 0.12
    kp_y1 = 0.08
    kp_z1 = 0.10
    kp_x2 = 0.08
    kp_y2 = 0.08
    kp_z2 = 0.15

    pub = rospy.Publisher(hector_cmdvel, Twist, queue_size=10)

    global flag
    flag = 0

    while not rospy.is_shutdown():
        #desired_x1 = 50
        #desired_y1 = 50
        #desired_z1 = 50
        #desired_x2 = 0
        #desired_y2 = 0
        #desired_z2 = 0

        error_x1 = (desired_x1 - pose_x)
        error_x2 = (desired_x2 - pose_x)
        error_y1 = (desired_y1 - pose_y)
        error_y2 = (desired_y2 - pose_y)
        error_z1 = (desired_z1 - pose_z)
        error_z2 = (desired_z2 - pose_z)

	print("x1", round(error_x1, 2), "y1", round(error_y1, 2), "z1", round(error_z1, 2), "x2", round(error_x2, 2), "y2", round(error_y2, 2), flag)


        if flag == 0 or flag == 1:
            traj.linear.z = kp_z1 * error_z1
            pub.publish(traj)


        if int(abs(error_z1) < 0.5) and int(abs(error_y1) > 0.5) and flag == 0:
            traj.linear.x = kp_x1 * error_x1
            pub.publish(traj)
            print("Pub X1")

            if int(abs(error_z1) < 0.5) and int(abs(error_x1) < 0.5) and int(abs(error_y1) > 0.5) and flag == 0:
                traj.linear.y = kp_y1 * error_y1
                pub.publish(traj)
            	print("Pub Y1")

		if int(abs(error_x1) < 0.5) and int(abs(error_y1) < 1.0):
		    flag = 1


        elif flag == 1:
            if int(abs(error_x2) > 0.1) and int(abs(error_y2) > 0.1) and int(abs(error_z1) < 1.0):
                print("Pub X2")
                traj.linear.x = kp_x2 * error_x2
                pub.publish(traj)

                if int(abs(error_x2) < 2.5) and int(abs(error_y2) <> (0.02*desired_x1)) and abs(int(error_z1) < 1.0):
                    print("Pub Y2")
                    traj.linear.y = kp_y2 * error_y2
                    pub.publish(traj)

            if int(abs(error_x2) < 0.5) and int(abs(error_y2) < 0.5):
                flag = 2


    	elif flag == 2:
    	    traj.linear.z = kp_z2 * error_z2
            pub.publish(traj)
            print("Pub Z2")

            if int(abs(error_x2) > 0.1) and int(abs(error_y2) > 0.1) and int(abs(error_z1) < 1.0):
                print("Pub X2")
                traj.linear.x = kp_x2 * error_x2
                pub.publish(traj)

                if int(abs(error_x2) < 2.5) and int(abs(error_y2) <> (0.02*desired_x1)) and abs(int(error_z1) < 1.0):
                    print("Pub Y2")
                    traj.linear.y = kp_y2 * error_y2
                    pub.publish(traj)

    rospy.spin()
