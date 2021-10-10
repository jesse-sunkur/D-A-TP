#!/usr/bin/env python

import rospy
import message_filters
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
import sys

def callback(traj):
    global pose_x
    global pose_y
    global pose_z
    global heading_x
    global heading_y
    global heading_z

    pose_x = (traj.pose.position.x)
    pose_y = (traj.pose.position.y)
    pose_z = (traj.pose.position.z)
    rotation_x = (traj.pose.orientation.x)
    rotation_y = (traj.pose.orientation.y)
    rotation_z = (traj.pose.orientation.z)

def service(traj1):
    global desired_x
    global desired_y
    global desired_z
    desired_x = traj1.linear.x
    desired_y = traj1.linear.y
    desired_z = traj1.linear.z
   
if __name__=="__main__":
    rospy.init_node("trajectory_control", anonymous=True)

    #Desired Drone Waypoints
    desired_x = 0  
    desired_y = 0
    desired_z = 0
    
    rospy.Subscriber("trajectory_control_service", Twist, service)
    
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
    traj = Twist()
    kp_x = 0.95
    kp_y = 0.95
    kp_z = 0.95
    
    pub = rospy.Publisher(hector_namespace,Twist,queue_size=10)

    while not rospy.is_shutdown():
        error_x = desired_x - pose_x
        error_y = desired_y - pose_y
	error_z = desired_z - pose_z
        
       
        print("Error X: ", round(error_x, 5), "Error Y: ", round(error_y, 5), "Error Z: ", round(error_z, 5))

        traj.linear.z = kp_z * error_z
        pub.publish(traj)
        if error_z < 0.05:
            traj.linear.x = kp_x * error_x
            pub.publish(traj)
            if int(error_z < 0.5) & int(error_x < 0.6):
                traj.linear.y = kp_y * error_y
                pub.publish(traj)
        else:
            pass
    rospy.spin()





























     
