#!/usr/bin/env python

from MoveGroupPythonInterface import *
from utils import *

import time
import numpy as np
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from geometry_msgs.msg import TwistStamped
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion

SHOULDER_OFFSET = 0.1198

rospy.init_node('jogarm_mirror', anonymous=True)
rate = rospy.Rate(10) 

pub = rospy.Publisher('/jog_arm_server/delta_jog_cmds', TwistStamped, queue_size=1)

robot = MoveGroupPythonInterface()
joint_init = [-np.pi,-2.44,0.72,-0.87,np.pi/2,0]
robot.go_to_init_pos(joint_init)

security_mode = 0
data_backup = 0 
L = []
def callback(data):
	global data_backup
	global security_mode
	global L

	go = 1

	ee_pos = robot.move_group.get_current_pose().pose
	current_joints = robot.move_group.get_current_joint_values()

	if security_halt(current_joints) : 
		print("security fail")
		go = 0

		msg = TwistStamped()
		msg.header.stamp = rospy.get_rostime()
		msg.twist.linear.x = 0
		msg.twist.linear.y = 0
		msg.twist.linear.z = 0.2/10
		msg.twist.angular.x = 0
		msg.twist.angular.y = 0
		msg.twist.angular.z = 0

		pub.publish(msg)
		security_mode = 1

	if data.data[0] == -1000 :		# if the message is a failed message we reuse the old message
		data.data = data_backup
		if security_mode == 1 :
			go = 0
	else :
		data_backup = data.data
		L+=[ [ee_pos.position.x, ee_pos.position.y, ee_pos.position.z, current_joints[0]] ]
		np.save('./out/ee_robot', L)

	if go :
		security_mode = 0
		deltaZ = -ee_pos.position.z + max(0,data.data[1])
		deltaX = -ee_pos.position.x + data.data[0] - np.sin(current_joints[0]) * SHOULDER_OFFSET
		deltaY = -ee_pos.position.y + data.data[2] + np.cos(current_joints[0]) * SHOULDER_OFFSET

		tmp = max([np.abs(deltaX), np.abs(deltaY), np.abs(deltaZ)])
		if tmp > 1:
			deltaX = deltaX / tmp
			deltaY = deltaY / tmp
			deltaZ = deltaZ / tmp

		deltaA = [0,0,0]
		deltaA[0] = 	(-current_joints[4] + joint_init[4])*np.cos(current_joints[0])    -	(-current_joints[3] + joint_init[3])*np.sin(current_joints[0])	#+  (current_joints[5] - np.pi)
		deltaA[1] = 	(-current_joints[4] + joint_init[4])*np.sin(current_joints[0]) 	  + 	(-current_joints[3] + joint_init[3])*np.cos(current_joints[0])	#+  (current_joints[5] - np.pi)
		deltaA[2] = 	0

		tmp = max([np.abs(deltaA[0]), np.abs(deltaA[1]), np.abs(deltaA[2])])
		if tmp > 1:
			deltaA = deltaA / tmp
		
		msg = TwistStamped()
		msg.header.stamp = rospy.get_rostime()
		msg.twist.linear.x = deltaX
		msg.twist.linear.y = deltaY
		msg.twist.linear.z = deltaZ
		msg.twist.angular.x = deltaA[0]
		msg.twist.angular.y = deltaA[1]
		msg.twist.angular.z = deltaA[2]
	
		rospy.loginfo(msg)
		pub.publish(msg)

	#rate.sleep()
	
rospy.Subscriber("ee_pos", Float64MultiArray, callback,  queue_size=1, buff_size=52428800)
rospy.spin()

