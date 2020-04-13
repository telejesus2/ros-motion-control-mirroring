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

rospy.init_node('jogarm_mirror', anonymous=True)
rate = rospy.Rate(10) 

pub = rospy.Publisher('/jog_arm_server/delta_jog_cmds', TwistStamped, queue_size=1)

robot = MoveGroupPythonInterface()
joint_init = [-np.pi,-2.44,0.72,-0.87,np.pi/2,0]
robot.go_to_init_pos(joint_init)

security_mode = 0
def callback(data):
	global security_mode
	#print(data.data)

	ee_pos = robot.move_group.get_current_pose().pose
	current_joints = robot.move_group.get_current_joint_values()

	go = 1
	deltaZ = -ee_pos.position.z + max(0,data.data[1])
	if security_halt(current_joints) : 
		if deltaZ < 0 :
			print("security fail")
			go = 0
			if security_mode == 0 :
				msg = TwistStamped()
				msg.header.stamp = rospy.get_rostime()
				msg.twist.linear.x = 0
				msg.twist.linear.y = 0
				msg.twist.linear.z = deltaZ/10
				msg.twist.angular.x = 0
				msg.twist.angular.y = 0
				msg.twist.angular.z = 0

				pub.publish(msg)
				security_mode = 1

	if go :
		security_mode = 0
		deltaX = -ee_pos.position.x + data.data[0]
		if np.abs(deltaX) > 1:
			deltaX = deltaX / np.abs(deltaX)
			deltaZ = deltaZ / np.abs(deltaX)
		if np.abs(deltaZ) > 1:
			deltaX = deltaX / np.abs(deltaZ)
			deltaZ = deltaZ / np.abs(deltaZ)

		msg = TwistStamped()
		msg.header.stamp = rospy.get_rostime()
		msg.twist.linear.x = deltaX
		msg.twist.linear.y = 0
		msg.twist.linear.z = deltaZ
		msg.twist.angular.x = 0
		msg.twist.angular.y = 0
		msg.twist.angular.z = 0

		#rospy.loginfo(msg)
		pub.publish(msg)

	#rate.sleep()
	
rospy.Subscriber("ee_pos", Float64MultiArray, callback,  queue_size=1, buff_size=52428800)
rospy.spin()

