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
from jog_msgs.msg import JogJoint
from geometry_msgs.msg import TwistStamped
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

rospy.init_node('jogarm_mirror', anonymous=True)
rate = rospy.Rate(10) 

pub = rospy.Publisher('jog_arm_server/joint_delta_jog_cmds', JogJoint, queue_size=1)
pub2 = rospy.Publisher('/jog_arm_server/delta_jog_cmds', TwistStamped, queue_size=1)
pub_gripper = rospy.Publisher('gripper_state', Float64MultiArray, queue_size=1)

robot = MoveGroupPythonInterface()
joint_init = [-np.pi,-2.44,0.72,-0.87,np.pi/2,np.pi]
robot.go_to_init_pos(joint_init)
for i in range(20) :
	msg = TwistStamped()
	msg.header.stamp = rospy.get_rostime()
	msg.twist.linear.x = 0
	msg.twist.linear.y = 0
	msg.twist.linear.z = 0
	msg.twist.angular.x = 0
	msg.twist.angular.y = 0
	msg.twist.angular.z = 0
	pub2.publish(msg)
	rate.sleep()
print("ok")


data_backup = 3*[0]
L = []
gripper = 1.0 # 1 open -1 close
time_gripper = 0
def callback(data):
	global L
	global data_backup
	global gripper
	global time_gripper
	
	if (data.data[-2] != gripper) and (time.time() - time_gripper > 2) :
			time.sleep(0.5)
			msg = Float64MultiArray()
			dim = [MultiArrayDimension()]
			dim[0].label = "height"
			dim[0].size = 1
			dim[0].stride = 1
			msg.layout.dim = dim
			msg.layout.data_offset = 0;
			msg.data = [data.data[-2]]
			pub_gripper.publish(msg)
			gripper = data.data[-2]
			time_gripper = time.time()
			time.sleep(0.5)
	else :
	
		#print(data.data)

		ee_pos = robot.move_group.get_current_pose().pose
		current_joints = robot.move_group.get_current_joint_values()
		L += [current_joints]
		np.save('./out/angles_robot', L)

		if security_halt(current_joints) : 
			print("security fail")
			msg = JogJoint()
			msg.header.stamp = rospy.get_rostime()	
			msg.joint_names = ['shoulder_lift_joint']
			msg.deltas = [0.05]
			pub.publish(msg)
		else :

			msg = JogJoint()
			msg.header.stamp = rospy.get_rostime()
			names = []
			deltas = []

			if data.data[-5] == 1 :			# if the base angle is correct
				data_backup[2] = -current_joints[0] + (data.data[0] - np.pi)
			names = names + [JOINT_NAMES[0]]
			deltas = deltas + [data_backup[2]]
	
			if data.data[-4] == 1:			# if the arm was found
				data_backup[:2] = [-current_joints[1] + (-data.data[1] - np.pi), -current_joints[2] + (-data.data[2])]
			names = names + [JOINT_NAMES[1], JOINT_NAMES[2]]
			deltas = deltas + data_backup[:2]

			if data.data[-3] == 1 : 		# if the hand was found
				names = names + [JOINT_NAMES[3], JOINT_NAMES[5]]
				deltas = deltas + [-current_joints[3] + (-data.data[3]), -current_joints[5] + (data.data[4] + np.pi)]

			norm_deltas = [max(min(x, 1), -1) for x in deltas]
			msg.joint_names = names
			msg.deltas = norm_deltas
			#rospy.loginfo(msg)
			pub.publish(msg)
	
	#rate.sleep()
	
rospy.Subscriber("angles_FK", Float64MultiArray, callback,  queue_size=1, buff_size=52428800)
rospy.spin()

