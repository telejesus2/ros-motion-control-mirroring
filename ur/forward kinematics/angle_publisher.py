#!/usr/bin/env python

# hmr
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
import sys
from absl import flags
import numpy as np
import skimage.io as io
import tensorflow as tf
from src.util import renderer as vis_util
from src.util import image as img_util
from src.util import openpose as op_util
import src.config
from src.RunModel import RunModel
import demo2
import time

from utils import *

# openpose and utilities
import pyopenpose as op
import cv2
import numpy as np
import random

# camera
import pyrealsense2 as rs

# ros
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
  
ARM_LENGTH = 0.54225		# length of the true robot in meters (wrist_2_length + forearm_length + upper_arm_length) 0.54225
SHOULDER_HEIGHT = 0.1519	# in meters

params = set_params()

#Constructing OpenPose object allocates GPU memory

opWrapper = op.WrapperPython()
opWrapper.configure(params)
opWrapper.start()
 
# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Creating the node and the topic
pub = rospy.Publisher('angles_FK', Float64MultiArray, queue_size=1)
pub_gripper = rospy.Publisher('gripper_state', Float64MultiArray, queue_size=1)
rospy.init_node('angle_publisher', anonymous=True)
rate = rospy.Rate(10) # 10 Hz (the camera rate is 30 Hz for the resolution 640*480)

# Creating the tf session for HMR
config2 = flags.FLAGS
config2(sys.argv)
config2.load_path = src.config.PRETRAINED_MODEL
config2.batch_size = 1

# calibration step
print("press ENTER when ready for calibration")

calibrated = 0
UPPER_ARM_PIXEL_LENGTH = 0
ANGLE_HAND = 0
def calibrate() :
	global calibrated
	global UPPER_ARM_PIXEL_LENGTH
	global ANGLE_HAND
	
	frames = pipeline.wait_for_frames()
	color_frame = frames.get_color_frame()
	#if not color_frame:
	#    continue
	color_image = np.asanyarray(color_frame.get_data())

	datum = op.Datum()
	datum.cvInputData = color_image
	opWrapper.emplaceAndPop([datum])
	keypoints, output_image = datum.poseKeypoints, datum.cvOutputData
	keypoints = keypoints.astype(np.float64)
	handKeypoints = datum.handKeypoints[1]	# right hand
	handKeypoints = handKeypoints.astype(np.float64)

	if len(keypoints.shape)>0:
		rightHand = keypoints[0,4]
		rightShoulder = keypoints[0,2]
		rightElbow = keypoints[0,3]
		chest = keypoints[0,1]
		heartFinger = handKeypoints[0,12]
		rightWrist = handKeypoints[0,0]
		fatFinger = handKeypoints[0,4]
		pinkyFinger = handKeypoints[0,4]

		if (rightHand[2] > 0.5) and (rightElbow[2] > 0.3) and (rightShoulder[2] > 0.3) and (fatFinger[2] > 0.3) and (pinkyFinger[2] > 0.3): 
			UPPER_ARM_PIXEL_LENGTH = np.linalg.norm(rightShoulder[:-1] - rightElbow[:-1])
			HAND_PIXEL_LENGTH = np.linalg.norm(pinkyFinger[:-1] - fatFinger[:-1])
			h = fatFinger[:-1] - rightWrist[:-1]
			u = rightHand[:-1] - rightElbow[:-1]
			#ANGLE_HAND = np.abs(angle_between(h,u))
			ANGLE_HAND = 0.785
			calibrated = 1

while calibrated == 0 :
	time.sleep(5)
	calibrate()
print("calibration done")

print(ANGLE_HAND)		
	

count = 0
model = None
old_data = 9*[0]
#L = []
gripper = 1.0 # 1 open -1 close
hand_open = 1
while not rospy.is_shutdown():

	while calibrated == 0 :
		time.sleep(5)
		calibrate()

	sent = 0

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
	#depth_image = cv2.convertScaleAbs(depth_image, alpha=0.03)
        color_image = np.asanyarray(color_frame.get_data())

	# Output keypoints
	datum = op.Datum()
	datum.cvInputData = color_image
	opWrapper.emplaceAndPop([datum])
        keypoints, output_image = datum.poseKeypoints, datum.cvOutputData
	keypoints = keypoints.astype(np.float64)

	# hand keypoints
	handKeypoints = datum.handKeypoints[1]	# right hand
	handKeypoints = handKeypoints.astype(np.float64)

	if len(keypoints.shape)>0:
		rightHand = keypoints[0,4]
		rightShoulder = keypoints[0,2]
		rightElbow = keypoints[0,3]
		chest = keypoints[0,1]
		heartFinger = handKeypoints[0,12]
		heartFinger9 = handKeypoints[0,9]
		heartFinger10 = handKeypoints[0,10]
		heartFinger11 = handKeypoints[0,11]
		pinkyFingerBase = handKeypoints[0,17]
		pinkyFingerTip = handKeypoints[0,20]
		indexFingerBase = handKeypoints[0,5]
		indexFingerTip  = handKeypoints[0,8]
		rightWrist = handKeypoints[0,0]
		fatFinger = handKeypoints[0,1] #4
		pinkyFinger = handKeypoints[0,20]
		
		found_hand = 0
		found_base = 0
		found_arm = 0

		if (rightHand[2] > 0.5) and (rightElbow[2] > 0.3) and (rightShoulder[2] > 0.3): 	# if the detection is confident enough
			
			if (heartFinger[2] > 0.3) and (fatFinger[2] > 0.3) :
				found_hand = 1

			# HMR
			model, joints3d, joints = demo2.main(color_image, keypoints, config2, model=model, vis=False)
			hmrRightWrist = joints3d[0,6]
			hmrRightElbow = joints3d[0,7]
			hmrRightShoulder = joints3d[0,8]
			hmrLeftShoulder = joints3d[0,9]
			hmrLeftElbow = joints3d[0,10]
			hmrLeftWrist = joints3d[0,11]

			
			if found_hand :
				hand_open = 1
				if np.sign(np.dot(heartFinger[:2] - rightWrist[:2] , heartFinger[:2] - heartFinger10[:2])) < 0 or np.sign(np.dot(heartFinger[:2] - rightWrist[:2] , heartFinger[:2] - heartFinger9[:2])) < 0 or np.sign(np.dot(heartFinger[:2] - rightWrist[:2] , heartFinger[:2] - heartFinger11[:2])) < 0 or np.sign(np.dot(heartFinger[:2] - rightHand[:2] , heartFinger[:2] - heartFinger10[:2])) < 0 or np.sign(np.dot(heartFinger[:2] - rightHand[:2] , heartFinger[:2] - heartFinger9[:2])) < 0 or np.sign(np.dot(heartFinger[:2] - rightHand[:2] , heartFinger[:2] - heartFinger11[:2])) < 0:
					hand_open = -1
			"""
			if found_hand :
				hand_open = 1
				if angle_between(pinkyFingerTip[:2] - pinkyFingerBase[:2], indexFingerTip[:2] - indexFingerBase[:2]) < 0.1 :
					hand_open = -1
			"""

			print(hand_open)
			"""
			if (hand_open != gripper) :
				time.sleep(0.5)
				msg = Float64MultiArray()
				dim = [MultiArrayDimension()]
				dim[0].label = "height"
				dim[0].size = 1
				dim[0].stride = 1
				msg.layout.dim = dim
				msg.layout.data_offset = 0;
				msg.data = [hand_open]
				pub_gripper.publish(msg)
				gripper = hand_open
				print("olaaaaa")
				time.sleep(0.5)
			"""
			x = np.linalg.norm(rightShoulder[:-1] - rightElbow[:-1]) / UPPER_ARM_PIXEL_LENGTH
			#angle_base = np.sign(hmrRightShoulder[2] - hmrRightWrist[2]) * np.arccos(max(min(x, 1), -1))
			#angle_base = np.sign(depth_image[int(rightHand[1]), int(rightHand[0])] - depth_image[int(rightShoulder[1]), int(rightShoulder[0])]) * np.arccos(max(min(x, 1), -1))
			#angle_base = np.sign(np.linalg.norm(pinkyFinger[:-1] - fatFinger[:-1]) - HAND_PIXEL_LENGTH) * np.arccos(max(min(x, 1), -1))
			angle_base = np.arccos(max(min(x, 1), -1))

			if angle_base <= 1 :
				found_base = 1
				found_arm = 1

			"""

			if angle_base <= 3*np.pi/8 :
				found_arm = 1
			"""
			u = rightHand[:-1] - rightElbow[:-1]
			v = rightShoulder[:-1] - rightElbow[:-1]
			w = rightShoulder[:-1] - chest[:-1]				# np.array([1,0])
			p = heartFinger[:-1] - rightWrist[:-1]
			h = fatFinger[:-1] - rightWrist[:-1]

			angle_shoulder = angle_between(v,-w) 	
			angle_elbow = angle_between(-u,v) 	
			angle_wrist = angle_between(p,u)
			angle_wrist_3 = -angle_between(h,u) * (np.pi/2) / ANGLE_HAND

			msg = Float64MultiArray()
			dim = [MultiArrayDimension()]
			dim[0].label = "height"
			dim[0].size = 10
			dim[0].stride = 1
			msg.layout.dim = dim
			msg.layout.data_offset = 0;
			msg.data = [angle_base, angle_shoulder, angle_elbow, angle_wrist, angle_wrist_3, found_base, found_arm, found_hand, hand_open, count]

			old_data = msg.data
			#rospy.loginfo(msg)
			pub.publish(msg)
			sent = 1
	else :
		calibrated = 0

	if sent == 0 :
		msg = Float64MultiArray()
		dim = [MultiArrayDimension()]
		dim[0].label = "height"
		dim[0].size = 10
		dim[0].stride = 1
		msg.layout.dim = dim
		msg.layout.data_offset = 0;
		msg.data = old_data
		pub.publish(msg)
	#L+=[msg.data]

        # Display the stream
        cv2.imshow('Human Pose Estimation',np.fliplr(output_image))
        key = cv2.waitKey(1)
        if key==ord('q'):
    		#np.save('./out/angles', L)
                break

	rate.sleep()
	count +=1

