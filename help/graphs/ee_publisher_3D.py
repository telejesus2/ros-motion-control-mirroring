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
pub = rospy.Publisher('ee_pos', Float64MultiArray, queue_size=1)
rospy.init_node('ee_publisher', anonymous=True)
rate = rospy.Rate(10) # 10 Hz (the camera rate is 30 Hz for the resolution 640*480)

# Creating the tf session for HMR
config2 = flags.FLAGS
config2(sys.argv)
config2.load_path = src.config.PRETRAINED_MODEL
config2.batch_size = 1

beta = 0.7
old_data = [0,0,0,0]
count = 0
model = None
start = time.time()
L=[]
T=[]
while not rospy.is_shutdown():	

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

	sent = 0
	if len(keypoints.shape)>0:
		rightHand = keypoints[0,4] 		# right hand = [x horizontal value between 0 and 639, z vertical value between 0 and 479, confidence between 0 and 1]
		rightShoulder = keypoints[0,2] 
		rightElbow = keypoints[0,3] 
		if (rightHand[2] > 0.5) and (rightElbow[2] > 0.3) and (rightShoulder[2] > 0.3): 
			
			# HMR
			model, joints3d, joints = demo2.main(color_image, keypoints, config2, model=model, vis=False)
			hmrRightWrist = joints3d[0,6]
			hmrRightElbow = joints3d[0,7]
			hmrRightShoulder = joints3d[0,8]
			hmrLeftShoulder = joints3d[0,9]
			hmrLeftElbow = joints3d[0,10]
			hmrLeftWrist = joints3d[0,11]
			
			if hmrRightShoulder[0] > hmrLeftShoulder[0] :
				print("backwards")
			else :
				armPixelLength3D = arm_length(hmrRightWrist, hmrRightElbow, hmrRightShoulder) 
				armPixelLength = arm_length(rightHand[:-1], rightElbow[:-1], rightShoulder[:-1])	
				
				u = rightShoulder[:-1] - rightHand[:-1]

				v = hmrRightShoulder[:-1] - hmrRightWrist[:-1]
				#v = np.array( [ (hmrRightShoulder[0]+1)*320 , (hmrRightShoulder[1]+1)*240 ] ) - np.array( [ (hmrRightWrist[0]+1)*320 , (hmrRightWrist[1]+1)*240 ] )
				xz_h = u * np.linalg.norm(v) / np.linalg.norm(u) * ARM_LENGTH / armPixelLength3D + np.array([0, SHOULDER_HEIGHT])

				xz_tmp = u * ARM_LENGTH / armPixelLength + np.array([0, SHOULDER_HEIGHT])

				y_h = (hmrRightShoulder[2] - hmrRightWrist[2]) * ARM_LENGTH / armPixelLength3D

				if np.abs(angle_between(u,v)) < 7.35:
					print(count)
				
					msg = Float64MultiArray()
					dim = [MultiArrayDimension()]

					dim[0].label = "height"
					dim[0].size = 4
					dim[0].stride = 1
					msg.layout.dim = dim
					msg.layout.data_offset = 0;
					msg.data = [beta*xz_h[0] + (1-beta)*old_data[0], beta*xz_h[1] + (1-beta)*old_data[1], beta*y_h + (1-beta)*old_data[2], count]
					
					old_data = msg.data
					#rospy.loginfo(msg)
					pub.publish(msg)
					sent = 1
					T += [[xz_tmp[0], xz_tmp[1], y_h]]
					L += [msg.data]
	if sent==0 :
		msg = Float64MultiArray()
		dim = [MultiArrayDimension()]
		dim[0].label = "height"
		dim[0].size = 4
		dim[0].stride = 1
		msg.layout.dim = dim
		msg.layout.data_offset = 0;
		msg.data = [-1000,0,0, count]			# send a negative message if failed
		pub.publish(msg)
		print(count)
	
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((output_image, depth_colormap))
	
        # Display the stream
        cv2.imshow('Human Pose Estimation',np.fliplr(output_image))
        key = cv2.waitKey(1)
        if key==ord('q'):
		print(time.time()-start)
		np.save('./out/ee', L)
		np.save('./out/ee_tmp', T)
                break

	#rate.sleep()
	count +=1
	


