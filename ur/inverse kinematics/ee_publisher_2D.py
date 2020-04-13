#!/usr/bin/env python

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

count = 0
while not rospy.is_shutdown():	

	# Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())

	# Output keypoints
	datum = op.Datum()
	datum.cvInputData = color_image
	opWrapper.emplaceAndPop([datum])
        keypoints, output_image = datum.poseKeypoints, datum.cvOutputData
	keypoints = keypoints.astype(np.float64)

	if len(keypoints.shape)>0:
		rightHand = keypoints[0,4]
		rightShoulder = keypoints[0,2]
		rightElbow = keypoints[0,3]
		if (rightHand[2] > 0.5) and (rightElbow[2] > 0.3) and (rightShoulder[2] > 0.3): 
			
			armPixelLength = arm_length(rightHand[:-1], rightElbow[:-1], rightShoulder[:-1])

			xz_h = (rightShoulder[:-1] - rightHand[:-1]) * ARM_LENGTH / armPixelLength + np.array([0, SHOULDER_HEIGHT])

			msg = Float64MultiArray()
			dim = [MultiArrayDimension()]

			dim[0].label = "height"
			dim[0].size = 3
			dim[0].stride = 1
			msg.layout.dim = dim
			msg.layout.data_offset = 0;
			msg.data = [xz_h[0], xz_h[1], count]

			rospy.loginfo(msg)
			pub.publish(msg)

        # Display the stream
        cv2.imshow('Human Pose Estimation',np.fliplr(output_image))
        key = cv2.waitKey(1)
        if key==ord('q'):
                break

	rate.sleep()
	count +=1

