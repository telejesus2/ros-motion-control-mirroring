import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion


#Setting OpenPose parameters
def set_params():

        params = dict()
        params["logging_level"] = 3
        params["output_resolution"] = "-1x-1"
        params["net_resolution"] = "-1x368"
        params["alpha_pose"] = 0.6
        params["scale_gap"] = 0.3
        params["scale_number"] = 1
        params["render_threshold"] = 0.05
        # If GPU version is built, and multiple GPUs are available, set the ID here
        params["num_gpu_start"] = 0
        params["disable_blending"] = False
        # Ensure you point to the correct path where models are located
        params["model_folder"] = "/home/aimove/openpose/models/"
	params["number_people_max"] = 1
	params["model_pose"] = "COCO"
	#params["keypoint_scale"] = 4
	params["hand"] = True
	params["hand_detector"] = 0

        return params


#Euclidean pixel length of the arm
def arm_length(hand, elbow, shoulder):
	return np.linalg.norm(hand - elbow) + np.linalg.norm(elbow - shoulder)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2' """

    def unit_vector(vector):
    	""" Returns the unit vector of the vector.  """
    	return vector / np.linalg.norm(vector)

    # v1_u = unit_vector(v1)
    # v2_u = unit_vector(v2)
    # return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
    return np.math.atan2(np.linalg.det([v1,v2]),np.dot(v1,v2))

UPPER_ARM_LENGTH = 0.24365	
FOREARM_LENGTH = 0.21325
WRIST_2_LENGTH = 0.08535
SHOULDER_HEIGHT = 0.1519	
PINZA_LENGTH = 0.33
SHOULDER_OFFSET = 0.1198

def security_halt(joints):
	threshold = 0.02
	theta = joints[1]
	phi = joints[2]
	psi = joints[3]
	y = SHOULDER_HEIGHT - UPPER_ARM_LENGTH * np.sin(-np.pi - theta)
	if y < threshold :
		return True
	z = y + FOREARM_LENGTH * np.sin(np.pi + theta + phi)
	if z < threshold :
		return True
	w = z + WRIST_2_LENGTH * np.sin(3*np.pi/2 + psi + phi + theta)
	if w < threshold :
		return True
	h = w - PINZA_LENGTH * np.sin(-np.pi - psi - phi - theta)
	if h < threshold :
		return True
	return False

def tcp_orientation(joints):
	alpha = joints[0]
	theta = joints[1]
	phi = joints[2]

	z1 = SHOULDER_HEIGHT - UPPER_ARM_LENGTH * np.sin(-np.pi - theta)
	z2 = z1 + FOREARM_LENGTH * np.sin(np.pi + theta + phi)
	x1 = UPPER_ARM_LENGTH*np.sin(theta + 3*np.pi/2)
	x2 = FOREARM_LENGTH*np.cos(theta + phi + np.pi)

	A = np.array([- np.sin(alpha)*SHOULDER_OFFSET + np.cos(alpha)*x1,  np.cos(alpha)*SHOULDER_OFFSET +np.sin(alpha)*x1,  z1])
	B = np.array([- np.sin(alpha)*SHOULDER_OFFSET + np.cos(alpha)*(x1+x2),  np.cos(alpha)*SHOULDER_OFFSET +np.sin(alpha)*(x1+x2),  z2])
	u =  A - B
	return [+0.707106*np.sin(alpha), -0.707106*np.cos(alpha),0,0.707106]




