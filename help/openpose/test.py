import sys
# sys.path.remove('/usr/local/python/cv2/python-2.7')
# sys.path.append('/home/aimove/openpose/build/python/openpose')

import cv2
import os
import argparse
import pyopenpose as op


# Flags
parser = argparse.ArgumentParser()
parser.add_argument("--image_path", default="/home/salwa/installations/openpose/examples/media/COCO_val2014_000000000192.jpg", help="Process an image. Read all standard formats (jpg, png, bmp, etc.).")
parser.add_argument("--no_display", default=False, help="Enable to disable the visual display.")
args = parser.parse_known_args()

params = dict()
params["model_folder"] = "/home/aimove/openpose/models/"
params["write_json"] = "./results/"
params["model_pose"] = "MPI"
# params["net_resolution"]
params["hand"] = True
params["hand_detector"] = 0
params["body"] = 1

opWrapper = op.WrapperPython()
opWrapper.configure(params)
opWrapper.start()

# Process Image
datum = op.Datum()
imageToProcess = cv2.imread(args[0].image_path)
datum.cvInputData = imageToProcess
datum.name = "image2"
opWrapper.emplaceAndPop([datum])

# Display Image
print("Body keypoints: \n" + str(datum.poseKeypoints))
cv2.imshow("OpenPose 1.4.0 - Tutorial Python API", datum.cvOutputData)
cv2.imwrite("output.png",datum.cvOutputData)
cv2.waitKey(0)
