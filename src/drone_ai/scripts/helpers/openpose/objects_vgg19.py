#! /usr/bin/env python

import cv2
import os
import rospy
import rospkg
import time
from math import *

rospack = rospkg.RosPack()
openpose_folder = os.path.join(rospack.get_path("drone_ai"), "scripts/helpers/openpose/models/")
protoFile = openpose_folder + "pose_deploy_linevec.prototxt"
weightsFile = openpose_folder + "pose_iter_440000.caffemodel"
net = cv2.dnn.readNetFromCaffe(protoFile, weightsFile)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
nPoints = 18
threshold = 0.1
inputSize = 300

# COCO Output Format
keypointsMapping = ['Nose', 'Neck', 'R-Sho', 'R-Elb', 'R-Wr', 'L-Sho', 
                    'L-Elb', 'L-Wr', 'R-Hip', 'R-Knee', 'R-Ank', 'L-Hip', 
                    'L-Knee', 'L-Ank', 'R-Eye', 'L-Eye', 'R-Ear', 'L-Ear']

POSE_PAIRS = [[1,2], [1,5], [2,3], [3,4], [5,6], [6,7],
              [1,8], [8,9], [9,10], [1,11], [11,12], [12,13],
              [1,0], [0,14], [14,16], [0,15], [15,17],
              [2,17], [5,16] ]

# index of pafs correspoding to the POSE_PAIRS
mapIdx = [[31,32], [39,40], [33,34], [35,36], [41,42], [43,44], 
          [19,20], [21,22], [23,24], [25,26], [27,28], [29,30], 
          [47,48], [49,50], [53,54], [51,52], [55,56], 
          [37,38], [45,46]]

colors = [ [0,100,255], [0,100,255], [0,255,255], [0,100,255], [0,255,255], [0,100,255],
         [0,255,0], [255,200,100], [255,0,255], [0,255,0], [255,200,100], [255,0,255],
         [0,0,255], [255,0,0], [200,200,0], [255,0,0], [200,200,0], [0,0,0]]



class OpenPoseVGG():
    def __init__(self):
        self.goal = 0.0  # [angle]

    def detect(self, cv_image):
        frameWidth = cv_image.shape[1]
        frameHeight = cv_image.shape[0]

        inHeight = 368
        inWidth = int((inHeight/frameHeight)*frameWidth)

        net.setInput(cv2.dnn.blobFromImage(cv_image, 1.0/255, (inWidth, inHeight), (0, 0, 0), swapRB=False, crop=False))
        output = net.forward()
        output = output[:, :nPoints, :, :]

        H = output.shape[2]
        W = output.shape[3]

        points = []
        for i in range(nPoints):
            probMap = output[0, i, :, :]
            minVal, prob, minLoc, point = cv2.minMaxLoc(probMap)
            x = (frameWidth * point[0]) / W
            y = (frameHeight * point[1]) / H

            points.append((int(x), int(y)) if prob > threshold else None)

        return points

    def calcYawAngle(self, position):
        new_goal = degrees(atan(float(320-position[0])/(480-position[1])))
        # yaw = new_goal + self.goal
        yaw_angle = new_goal
        self.goal = yaw_angle
        return yaw_angle