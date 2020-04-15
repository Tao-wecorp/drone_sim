#! /usr/bin/env python3

import cv2
from cv_bridge import CvBridge, CvBridgeError
import os
import rospy
import rospkg
rospack = rospkg.RosPack()
from sensor_msgs.msg import Image

openpose_folder = os.path.join(rospack.get_path("jetbot_dqn"), "scripts/helpers/openpose/models/")
net = cv2.dnn.readNetFromTensorflow(openpose_folder + "graph_opt.pb")
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
nPoints = 18
threshold = 0.1
inHeight = 368

class OpenPose():
    def detect(self, cv_image):
        frameWidth = cv_image.shape[1]
        frameHeight = cv_image.shape[0]

        inWidth = int((inHeight/frameHeight)*frameWidth)
        net.setInput(cv2.dnn.blobFromImage(cv_image, 1.0, (inWidth, inHeight), (127.5, 127.5, 127.5), swapRB=True, crop=False))
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

            if (prob > threshold):
                points.append((int(x), int(y)))
            else :
                points.append(None)
        
        return points