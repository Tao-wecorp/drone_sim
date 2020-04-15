#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge, CvBridgeError
from copy import deepcopy
import os
import rospy
from sensor_msgs.msg import Image
import time

from geometry_msgs.msg import Pose, Quaternion
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState

from helpers.openpose import OpenPose
openpose = OpenPose()
from helpers.qlearning import QLearning
q = QLearning()
x_fpv, y_fpv = [320, 480]

class Pose(object):
    def __init__(self):
        rospy.init_node('pose_node', anonymous=True)

        self.state_sub = rospy.Subscriber("/gazebo/model_states",ModelStates,self.state_callback)

        self.image_sub = rospy.Subscriber("/robot/camera1/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()
        self.frame = None

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.frame is not None:
                # start_time = time.time()

                frame = deepcopy(self.frame)
                # x_hip, y_hip = openpose.detect(frame)[11]
                # yaw_angle = q.yaw([x_hip, y_hip])
                # print(yaw_angle)

                # frame = cv2.circle(frame, (int(x_hip), int(y_hip)), 3, (0, 255, 255), thickness=-1, lineType=cv2.FILLED)
                # frame = cv2.circle(frame, (int(x_fpv), int(y_fpv)), 5, (255, 0, 255), thickness=-1, lineType=cv2.FILLED)
                # cv2.imshow("", frame)
                # cv2.waitKey(1)

                # print("%s seconds" % (time.time() - start_time))
            rate.sleep()
    
    def camera_callback(self,data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        self.frame = cv_image

    def state_callback(self,data):
        try:
            jetbot_orientation = data.pose[2].orientation.w
            print(jetbot_orientation)
        except CvBridgeError as e:
            print(e)

def main():
    try:
        Pose()
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()