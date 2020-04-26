#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

from copy import deepcopy
from cv_bridge import CvBridge, CvBridgeError
import cv2

from math import *
import numpy as np
import time

from helpers.openpose import OpenPoseVGG
openpose = OpenPoseVGG()

from helpers.control import Control
control = Control()

POSE_PAIRS = [[1,2], [1,5], [2,3], [3,4], [5,6], [6,7],
              [1,8], [8,9], [9,10], [1,11], [11,12], [12,13],
              [1,0], [0,14], [14,16], [0,15], [15,17],
              [2,17], [5,16] ]
colors = [ [0,100,255], [0,100,255], [0,255,255], [0,100,255], [0,255,255], [0,100,255],
         [0,255,0], [255,200,100], [255,0,255], [0,255,0], [255,200,100], [255,0,255],
         [0,0,255], [255,0,0], [200,200,0], [255,0,0], [200,200,0], [0,0,0]]

class Tracking(object):
    def __init__(self):
        rospy.init_node('tracking_node', anonymous=True)
        self.rate = rospy.Rate(10)

        rospy.Subscriber("/drone/front_camera/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()
        self.frame = None

        control.takeoff()
        rospy.on_shutdown(self.shutdown)

        while not rospy.is_shutdown():
            if self.frame is not None:
                start_time = time.time()
                frame = deepcopy(self.frame)
                
                # personwiseKeypoints,  keypoints_list= openpose.detect(frame)
                # for i in range(17):
                #     for n in range(len(personwiseKeypoints)):
                #         index = personwiseKeypoints[n][np.array(POSE_PAIRS[i])]
                #         if -1 in index:
                #             continue
                #         B = np.int32(keypoints_list[index.astype(int), 0])
                #         A = np.int32(keypoints_list[index.astype(int), 1])
                #         cv2.line(frame, (B[0], A[0]), (B[1], A[1]), colors[i], 3, cv2.LINE_AA)
                
                detected_keypoints = openpose.detect(frame)
                
                for i in range(18):
                    for j in range(len(detected_keypoints[i])):
                        cv2.circle(frame, detected_keypoints[i][j][0:2], 3, [0,0,255], -1, cv2.LINE_AA)

                cv2.imshow("", frame)
                cv2.waitKey(1)
                # print("%s seconds" % (time.time() - start_time))
            self.rate.sleep()
    
    def camera_callback(self,data):
        try:
            cv_img = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        self.frame = cv_img

    def shutdown(self):
        control.land()


def main():
    try:
        Tracking()
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    

if __name__ == '__main__':
    main()