#! /usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_srvs.srv import Empty

from copy import deepcopy
from cv_bridge import CvBridge, CvBridgeError
import cv2
from cvlib.object_detection import draw_bbox

from math import *
import numpy as np
import time

from helpers.detection import YoloV3
detect_obj = YoloV3()

from helpers.tracking import DaSiamRPN
# track_obj = DaSiamRPN()

from helpers.control import Control
control_obj = Control()


class Track(object):
    def __init__(self):
        rospy.init_node('track_node', anonymous=True)
        self.rate = rospy.Rate(10)
        self.current_yaw = 0.0

        rospy.Subscriber("/drone/front_camera/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()
        self.frame = None

        control_obj.takeoff()
        rospy.on_shutdown(self.shutdown)

        while not rospy.is_shutdown():
            if self.frame is not None:
                start_time = time.time()
                frame = deepcopy(self.frame)
                current_yaw = deepcopy(self.current_yaw)
                
                bboxes, indices = detect_obj.detect(frame)

                for i in indices:
                    i = i[0]
                    bbox = bboxes[i]
                    x1, y1, x2, y2 = bbox
                    if i==0:
                        frame = cv2.rectangle(frame, (x1,y1), (x2, y2), (0, 0, 255), 2)
                    else:
                        frame = cv2.rectangle(frame, (x1,y1), (x2, y2), (0, 255, 0), 2)
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
        cv2.destroyAllWindows()
        control_obj.land()

def main():
    try:
        Track()
    except KeyboardInterrupt:
        pass
    

if __name__ == '__main__':
    main()