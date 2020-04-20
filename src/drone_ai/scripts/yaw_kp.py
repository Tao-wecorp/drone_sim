#! /usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_srvs.srv import Empty

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point, Pose, Quaternion, Twist

from copy import deepcopy
from cv_bridge import CvBridge, CvBridgeError
import cv2

from math import *
import numpy as np
import time

from helpers.openpose import OpenPose
openpose = OpenPose()
pose = Pose()
x_fpv, y_fpv = [320, 480]

from helpers.control import Control
control = Control()
kp = 0.5

class Yaw(object):
    def __init__(self):
        rospy.init_node('yaw_node', anonymous=True)
        self.rate = rospy.Rate(10)
        self.yaw = 0.0

        # self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        # self.reset_simulation()

        self.img_sub = rospy.Subscriber("/drone/front_camera/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()
        self.frame = None

        rospy.Subscriber ('/drone/gt_pose', Pose, self.pose_callback)
        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._move_msg = Twist()

        control.takeoff()
        rospy.on_shutdown(self.shutdown)

        while not rospy.is_shutdown():
            if self.frame is not None:
                frame = deepcopy(self.frame)
                yaw = deepcopy(self.yaw)
                
                # To-do: openpose service
                start_time = time.time()
                points = openpose.detect(frame)
                # print("%s seconds" % (time.time() - start_time))
                # time.sleep(round((time.time() - start_time), 1))

                # To-do: yaw action
                if points[11] is None: # hip point
                    continue
                else:
                    x_hip, y_hip = points[11]
                    yaw_angle = openpose.yaw([x_hip, y_hip])
                    # To-do: Investigation on proportional controller
                    self._move_msg.angular.z = kp * (yaw_angle*pi/180 - yaw)
                    self._pub_cmd_vel.publish(self._move_msg)

                for i in range(len(points)):
                    if points[i] is not None:
                        frame = cv2.circle(frame, (int(points[i][0]), int(points[i][1])), 3, (0, 255, 255), thickness=-1, lineType=cv2.FILLED)
                frame = cv2.circle(frame, (int(x_fpv), int(y_fpv)), 10, (255, 0, 255), thickness=-1, lineType=cv2.FILLED)
                cv2.imshow("", frame)
                cv2.waitKey(1)

            self.rate.sleep()
    
    def camera_callback(self,data):
        try:
            cv_img = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        self.frame = cv_img

    def pose_callback(self, data):
        orientation = data.orientation
        self.yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]

    def shutdown(self):
        cv2.destroyAllWindows()
        control.land()

def main():
    try:
        Yaw()
    except KeyboardInterrupt:
        pass
    

if __name__ == '__main__':
    main()