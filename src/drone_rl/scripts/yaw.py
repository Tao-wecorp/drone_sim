#! /usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
# from std_srvs.srv import Empty
from std_msgs.msg import Empty

from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Pose, Quaternion, Twist

from copy import deepcopy
from cv_bridge import CvBridge, CvBridgeError
import cv2

from math import *
import numpy as np
import random
import time

from helpers.openpose import OpenPose
openpose = OpenPose()
pose = Pose()
x_fpv, y_fpv = [320, 480]

# from helpers.control import Control
# control = Control()


class Yaw(object):
    def __init__(self):
        rospy.init_node('yaw_node', anonymous=True)
        self.ctrl_c = False
        self.rate = rospy.Rate(30)

        # self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        # self.reset_simulation()

        self.img_sub = rospy.Subscriber("/drone/front_camera/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()
        self.frame = None
        
        self.states_sub = rospy.Subscriber("/gazebo/model_states",ModelStates,self.states_callback)
        self.robot_position = None

        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._move_msg = Twist()
        self._pub_takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
        self._takeoff_msg = Empty()

        # i=0
        # while not i == 3:
        #     self._pub_takeoff.publish(self._takeoff_msg)
        #     time.sleep(1)
        #     i += 1

        while not rospy.is_shutdown():
            if self.frame is not None:
                start_time = time.time()
                frame = deepcopy(self.frame)
                
                points = openpose.detect(frame)
                if points[11] is None:
                    continue
                else:
                    x_hip, y_hip = points[11]
                    print x_hip, y_hip
                    # yaw_angle = openpose.yaw([x_hip, y_hip])
                    # yaw = yaw_angle*pi/180
                    # print yaw_angle, yaw
                    # self.turn_drone(yaw)

                # for i in range(len(points)):
                #     if points[i] is not None:
                #         frame = cv2.circle(frame, (int(points[i][0]), int(points[i][1])), 3, (0, 255, 255), thickness=-1, lineType=cv2.FILLED)
                frame = cv2.circle(frame, (int(x_fpv), int(y_fpv)), 10, (255, 0, 255), thickness=-1, lineType=cv2.FILLED)
                cv2.imshow("", frame)
                cv2.waitKey(1)

                print("%s seconds" % (time.time() - start_time))
                time.sleep(round((time.time() - start_time), 1))
            self.rate.sleep()
    
    def camera_callback(self,data):
        try:
            cv_img = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        self.frame = cv_img

    def states_callback(self,data):
        self.robot_position = data.twist[2]
        # print self.robot_position

    def turn_drone(self, yaw):
        rospy.loginfo("Turning...")
        self._move_msg.linear.x = 0.0
        self._move_msg.linear.y = 0.0
        self._move_msg.linear.z = 0.0
        self._move_msg.angular.z = yaw
        self._pub_cmd_vel.publish(self._move_msg)


def main():
    try:
        Yaw()
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()