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

from threading import Thread

from helpers.openpose import OpenPose
openpose = OpenPose()
pose = Pose()
x_fpv, y_fpv = [320, 480]

from helpers.control import Control
control = Control()
kp = 0.5

detection_freq = 1
setpoint_freq = 10

class Yaw(object):

    def __init__(self):
        rospy.init_node('yaw_node', anonymous=True)
        self.yaw = 0.0
        self.yaw_angle = 0.0

        # self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        # self.reset_simulation()

        self.img_sub = rospy.Subscriber("/drone/front_camera/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()
        self.frame = None

        self._sub = rospy.Subscriber ('/drone/gt_pose', Pose, self.pose_callback)
        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        control.takeoff()
        rospy.on_shutdown(self.shutdown) 

        # Create target_yaw detection thread
        self._thr = Thread(target=self.determine_target_yaw, args=())
        self._thr.daemon = True
        self._thr.start()

        self.publish_yaw_velocity()
    
    def camera_callback(self,data):
        try:
            cv_img = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        self.frame = cv_img

    def pose_callback(self, data):
        orientation = data.orientation
        self.yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]

    def determine_target_yaw(self):
        r = rospy.Rate(detection_freq)

        while not rospy.is_shutdown():
            if self.frame is not None:
                frame = deepcopy(self.frame)

                start_time = time.time()
                points = openpose.detect(frame)
                # print("%s seconds" % (time.time() - start_time))

                # To-do: multithread or action node
                if points[11] is None: # hip point
                    continue
                else:
                    x_hip, y_hip = points[11]
                    self.yaw_angle = openpose.calcYawAngle([x_hip, y_hip])*pi/180

            r.sleep()

    def publish_yaw_velocity(self):
        r = rospy.Rate(setpoint_freq)
        move_msg = Twist()

        # required factor to set velocity such that target_yaw_angle is achieved by next iteration
        # reduce factor by ratio between detection & setpoint frequencies in attempt to smooth motion
        pos_offset_to_rate_setpoint_factor = (detection_freq/setpoint_freq)*1.0/setpoint_freq
        print pos_offset_to_rate_setpoint_factor

        while not rospy.is_shutdown():
            yaw = deepcopy(self.yaw)
            target_yaw_angle = deepcopy(self.yaw_angle)
			
			# Implement EKF to interpolate

            # calculate velocity from setpoint and estimated positions
            move_msg.angular.z = pos_offset_to_rate_setpoint_factor*(target_yaw_angle - yaw)

            self._pub_cmd_vel.publish(move_msg)

            r.sleep()

    def shutdown(self):
        cv2.destroyAllWindows()
        control.land()
        self._thr.join()

def main():
    try:
        Yaw()
    except KeyboardInterrupt:
        pass
    

if __name__ == '__main__':
    main()