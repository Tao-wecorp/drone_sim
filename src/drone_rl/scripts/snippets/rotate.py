#! /usr/bin/env python

from math import *
import time
import rospy
from geometry_msgs.msg import Pose, Quaternion, Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from helpers.control import Control
control = Control()
kp = 0.5

class Rotate(object):
    def __init__(self):
        rospy.init_node('rotate_node')
        self.rate = rospy.Rate(30)

        self.yaw_angle = 80
        self.yaw = 0.0

        rospy.Subscriber ('/drone/gt_pose', Pose, self.pose_callback)
        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._move_msg = Twist()

        control.takeoff()
        rospy.on_shutdown(self.land)

        while not rospy.is_shutdown():
            yaw = self.yaw_angle*pi/180
            self._move_msg.angular.z = kp * (yaw - self.yaw)
            self._pub_cmd_vel.publish(self._move_msg)
            self.rate.sleep()

    def pose_callback(self, data):
        orientation = data.orientation
        roll, pitch, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.yaw = yaw

    def land(self):
        control.land()
        
def main():
    try:
        Rotate()
    except KeyboardInterrupt:
        pass
        

if __name__ == '__main__':
    main()