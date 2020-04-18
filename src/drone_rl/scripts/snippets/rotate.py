#! /usr/bin/env python

from math import *
import time
import rospy
from geometry_msgs.msg import Pose, Quaternion, Twist
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Rotate(object):
    def __init__(self):
        rospy.init_node('rotate_node')
        self.rate = rospy.Rate(30)
        self.yaw = 0.0

        rospy.Subscriber ('/drone/gt_pose', Pose, self.pose_callback)
        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._move_msg = Twist()
        self._pub_takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
        self._takeoff_msg = Empty()
        self._pub_land = rospy.Publisher('/drone/land', Empty, queue_size=1)
        self._land_msg = Empty()

        self.takeoff()
        rospy.on_shutdown(self.land)

        while not rospy.is_shutdown():
            yaw = 90*pi/180
            self._move_msg.angular.z = 0.5 * (yaw - self.yaw)
            self._pub_cmd_vel.publish(self._move_msg)
            self.rate.sleep()

    def pose_callback(self, data):
        orientation = data.orientation
        roll, pitch, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.yaw = yaw

    def takeoff(self):
        rospy.loginfo('Taking off...')
        i=0
        while not i == 3:
            self._pub_takeoff.publish(self._takeoff_msg)
            time.sleep(1)
            i += 1

    def land(self):
        rospy.loginfo('Landing...')
        i=0
        while not i == 3:
            self._pub_land.publish(self._land_msg)
            time.sleep(1)
            i += 1
        
def main():
    try:
        Rotate()
    except KeyboardInterrupt:
        pass
        

if __name__ == '__main__':
    main()