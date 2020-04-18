#! /usr/bin/env python

import rospy
import time
from std_msgs.msg import Empty


class TakeOff(object):
    def __init__(self):
        rospy.init_node('takeoff_node')
        self.ctrl_c = False
        self.rate = rospy.Rate(30)

        self.takeoff_pub = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
        self.takeoff_msg = Empty()
        self.takeoff()

    def takeoff(self):
        i=0
        while not i == 3:
            self.takeoff_pub.publish(self.takeoff_msg)
            rospy.loginfo('Taking off...')
            time.sleep(1)
            i += 1

def main():
    try:
        TakeOff()
    except KeyboardInterrupt:
        pass
        

if __name__ == '__main__':
    main()
