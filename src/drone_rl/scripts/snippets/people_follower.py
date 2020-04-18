#!/usr/bin/env python3
import sys
import math
# ROS imports
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# OpenPose objects
from openpose_ros_msgs.msg import PersonDetection, BodypartDetection

from numpy import interp

import random

# PID control
# TODO: Make the PID faster
#from pid_control import PID
from tail_wag_pub import TailPubWag

class IgnisPoepleFollower(object):

    def __init__(self, max_angular_speed, move_speed):

        self.person_found = False

        # This is based on the peopletracker image pixel size 224x224
        self.MAX_POS = 224
        self.POS_CENTER = self.MAX_POS / 2

        # Moving Speed
        self.MOVE_SPEED = move_speed
        self.MAX_ANGULAR_SPEED = max_angular_speed
        self.MAX_MOVEMENT_TIME = 3.0

        self.ANGULAR_DELTA_RANGES = {"max":30.0,
                                        "min":20.0}

        self.not_found_counter = 0
        self.found_counter = 0
        # Times you need to detect a person or not detect to change state
        self.delta_counter = 5

        # Camera Pan related values:
        self.CAMERA_PITCH_SIM_RANGES = {"max":0.86,
                                        "min":-1.21}
        self.pan_angle = 0.0

        # Eye distance range of detection
        self.EYE_DIST_RANGES = {"max":20.0,
                                "min":6.0}
        self.TAIL_WAG_RANGES = {"max":8.0,
                                "min":0.5}

        # Shoulder dist
        self.SHOULDER_RANGES = {"max":60.0,
                                "middle_lower": 50.0,
                                "middle_upper": 60.0,
                                "min":3.0}
        self.SHOULDER_DIST_PAN_ANGLE = {"lower": -1.1}

        self.shoulder_dist_msg = Float64()
        self.shoulder_dist_pub = rospy.Publisher("/shoulder_distance", Float64, queue_size=1)

        self.eye_dist_msg = Float64()
        self.eye_dist_pub = rospy.Publisher("/eye_distance", Float64, queue_size=1)


        self.common_rate = rospy.Rate(10)
        self.global_detection_topic = '/ignis_peopletracker_ros/global_detection'
        #self._check_global_detection_ready()
        rospy.Subscriber(self.global_detection_topic, BodypartDetection, self.global_persondetection_callback)

        self.poses_keypoints_topic = '/ignis_peopletracker_ros/detected_poses_keypoints'
        #self._check_poses_keypoints_ready()
        rospy.Subscriber(self.poses_keypoints_topic, PersonDetection, self.poses_keypoints_callback)

        # Movement Topics publishers
        self.camera_tilt_pub = rospy.Publisher("/jetdog/camera_tilt_joint_position_controller/command", Float64, queue_size=1)
        self.tail_wag_obj = TailPubWag()
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # We set Robot to start searching
        self.move_camera_pan(-0.1)

        rospy.loginfo("IgnisPoepleFollower READY")



    def _check_global_detection_ready(self):
        self.global_detection = None
        rospy.logdebug("Waiting for "+self.global_detection_topic+" to be READY...")
        while self.global_detection is None and not rospy.is_shutdown():
            try:
                self.global_detection = rospy.wait_for_message(self.global_detection_topic, BodypartDetection, timeout=1.0)
                rospy.logdebug("Current "+self.global_detection_topic+" READY=>")

            except:
                rospy.logerr("Current "+self.global_detection_topic+" not ready yet, retrying...")

        self.check_person_found(self.global_detection)


    def _check_poses_keypoints_ready(self):
        self.poses_keypoints = None
        rospy.logdebug("Waiting for "+self.poses_keypoints_topic+" to be READY...")
        while self.poses_keypoints is None and not rospy.is_shutdown():
            try:
                self.poses_keypoints = rospy.wait_for_message(self.poses_keypoints_topic, PersonDetection, timeout=1.0)
                rospy.logdebug("Current "+self.poses_keypoints_topic+" READY=>")

            except:
                rospy.logerr("Current "+self.poses_keypoints_topic+" not ready yet, retrying...")

        self.check_nose_found(self.poses_keypoints)

    def check_nose_found(self, poses_msg):

        self.nose_found = poses_msg.nose.x != 0 or poses_msg.nose.y != 0


    def check_person_found(self, bodypart_msg):

        if bodypart_msg.x != 0 or bodypart_msg.y != 0:
            if self.not_found_counter > 0:
                self.not_found_counter -= 1
            self.found_counter += 1
            # To avoid value getting too big
            if self.found_counter > self.delta_counter:
                self.found_counter = self.delta_counter
        else:
            self.not_found_counter += 1
            # To avoid value getting too big
            if self.not_found_counter > self.delta_counter:
                self.not_found_counter = self.delta_counter

            if self.found_counter > 0:
                self.found_counter -= 1

        if self.found_counter >= self.delta_counter:
            self.not_found_counter = 0
            self.person_found = True
            rospy.logdebug("Person FOUND!")
        elif self.not_found_counter >= self.delta_counter:
            self.found_counter = 0
            self.person_found = False
            rospy.logdebug("PersonLost")
        else:
            rospy.logdebug("No Person change")


    def global_persondetection_callback(self, msg):
        """
        This call back processes theglobal person detection.
        This is a mean of all the body parts detection of all the people detected.
        Its the person detection equivalent to blobs.
        """
        self.global_detection = msg
        self.check_person_found(self.global_detection)


    def poses_keypoints_callback(self, msg):
        """
        Get positions of all the keypoints of the person detected and has the Person Id
        and number of persons detected
        """
        self.poses_keypoints = msg
        self.check_nose_found(self.poses_keypoints)

    def reset_movements(self):
        """
        We stop everything and set camera to a neutral position
        """
        self.move_cmd_vel(linear=0.0,angular=0.0)
        self.tail_wag_obj.tail_hz_update(hz_value=0.0)
        self.pan_angle = self.CAMERA_PITCH_SIM_RANGES["min"] / 2.0
        self.move_camera_pan(self.pan_angle)

    def wait_for_time(self, time_seconds):

        init_seconds = rospy.get_time()
        now_seconds = rospy.get_time()
        delta = now_seconds - init_seconds
        wait_rate = rospy.Rate(10)
        while delta < time_seconds and not rospy.is_shutdown() and not self.person_found:
            wait_rate.sleep()
            now_seconds = rospy.get_time()
            delta = now_seconds - init_seconds

    def move_randomly(self):
        """
        Turns and moves forwards a random ammount of time
        """
        turn_random_time = random.random() * self.MAX_MOVEMENT_TIME
        forwards_random_time = random.random() * self.MAX_MOVEMENT_TIME

        if not self.person_found:
            # Turn
            self.move_cmd_vel(linear=0.0,angular=self.MAX_ANGULAR_SPEED)
            self.wait_for_time(time_seconds=turn_random_time)
            self.move_cmd_vel(linear=0.0,angular=0.0)

        if not self.person_found:
            # Move Forwards
            self.move_cmd_vel(linear=self.MOVE_SPEED,angular=0.0)
            self.wait_for_time(time_seconds=forwards_random_time)
            self.move_cmd_vel(linear=0.0,angular=0.0)

    def person_search(self):
        rospy.logdebug("Starting PersonSearch...")
        self.reset_movements()

        while not self.person_found and not rospy.is_shutdown():
            self.move_randomly()
            if not self.person_found:
                self.common_rate.sleep()
            else:
                break

        rospy.logdebug("PersonSearch finished. PersonFound")


    def convert_pixel_to_speed(self, pixel_delta):
        """
        This converts the pixeldelta in values acceptable for speeds in real robot.
        """
        # Deltas wont exceed the pos center value
        value = interp(pixel_delta,[-self.ANGULAR_DELTA_RANGES["max"],self.ANGULAR_DELTA_RANGES["max"]], [-self.MAX_ANGULAR_SPEED,self.MAX_ANGULAR_SPEED])
        # We multiply it to magnify the speed, originals are too low.

        return value

    def convert_pixel_to_angle(self, pixel_delta):
        """
        This converts the pixeldelta in values acceptable for angles pan in real robot.
        """
        # Deltas wont exceed the pos center value
        value = interp(pixel_delta,[0.0,self.POS_CENTER], [self.CAMERA_PITCH_SIM_RANGES["min"],self.CAMERA_PITCH_SIM_RANGES["max"]])
        return value

    def move_cmd_vel(self, linear,angular):
        cmd_vel_obj = Twist()
        cmd_vel_obj.linear.x = linear
        cmd_vel_obj.angular.z = angular
        self.cmd_vel_pub.publish(cmd_vel_obj)

    def move_camera_pan(self, angle):
        pan_angle_obj = Float64()
        pan_angle_obj.data= angle
        self.camera_tilt_pub.publish(pan_angle_obj)


    def get_pan_increment(self,delta_nose_y, nominal_incr = 0.05, delta_percentage = 0.25):
        """
        Given a Delta Pixel of nose position respect the center of image
        we decide if we increment, decremet or dont change the camera pan angle.
        nominal_incr: increment of angle in radians
        """
        activation_gap = 0.25 * self.POS_CENTER

        pan_increment = 0.0
        if delta_nose_y > activation_gap:
            pan_increment = nominal_incr
        elif delta_nose_y < (-1 * activation_gap):
            pan_increment = -1*nominal_incr
        else:
            pan_increment = 0.0

        return pan_increment

    def pubish_tail_wag_based_on_eyes(self, publish_eye_dist_topic=False):
        """
        Using the distance of between the two eyes , we make the tail wag proportional
        The bigger the distance the closer the person is to the robot and therefore
        the tail wag should be faster
        """

        # If both x and y are zero, that enas there is NO detection. Although 0 value is realy rare
        right_eye_detected = self.poses_keypoints.right_eye.x != 0 or self.poses_keypoints.right_eye.y != 0
        left_eye_detected = self.poses_keypoints.left_eye.x != 0 or self.poses_keypoints.left_eye.y != 0
        both_eyes_detected = right_eye_detected and left_eye_detected
        rospy.logdebug("RE="+str(right_eye_detected)+",LE="+str(left_eye_detected))

        if both_eyes_detected:
            #rospy.logdebug(str(self.poses_keypoints.left_eye)+","+str(self.poses_keypoints.right_eye))

            x_eyes = self.poses_keypoints.left_eye.x - self.poses_keypoints.right_eye.x
            y_eyes = self.poses_keypoints.left_eye.y - self.poses_keypoints.right_eye.y


            x_eyes_2 = math.pow(x_eyes,2)
            y_eyes_2 = math.pow(y_eyes,2)
            pixel_dist_eyes = math.sqrt(x_eyes_2 + y_eyes_2)



            # Generate tail wag frequency based on eye distance
            if pixel_dist_eyes < self.EYE_DIST_RANGES["min"] and pixel_dist_eyes > 0.0:
                if self.pan_angle < self.SHOULDER_DIST_PAN_ANGLE["lower"]:
                    # You are detecting person far away, but the camera is pointing really up
                    # This means that the person is on top, therefore you should wag tail faster
                    tail_wag_frequency = self.TAIL_WAG_RANGES["max"] / 2.0
                else:
                    tail_wag_frequency = self.TAIL_WAG_RANGES["min"]

            elif pixel_dist_eyes >= self.EYE_DIST_RANGES["min"] and pixel_dist_eyes < self.EYE_DIST_RANGES["max"]:
                tail_wag_frequency = interp(pixel_dist_eyes,[self.EYE_DIST_RANGES["min"],self.EYE_DIST_RANGES["max"]], [self.TAIL_WAG_RANGES["min"],self.TAIL_WAG_RANGES["max"]])
            elif pixel_dist_eyes >= self.EYE_DIST_RANGES["max"]:
                tail_wag_frequency = self.TAIL_WAG_RANGES["max"]
            else:

                rospy.logdebug("LE-X="+str(self.poses_keypoints.left_eye.x)+",RE-X="+str(self.poses_keypoints.right_eye.x))
                rospy.logdebug("LE-Y="+str(self.poses_keypoints.left_eye.y)+",RE-Y="+str(self.poses_keypoints.right_eye.y))
                rospy.logdebug("x_eyes="+str(x_eyes)+",y_eyes="+str(y_eyes))
                # We have to check if its zero because they are the same point or because they wre not detected:
                if self.poses_keypoints.left_eye.x > 0.0 and self.poses_keypoints.right_eye.x > 0.0:
                    if self.poses_keypoints.left_eye.y > 0.0 and self.poses_keypoints.right_eye.y > 0.0:
                        tail_wag_frequency = self.TAIL_WAG_RANGES["max"] / 2.0
                else:
                    rospy.logerr("This shouldnt happen in pubish_tail_wag_based_on_eyes ")
                    tail_wag_frequency = 0.0

            rospy.logdebug("EyeDist="+str(pixel_dist_eyes))
            if publish_eye_dist_topic:
                self.eye_dist_msg.data = pixel_dist_eyes
                self.eye_dist_pub.publish(self.eye_dist_msg)

        else:
            rospy.logdebug("Both eyes were not detected!")
            tail_wag_frequency = 0.0




        #rospy.logdebug("TailWagFreq="+str(tail_wag_frequency))
        self.tail_wag_obj.tail_hz_update(hz_value=tail_wag_frequency)

    def publish_cmd_vel_based_on_detect(self, x_val, y_val, shoulder_dist_publish=False):
        """
        Based on the detections we publish the movement Fowards, backwards , turn left or right
        """


        # TURNING LEFT OR RIGHT
        # ---------------------
        # delta_x > 0 , have to turn right, delta_x = 0 centered, delta_x < 0 turn left
        delta_x =  self.POS_CENTER - x_val

        if abs(delta_x) <=  self.ANGULAR_DELTA_RANGES["max"]:
            angular = 0.0
        else:
            # Bigger than the max, then turn max throttle
            sign = lambda a: (a>0) - (a<0)
            angular = sign(delta_x) * self.MAX_ANGULAR_SPEED


        rospy.logdebug("delta_x="+str(delta_x)+", angular==>"+str(angular))
        #self.move_cmd_vel(linear=0.0,angular=angular)

        # MOVE FORWARDS OR BACKWARDS
        # We get the shoulders distance
        dist_sholders, both_parts_detected = self.distance_between_two_parts(part1=self.poses_keypoints.left_shoulder,
                                                                            part2=self.poses_keypoints.right_shoulder)
        if both_parts_detected:

            # Generate tail wag frequency based on eye distance
            if dist_sholders < self.SHOULDER_RANGES["middle_lower"] and dist_sholders > 0.0:
                # Move forward to the person
                linear_speed = self.MOVE_SPEED
                if self.pan_angle < self.SHOULDER_DIST_PAN_ANGLE["lower"]:
                    # You are detecting person far away, but the camera is pointing really up
                    # This means that the person is on top, therefore you shouldnt keep moving worwards

                    linear_speed = 0.0
                else:
                    pass

            elif dist_sholders >= self.SHOULDER_RANGES["middle_lower"] and dist_sholders < self.SHOULDER_RANGES["middle_upper"]:
                linear_speed = 0.0
            elif dist_sholders >= self.SHOULDER_RANGES["middle_upper"]:
                linear_speed = -1 * self.MOVE_SPEED
            else:
                # We have to check if its zero because they are the same point or because they wre not detected:
                if self.poses_keypoints.left_shoulder.x > 0.0 and self.poses_keypoints.right_shoulder.x > 0.0:
                    if self.poses_keypoints.left_shoulder.y > 0.0 and self.poses_keypoints.right_shoulder.y > 0.0:
                        # This meand that the two shoulder points are the same, so its really far away
                        linear_speed = self.MOVE_SPEED
                else:
                    rospy.logerr("This shouldnt happen in publish_cmd_vel_based_on_detect ")
                    linear_speed = 0.0


            rospy.logdebug("dist_sholders==="+str(dist_sholders)+",lspeed="+str(linear_speed))

            if shoulder_dist_publish:
                self.shoulder_dist_msg.data = dist_sholders
                self.shoulder_dist_pub.publish(self.shoulder_dist_msg)


        else:
            linear_speed = 0.0

        self.move_cmd_vel(linear=linear_speed,angular=angular)


    def distance_between_two_parts(self, part1, part2):

        # If both x and y are zero, that enas there is NO detection. Although 0 value is realy rare
        part1_detected = part1.x != 0 or part1.y != 0
        part2_detected = part2.x != 0 or part2.y != 0
        both_parts_detected = part1_detected and part2_detected
        rospy.logdebug("P1="+str(part1_detected)+",P2="+str(part2_detected))

        x_part = part1.x - part2.x
        y_part = part1.y - part2.y

        x_part_2 = math.pow(x_part,2)
        y_part_2 = math.pow(y_part,2)
        pixel_dist_part = math.sqrt(x_part_2 + y_part_2)

        return pixel_dist_part, both_parts_detected


    def person_follow(self):
        """
        Based on the person global_detection, we move the camera pan and the wheels
        """
        self.move_camera_pan(self.pan_angle)


        while self.person_found and not rospy.is_shutdown():
            x_val = self.global_detection.x
            y_val = self.global_detection.y
            # Due to the fact that non detection is inmidate, but considere lost no, we have to filter that
            not_null_values = x_val != 0 or y_val != 0

            if self.person_found and not_null_values:

                # LOOKING UP DOWN with Camera Pan
                if self.nose_found:
                    delta_nose_y = self.poses_keypoints.nose.y - self.POS_CENTER
                    pan_increment = self.get_pan_increment(delta_nose_y=delta_nose_y, nominal_incr = 0.06, delta_percentage = 0.25)
                    self.pan_angle += pan_increment
                    self.move_camera_pan(self.pan_angle)

                    # We publish the tail hz based on eye distance
                    self.pubish_tail_wag_based_on_eyes()

                else:
                    rospy.logdebug("Nose NOT FOUND")


                # Move Forwards or Backwards, left and right
                #---------------------------
                self.publish_cmd_vel_based_on_detect(x_val,y_val, True)


            else:
                rospy.logdebug("You need a person to follow, find it!")
                self.move_cmd_vel(linear=0.0,angular=0.0)
                self.tail_wag_obj.tail_hz_update(hz_value=0.0)

            self.common_rate.sleep()



    def start_follow_person_loop(self):

        while not self.person_found and not rospy.is_shutdown():
            self.person_search()
            self.person_follow()



if __name__ == "__main__":
    rospy.init_node("peoplet_follower_test_node", log_level=rospy.INFO)
    if len(sys.argv) < 3:
        print("usage: people_follower.py max_angular_speed move_speed")
    else:
        max_angular_speed = float(sys.argv[1])
        move_speed = float(sys.argv[2])

        pf_object = IgnisPoepleFollower(max_angular_speed, move_speed)
        pf_object.start_follow_person_loop()
