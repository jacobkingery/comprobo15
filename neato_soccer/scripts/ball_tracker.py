#!/usr/bin/env python

""" This is a script that walks through some of the basics of working with images
    with opencv in ROS. """

import rospy
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3

class BallTracker(object):
    """ The BallTracker is a Python object that encompasses a ROS node 
        that can process images from the camera and search for a ball within.
        The node will issue motor commands to move forward while keeping
        the ball in the center of the camera's field of view. """

    def __init__(self, image_topic):
        """ Initialize the ball tracker """
        rospy.init_node('ball_tracker')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        cv2.namedWindow('video_window')
        cv2.namedWindow('threshold_image')
        cv2.namedWindow('slider_window')

        self.center_x = 0.0
        self.center_y = 0.0

        self.hue_lower_bound = 95
        cv2.createTrackbar('hue lower bound', 'slider_window', 95, 255, self.set_hue_lower_bound)
        self.hue_upper_bound = 151
        cv2.createTrackbar('hue upper bound', 'slider_window', 151, 255, self.set_hue_upper_bound)
        self.saturation_lower_bound = 127
        cv2.createTrackbar('saturation lower bound', 'slider_window', 127, 255, self.set_saturation_lower_bound)
        self.saturation_upper_bound = 255
        cv2.createTrackbar('saturation upper bound', 'slider_window', 255, 255, self.set_saturation_upper_bound)
        self.value_lower_bound = 0
        cv2.createTrackbar('value lower bound', 'slider_window', 0, 255, self.set_value_lower_bound)
        self.value_upper_bound = 38
        cv2.createTrackbar('value upper bound', 'slider_window', 38, 255, self.set_value_upper_bound)

        rospy.Subscriber(image_topic, Image, self.process_image)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def set_hue_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the hue lower bound """
        self.hue_lower_bound = val

    def set_hue_upper_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the hue upper bound """
        self.hue_upper_bound = val

    def set_saturation_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the saturation lower bound """
        self.saturation_lower_bound = val

    def set_saturation_upper_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the saturation upper bound """
        self.saturation_upper_bound = val

    def set_value_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the value lower bound """
        self.value_lower_bound = val

    def set_value_upper_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the value uppper bound """
        self.value_upper_bound = val

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imshow('video_window', self.cv_image)

        self.hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

        lower_bounds = (self.hue_lower_bound, self.saturation_lower_bound, self.value_lower_bound)
        upper_bounds = (self.hue_upper_bound, self.saturation_upper_bound, self.value_upper_bound)
        binary_image = cv2.inRange(self.hsv_image, lower_bounds, upper_bounds)

        moments = cv2.moments(binary_image)
        if moments['m00'] != 0:
            self.center_x, self.center_y = moments['m10']/moments['m00'], moments['m01']/moments['m00']
            self.center_x = self.center_x / binary_image.shape[1] - 0.5

        cv2.imshow('threshold_image', binary_image)
        cv2.waitKey(5)

    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        my_twist = Twist()
        my_twist.linear.x = 0.2
        while not rospy.is_shutdown():
        	# turn based on center of mass of ball
            my_twist.angular.z = -self.center_x
            self.pub.publish(my_twist)
            r.sleep()

if __name__ == '__main__':
    node = BallTracker("/camera/image_raw")
    node.run()
