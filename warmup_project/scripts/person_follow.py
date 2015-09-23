#!/usr/bin/env python

""" Person-following Neato using simple proportional control """

import math
from numpy import mean
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def angle_normalize(z):
    """ convenience function to map an angle to the range [-pi,pi] """
    return math.atan2(math.sin(z), math.cos(z))

def to_rad(d):
    """ map an angle in degrees to a normalized angle in radians """
    return angle_normalize(math.radians(d))

class PersonFollow(object):
    """ A ROS node that implements a proportional controller to set the
        velocity of the Neato so that it follows the thing in front of it at a
        fixed distance """
    def __init__(self, distance=1.0):
        """ Initialize a node """
        rospy.init_node('person_follow')
        rospy.Subscriber('/scan', LaserScan, self.process_laser_scan)
        self.twist = Twist()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.target_distance = distance
        self.max_distance = 1.5
        self.max_angle = math.pi / 8
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def process_laser_scan(self, msg):
        """ Process laser scan messages to find the center of mass of the
            object in front of the robot and proportionally (K = 1) set the
            linear and angular velocities so that the object stays directly
            ahead of the robot """
        # Ignore the last value (it's the same as the first) and retain angle information
        vectors = [(to_rad(x[0]), x[1]) for x in enumerate(msg.ranges[:-1])]

        # Only keep readings within some angle range not having a reading of 0.0 or greater than some max distance
        filtered = [x for x in vectors if abs(x[0]) < self.max_angle and x[1] and x[1] < self.max_distance]

        # Set defaults for no velocity if there is no object seen
        average_angle, average_distance = 0, self.target_distance
        if filtered:
            average_angle, average_distance = mean(filtered, axis=0)

        # Set velocities based on error
        self.linear_velocity = average_distance - self.target_distance
        self.angular_velocity = average_angle # Target angle is 0

    def run(self):
        """ Our main 5Hz run loop """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.twist.linear.x = self.linear_velocity
            self.twist.angular.z = self.angular_velocity
            self.publisher.publish(self.twist)
            r.sleep()

if __name__ == '__main__':
    node = PersonFollow()
    node.run()
