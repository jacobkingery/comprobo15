#!/usr/bin/env python

""" Wall-and-person-following Neato using simple proportional control and a 
    finite-state controller to switch between modes"""

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

class WallAndPersonFollow(object):
    """ A ROS node that implements a finite-state controller to switch
        between wall-following and person-following """
    def __init__(self):
        """ Initialize a node """
        rospy.init_node('wall_and_person_follow')
        rospy.Subscriber('/scan', LaserScan, self.process_laser_scan)
        self.twist = Twist()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.current_state = self.follow_wall # start out wall-following

        self.target_distance = 1.0
        self.max_distance = 1.5
        self.max_angle = math.pi / 8

        self.ranges = [0.0] * 361
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def process_laser_scan(self, msg):
        """ Save the ranges reported by the LIDAR for use by the velocity-
            setting functions """
        self.ranges = msg.ranges

    def follow_wall(self):
        """ Determine if it is angled towards or away from the wall on its left
            and proportionally (K = 1) set the angular velocity accordingly """
        # Look in the front-right area (so the wall on the left is not counted)
        # to see if a person has appeared, and switch to person-following if so
        front_right_distances = [x for x in self.ranges[-20:] if x and x < self.max_distance]
        if front_right_distances:
            self.current_state = self.follow_person
            return

        # Otherwise, do the wall-following
        front_left_distances = self.ranges[70:75] # use 5 points for glitch avoidance
        back_left_distances = self.ranges[106:111] # same thing

        errors = []
        for i in range(5):
            f = front_left_distances[i] # These variable names are too darn long
            b = back_left_distances[i]
            # Only use measurements where there wasn't a 0.0 reading from either
            if f and b:
                errors.append(f - b) 

        # Use the average for angular velocity
        if errors:
            self.angular_velocity = mean(errors)
        # Use a constant linear velocity
        self.linear_velocity = 0.1

    def follow_person(self):
        """ Find the center of mass of the object in front of the robot and
            proportionally (K = 1) set the linear and angular velocities so
            that the object stays directly ahead of the robot """
        # Ignore the last value (it's the same as the first) and retain angle information
        vectors = [(to_rad(x[0]), x[1]) for x in enumerate(self.ranges[:-1])]

        # Only keep readings within some angle range not having a reading of 0.0 or greater than some max distance
        filtered = [x for x in vectors if abs(x[0]) < self.max_angle and x[1] and x[1] < self.max_distance]

        average_angle, average_distance = 0, self.target_distance
        if filtered:
            average_angle, average_distance = mean(filtered, axis=0)
        else:
            # Switch to wall-following if nothing is seen
            self.current_state = self.follow_wall
            return

        # Set velocities based on error
        self.linear_velocity = average_distance - self.target_distance
        self.angular_velocity = average_angle # Target angle is 0

    def run(self):
        """ Our main 5Hz run loop """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.current_state()
            self.twist.linear.x = self.linear_velocity
            self.twist.angular.z = self.angular_velocity
            self.publisher.publish(self.twist)
            r.sleep()

if __name__ == '__main__':
    node = WallAndPersonFollow()
    node.run()
