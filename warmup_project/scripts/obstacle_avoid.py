#!/usr/bin/env python

""" Obstacle-avoiding Neato using potential fields and simple proportional
    control """

import math
from numpy import exp, angle
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def angle_normalize(z):
    """ convenience function to map an angle to the range [-pi,pi] """
    return math.atan2(math.sin(z), math.cos(z))

def to_rad(d):
    """ map an angle in degrees to a normalized angle in radians """
    return angle_normalize(math.radians(d))

class ObstacleAvoid(object):
    """ A ROS node that implements a proportional controller to set the
        velocity of the Neato so that it avoids obstacles while moving
        forward """
    def __init__(self, distance=1.0):
        """ Initialize a node """
        rospy.init_node('obstacle_avoid')
        rospy.Subscriber('/scan', LaserScan, self.process_laser_scan)
        self.twist = Twist()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.forward_force = 10.0 # magnitude 10.0, angle 0.0
        self.max_distance = 1.0
        self.max_angle = 45 # degrees
        self.k_linear = 0.01 # slow it down
        self.k_angular = 0.1
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def process_laser_scan(self, msg):
        """ Process laser scan messages to locate obstacles and create a
            potential field to repel the robot from the obstacles while
            generally moving forward by proportionally setting the
            linear and angular velocities based on the sum of forces """
        # Ignore the last value (it's the same as the first) and retain angle 
        # information, only looking in front of the robot and ignoring readings
        # of 0.0 or greater than the max distance
        readings = [x for x in enumerate(msg.ranges[:-1])
            if (x[0] < self.max_angle or x[0] > (360 - self.max_angle))
            and x[1] and x[1] < self.max_distance]

        # Make the magnitude increase as distance decreases, add 180deg to the
        # angle to repel the robot, and save as complex number for summing later
        complex = [(self.max_distance - x[1]) * exp(1j * to_rad(x[0] + 180)) for x in readings]

        # Sum the obstacle forces
        obstacle_force = 0
        if complex:
            obstacle_force = sum(complex)

        # Add the result to the contant forward force
        net_force = self.forward_force + obstacle_force
        net_magnitude = abs(net_force)
        net_angle = angle(net_force)
        # If the net force is pointing behind the robot, don't move forward
        # (or backward because driving the Neatos backwards is bad)
        if abs(net_angle) > math.pi / 2:
            net_magnitude = 0

        self.linear_velocity = self.k_linear * net_magnitude
        self.angular_velocity = self.k_angular * net_angle

    def run(self):
        """ Our main 5Hz run loop """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.twist.linear.x = self.linear_velocity
            self.twist.angular.z = self.angular_velocity
            self.publisher.publish(self.twist)
            r.sleep()

if __name__ == '__main__':
    node = ObstacleAvoid()
    node.run()
