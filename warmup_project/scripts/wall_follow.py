#!/usr/bin/env python

""" Wall-following Neato using simple proportional control """

from numpy import mean
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollow(object):
    """ A ROS node that implements a proportional controller to set the angular
        velocity of the Neato so that it aligns itself parallel to the wall on
        its left as it drives forward """
    def __init__(self):
        """ Initialize a node """
        rospy.init_node('wall_follow')
        rospy.Subscriber('/scan', LaserScan, self.process_laser_scan)
        self.twist = Twist()
        self.twist.linear.x = 0.1
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.angular_velocity = 0.0

    def process_laser_scan(self, msg):
        """ Process laser scan messages to determine if it is angled towards or
            away from the wall on its left and proportionally (K = 1) set the
            angular velocity accordingly """
        front_left_distances = msg.ranges[70:75] # use 5 points for glitch avoidance
        back_left_distances = msg.ranges[106:111] # same thing

        errors = []
        for i in range(5):
            f = front_left_distances[i] # These variable names are too darn long
            b = back_left_distances[i]
            # Only use measurements where there wasn't a 0.0 reading from either
            if f and b:
                errors.append(f - b) 

        # Use the average
        if errors:
            self.angular_velocity = mean(errors)

    def run(self):
        """ Our main 5Hz run loop """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.twist.angular.z = self.angular_velocity
            self.publisher.publish(self.twist)
            r.sleep()

if __name__ == '__main__':
    node = WallFollow()
    node.run()
