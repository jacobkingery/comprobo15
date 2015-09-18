#!/usr/bin/env python

""" This ROS node uses proportional control to guide a robot to a specified
    distance from the obstacle immediately in front of it """

from numpy import mean
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WallApproach(object):
    """ A ROS node that implements a proportional controller to approach an obstacle
        immediately in front of the robot """
    def __init__(self, target_distance=1.0):
        """ Initialize a node with the specified target distance
            from the forward obstacle """
        rospy.init_node('wall_approach')
        rospy.Subscriber('/scan', LaserScan, self.process_laser_scan)
        self.twist = Twist()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.target_distance = target_distance
        self.linear_velocity = 0.0

    def process_laser_scan(self, msg):
        """ Process laser scan messages to extract the distance away from the
            wall in front of the robot and convert this to a linear velocity
            that is proportional (K = 1) to the error from the target distance """
        distances = msg.ranges[:5] + msg.ranges[-5:] # only take the readings from the front
        distances = [x for x in distances if x] # filter out the 0.0 readings

        if distances:
            mean_distance = mean(distances) # average the front readings for glitch avoidance
            self.linear_velocity = mean_distance - self.target_distance

    def run(self):
        """ Our main 5Hz run loop """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.twist.linear.x = self.linear_velocity
            self.publisher.publish(self.twist)
            r.sleep()

if __name__ == '__main__':
    node = WallApproach(1.5)
    node.run()
