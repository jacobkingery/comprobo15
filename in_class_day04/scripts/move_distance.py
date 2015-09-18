#!/usr/bin/env python

""" This ROS node uses proportional control to drive a robot by a specified 
    change in x position """

from numpy import mean
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class MoveDistance(object):
    """ A ROS node that implements a proportional controller to drive the robot
        by a specified change in x position """
    def __init__(self, distance=1.0):
        """ Initialize a node with the specified distance """
        rospy.init_node('drive_forward')
        rospy.Subscriber('/odom', Odometry, self.process_odom)
        self.twist = Twist()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.distance = distance
        self.linear_velocity = 0.0
        self.initial_x = False
        self.target_x = 0.0

    def process_odom(self, msg):
        """ Process odometry messages to extract the current position and
            convert this to a linear velocity that is proportional (K = 1)
            to the error from the target position """
        x_position = msg.pose.pose.position.x

        # Use the first message to set the initial position
        if not self.initial_x:
            self.target_x = x_position + self.distance
            self.initial_x = True

        self.linear_velocity = self.target_x - x_position

    def run(self):
        """ Our main 5Hz run loop """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.twist.linear.x = self.linear_velocity
            self.publisher.publish(self.twist)
            r.sleep()

if __name__ == '__main__':
    node = MoveDistance(0.5)
    node.run()
