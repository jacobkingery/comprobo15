#!/usr/bin/env python

""" Wall bumping FSM """

import rospy
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallBumping(object):
    def __init__(self):
        rospy.init_node('wall_bumping')
        rospy.Subscriber('/bump', Bump, self.process_bump)
        rospy.Subscriber('/scan', LaserScan, self.process_laser_scan)
        self.twist = Twist()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.current_state = self.go_forward

        self.linear_speed = 0.5
        self.angular_speed = 0.5
        self.distance_limit = 0.5
        self.turn_time_limit = rospy.Duration(1)

        self.has_bumped = False
        self.is_far = False

    def process_bump(self, msg):
        sensors = (
            msg.leftFront,
            msg.leftSide,
            msg.rightFront,
            msg.rightSide
        )
        self.has_bumped = (1 in sensors)

    def process_laser_scan(self, msg):
        distances = msg.ranges[:5] + msg.ranges[-5:] # only take the readings from the front
        distances = [x for x in distances if x] # filter out the 0.0 readings

        if distances:
            min_distance = min(distances)
            self.is_far = (min_distance > self.distance_limit)

    def go_forward(self):
        if self.has_bumped:
            # If we bumped, change state to go backward
            self.current_state = self.go_backward
            return self.current_state()
        else:
            return (self.linear_speed, 0)

    def go_backward(self):
        if self.is_far:
            # If we're far enough away, change state to turn left
            self.current_state = self.turn_left
            self.turn_started_time = rospy.Time.now()
            return self.current_state()
        else:
            return (-self.linear_speed, 0)

    def turn_left(self):
        now = rospy.Time.now()
        if now - self.turn_started_time > self.turn_time_limit:
            # If it's been long enough, change state to go forward
            self.current_state = self.go_forward
            return self.current_state()
        else:
            return (0, self.angular_speed)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            linear, angular = self.current_state()
            self.twist.linear.x = linear
            self.twist.angular.z = angular

            self.publisher.publish(self.twist)
            r.sleep()

if __name__ == '__main__':
    node = WallBumping()
    node.run()
