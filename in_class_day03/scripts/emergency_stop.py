#!/usr/bin/env python

""" Emergency stop using the Neato's bump sensors """

import rospy
from neato_node.msg import Bump
from geometry_msgs.msg import Twist

class EmergencyStop(object):
    def __init__(self):
        rospy.init_node('emergency_stop')
        rospy.Subscriber('/bump', Bump, self.process_bump)
        self.twist = Twist()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.go_speed = 0.5
        self.has_bumped = False

    def process_bump(self, msg):
        sensors = (
            msg.leftFront,
            msg.leftSide,
            msg.rightFront,
            msg.rightSide
        )
        self.has_bumped = (1 in sensors)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.has_bumped:
                self.twist.linear.x = 0
            else:
                self.twist.linear.x = self.go_speed

            self.publisher.publish(self.twist)
            r.sleep()

if __name__ == '__main__':
    node = EmergencyStop()
    node.run()
