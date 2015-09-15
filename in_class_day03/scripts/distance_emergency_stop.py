#!/usr/bin/env python

""" Emergency stop using the Neato's LIDAR """

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class DistanceEmergencyStop(object):
    def __init__(self):
        rospy.init_node('distance_emergency_stop')
        rospy.Subscriber('/scan', LaserScan, self.process_laser_scan)
        self.twist = Twist()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.go_speed = 0.5
        self.distance_limit = 0.4
        self.is_close = False

    def process_laser_scan(self, msg):
        distances = [x for x in msg.ranges if x] # filter out the 0.0 readings

        # takes care of the case where all readings are filtered out
        if distances:
            min_distance = min(distances)
            self.is_close = (min_distance < self.distance_limit)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.is_close:
                self.twist.linear.x = 0
            else:
                self.twist.linear.x = self.go_speed

            self.publisher.publish(self.twist)
            r.sleep()

if __name__ == '__main__':
    node = DistanceEmergencyStop()
    node.run()
