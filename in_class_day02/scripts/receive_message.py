#!/usr/bin/env python

""" Investigate receiving a message using a callback function """

import rospy
from geometry_msgs.msg import PointStamped

def process_stamped_point(msg):
    print msg.point

rospy.init_node('receive_message')
rospy.Subscriber('/my_point', PointStamped, process_stamped_point)

r = rospy.Rate(1)
while not rospy.is_shutdown():
    r.sleep()
