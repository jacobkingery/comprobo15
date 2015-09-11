#!/usr/bin/env python

""" This script explores publishing ROS messages in ROS using Python """

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped

rospy.init_node('test_message')

my_header = Header(stamp=rospy.Time.now(), frame_id='odom')
my_point = Point(x=1.0, y=2.0)
my_point_stamped = PointStamped(header=my_header, point=my_point)

publisher = rospy.Publisher('/my_point', PointStamped, queue_size=10)
r = rospy.Rate(2)
while not rospy.is_shutdown():
    publisher.publish(my_point_stamped)
    r.sleep()
