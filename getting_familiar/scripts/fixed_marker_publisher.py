#!/usr/bin/env python

""" This script publishes messages for a Marker at 1m, 2m """

import rospy
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker

rospy.init_node('fixed_marker_publisher')

# Put a white Marker at (1,2)
my_marker = Marker()
my_marker.type = Marker.SPHERE
my_marker.header = Header(stamp=rospy.Time.now(), frame_id='odom')
my_marker.pose.position = Point(x=1.0, y=2.0)
my_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
my_marker.scale = Vector3(x=0.25, y=0.25, z=0.25)

publisher = rospy.Publisher('/my_marker', Marker, queue_size=10)
r = rospy.Rate(10)

# Continuously publish the message at 10Hz
while not rospy.is_shutdown():
    publisher.publish(my_marker)
    r.sleep()
