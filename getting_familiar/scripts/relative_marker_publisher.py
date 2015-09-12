#!/usr/bin/env python

""" This script publishes messages for a Marker 1m in front of the Neato """

import rospy
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker

rospy.init_node('relative_marker_publisher')

# Make a white Marker at (1,0) in the base_link frame
my_marker = Marker()
my_marker.type = Marker.SPHERE
my_marker.pose.position = Point(x=1.0)
my_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
my_marker.scale = Vector3(x=0.25, y=0.25, z=0.25)

publisher = rospy.Publisher('/my_marker', Marker, queue_size=10)
r = rospy.Rate(10)

# Continuously publish the message at 10Hz
while not rospy.is_shutdown():
    # This needs to be in the while loop so that the base_link frame is updated
    # based on the Neato's current position.
    my_marker.header = Header(stamp=rospy.Time.now(), frame_id='base_link')
    publisher.publish(my_marker)
    r.sleep()
