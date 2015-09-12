#!/usr/bin/env python

""" Tells the Neato to drive in a 1m by 1m square """

import rospy
from geometry_msgs.msg import Twist

# (linear, angular) velocities
FORWARD = (1, 0) # move forward
LEFT    = (0, 1) # turn in place to the left
STOP    = (0, 0) # stop

# 3.6s of going forward is ~1m and 1.4s of turning left is ~90deg.
# These numbers were found empirically. The stop delay is arbitrary, enough to
# ensure that it will actually stop.
LINEAR_DELAY = rospy.Duration(3.6)
ANGULAR_DELAY = rospy.Duration(1.4)
STOP_DELAY = rospy.Duration(1.0)

# Alternating going forward and turning left 4 times results in a square.
SEQUENCE = [(FORWARD, LINEAR_DELAY), (LEFT, ANGULAR_DELAY)] * 4
SEQUENCE.append((STOP, STOP_DELAY)) # Stop at the end

rospy.init_node('drive_square')
my_twist = Twist()
publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
r = rospy.Rate(10)

for command in SEQUENCE:
    velocities, delay = command
    my_twist.linear.x = velocities[0]
    my_twist.angular.z = velocities[1]

    # If the message is published only once, the Neato would sometimes not get
    # some of them. So, publish at 10Hz, but only for the specified duration.
    cmd_start = rospy.Time.now()
    while (rospy.Time.now() - cmd_start < delay):
        publisher.publish(my_twist)
        r.sleep()
