#!/usr/bin/env python

""" Gets keyboard input and teleoperates the Neato accordingly """

import tty
import select
import sys
import termios
import rospy
from geometry_msgs.msg import Twist

# (linear, angular) velocities
KEYBINDINGS = {
    'w': (1, 0),  # move forward
    'a': (0, 1),  # turn in place to the left
    's': (-1, 0), # move backward
    'd': (0, -1)  # turn in place to the right
}

# Provided getKey function
def getKey():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

rospy.init_node('teleop')
my_twist = Twist()
publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

settings = termios.tcgetattr(sys.stdin)
key = None

print "Use WASD to drive the Neato, Ctrl-C to quit"

while key != '\x03':
    key = getKey()

    try:
        # Get the linear and angular velocities for the pressed key
        linear, angular = KEYBINDINGS[key]
    except KeyError:
        # Any key not bound causes it to stop completely
        linear, angular = (0, 0)

    my_twist.linear.x = linear
    my_twist.angular.z = angular

    publisher.publish(my_twist)
