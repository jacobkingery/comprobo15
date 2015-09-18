#!/usr/bin/env python

""" This ROS node uses proportional control to turn a robot by a specified 
    angle """

from numpy import mean
import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def convert_pose_to_xy_and_theta(pose):
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x,
                         pose.orientation.y,
                         pose.orientation.z,
                         pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return pose.position.x, pose.position.y, angles[2]

def angle_normalize(z):
    """ convenience function to map an angle to the range [-pi,pi] """
    return math.atan2(math.sin(z), math.cos(z))

def angle_diff(a, b):
    """ Calculates the difference between angle a and angle b (both should be in radians)
        the difference is always based on the closest rotation from angle a to angle b
        examples:
            angle_diff(.1,.2) -> -.1
            angle_diff(.1, 2*math.pi - .1) -> .2
            angle_diff(.1, .2+2*math.pi) -> -.1
    """
    a = angle_normalize(a)
    b = angle_normalize(b)
    d1 = a-b
    d2 = 2*math.pi - math.fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if math.fabs(d1) < math.fabs(d2):
        return d1
    else:
        return d2

class TurnByAngle(object):
    """ A ROS node that implements a proportional controller to turn the robot
        by a specified angle """
    def __init__(self, angle=0.5):
        """ Initialize a node with the specified angle (in radians) """
        rospy.init_node('turn_by_angle')
        rospy.Subscriber('/odom', Odometry, self.process_odom)
        self.twist = Twist()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.target_amount = angle
        self.amount_turned = 0.0
        self.last_angle = 0.0
        self.initial_angle = True
        self.angular_velocity = 0.0

    def process_odom(self, msg):
        """ Process odometry messages to extract the current angle and
            convert this to an angular velocity that is proportional (K = 1)
            to the error of how far the robot should turn compared to how far
            it has turned """
        x, y, theta = convert_pose_to_xy_and_theta(msg.pose.pose)

        # Use the first message to set the initial angle
        if self.initial_angle:
            self.last_angle = theta
            self.initial_angle = False
            return

        # See how far we turned since last time (taking potential wraparounds into account)
        delta = angle_diff(theta, self.last_angle)
        self.amount_turned += delta

        self.last_angle = theta
        self.angular_velocity = self.target_amount - self.amount_turned

    def run(self):
        """ Our main 5Hz run loop """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.twist.angular.z = self.angular_velocity
            self.publisher.publish(self.twist)
            r.sleep()

if __name__ == '__main__':
    node = TurnByAngle(3 * math.pi)
    node.run()
