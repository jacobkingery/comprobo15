#!/usr/bin/env python

""" This Python script shows how to use interruptable_bag_player """

from interruptable_bag_player import InterruptableBagPlayer
import sys
import rospy
from sensor_msgs.msg import LaserScan
import pdb
import rospkg

class ExampleBagPausing(object):
    """ This is an example of a ROS Node that has the ability to play a bag file and pause the bag
        file programmatically when we want to debug the program.
        Every time you want to stop the program's execution, you should toggle the pause state
        of the bag, call pdb.set_trace(), and then when you are done with the section you want to
        debug you should toggle the pause state of the bag again """

    def __init__(self):
        """ Initialize the node, right now we are hardcoding a bag file, but you could pass it in using
            a ROS argument or parameter """
        rp = rospkg.rospack.RosPack()
        rospy.init_node('mynode')
        rospy.Subscriber('scan', LaserScan, self.process_scan)
        self.bag = InterruptableBagPlayer(rp.get_path('python_debugging_practice') + "/bags/2015-09-28-15-33-57.bag")

    def run(self):
        """ the main run loop, just hangout """     
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()

    def process_scan(self, scan):
        """ Process scan data """
        print scan.header.seq
        if scan.header.seq % 10 == 0:
            # for some reason we want to debug the callback every 10th scan
            # pause the bag
            self.bag.toggle_pause()
            # enter PDB
            pdb.set_trace()
            # unpause the bag
            self.bag.toggle_pause()

if __name__ == '__main__':
    node = ExampleBagPausing()
    node.run()