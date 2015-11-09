#!/usr/bin/env python

""" Provides a very simple interface that plays a specified bag_file and provides a single method
    that pauses / unpauses playback of the bag file programmatically """

import subprocess

class InterruptableBagPlayer(object):
    """ A class that allows a bag file to be played and paused / unpaused programmatically """
    def __init__(self, bag_file):
        p = subprocess.Popen(['catkin_find','rosbag','play'], stdout=subprocess.PIPE, stdin=subprocess.PIPE, stderr=subprocess.STDOUT)
        output = p.communicate()
        rosbag_path = output[0].split()[0]
        self.p2 = subprocess.Popen([rosbag_path, bag_file, '--clock'],stdout=subprocess.PIPE, stdin=subprocess.PIPE, stderr=subprocess.STDOUT)

    def toggle_pause(self):
        """ Pause the bag file playback if it is playing, unpause it if it is paused """
        if self.p2.poll() == None:
            self.p2.stdin.write(' \n')