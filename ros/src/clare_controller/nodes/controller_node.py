#!/usr/bin/env python
 
import rospy
import time
from std_msgs.msg import String, Bool, Float32
import argparse

class ClareController(object):
    def __init__(self) -> None:
        pass

    def start(self):
        pass


if __name__ == '__main__':
    rospy.init_node('clare_controller', anonymous=False)

    c = ClareController()

    while not rospy.is_shutdown():
        rospy.spin()
