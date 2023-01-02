#!/usr/bin/env python

import time
import rospy
import subprocess
from std_msgs.msg import String
from os import path

class ImgViewer(object):
    def __init__(self):
        super(ImgViewer, self).__init__()

        rospy.init_node("clare_imgview", anonymous=False, disable_signals=False)
        self._root = path.expanduser(rospy.get_param("~root", "~"))
        self._display = rospy.get_param("~display", ":0.0")
        self._imgview_sub = rospy.Subscriber("clare/imgview", String, self._imgview_callback)

    def show_image(self, impath):
        if not path.isabs(impath):
            impath = path.join(self._root, impath)

        # impath = path.abspath(impath)

        if not path.exists(impath):
            rospy.logwarn(f"Image not found {impath}")
            self.show_not_found()
        else:
            rospy.loginfo(f"Showing image {impath}")
            subprocess.call("pkill feh", shell=True)
            subprocess.call(f"DISPLAY={self._display} feh --fullscreen --hide-pointer -F {impath} &", shell=True)

    def show_not_found(self):
        self.show_image("not_found.jpg")

    def show_default(self):
        self.show_image("happy-robot.png")

    def _imgview_callback(self, msg):
        impath = msg.data
        self.show_image(impath)

    def run(self):
        self.show_default()
        rospy.logdebug('ImgView node ready and listening')
        rospy.spin()

if __name__ == "__main__":
  
    n = ImgViewer()
    try:
        n.run()
    except rospy.ROSInterruptException:
        pass
