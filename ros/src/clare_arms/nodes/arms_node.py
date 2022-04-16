#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import String
from clare_arms.msg import ArmMovement
from clare_common.utils import ServoController

class ArmsController(ServoController):
    def __init__(self):
        super(ArmsController, self).__init__()

        rospy.init_node("clare_arms", anonymous=False, disable_signals=False)
        
        self._servo_map = {
            # name : [index, limits, mapper_fcn]
            "sh_left_fb":  [0, [0, 180], None],
            "sh_right_fb": [1, [0, 180], None],
            "sh_left_ud":  [7, [0, 180], None],
            "sh_right_ud": [9, [0, 180], None],
            "el_left":     [4, [0, 180], None],
            "el_right":    [10, [0, 180], None],
            "wr_left":     [5, [0, 180], None],
            "wr_right":    [8, [0, 180], None],
            "gr_left": [6, [50, 93], None],
            "gr_right": [11, [50, 93], None]
        }    

        self._setup_servos()
        self._arm_sub = rospy.Subscriber("clare/arms", ArmMovement, self._arm_callback)

    def _arm_callback(self, msg):
        # Set the servos directly, takes logical values

        # Shoulders
        # Make servo's rotate in the same direction (mirror image mount)
        self.set_servo_logical("sh_left_fb", msg.sh_left_fb, flip=True)
        self.set_servo_logical("sh_right_fb", msg.sh_right_fb)

        self.set_servo_logical("sh_left_ud", msg.sh_left_ud, flip=True)
        self.set_servo_logical("sh_right_ud", msg.sh_right_ud)

        # Elbows
        self.set_servo_logical("el_left", msg.el_left, flip=False)
        self.set_servo_logical("el_right", msg.el_right)

        # Wrist
        self.set_servo_logical("wr_left", msg.wr_left, flip=False)
        self.set_servo_logical("wr_right", msg.wr_right)

        # Gripper
        self.set_servo_logical("gr_left", msg.gr_left, flip=False)
        self.set_servo_logical("gr_right", msg.gr_right)


    def run(self):
        rospy.logdebug('Arms node ready and listening')
        rospy.spin()

if __name__ == "__main__":
  
    n = ArmsController()
    try:
        n.run()
    except rospy.ROSInterruptException:
        pass

