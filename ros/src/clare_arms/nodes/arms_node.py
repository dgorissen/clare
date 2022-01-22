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
            # index, limits, mapper_fcn
            "left": [0, [0, 180],  None],
            "right": [1, [0, 180],  None]
        }    

        self._setup_servos()
        self._arm_sub = rospy.Subscriber("clare/arms", ArmMovement, self._arm_callback)

    # Takes logical values
    def set_shoulders(self, pos_left, pos_right):
        # Make servo's rotate in the same direction (mirror image mount)
        self.set_servo_logical("left", pos_left, flip=True)
        self.set_servo_logical("right", pos_right)

    def _arm_callback(self, msg):
        self.set_shoulders(msg.shoulder_left, msg.shoulder_right)

    def run(self):
        rospy.logdebug('Arms node ready and listening')
        rospy.spin()

if __name__ == "__main__":
  
    n = ArmsController()
    try:
        n.run()
    except rospy.ROSInterruptException:
        pass

