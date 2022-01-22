#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import String
from adafruit_extended_bus import ExtendedI2C as I2C
from clare_neck.msg import NeckMovement
from clare_common.utils import ServoController
from adafruit_servokit import ServoKit


class NeckController(ServoController):
    def __init__(self):
        super(NeckController, self).__init__()

        rospy.init_node("clare_neck", anonymous=False, disable_signals=False)
        
        self._servo_map = {
            # index, limits, mapper_fcn
            "z": [14, [36, 90],  None],
            "y": [13, [46, 134],  None]
        }    

        self._setup_servos()
        self._neck_sub = rospy.Subscriber("clare/neck", NeckMovement, self._neck_callback)

    # Takes logical values, negative indicates None / ignore
    def set_neck(self, z, y):
        self.set_servo_logical("z", z)
        self.set_servo_logical("y", y)

    def _neck_callback(self, msg):
        self.set_neck(msg.z, msg.y)

    def run(self):
        rospy.logdebug('Neck node ready and listening')
        rospy.spin()

if __name__ == "__main__":
  
    n = NeckController()
    try:
        n.run()
    except rospy.ROSInterruptException:
        pass
