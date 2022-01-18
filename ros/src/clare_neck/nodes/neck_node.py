#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import String
from adafruit_extended_bus import ExtendedI2C as I2C
from clare_neck.msg import NeckMovement
from adafruit_servokit import ServoKit


class NeckController(object):
    def __init__(self):
        super(NeckController, self).__init__()

        rospy.init_node("clare_neck", anonymous=False, disable_signals=False)
        i2c_bus = rospy.get_param("~i2c_bus", 3)

        # Setup servo connection
        i2c = I2C(i2c_bus)
        self._servokit = ServoKit(channels=16, i2c=i2c)

        self._arm_sub = rospy.Subscriber("clare/neck", NeckMovement, self._neck_callback)

    def set_neck_pos(self, z, y):
        # Respect physical limits
        if (25 <= z <= 100) and (0 <= y <= 150):
            # Make servo's rotate in the same direction (mirror image mount)
            self._servokit.servo[12].angle = z
            self._servokit.servo[13].angle = y
        else:
            rospy.logerr(f"Invalid neck position commanded, z/up/down: {z} y/left/right: {y}")

    def _neck_callback(self, msg):
        self.set_neck_pos(msg.z_angle, msg.y_angle)

    def run(self):
        rospy.logdebug('Neck node ready and listening')
        rospy.spin()

if __name__ == "__main__":
  
    n = NeckController()
    try:
        n.run()
    except rospy.ROSInterruptException:
        pass
