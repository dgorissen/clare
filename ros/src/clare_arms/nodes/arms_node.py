#!/usr/bin/env python

import time
from xml.dom import INDEX_SIZE_ERR
import rospy
from std_msgs.msg import String
from clare_arms.msg import ArmMovement
from clare_arms.srv import ArmsToNeutral
from clare_common.utils import ServoController, ServoJoint


class ArmsController(ServoController):
    # Wrapper class for controlling arms. No feedback information so servo's run
    # open loop *fingers crossed*
    def __init__(self):
        super(ArmsController, self).__init__()

        rospy.init_node("clare_arms", anonymous=False, disable_signals=False)
        
        # All limits and positions are in physical space
        self._servo_map = {
            "sh_fb_left":  ServoJoint(0, neutral_pos=162),
            "sh_fb_right": ServoJoint(1, neutral_pos=18),
            "sh_ud_left": ServoJoint(7, neutral_pos=131),
            "sh_ud_right": ServoJoint(9, neutral_pos=54),
            "el_left": ServoJoint(4, neutral_pos=77),
            "el_right": ServoJoint(10, neutral_pos=83),
            "wr_left": ServoJoint(5, neutral_pos=0),
            "wr_right": ServoJoint(8, neutral_pos=4),
            "gr_left": ServoJoint(6, [50, 93], neutral_pos=93),
            "gr_right":ServoJoint(11, [50, 90], neutral_pos=90),
        }

        self._setup_servos()
        self._arm_sub = rospy.Subscriber("clare/arms", ArmMovement, self._arm_callback)
        self._arm_pub = rospy.Publisher("clare/arms/cur_position", ArmMovement, queue_size=10)
        self._arm_reset_service = rospy.Service('/clare/arms/arms_to_neutral', ArmsToNeutral, self.handle_arms_to_neutral)

    def handle_arms_to_neutral(self, req):
        try:
            self.reset_to_neutral()
        except Exception as e:
            rospy.logerr(e)
            return False
        return True
  
    def reset_to_neutral(self):
        # Do in a specific order to reduce risk of self collisions
        keys = ["gr", "wr", "el", "sh_ud", "sh_fb"]
        for k in keys:
            # Postion left/right together, leave time between joints
            kl = k + "_left"
            kr = k + "_right"
            self.set_servo(kl, self._servo_map[kl].neutral_pos)
            self.set_servo(kr, self._servo_map[kr].neutral_pos)
            time.sleep(2)
        
        self._publish_arm_state()

    def get_logical_joint_positions(self):
        res = [(k, v.get_logical_pos()) for k,v in self._servo_map.items()]
        return res
    
    def _publish_arm_state(self):
        msg = ArmMovement()
        for k, v in self.get_logical_joint_positions():
            setattr(msg, k, v)
        self._arm_pub.publish(msg)

    def _arm_callback(self, msg):
        # Set the servos directly, takes logical values

        # Shoulders
        # Make servo's rotate in the same direction (mirror image mount)
        self.set_servo_group({"sh_fb_left": msg.sh_fb_left, "sh_fb_right": msg.sh_fb_right},
                             {"sh_fb_left": True},
                             True)


        self.set_servo_group({"sh_ud_left": msg.sh_ud_left, "sh_ud_right": msg.sh_ud_right},
                             {"sh_ud_left": True},
                             True)

        # Elbows
        self.set_servo_group({"el_left": msg.el_left, "el_right": msg.el_right},
                             {},
                             True)

        # Wrist
        self.set_servo_group({"wr_left": msg.wr_left, "wr_right": msg.wr_right},
                             {},
                             True)

        # Gripper
        self.set_servo_group({"gr_left": msg.gr_left, "gr_right": msg.gr_right},
                             {},
                             True)

        # Publish latest positions
        self._publish_arm_state()
        
    def _arm_callback_synchronous(self, msg):
        # Set the servos directly, takes logical values

        # Shoulders
        # Make servo's rotate in the same direction (mirror image mount)
        self.set_servo_logical("sh_fb_left", msg.sh_fb_left, flip=True)
        self.set_servo_logical("sh_fb_right", msg.sh_fb_right)

        self.set_servo_logical("sh_ud_left", msg.sh_ud_left, flip=True)
        self.set_servo_logical("sh_ud_right", msg.sh_ud_right)

        # Elbows
        self.set_servo_logical("el_left", msg.el_left)
        self.set_servo_logical("el_right", msg.el_right)

        # Wrist
        self.set_servo_logical("wr_left", msg.wr_left)
        self.set_servo_logical("wr_right", msg.wr_right)

        # Gripper
        self.set_servo_logical("gr_left", msg.gr_left)
        self.set_servo_logical("gr_right", msg.gr_right)

        # Publish latest positions
        self._publish_arm_state()
    
    def run(self):
        rospy.logdebug('Arms node ready and listening')
        rospy.spin()

if __name__ == "__main__":
  
    n = ArmsController()
    try:
        n.run()
    except rospy.ROSInterruptException:
        pass

