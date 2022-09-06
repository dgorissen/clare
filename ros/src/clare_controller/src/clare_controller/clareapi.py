import rospy
import time
import argparse
from diagnostic_msgs.msg import KeyValue
from clare_arms.msg import ArmMovement
from clare_arms.srv import ArmsToNeutral
from clare_neck.msg import NeckMovement
from clare_fan.msg import FanControl
from clare_lightring.msg import LightRingMessage
from clare_env.msg import BME680Message
from clare_middle.msg import GasMessage, UltrasoundMessage
from clare_tracks.msg import EncoderMessage, JoystickInput, MotorSpeeds
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from sensor_msgs.msg import CompressedImage
from clare_head.msg import NoseMessage, EvoMessage, FaceMessage, EarsMessage, IRMessage
from clare_head.srv import ListExpressions
from sensor_msgs.msg import Imu
import webcolors


class ClareAPI(object):   

    def __init__(self) -> None:
        self._arm_pub = rospy.Publisher("/clare/arms", ArmMovement, queue_size=5)
        self._neck_pub = rospy.Publisher("/clare/neck", NeckMovement, queue_size=5)
        self._fan_pub = rospy.Publisher("/clare/fan", FanControl, queue_size=5)
        self._lightring_pub = rospy.Publisher("/clare/lightring", LightRingMessage, queue_size=5)
        self._tts_pub = rospy.Publisher("/clare/tts", String, queue_size=5)
        self._face_pub = rospy.Publisher("/clare/head/face", FaceMessage, queue_size=5)
        self._ears_pub = rospy.Publisher("/clare/head/ears", EarsMessage, queue_size=5)

        rospy.Subscriber("/clare/head/noise", Bool, self._noise_cb)
        rospy.Subscriber("/clare/head/nose", NoseMessage, self._nose_cb)
        rospy.Subscriber("speech_to_text", SpeechRecognitionCandidates, self._speech_cb)
        rospy.Subscriber("/clare/buttons", KeyValue, self._button_cb)

        self.reset_arms_service = rospy.ServiceProxy('/clare/arms/arms_to_neutral', ArmsToNeutral)
        self.list_expressions_service = rospy.ServiceProxy('/clare/head/list_face_expressions', ListExpressions)

        #self._exps = self.list_expressions_service().expressions.split(",");

    def set_expression(self, exp):
        m = FaceMessage()
        m.expression = exp
        self._face_pub.publish(m)

    def set_ears_from_cname(self, n):
        h = webcolors.name_to_hex(n).replace('#','0x')
        rospy.loginfo(f"{n}->{h}")
        self.set_ears(h,h,h,h)

    # Assumes hex strings
    def set_ears(self, le, la, re, ra):
        m = EarsMessage()
        m.left_ear_col = int(le, base=0)
        m.left_antenna_col = int(la, base=0)
        m.right_ear_col = int(re, base=0)
        m.right_antenna_col = int(ra, base=0)
        self._ears_pub.publish(m)

    def reset_arms(self):
        res = self.reset_arms_service()
        return res

    def set_arms(self, dict):
        m = ArmMovement()

        # Dont take any chances, set all to -1 first
        m.sh_fb_left = -1
        m.sh_fb_right = -1
        m.sh_ud_left = -1
        m.sh_ud_right = -1
        m.el_left = -1
        m.el_right = -1
        m.wr_left = -1
        m.wr_right = -1
        m.gr_left = -1
        m.gr_right = -1

        for k, v in dict.items():
            setattr(m, k, v)
        self._arm_pub.publish(m)

    def set_neck(self, z, y):
        m = NeckMovement()
        m.z = z
        m.y = y
        self._neck_pub.publish(m)

    def set_fan(self, state, dur):
        m = FanControl()
        m.state = state
        m.duration = dur
        self._fan_pub.publish(m)
    
    def set_lightring(self, pat):
        m = LightRingMessage()
        m.pattern = pat
        self._lightring_pub.publish(m)
    
    def speak(self, txt):
        self._tts_pub.publish(txt)
    
    ####################################
    # Callbacks
    def _nose_cb(self, msg):
        t = msg.temp
        h = msg.humidity
        self.nose_handler(t, h)
    
    def _button_cb(self, msg):
        self.button_handler()

    def _noise_cb(self, msg):
        self.noise_handler()

    def _speech_cb(self, data):
        txt = data.transcript[0].lower().strip()
        self.speech_handler(txt)

    ##################
    # Should be overridden
    def nose_handler(self, temp, hum):
        pass
    
    def button_handler(self):
        pass

    def noise_handler(self):
        pass

    def speech_handler(self, txt):
        pass
