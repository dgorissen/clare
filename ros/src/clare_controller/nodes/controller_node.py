#!/usr/bin/env python
 
import rospy
import time
import argparse
from threading import Thread
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
from transitions import Machine, State
from enum import Enum, auto
import webcolors

class States(Enum):
    idle = auto()
    surprised = auto()
    listening = auto()
    sad = auto()
    hooray = auto()

states = [
    State(name=States.idle, on_enter='do_idle'),
    State(name=States.surprised, on_enter='do_surprised'),
    State(name=States.listening, on_enter='do_listening'),
    State(name=States.hooray, on_enter='do_hooray')
]

transitions = [
    ['noise', States.idle, States.surprised],
    ['trigger_phrase', States.idle, States.listening],
    ['hooray', States.listening, States.hooray],
    ['to_idle', '*', States.idle]
]

class ClareController(object):   

    def __init__(self) -> None:
        self._arm_pub = rospy.Publisher("/clare/arms", ArmMovement, queue_size=5)
        self._neck_pub = rospy.Publisher("/clare/neck", NeckMovement, queue_size=5)
        self._fan_pub = rospy.Publisher("/clare/fan", FanControl, queue_size=5)
        self._lightring_pub = rospy.Publisher("/clare/lightring", LightRingMessage, queue_size=5)
        self._tts_pub = rospy.Publisher("/clare/tts", String, queue_size=5)
        self._face_pub = rospy.Publisher("/clare/head/face", FaceMessage, queue_size=5)
        self._ears_pub = rospy.Publisher("/clare/head/ears", EarsMessage, queue_size=5)

        self.machine = Machine(model=self, states=States)

    def start(self) -> None:
        rospy.Subscriber("/clare/head/noise", Bool, self._noise_cb)
        rospy.Subscriber("/clare/head/nose", NoseMessage, self._nose_cb)
        rospy.Subscriber("speech_to_text", SpeechRecognitionCandidates, self.speech_cb)
        rospy.Subscriber("/clare/buttons", KeyValue, self.button_cb)

        self.reset_arms_service = rospy.ServiceProxy('/clare/arms/arms_to_neutral', ArmsToNeutral)
        self.list_expressions_service = rospy.ServiceProxy('/clare/head/list_face_expressions', ListExpressions)

        self._exps = self.list_expressions_service().expressions.split(",");

        # Give some time
        time.sleep(1)
    
        # Kick off state machine
        self.machine.to_idle()

        while not rospy.is_shutdown():
            rospy.spin()

    def do_idle(self):
        self.set_expression("happyblink")
        self.set_ears_from_cname("green")

    def do_surprised(self):
        self.speak("What was that?")
        self.set_expression("surprised")
        self.set_ears_from_cname("orange")
        time.sleep(5)
        self.trigger('to_idle')

    def do_listening(self):
        self.set_expression("sceptical")
        self.set_ears_from_cname("blue")

    def do_hooray(self):
        self.set_expression("bighappy")
        self.set_ears_from_cname("green")
        self.speak("Hip hip hooray!")
        time.sleep(3)
        self.trigger("to_idle")

    def _noise_cb(self, msg):
        self.trigger('noise')

    def speech_cb(self, data):
        txt = data.transcript[0]

        if self.state == States.listening:
            st_fun = self.text_to_state(txt.lower())
            st_fun()
        elif self.state == States.idle:
            if txt.strip() == 'hey clare':
                self.trigger('tigger_phrase')
            else:
                self.speak("You must say hey clare first")
        else:
            pass

    def do_confused(self):
        self.speak("Im confused")
        self.set_expression("confused")
        self.set_ears_from_cname("red")
        time.sleep(3)
        self.to_idle

    def text_to_state(self, txt):
        if "hooray" in txt:
            return self.to_hooray
        else:
            self.do_confused()

    def _nose_cb(self, msg):
        t = msg.temp
        h = msg.humidity

    def set_expression(self, exp):
        m = FaceMessage()
        m.expression = exp
        self._face_pub.publish(m)

    def set_ears_from_cname(self, n):
        h = webcolors.name_to_hex(n).replace('#','0x')
        self.set_ears(h)

    def set_ears(self, cstr):
        c = int(cstr, base=16);
        self.set_ears(c,c,c,c)

    # Assumes hex strings
    def set_ears(self, le, la, re, ra):
        m = EarsMessage()
        m.left_ear_col = int(le, base=0)
        m.left_antenna_col = int(la, base=0)
        m.right_ear_col = int(re, base=0)
        m.right_antenna_col = int(ra, base=0)
        self._ears_pub.publish(m)

    def set_arms(self, dict):
        m = ArmMovement()
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

    def button_cb(self, msg):
        self.speak("I like when you push my buttons")


if __name__ == '__main__':
    rospy.init_node('clare_controller', anonymous=False)

    c = ClareController()
    c.start()
