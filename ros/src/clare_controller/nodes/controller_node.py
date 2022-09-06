#!/usr/bin/env python
 
import rospy
import time
import argparse
from threading import Thread
from transitions import Machine, State
from enum import Enum, auto
import logging
from clare_controller.clareapi import ClareAPI


class States(Enum):
    dummy = auto()
    init = auto()
    idle = auto()
    surprised = auto()
    listening = auto()
    sad = auto()
    hooray = auto()
    confused = auto()
    farted = auto()


states = [
    State(name=States.dummy),
    State(name=States.init),
    State(name=States.idle),
    State(name=States.surprised),
    State(name=States.listening),
    State(name=States.hooray),
    State(name=States.confused),
    State(name=States.farted)
]

transitions = [
    ['initialise', States.dummy, States.init],
    ['initialised', States.init, States.idle],
    ['noise', States.idle, States.surprised],
    ['trigger_phrase', States.idle, States.listening],
    ['hooray', States.listening, States.hooray],
    ['confused', States.listening, States.confused],
    ['fartdefense', States.listening, States.farted],
    ['to_idle', '*', States.idle]
]

class ClareController(ClareAPI):   

    def __init__(self) -> None:
        super().__init__()
        self._first_boot = True
        self.machine = Machine(model=self, states=States, transitions=transitions, initial=States.dummy)

    def start(self) -> None:
        # Kick off state machine
        self.trigger("initialise")

    def nose_handler(self, temp, hum):
        pass
    
    def button_handler(self):
        #self.speak("Hey, you touched my buttons!")
        pass

    def noise_handler(self):
        rospy.loginfo("Heard noise")
        self.trigger("noise")

    def speech_handler(self, txt):
        if self.state == States.listening:
            s = self.text_to_state(txt)
            self.trigger(s)
        elif self.state == States.idle:
            if txt in ['hey clare', 'hey claire']:
                rospy.loginfo("Trigger phrase detected")
                self.trigger("trigger_phrase")
            else:
                pass
        else:
            pass

    def on_enter_init(self):
        rospy.loginfo("Init state")
        time.sleep(2)
        self.speak("Hello, I am Clare. Please wait for me to setup")
        self.set_expression("noexpression")
        self.set_ears_from_cname("red")
        self.reset_arms()
        time.sleep(6)
        self.trigger("to_idle")

    def on_enter_idle(self):
        rospy.loginfo("Idle state")
        self.set_ears_from_cname("green")
        self.set_lightring("green")
        self.set_expression("happyblink")
        if self._first_boot:
            self.speak("Setup complete")
            self._first_boot = False

    def on_enter_surprised(self):
        rospy.loginfo("Surprised state")
        self.speak("What was that?")
        self.set_expression("surprised")
        self.set_ears_from_cname("orange")
        self.set_lightring("orange")

        time.sleep(5)
        self.trigger("to_idle")

    def on_enter_listening(self):
        rospy.loginfo("Listening state")
        self.set_expression("sceptical")
        self.set_ears_from_cname("blue")
        self.set_lightring("blue")

    def on_enter_hooray(self):
        rospy.loginfo("Hooray state")
        self.set_arms({"sh_fb_left":80, "sh_fb_right":80})
        self.speak("Oooh! This is exciting")
        self.set_expression("happy")
        self.set_ears_from_cname("yellow")
        time.sleep(4)
        self.speak("Hip hip hooray! Hip Hip Hooray!")
        self.set_expression("bighappy")
        self.set_ears_from_cname("yellow")
        self.set_lightring("rainbow")
        self.set_arms({"sh_fb_left":10, "sh_fb_right":10})
        time.sleep(4)
        self.trigger("to_idle")

    def on_enter_confused(self):
        self.set_expression("confused")
        self.set_ears_from_cname("red")
        self.set_lightring("red")
        self.speak("I'm confused")
        time.sleep(2)
        self.trigger("to_idle")

    def on_enter_farted(self):
        rospy.loginfo("Farted state")
        self.speak("Oh no! Let me take care of that")
        self.set_expression("ugh")
        self.set_ears_from_cname("brown")
        self.set_lightring("blue")
        self.set_fan(True, 5)
        time.sleep(3)
        self.trigger("to_idle")

    # TODO - figure out proper model
    def text_to_state(self, txt):
        if "hooray" in txt:
            return "hooray"
        elif "farted" in txt or "fart" in txt:
            return "fartdefense"
        else:
            return "confused"

if __name__ == '__main__':
    rospy.init_node('clare_controller', anonymous=False)

    c = ClareController()
    c.start()

    while not rospy.is_shutdown():
        rospy.spin()
