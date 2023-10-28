#!/usr/bin/env python
 
import rospy
import time
import argparse
from threading import Thread
from transitions import Machine, State
from enum import Enum, auto
import logging
from clare_controller.clareapi import ClareAPI
from clare_controller.gptclient import GPTClient
from threading import Timer
import random

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
    insulted = auto()
    love = auto()
    gpt = auto()


states = [
    State(name=States.dummy),
    State(name=States.init),
    State(name=States.idle),
    State(name=States.surprised),
    State(name=States.listening),
    State(name=States.hooray),
    State(name=States.confused),
    State(name=States.farted),
    State(name=States.insulted),
    State(name=States.love),
    State(name=States.gpt)
]

transitions = [
    ['initialise', States.dummy, States.init],
    ['initialised', States.init, States.idle],
    ['noise', States.idle, States.surprised],
    ['trigger_phrase', States.idle, States.listening],
    ['hooray', States.listening, States.hooray],
    ['confused', States.listening, States.confused],
    ['fartdefense', States.listening, States.farted],
    ['insult', States.listening, States.insulted],
    ['inlove', States.listening, States.love],
    ['chatgpt', States.listening, States.gpt],
    ['gpterror', States.gpt, States.confused],
    ['to_idle', '*', States.idle]
]

class ClareController(ClareAPI):   

    def __init__(self) -> None:
        super().__init__()
        self._first_boot = True
        self.machine = Machine(model=self, states=States, transitions=transitions, initial=States.dummy)
        self._idle_timer = None
        self._gptclient = GPTClient()
        self._speech_direction = 0

    def start(self) -> None:
        # Kick off state machine
        self.trigger("initialise")

    def nose_handler(self, temp, hum):
        pass
    
    def button_handler(self):
        #self.speak("Hey, you touched my buttons!")
        pass

    def noise_handler(self):
        if self.state == States.idle:
            # dont interrupt other states
            rospy.loginfo("Heard noise")
            self.trigger("noise")

    def speech_handler(self, txt):
        if self.state == States.listening:
            s = self.text_to_state(txt)
            if s:
                self.trigger(s)
            else:
                try:
                    self.trigger("chatgpt", txt=txt)
                except Exception as e:
                    self.trigger("gpterror")
        elif self.state == States.idle:
            if txt in ['hey clare', 'hey claire']:
                rospy.loginfo("Trigger phrase detected")
                self.trigger("trigger_phrase")
            else:
                pass
        else:
            pass

    def speech_direction_handler(self, angle):
        #rospy.loginfo(f"Speech angle detected: {angle}")
        self._speech_direction = angle

    def on_enter_init(self):
        rospy.loginfo("Init state")
        time.sleep(2)
        self.speak("Hello, I am Clare. Please wait for me to setup")
        self.set_expression("noexpression")
        self.set_ears_from_cname("red")
        self.reset_arms()
        self.reset_neck()
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

        self._idle_timer = RepeatTimer(7, self._idle_animation)
        self._idle_timer.start()
        
    def on_exit_idle(self):
        if self._idle_timer:
            self._idle_timer.cancel()
            self._idle_timer = None

        self.reset_neck()
        
    def _idle_animation(self):
        if self.state != States.idle:
            return

        self.set_expression("happyblink")
        self.set_lightring("rainbow")
        
        z = random.randint(25, 75)
        y = random.randint(25, 75)

        self.set_neck(z, y)

    def on_enter_gpt(self, txt=""):
        rospy.loginfo("GPT state")
        r = self._gptclient.ask_gpt(txt)
        rospy.loginfo(str(r))
        e = r.get("expression", "confused")
        c1 = r.get("colour1","0xff0000")
        c2 = r.get("colour2",c1)
        resp = r.get("response","Oops...I forgot...")
        self.set_expression(e)
        self.set_ears(c1,c1,c2,c2)
        self.speak(resp)
        time.sleep(3)
        self.trigger("to_idle")

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
        self.set_neck(50,self._speech_direction)
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

    def on_enter_insulted(self):
        rospy.loginfo("Insulted state")
        self.set_arms({"sh_fb_left":50, "sh_fb_right":50})
        self.set_expression("angry")
        self.set_lightring("orange")
        self.set_ears_from_cname("orange")
        self.speak("Well, they are just player hating.")
        time.sleep(1)
        self.speak("I will pinch them in their bottoms.")
        self.set_arms({"gr_left":50, "gr_right":50, "wr_left":50, "wr_right":50})
        time.sleep(6)
        self.speak("Until I hear them beg for mercy.")
        self.set_arms({"gr_left":100, "gr_right":100, "wr_left":0, "wr_right":0})
        self.set_arms({"sh_fb_left":10, "sh_fb_right":10})
        time.sleep(2)
        self.speak("And I will suck their blood.")
        self.set_expression("vampire")
        self.set_ears_from_cname("red")
        self.set_lightring("red")
        time.sleep(7)
        self.trigger("to_idle")

    def on_enter_love(self):
        rospy.loginfo("Love state")
        self.set_arms({"sh_fb_left":50, "sh_fb_right":50})
        self.set_expression("kiss")
        self.set_lightring("crimson")
        self.set_ears_from_cname("crimson")
        self.speak("Oooooh, yes I like you very much.")
        time.sleep(2)
        self.speak("I will pinch you in you're bottom.")
        self.set_arms({"gr_left":50, "gr_right":50, "wr_left":50, "wr_right":50})
        time.sleep(3)
        self.set_arms({"gr_left":100, "gr_right":100, "wr_left":0, "wr_right":0})
        self.set_arms({"sh_fb_left":10, "sh_fb_right":10})
        time.sleep(3)
        self.trigger("to_idle")

    # TODO - some simple pre-canned actions, figure out proper model
    def text_to_state(self, txt):
        if "hooray" in txt:
            return "hooray"
        elif "farted" in txt or "fart" in txt:
            return "fartdefense"
        elif "work" in txt:
            return "insult"
        elif "like" in txt:
            return "inlove"
        else:
            return None

class RepeatTimer(Timer):
    def run(self):
        while not self.finished.wait(self.interval):
            self.function(*self.args, **self.kwargs)


if __name__ == '__main__':
    rospy.init_node('clare_controller', anonymous=False)

    c = ClareController()
    c.start()

    while not rospy.is_shutdown():
        rospy.spin()
