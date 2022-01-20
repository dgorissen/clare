#!/usr/bin/env python
 
import rospy
from std_msgs.msg import String
from gtts import gTTS
from tempfile import NamedTemporaryFile
from playsound import playsound
import os
import subprocess
import re
 
def shell_cmd(cmd):
    result = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE)
    return result.stdout.decode("utf-8")

def speak_gtts(txt):
    lang = 'en'
    with NamedTemporaryFile() as voice:
        gTTS(text=txt, lang=lang).write_to_fp(voice)
        playsound(voice.name)

def speak_festival(txt):
    # No naughty strings
    txt = re.sub(r'[^0-9a-zA-Z ]+', '', txt)
    if txt:
        cmd = f"ALSA_CARD=Headphones festival -b '(voice_cmu_us_slt_arctic_hts)' '(SayText \"{txt}\")'"
        return shell_cmd(cmd)

def speak(backend, msg):
    txt = msg.data
    if backend == "gtts":
        speak_gtts(txt)
    elif backend == "festival":
        speak_festival(txt)
    else:
        rospy.logwarn("Invalid backend specified: ", backend)

if __name__ == '__main__':
    rospy.init_node('clare_tts', anonymous=False)
    backend = rospy.get_param("~backend", 'gtts')
    rospy.Subscriber('clare/tts', String, lambda x : speak(backend, x))
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
