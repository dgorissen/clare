# Simple integration test app
 
from flask import Flask, Response, render_template, session, request, jsonify, send_file
from random import randint
import subprocess
import json
import time
from flask_cors import CORS
from clare.state import Tracks, ClareMiddle, HeadCamera, ClareVoice, RealsenseDepth, ClareTop, ClareHead
import rospy
from functools import wraps
import datetime
from clare.utils import shell_cmd
import re
import threading
import RPi.GPIO as GPIO
import secrets

class ClareState:
    def __init__(self):
        self.tracks = None
        self.middle = None
        self.camhead = None
        self.voice = None
        self.realsense = None
        self.top = None
        self.head = None

def is_connected(f):
    @wraps(f)
    def decorated_function(*args, **kwargs):
        if connected():
            return f(*args, **kwargs)
        else:
            return "Not connected", 403
    return decorated_function

# configuration
DEBUG = True
# TODO global vars
STATE = None

# instantiate the app and enable CORS
app = Flask(__name__)
app.config.from_object(__name__)
CORS(app, resources={r'/*': {'origins': '*'}})

# Create ros node
threading.Thread(target=lambda: rospy.init_node('clare_brain_backend', disable_signals=True)).start()

# Pin to control the speaker/amp
MUTE_PIN=40

def mute():
    GPIO.output(MUTE_PIN, GPIO.LOW)

def unmute():
    GPIO.output(MUTE_PIN, GPIO.HIGH)

def init_state():
    global STATE

    if GPIO.getmode() is None:
        GPIO.setwarnings(True)
        
        # Use physical pin numbering
        GPIO.setmode(GPIO.BOARD)

        # Setup the pin we use to control the amp / speaker
        GPIO.setup(MUTE_PIN, GPIO.OUT)
        unmute()


    if STATE is not None:
        raise Exception("init_state() called on already initialised STATE")

    cs = ClareState()
    cs.tracks = Tracks()
    cs.middle = ClareMiddle()
    cs.camhead = HeadCamera()
    cs.voice = ClareVoice()
    cs.realsense = RealsenseDepth()
    cs.top = ClareTop()
    cs.head = ClareHead()

    STATE = cs

    rospy.loginfo("State initialised")

def connected():
    return STATE is not None

@app.route("/")
def index():
    return jsonify({
        "cbrain": "CLARE API v0.1",
        "connected": connected()
    })

@app.route("/connect")
def connect():
    if not connected():
        init_state()

    return ('Connected',200)

@app.route("/disconnect")
@is_connected
def disconnect():
    return ('Not implemented yet',500)

@app.route("/tracks/headlights")
@is_connected
def headlights():
    val = STATE.tracks.get_headlights() if STATE.tracks else None
    return jsonify({
        "value": val
    })

@app.route("/tracks/headlights/<status>")
@is_connected
def set_headlights(status):
    if status == "on":
        STATE.tracks.set_headlights(True)
    elif status == "off":
        STATE.tracks.set_headlights(False)
    else:
        return "Invalid headlight status", 500

    return f'Headlights set to {status}', 200

@app.route("/tracks/move")
@is_connected
def tracks_move():
    x = request.args.get('x', default=0, type=float)
    y = request.args.get('y', default=0, type=float)

    if abs(x) > 100 or abs(y) > 100:
        return "Position out of range", 500

    if x > 1 or y > 1:
        STATE.tracks.send_move_command(x, y)
    
    return "",200

@app.route("/tracks/state")
@is_connected
def tracks_state():
    return jsonify(STATE.tracks.get_state())

@app.route("/middle/state")
@is_connected
def middle_state():
    return jsonify(STATE.middle.get_state())

@app.route("/top/state")
@is_connected
def top_state():
    return jsonify(STATE.top.get_state())

@app.route("/head/list_face_expressions")
@is_connected
def list_face_expressions():
    return jsonify(STATE.head.get_expressions())


def randhex(n):
    return secrets.token_hex(n)

@app.route("/head/set_ears")
@is_connected
def set_ears():

    def rand_col_str():
        return "0x" + str(randhex(3))

    le = request.args.get('le', default=rand_col_str(), type=str)
    la = request.args.get('la', default=rand_col_str(), type=str)
    re = request.args.get('re', default=rand_col_str(), type=str)
    ra = request.args.get('ra', default=rand_col_str(), type=str)

    STATE.head.set_ears(le, la, re, ra)
    return "", 200

@app.route("/head/send_ir")
@is_connected
def send_ir():

    def rand_hex():
        return "0x" + str(randhex(3))

    cmd = request.args.get('cmd', default=rand_hex(), type=str)
    addr = request.args.get('cmd', default=rand_hex(), type=str)
    repeat = request.args.get('cmd', default=5, type=int)

    STATE.head.send_ir(addr, cmd, repeat)
    return "", 200

@app.route("/system/audio_devices")
def audio_devices():
    return shell_cmd(["aplay", "-l"])

@app.route("/system/usb_devices")
def usb_devices():
    return shell_cmd(["lsusb"])

@app.route("/voice/speak")
@is_connected
def speak():
    tts = request.args.get('tts', default="", type=str)
    # No naughty strings
    tts = re.sub(r'[^0-9a-zA-Z ]+', '', tts)
    STATE.top.speak(tts)
    return "", 200

@app.route("/<source>/stream")
@is_connected
def get_stream(source):
    if source == "tracks":
        stream = make_stream("tracks", STATE.tracks.get_state)
    elif source == "middle":
        stream = make_stream("middle", STATE.middle.get_state)
    elif source == "top":
        stream = make_stream("top", STATE.top.get_state)
    elif source == "head":
        stream = make_stream("top", STATE.head.get_state)
    elif source == "voice":
        stream = make_stream("voice", STATE.voice.get_state)
    else:
        return f"Invalid stream source {source}", 500

    return Response(stream(), mimetype="text/event-stream")

def jpeg_stream(state_getter, field):
    last_ts = -1
    while True:
        # Get the current state
        tstate = state_getter()
        cur_ts = tstate.get('ts', -1)
        if (cur_ts > 0) and (last_ts != cur_ts):
            img = tstate.get(field, None)
            if img is None:
                time.sleep(0.5)
            else:
                last_ts = cur_ts
                yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(img) + b'\r\n')
        else:
            time.sleep(0.5)

@app.route("/head/facefeed")
@is_connected
def facefeed():
	return Response(jpeg_stream(STATE.camhead.get_state, "image"),
		mimetype = "multipart/x-mixed-replace; boundary=frame")

@app.route("/body/depthfeed")
@is_connected
def depthfeed():
	return Response(jpeg_stream(STATE.realsense.get_state, "depth"),
		mimetype = "multipart/x-mixed-replace; boundary=frame")

def make_stream(msg_name, state_getter):
    def _stream():
        last_ts = 0
        while True:
            # Get the current state
            tstate = state_getter()
            ts = tstate.get("ts", -1)

            if ts > 0 and ts != last_ts:
                last_ts = ts
                yield f"event: {msg_name}" + "\ndata: " + json.dumps(tstate) + "\n\n"
                # No need for full throttle
                time.sleep(0.5)
            else:
                time.sleep(0.2)

    return _stream

@app.route("/body/move_arms")
@is_connected
def move_arms():
    d = request.args.to_dict()
    for k, v in d.items():
        d[k] = int(v)
    STATE.top.set_arms(d)
    return "", 200

@app.route("/body/arms/reset")
@is_connected
def reset_arms():
    STATE.top.reset_arms()
    return "", 200

@app.route("/body/move_neck")
@is_connected
def move_neck():
    z = request.args.get('z', default=-1, type=int)
    y = request.args.get('y', default=-1, type=int)
    STATE.top.set_neck(z, y)
    return "", 200

@app.route("/body/fan/<state>")
@is_connected
def set_fan(state):
    s = True if state == "on" else False
    d = request.args.get('dur', default=3, type=int)
    STATE.top.set_fan(s, d)
    return "", 200

@app.route("/body/lightring")
@is_connected
def read_env():
    p = request.args.get('pat', default="rainbow", type=str)
    STATE.top.set_lightring(p)
    return "", 200

@app.route("/head/face/<expression>")
@is_connected
def set_face(expression):
    STATE.head.set_expression(expression)
    return "", 200

@app.route("/head/ears/<col>")
@is_connected
def set_ears_single(col):
    STATE.head.set_ears(col)
    return "", 200

if __name__ == "__main__":
    try:
        app.run(host='0.0.0.0', threaded=True)
    finally:
        GPIO.cleanup()
