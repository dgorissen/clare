#!/usr/bin/env python

import time
import rospy
from clare_lightring.msg import LightRingMessage
from threading import Thread
import neopixel
import board

def wheel(pos):
    ORDER = neopixel.GRB
    
    # Input a value 0 to 255 to get a color value.
    # The colours are a transition r - g - b - back to r.
    if pos < 0 or pos > 255:
        r = g = b = 0
    elif pos < 85:
        r = int(pos * 3)
        g = int(255 - pos * 3)
        b = 0
    elif pos < 170:
        pos -= 85
        r = int(255 - pos * 3)
        g = 0
        b = int(pos * 3)
    else:
        pos -= 170
        r = 0
        g = int(pos * 3)
        b = int(255 - pos * 3)
    return (r, g, b) if ORDER in (neopixel.RGB, neopixel.GRB) else (r, g, b, 0)


class LightringController(object):
    def __init__(self):
        super(LightringController, self).__init__()

        rospy.init_node("clare_lightring", anonymous=False, disable_signals=False)

        self._num_pixels = 24
        self._pixels = neopixel.NeoPixel(board.D18, self._num_pixels)
        self._led_sub = rospy.Subscriber("clare/lightring", LightRingMessage, self._lightring_callback)

    def _lightring_callback(self, msg):
        if msg.pattern == "rainbow":
            self.rainbow_cycle()
        else:
            self.fill_solid(msg.pattern)

    def rainbow_cycle(self, wait=0.001):
        for j in range(255):
            for i in range(self._num_pixels):
                pixel_index = (i * 256 // self._num_pixels) + j
                self._pixels[i] = wheel(pixel_index & 255)
            self._pixels.show()
            time.sleep(wait)

    def fill_solid(self, val):
        if val == "red":
            self._pixels.fill((255, 0, 0))
            self._pixels.show()
        if val == "green":
            self._pixels.fill((0, 255, 0))
            self._pixels.show()
        if val == "blue":
            self._pixels.fill((0, 0, 255))
            self._pixels.show()
        elif not val:
            self._pixels.fill((0, 0, 0))
        else:
            pass

    def run(self):
        rospy.logdebug('Lightring node ready and listening')
        rospy.spin()

if __name__ == "__main__":
  
    n = LightringController()
    try:
        n.run()
    except rospy.ROSInterruptException:
        pass
