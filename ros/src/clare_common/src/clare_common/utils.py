import rospy
from adafruit_extended_bus import ExtendedI2C as I2C
from clare_arms.msg import ArmMovement
from adafruit_servokit import ServoKit
import time
import numpy as np
import math
import concurrent.futures

def make_interpolater(left_min, left_max, right_min, right_max): 
    # Figure out how 'wide' each range is  
    leftSpan = left_max - left_min  
    rightSpan = right_max - right_min  

    # Compute the scale factor between left and right values 
    scaleFactor = float(rightSpan) / float(leftSpan) 

    # create interpolation function using pre-calculated scaleFactor
    def interp_fn(value):
        val = right_min + (value-left_min)*scaleFactor
        return val
    
    return interp_fn

class ServoJoint:
    # All limits and positions are in physical space
    def __init__(self, index, limits=[0, 180], neutral_pos=0) -> None:
        self.index = index
        self.limits = limits
        self.neutral_pos = neutral_pos
        self.current_pos = -1;
        self.mapper_fcn = make_interpolater(0, 100, limits[0],limits[1])
    
    def get_physical_pos(self):
        return self.current_pos
    
    def get_logical_pos(self):
        return self.mapper_fcn(self.current_pos)

class ServoController(object):
    def __init__(self):
        super(ServoController, self).__init__()
        self._servo_map = None

    def _setup_servos(self):
        # Setup servo connection
        i2c_bus = rospy.get_param("~i2c_bus", 3)
        i2c = I2C(i2c_bus)
        self._servokit = ServoKit(channels=16, i2c=i2c)
        
        for k, v in self._servo_map.items():
            idx = v.index
            # Tuned for diymore mg996r servos        
            self._servokit.servo[idx].set_pulse_width_range(500,2500)

    # Interpolate between starting and ending positions
    def _build_servo_sequence(self, start, end):
        # Split the interval into a fixed number of steps or according
        # to a particular resolution
        # I.e., no point in taking 40 steps in going from 5 degrees to 7
        resolution = 0.2 # degrees
        max_steps = 50
        nres = abs(end - start) / resolution
        steps = math.floor(min(max_steps, nres))

        if steps < 1:
            return []

        # Speed to run at
        v = 90/3 # degrees per second

        # Angular distance to cover
        dist = abs(end - start)

        # Time needed = dist / speed
        t = dist / v

        angles = np.linspace(start, end, num=steps)
        times = [t / steps] * len(angles)
        return zip(angles, times)

    # Takes raw servo angles
    def set_servo(self, key, value, flip=False):
        joint = self._servo_map[key]
        lb, ub = joint.limits
        cur_pos = joint.current_pos

        rospy.logdebug(f"Setting servo {key} to angle {value}")

        # Reverse direction
        v = 180 - value if flip else value

        if (lb <= v <= ub):
            if (lb <= cur_pos <= ub) and (cur_pos > 0):
                for pos, sleep in self._build_servo_sequence(cur_pos, v):
                    self._servokit.servo[joint.index].angle = pos
                    joint.current_pos = pos
                    time.sleep(sleep)
            else:
                # Easing not possible / safe
                self._servokit.servo[joint.index].angle = v
                joint.current_pos = v
        else:
            rospy.logerr(f"Out of range servo angle commanded: {key}:{v}")

    # Set multipe servos in parallel
    def set_servo_group(self, pos_map, flips, logical):
        futures = {}

        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
            for key, value in pos_map.items():
                flip = flips.get(key, False)

                if logical:
                    future = executor.submit(self.set_servo_logical, key, value, flip)
                else:
                    future = executor.submit(self.set_servo, key, value, flip)
                
                futures[future] = key

            for fut in concurrent.futures.as_completed(futures):
                key = futures[fut]
                rospy.info(f"Joint {key}, position reached")

    # Takes values in 0-100, negative indicates None / ignore
    def set_servo_logical(self, key, value, flip=False):
        if value < 0:
            pass
        else:
            v = self._servo_map[key].mapper_fcn(value)
            self.set_servo(key, v, flip=flip)
