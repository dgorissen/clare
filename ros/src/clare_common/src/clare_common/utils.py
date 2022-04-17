import rospy
from adafruit_extended_bus import ExtendedI2C as I2C
from clare_arms.msg import ArmMovement
from adafruit_servokit import ServoKit
import time
import numpy as np

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
    # All limits and positions are in phyical space
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
    def _build_servo_sequence(self, start, end, toa, steps=50):
        if start
        angles = np.linsppace(start, end, num=steps)
        times = toa / steps
        return zip(angles, times)

    # Takes raw servo angles
    def set_servo(self, key, value, flip=False, toa=2):
        joint = self._servo_map[key]
        lb, ub = joint.limits
        cur_pos = joint.current_pos

        # Reverse direction
        v = 180 - value if flip else value

        if (lb <= v <= ub):
            if (lb <= cur_pos <= ub) and (cur_pos > 0) and (toa > 0):
                for pos, sleep in self._build_servo_sequence(cur_pos, v, toa)
                    self._servokit.servo[joint.index].angle = pos
                    joint.current_pos = pos
                    time.sleep(sleep)
            else:
                # Easing not possible / safe
                self._servokit.servo[joint.index].angle = v
                joint.current_pos = v
        else:
            rospy.logerr(f"Out of range servo angle commanded: {key}:{v}")

    # Takes values in 0-100, negative indicates None / ignore
    def set_servo_logical(self, key, value, flip=False):
        if value < 0:
            pass
        else:
            v = self._servo_map[key].mapper_fcn(value)
            self.set_servo(key, v, flip=flip)
