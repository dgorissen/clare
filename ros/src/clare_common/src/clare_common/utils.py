import rospy
from adafruit_extended_bus import ExtendedI2C as I2C
from clare_arms.msg import ArmMovement
from adafruit_servokit import ServoKit


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


class ServoController(object):
    def __init__(self):
        super(ServoController, self).__init__()
        self._servo_map = None

    def _setup_servos(self):
        # Setup servo connection
        i2c_bus = rospy.get_param("~i2c_bus", 3)
        i2c = I2C(i2c_bus)
        self._servokit = ServoKit(channels=16, i2c=i2c)
        
        # Create a function that maps from logical range (0-100) to
        # raw angles
        for k, v in self._servo_map.items():
            idx, (lb, ub), fun = v
            v[2] = make_interpolater(0, 100, lb, ub)

            # Tuned for diymore mg996r servos        
            self._servokit.servo[idx].set_pulse_width_range(500,2500)

    
    # Takes raw servo angles
    def set_servo(self, key, value, flip=False):
        idx, (lb, ub), fun = self._servo_map[key]
        # Reverse direction
        v = 180 - value if flip else value

        if (lb <= v <= ub):
            self._servokit.servo[idx].angle = v
        else:
            rospy.logerr(f"Out of range servo angle commanded: {key}:{v}")

    # Takes values in 0-100, negative indicates None / ignore
    def set_servo_logical(self, key, value, flip=False):
        if value < 0:
            pass
        else:
            v = self._servo_map[key][2](value)
            self.set_servo(key, v, flip=flip)