# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time
from flask import Flask

app = Flask(__name__)

# Import the PCA9685 module.
import Adafruit_PCA9685

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Configure min and max servo pulse lengths
servo_min = 150  # Min pulse length out of 4096
servo_max = 600  # Max pulse length out of 4096

# Mapping servo channels to their name
BASE = 0
SHOULDER = 1
ELBOW = 2
GRIPPER = 4

# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)
    
def angle_to_pwm(angle):
    
    pwm_length = servo_min + ((servo_max-servo_min)/180.*angle)
    
    return pwm_length

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

@app.route('/servo/<servo_id>/<angle>')
def servo(servo_id,angle):
    
    servo_id = int(servo_id)
    angle = int(angle)
    
    pwm_length = int(angle_to_pwm(angle))
    print(pwm_length)
    pwm.set_pwm(servo_id, 0, pwm_length)
    
    return "Servo {} is at a {} angle.".format(servo_id, angle)

@app.route('/dance')
def sequence():
    servo(0,0)
    servo(1,0)
    servo(2,0)
    servo(3,0)

    time.sleep(3)

    servo(0,90)
    time.sleep(3)
    
    servo(1,90)
    time.sleep(3)

    servo(2,45)
    time.sleep(3)

    return servo(3,180)

@app.route('/scan')
def scan():
    # How many degrees per step
    STEP_SIZE = 45

    # Init elbow to be straight and gripper open
    servo(ELBOW,90)
    servo(GRIPPER,90)

    # Calculate how many steps
    STEPS = int(180/STEP_SIZE + 1)

       
    # Sweep through angles for shoulder
    for shoulder_step in range(STEPS-3):
        shoulder_angle = (5-shoulder_step) * STEP_SIZE
        servo(SHOULDER,shoulder_angle)
        print("shoulder = {}".format(shoulder_angle))
        time.sleep(1)
        # Sweep through angles for base
        for base_step in range(STEPS):
            base_angle = base_step * STEP_SIZE 
            servo(BASE,base_angle)
            print("base = {}".format(base_angle))
            time.sleep(1)
 


    return 'Scanning...'
    

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)
