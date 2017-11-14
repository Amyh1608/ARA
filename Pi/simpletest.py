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
    
    pwm_length = servo_min + servo_max/180.*angle
    
    return pwm_length

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

@app.route('/servo/<servo_id>/<angle>')
def servo0(servo_id,angle):
    
    servo_id = int(servo_id)
    angle = int(angle)
    
    pwm_length = angle_to_pwm(angle)
    pwm.set_pwm(servo_id, 0, pwm_length
    
    return "Servo {} is at a {} angle.".format(servo_id, angle)
    
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)
