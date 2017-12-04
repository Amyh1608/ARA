# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time
import os
import numpy as np
import cv2
from ball_tracking_red import detect
from flask import Flask, send_file

# Import Pi Camera module.
from picamera.array import PiRGBArray
from     picamera import PiCamera
CAMERA_WIDTH = 800
CAMERA_HEIGHT = 600
camera = PiCamera()
camera.resolution = (CAMERA_WIDTH, CAMERA_HEIGHT)
camera.framerate = 30
camera.rotation = 180
rawCapture = PiRGBArray(camera, size=(CAMERA_WIDTH, CAMERA_HEIGHT))
time.sleep(1.0)

# Centering constants
x_center = CAMERA_WIDTH/2
y_center = CAMERA_HEIGHT/2
x_thresh = 10
y_thresh = 10
x_step = 1 # Degrees to step
y_step = 1

IMAGE_DIR = 'images'
image_count = 0

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
SHOULDER = 4
ELBOW = 8
GRIPPER = 12

# Servo mapped limits
GRIPPER_OPEN = 160
GRIPPER_CLOSE = 90

class Arm():
    def __init__(self):
        self.base = 0
        self.shoulder = 0
        self.elbow = 0
        self.gripper = 0

    def update(self, servo_id, angle):

        servo_id = int(servo_id)
        angle = int(angle)    
    
        if(servo_id == BASE):
            self.base = angle
        elif(servo_id == SHOULDER):
            self.shoulder = angle
        elif(servo_id == ELBOW):
            self.elbow = angle
        elif(servo_id == GRIPPER):
            self.gripper = angle
        else:
            print("Error in Arm.update()")

        set_servo(servo_id, angle)
        
arm = Arm()

def init_arm():
    arm.update(BASE, 90)
    arm.update(SHOULDER, 90)
    arm.update(ELBOW, 150)

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
    arm.update(servo_id, angle)

    return "Servo {} is at a {} angle.".format(servo_id, angle)

def set_servo(servo_id,angle):
    
    pwm_length = int(angle_to_pwm(angle))
    print(pwm_length)
    pwm.set_pwm(servo_id, 0, pwm_length)
    

@app.route('/dance')
def sequence():
    servo(BASE,0)
    servo(SHOULDER,0)
    servo(ELBOW,0)
    servo(GRIPPER,0)

    time.sleep(3)

    servo(BASE,90)
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
    servo(SHOULDER,80)
    servo(ELBOW,180)
    servo(GRIPPER,90)

    # Calculate how many steps
    STEPS = int(180/STEP_SIZE + 1)

    '''   
    # Sweep thr ugh angles forme in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
 shoulder
    for shoulder_step in range(STEPS-2):
        shoulder_angle = shoulder_step * (STEP_SIZE-30) + 15
        servo(SHOULDER,shoulder_angle)
        print("shoulder = {}".format(shoulder_angle))
        time.sleep(1)
        '''
    # Sweep through angles for base
    for base_step in range(STEPS):
        base_angle = base_step * STEP_SIZE 
        servo(BASE,base_angle)
        print("base = {}".format(base_angle))
        time.sleep(1)
        


    return 'Scanning...'

@app.route('/camera/shoot')
def take_picture():
    # Init camera
    camera.resolution = (640, 480)

    camera.start_preview()
    time.sleep(2) # Need to wait at least 2 seconds before capturing

    # Create save path
    global image_count
    image_name = 'capture_' + str(image_count) + '.jpg'
    image_count += 1
    image_path = os.path.join(IMAGE_DIR,image_name)

    # Capture
    camera.capture(image_path)
    camera.stop_preview()
    img_bgr = cv2.imread(image_path)
    pos_x, pos_y, radius = ball_tracking(img_bgr)
    detect
    # Return image html
    image_source = os.path.join('image',image_name)
    return "Took a shot...<br><img src='{}'>".format(image_source)    

@app.route('/camera/image/<image>')
def get_image(image):
    image_path = os.path.join(IMAGE_DIR,image)
    return send_file(image_path,mimetype='image/gif')

@app.route('/store')
def store():
    STEP_TIME = 1

    # Moves arm to face forward
    servo(BASE, 90)
    time.sleep(STEP_TIME)
    
    # Move shoulder to straight up
    servo(SHOULDER, 135)
    time.sleep(STEP_TIME)

    # Move elbow to bend 90 degrees back
    servo(ELBOW, 10) 
    time.sleep(3)
    
    # Release object
    open()
    
    return "Object has been stored"

@app.route('/close')
def close():
    '''
    Closes gripper
    '''
    servo(GRIPPER, GRIPPER_CLOSE)
    return "Gripper closed"

@app.route('/open')
def open():
    '''
    Opens gripper
    '''
    servo(GRIPPER, GRIPPER_OPEN)
    return "Gripper opened"

@app.route('/detect')
def detect_object():
    camera.start_preview()
    time.sleep(2) # Need to wait at least 2 seconds before capturing

    # Create save path
    global image_count
    image_name = 'capture_' + str(image_count) + '.jpg'
    image_count += 1
    image_path = os.path.join(IMAGE_DIR,image_name)

    # Capture
    camera.capture(image_path)
    camera.stop_preview()
    frame = cv2.imread(image_path)

    object_vals = detect(frame)
    distance = dist(object_vals[3])
    print("dist", distance) 
    #print(object_vals)
    if(type(object_vals) is tuple):
        output = "detected " + str(distance) 
    else:
        output = "fuQ"
    return output

@app.route('/center')
def center(x_axis=True):
    X_centered = False
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        frame = frame.array
       
        #TEST 
        #print(np.mean(frame, dtype=np.float64))

        object_vals = detect(frame)
        rawCapture.truncate(0)
        print(object_vals)
        if(type(object_vals) is tuple):
            x = object_vals[1]
            y = object_vals[2]
            radius = object_vals[3]
            if(x > (x_center + x_thresh) and not(X_centered) and x_axis):
                arm.update(BASE, arm.base - x_step)
            elif(x < (x_center - x_thresh) and not(X_centered) and x_axis):
                arm.update(BASE, arm.base + x_step)
            else:
                print("X centered")
                X_centered = True
                #if((arm.elbow > 180) or (arm.elbow < 0)):
                #    print("Elbow out of range.")
                #    break
                if(y > (y_center + y_thresh)):
                    print("Y down")
                    arm.update(ELBOW, arm.elbow + y_step)
                elif(y < (y_center - y_thresh)):
                    print("Y up")
                    arm.update(ELBOW, arm.elbow - y_step)
                else:
                    print("Y centered")
                    break
            
        else:
            print("Object not detected")
            return "Object not detected"
        #time.sleep(0.i5)

    #return "Centered"
    return str(radius)

def dist(radius):
    img_ref = cv2.imread('15cm.jpg')
    #ref_vals = detect(img_ref)
    ref_dist = 15
    #print("ref_vals",ref_vals)
    ref_vals = 70
    ratio = ref_dist*ref_vals
    #print("ratio", ratio)
    return ratio/radius    

@app.route('/zoom')
def zoom():
    #shoulder 5; arm 
    #shoulder 5; arm 3 (Prob best so far)
    #shoulder 8; arm 5
    #shoulder 10; arm 10
    shoulder_step = 5
    arm_step = 3
    arm.update(GRIPPER, GRIPPER_OPEN)
    while(True):
        arm.update(SHOULDER, arm.shoulder - shoulder_step)
        time.sleep(0.5)
        arm.update(ELBOW, arm.elbow - arm_step)
        radius = center(x_axis = False)
        print("SHOULDER: " + str(arm.shoulder))
        print("ELBOW: " + str(arm.elbow))
        if(is_num(radius)):
            dist_away = dist(float(radius))
            print("distance away: " + str(dist_away))
            if(dist_away < 7):
                #reduces noise at closer distances
                x_thresh = 15
                y_thresh = 15 
            if(dist_away < 4):
                break
        else:
            break

    #update this else where im fuqin dumb
    #x_thresh = 10
    #y_thresh = 10
    return "Finished Zooming In"

@app.route('/pick_up')
def pick_up():
    #NOTES: ~13-14cm from base
    arm.update(SHOULDER, 60)
    for i in range(5, 0, -1):
        arm.update(ELBOW, arm.elbow - 6)
        time.sleep(0.5)
        arm.update(SHOULDER, arm.shoulder - 6)
    arm.update(ELBOW, arm.elbow - 1)
    time.sleep(1)
    arm.update(SHOULDER, 40)
    time.sleep(1)
    close()
    time.sleep(0.5)
    store()

    return 'Finished sequence'

def is_num(input):
    try:
        float(input)
        return True
    except ValueError:
        return False

@app.route('/init')
def init():
    init_arm()
    return "Arm is in initial position"

if __name__ == '__main__':
    init_arm()
    app.run(host='0.0.0.0', port=5000, threaded=True)
