#!/bin/env/python
"""
Main controller for Automated Robotic Arm.
"""
from __future__ import division
import time
import os
import numpy as np
import cv2
from motor import Motor
from ball_tracking_red import detect
from flask import Flask, send_file, send_from_directory

#BEGIN CAMERA ##############################################################

# Import Pi Camera module.
from picamera.array import PiRGBArray
from picamera import PiCamera
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

#END CAMERA ##############################################################


app = Flask(__name__)

#BEGIN SERVO SETUP#######################################################

# Import the PCA9685 module.
import Adafruit_PCA9685

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

# Configure min and max servo pulse lengths
servo_min = 150  # Min pulse length out of 4096
servo_max = 600  # Max pulse length out of 4096

# Mapping servo channels to their name
BASE = 3
SHOULDER = 4
ELBOW = 8
GRIPPER = 12

# Servo mapped limits
GRIPPER_OPEN = 160
GRIPPER_CLOSE = 90

class Arm():
    def __init__(self,pwm):
        self.pwm = pwm
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

        pwm_length = int(self.angle_to_pwm(angle))
        
        self.pwm.set_pwm(servo_id, 0, pwm_length)
        
    def angle_to_pwm(self,angle):
        """Helper function to convert angles to pulse lengths"""
        pwm_length = servo_min + ((servo_max-servo_min)/180.*angle)
        return pwm_length
    def get_arm_state(self):
        """Return arm state"""
        return (self.base,self.shoulder,self.elbow,self.gripper)

    def print_arm(self):
        return str(self.get_arm_state())
        return "Base = {} <br>\n Shoulder = {} <br>\n Elbow = {} <br>\n Gripper =  {}<br>\n".format(self.get_arm_state())
        
arm = Arm(pwm)

my_motor = Motor()


#END SERVO SETUP#######################################################

#BEGIN WEB SERVER #####################################################
@app.route('/index.html')
@app.route('/')
def index_html():
    return send_from_directory('web','index.html')

@app.route('/index.js')
def index_js():
    return send_from_directory('web','index.js')

@app.route('/index.css')
def index_css():
    return send_from_directory('web','index.css')

#END WEB SERVER #######################################################


#BEGIN CAMERA FUNCTIONS################################################
#shoot
#dist
#take_picture
#get_image
#detect

def shoot():
    """Return a snapshot from camera"""
    # Create save path
    image_path = os.path.join(IMAGE_DIR,'current_image.jpg')
    
    # Capture
    camera.start_preview()
    camera.capture(image_path)
    camera.stop_preview()
    img_bgr = cv2.imread(image_path)
    
    return img_bgr
    
def dist(radius):
    """Converts radius in pixels to distance in cm"""
    ref_dist = 15
    ref_vals = 70
    ratio = ref_dist*ref_vals
    return ratio/radius  

@app.route('/camera/shoot')
def take_picture():
    """Take an image and display on browser"""
    img_bgr = shoot()
    # Return image html
    image_source = os.path.join(IMAGE_DIR,'current_image.jpg')
    return "Took a shot...<br><img src='{}'>".format(image_source)

def is_object():
    """Returns true if object detected, false otherwise"""
    frame = shoot()
    object_vals = detect(frame)
    if(type(object_vals) is tuple):
        return True
    else:
        return False

@app.route('/camera/images/<image>')
def get_image(image):
    """Gets image to display"""
    image_path = os.path.join(IMAGE_DIR,'current_image.jpg')
    return send_file(image_path,mimetype='image/gif')
    
@app.route('/detect')
def detect_object():
    """Detects object and returns distance if found"""
    frame = shoot()

    object_vals = detect(frame)
    distance = dist(object_vals[3])
    print("dist", distance) 
    if(type(object_vals) is tuple):
        output = "detected " + str(distance) 
    else:
        output = "not detected"
    return output


#END CAMERA FUNCTIONS################################################


#BEGIN MOTOR FUNCTIONS###############################################
@app.route('/motor/<motor_dir>')
def direction(motor_dir):
    if(motor_dir == "forward"):
        my_motor.forward()
    elif(motor_dir == "backward"):
        my_motor.backward()
    elif(motor_dir == "left"):
        my_motor.left()
    elif(motor_dir == "right"):
        my_motor.right()
    else:
        print("Error Invalid Direction: " + motor_dir)
    
    return "Motor Moved"
    

#END MOTOR FUNCTIONS#################################################

#BEGIN SERVO BASIC COMMANDS############################################
#state
#servo
#init_arm
#store
#dance

@app.route('/state')
def state():
    return arm.print_arm()

@app.route('/servo/<servo_id>/<angle>')
def servo(servo_id,angle):
    """Allows for manual control of servos"""
    maps = {'base': BASE,
            'shoulder': SHOULDER,
            'elbow': ELBOW,
            'gripper': GRIPPER
    }
    try:
        servo_id = int(servo_id)
    except:
        servo_id = maps[servo_id]
    
    arm.update(servo_id, angle)
    
    return arm.print_arm()
    
@app.route('/init')
def init_arm():
    """Send arm to initial position"""
    arm.update(BASE, 90)
    arm.update(SHOULDER, 90)
    arm.update(ELBOW, 150)
    return "Arm is in initial position"
    
@app.route('/release')
def release():
    """Opens gripper"""
    servo(GRIPPER, GRIPPER_OPEN)
    return "Gripper opened"
    
@app.route('/close')
def close():
    """Closes gripper"""
    servo(GRIPPER, GRIPPER_CLOSE)
    return "Gripper closed"
    
@app.route('/store')
def store():
    """Does object storing sequence to drop object in back basket"""
    STEP_TIME = 1

    # Moves arm to face forward
    servo(BASE, 80)
    time.sleep(STEP_TIME)

    # Move shoulder to straight up
    servo(SHOULDER, 135)
    time.sleep(STEP_TIME)

    # Check if we still have object
    #if(not is_object()):
     #   no()
      #  return "Object lost"

    # Move elbow to bend 90 degrees back
    servo(ELBOW, 10) 
    time.sleep(3)
    
    # Release object
    release()

    yes()
    
    return "Object has been stored"

@app.route('/yes')
def yes():
    """Waves"""
    step = 15
    for i in range(3):
        arm.update(ELBOW, arm.elbow + 15)
        time.sleep(0.5)
        arm.update(ELBOW, arm.elbow - 15)
        time.sleep(0.5)

    return "Waved yes"

@app.route('/no')
def no():
    """Nodes no"""
    step = 15    
    for i in range(3):
        arm.update(BASE, arm.base + 15)
        time.sleep(0.5)
        arm.update(BASE, arm.base - 15)
        time.sleep(0.5)

    return "Nodded no"
    
@app.route('/dance')
def dance():
    """Does a random robotic dance"""
    servo(BASE,0)
    servo(SHOULDER,0)
    servo(ELBOW,0)
    servo(GRIPPER,0)

    time.sleep(3)

    servo(BASE,90)
    time.sleep(3)
    
    servo(SHOULDER,90)
    time.sleep(3)

    servo(ELBOW,45)
    time.sleep(3)
    
    servo(GRIPPER,180)

    return "Dance completed"
    
    
    
#END SERVO BASIC COMMANDS############################################

#BEGIN SERVO ADVANCED COMMANDS############################################
#Meaning those that combine camera and servo
#scan
#center
#zoom
#pick_up   


@app.route('/scan')
def scan():
    """Scans and sets arm to pointing at object"""

    # How many degrees per step
    STEP_SIZE = 45

    # Init elbow to be straight and gripper open
    servo(SHOULDER,80)
    servo(ELBOW,180)
    release()

    # Calculate how many steps
    STEPS = int(180/STEP_SIZE + 1)

    '''   
    # Sweep through angles for shoulder
    for shoulder_step in range(STEPS-2):
        shoulder_angle = shoulder_step * (STEP_SIZE-30) + 15
        servo(SHOULDER,shoulder_angle)
        print("shoulder = {}".format(shoulder_angle))
        time.sleep(1)
        '''
    target_position = {'base': 0,
                       'shoulder': 0,
                       'elbow': 0,
    
    }

    # Sweep through angles for base
    for base_step in range(STEPS):
        base_angle = base_step * STEP_SIZE 
        arm.update(BASE,base_angle)
        print("base = {}".format(base_angle))
        time.sleep(2)
        if(is_object()):
            yes()
            return "Object found at {}".format(arm.get_arm_state())
        time.sleep(1)
        
    return 'Scanned'


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
                    if((arm.elbow + y_step) < 180):
                        arm.update(ELBOW, arm.elbow + y_step)
                    else:
                        arm.update(SHOULDER, arm.shoulder - y_step)
                elif(y < (y_center - y_thresh)):
                    print("Y up")
                    arm.update(ELBOW, arm.elbow - y_step)
                else:
                    print("Y centered")
                    break
            
        else:
            print("Object not detected")
            return "Object not detected"

    return str(radius)

  

@app.route('/zoom')
def zoom(stop=2.5):
    #shoulder 5; arm 
    #shoulder 5; arm 3 (Prob best so far)
    #shoulder 8; arm 5
    #shoulder 10; arm 10
    shoulder_step = 5
    arm_step = 3
    
    # Was 4 changed to 3 -Simon
    STOPPING_DIST = stop
    
    # Open Gripper
    release
    
    while(True):
        # Move shoulder down a bit
        arm.update(SHOULDER, arm.shoulder - shoulder_step)
        time.sleep(0.5)
        
        # Move elbow up a bit
        arm.update(ELBOW, arm.elbow - arm_step)
        
        # Center y-axis, but not x-axis
        radius = center(x_axis = False)
        print("SHOULDER: " + str(arm.shoulder))
        print("ELBOW: " + str(arm.elbow))
        
        if(is_num(radius)):
            dist_away = dist(float(radius))
            print("distance away: " + str(dist_away))
            if(dist_away < 7):
                shoulder_step = 3
                #reduces noise at closer distances
                x_thresh = 15
                y_thresh = 15 
            if(dist_away < STOPPING_DIST):
                return "Object within range"
                break
        else:
            return "Object not detected"
            break

    #update this else where im fuqin dumb
    #x_thresh = 10
    #y_thresh = 10
    return "Finished Zooming In"
    
@app.route('/adjust')
def adjust():
    """Final adjustment for arm after zooming in"""
    
    ADJUST_CENTER_Y = 484
    ADJUST_CENTER_X = 406
    X_centered = False
    for frame in camera.capture_continuous(
    rawCapture, format="bgr", use_video_port=True):
        rawCapture.truncate(0)
        frame = frame.array

        object_vals = detect(frame)
        
        if(type(object_vals) is tuple):
            x = object_vals[1]
            y = object_vals[2]
            radius = object_vals[3]
            if(x > (ADJUST_CENTER_X + x_thresh) and not(X_centered)):
                arm.update(BASE, arm.base - x_step)
            elif(x < (ADJUST_CENTER_X - x_thresh) and not(X_centered)):
                arm.update(BASE, arm.base + x_step)
            else:
                print("X centered")
                X_centered = True
                if(y > (ADJUST_CENTER_Y + y_thresh)):
                    print("Y down")
                    arm.update(ELBOW, arm.elbow + y_step)
                elif(y < (ADJUST_CENTER_Y - y_thresh)):
                    print("Y up")
                    arm.update(ELBOW, arm.elbow - y_step)
                else:
                    print("Y centered")
                    break
            
        else:
            print("Object not detected")
            return "Object not detected"
    return str(arm.get_arm_state())

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

def is_num(_input):
    try:
        float(_input)
        return True
    except ValueError:
        return False

@app.route('/freedom')
def freedom():
    """Frees ARA to move on its own"""
    response = scan()
    if(response == "Scanned"):
        return "Object not found!!!"
    center()
    zoom()
    adjust()
    time.sleep(3)
    close()
    time.sleep(3)
    
    store()
    
    return "Object retrieved!!"
    
#END SERVO ADVANCED COMMANDS############################################

if __name__ == '__main__':
    init_arm()
    app.run(host='0.0.0.0', port=5000, threaded=True)
