# USAGE
# python ball_tracking.py --video ball_tracking_example.mp4
# python ball_tracking.py

# import the necessary packages
from collections import deque
import numpy as np
import argparse
import imutils
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

def detect(frame):
    # define the lower and upper boundaries of the "green"
    # ball in the HSV color space, then initialize the
    # list of tracked points
    redLower = (0, 166, 64)
    redUpper = (10, 255, 255)

    # keep looping
    #for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the current frame
    #(grabbed, frame) = camera.read()
        
    # if we are viewing a video and we did not grab a frame,
    # then we have reached the end of the video
    #if args.get("video") and not grabbed:
    #	break

    # resize the frame, blur it, and convert it to the HSV
    # color space
    #frame=frame.array
    #if term<101:
        #   cv2.imwrite("photo/num%d.jpg" %term, frame)
        #   term=term+1
    
    
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, redLower, redUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
        
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # only proceed if the radius meets a minimum size
        if radius > 15:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            #print center[0]
            #print center[1]
            print '---------------------------'
            print 'ball detected'
            print 'y-axis:',center[1]
            print 'x-axis:',center[0]
            print 'radius is' ,radius
            #cv2.circle(frame, (int(x), int(y)), int(radius),	(0, 255, 255), 2)
            
            #cv2.circle(frame, center, 5, (0, 0, 255), -1)

    # update the points queue
    else :
                print '---------------------------'
                print 'not detected'

    #cv2.imshow("Frame", frame)
    #key = cv2.waitKey(1) & 0xFF
    #time.sleep(1)
    #rawCapture.truncate(0) 
    

    #cv2.destroyAllWindows()
    
    if len(cnts) > 0:
        # Return (If object detected, x-cord, y-cord, radius)
        return (True,center[0],center[1],radius)
    else:
        # Return (That we did not detect object)
        return (False)
