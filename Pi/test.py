camera = PiCamera()
camera.resolution = (800, 600)
camera.framerate = 30
camera.rotation = 180
rawCapture = PiRGBArray(camera, size=(800, 600))
time.sleep(1.0)
#term=from ball_tracking import *
import cv2

#reference info
img_ref = cv2.imread('12cm.jpg')
_, _, radius_ref = ball_tracking(img_ref)
dist_ref = 12
dia_ref = 151
focalLength = radius_ref*dist_ref/dia_ref

#input img
img = cv2.imread('test.jpg')
_, _, radius = ball_tracking(img)
print(dist_ref*dia_ref/radius)

