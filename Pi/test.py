from ball_tracking import *
import cv2

img = cv2.imread('test.jpg')
pos_x, pos_y, radius = ball_tracking(img)

