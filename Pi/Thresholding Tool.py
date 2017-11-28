import cv2
import numpy as np
import sys
import time

def nothing(x):
    pass

input_name = sys.argv[1]
extension = input_name[-3:]
video = 0

x = extension.upper()
cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
if (x == 'JPG') or (x == 'PNG'):
    frame_bgr = cv2.imread(input_name)
elif (x == 'MOV') or (x == 'MP4'):
    video = 1
    cap = cv2.VideoCapture(input_name)
else:
    print("Extension not supported.")
    cv2.destroyAllWindows()

cv2.createTrackbar('h_min', 'frame', 0, 179, nothing)
cv2.createTrackbar('s_min', 'frame', 0, 255, nothing)
cv2.createTrackbar('v_min', 'frame', 0, 255, nothing)
cv2.createTrackbar('h_max', 'frame', 0, 179, nothing)
cv2.createTrackbar('s_max', 'frame', 0, 255, nothing)
cv2.createTrackbar('v_max', 'frame', 0, 255, nothing)
if video == 1:
    switch = 'Slow-mo'
    cv2.createTrackbar(switch, 'frame', 0, 1, nothing)

cv2.setTrackbarPos('h_min', 'frame', 0)
cv2.setTrackbarPos('s_min', 'frame', 0)
cv2.setTrackbarPos('v_min', 'frame', 0)
cv2.setTrackbarPos('h_max', 'frame', 179)
cv2.setTrackbarPos('s_max', 'frame', 255)
cv2.setTrackbarPos('v_max', 'frame', 255)

h_min = 0
s_min = 0
v_min = 0
h_max = 0
s_max = 0
v_max = 0

while(True):
    h_min = cv2.getTrackbarPos('h_min','frame')
    s_min = cv2.getTrackbarPos('s_min','frame')
    v_min = cv2.getTrackbarPos('v_min','frame')
    h_max = cv2.getTrackbarPos('h_max','frame')
    s_max = cv2.getTrackbarPos('s_max','frame')
    v_max = cv2.getTrackbarPos('v_max','frame')

    hsv_lower = np.array([h_min, s_min, v_min])
    hsv_upper = np.array([h_max, s_max, v_max])

    #print("h_min = %d, s_min = %d, v_min = %d" % (h_min, s_min, v_min))
    #print("h_max = %d, s_max = %d, v_max = %d" % (h_max, s_max, v_max))

    #MAKE AN INVERT FEATURE (?)

    if video == 1:
        retval, frame_bgr = cap.read()
        pause = cv2.getTrackbarPos(switch, 'frame')
        if pause == 1:
            time.sleep(1)
    frame_hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    thresh = cv2.inRange(frame_hsv, hsv_lower, hsv_upper)
    output = cv2.bitwise_and(frame_bgr, frame_bgr, mask=thresh)
    cv2.imshow('frame', output)

    if cv2.waitKey(5) & 0xFF == ord('q'):
        break

if video == 1:
    vid.release()
cv2.destroyAllWindows()
