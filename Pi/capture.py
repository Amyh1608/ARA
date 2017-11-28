from picamera import PiCamera
import time

camera = PiCamera()
camera.start_preview()
time.sleep(2)
camera.capture('test.jpg')
'''
time.sleep(5)
camera.capture('test1.jpg')
time.sleep(5)
camera.capture('test2.jpg')
'''
camera.stop_preview()
