import serial
import time
from flask import Flask, request, session, g, redirect, url_for, abort, render_template, flash

app = Flask(__name__)

BAUD_RATE = 9600
PORT = '/dev/ttyUSB0'

arduino = serial.Serial(PORT, BAUD_RATE)

@app.route('/servo0/<angle>')
def servo0(angle):
    #servo_0 = int(raw_input("Angle: "))
    msg = str(angle)
    arduino.write(msg)
    time.sleep(3)
    return angle
    
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)
