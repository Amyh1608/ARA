# ARA
Project A.R.A. (Automated Robotic Arm) automates the a robotic arm so that it recognizes certain objects, grabs it, and stores it in its basket.

## Web Based Arm Control
We can control the robotic arm through HTTP GET requests on the host computer runnnig the server such as going to the following on a web brower:
```
100.80.245.38:5000/servo0/90
```

Replace '100.80.245.38' with whatever the IP address is currently.
### Software Dependencies
```
sudo apt-get install python-flask
```
Using ```flask``` for a web-server so that we can control the arm via browser requests.

### Execution
Upload robotic_arm.ino to Arduino.

```
sudo python Pi/control_servos.py
```

### Resources
[flask](http://flask.pocoo.org/docs/0.12/quickstart/)  
[toInt()](https://www.arduino.cc/en/Tutorial/StringToIntExample)  
[pySerial](http://pyserial.readthedocs.io/en/latest/pyserial_api.html)  
[readString()](http://www.instructables.com/id/Arduino-Function-Serialread-And-SerialreadString/)  
[Servo.h](https://www.arduino.cc/en/Tutorial/Sweep)
