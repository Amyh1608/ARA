# ARA
Project A.R.A. (Automated Robotic Arm) automates the a robotic arm so that it recognizes certain objects, grabs it, and stores it in its basket.

## Web Based Arm Control
We can control the robotic arm through HTTP GET requests on the host computer runnnig the server such as going to the following on a web brower:
```
<IP ADDRESS>:5000/servo/<Servo ID>/<Servo Angle>
```
Example:  
```
100.80.245.38:5000/servo/0/90
```
This sets servo with ID = 0 to 90 degrees.

### Mapping
|servo_id | servo_name |
|---------|------------|
|0        |base        |
|1        |shoulder    |
|2        |elbow       |
|3        |gripper     |

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
[Leading Zeros](https://stackoverflow.com/questions/733454/best-way-to-format-integer-as-string-with-leading-zeros)  
[Arduino substring()](https://www.arduino.cc/en/Reference/StringSubstring)  

## Servo Pi Hat Arm Control

### Software Dependencies
Need to set-up I2C on raspberry Pi by enabling I2C in Interfacing Options in ```raspi-config```.
```
sudo raspi-config
```


Install libraries.
```
sudo apt-get install python-smbus
sudo apt-get install i2c-tools
```

```
sudo pip install adafruit-pca9685

```
