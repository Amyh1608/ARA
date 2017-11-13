#include <Servo.h>

const int SERVO_0_PIN = 3;
const int SERVO_1_PIN = 5;
const int SERVO_2_PIN = 6;
const int SERVO_3_PIN = 9;

Servo servo_0;  // create servo object to control a servo
Servo servo_1;
Servo servo_2;
Servo servo_3;

int pos = 0;    // variable to store the servo position

void setup() {
  servo_0.attach(SERVO_0_PIN);  // attaches the servo on pin 9 to the servo object
  servo_1.attach(SERVO_1_PIN);
  servo_2.attach(SERVO_2_PIN);
  servo_3.attach(SERVO_3_PIN);
  
  Serial.begin(9600);
}

void loop() {
  String msg; 
  if (Serial.available() > 0) {
    // read the incoming byte:
    msg = Serial.readString();
    int angle = msg.toInt();
    servo_0.write(angle);
  }
}
