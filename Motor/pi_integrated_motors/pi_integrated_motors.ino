/*Listens for commands from UART and executes them on motors.
 * 
 */
#define PWM_SPEED 200   // Speed that motors will run at.
#define DELAY_TIME 1000 // Length of time that motors for run for in milliseconds (ms).

const int STBY=4;   // Master control. Set low to cut power to all motors.

const int AIN1=2;   // Controls clockwise/counter-clockwise rotation for motor A
const int AIN2=3;

const int BIN1=5;   // Controls clockwise/counter-clockwise rotation for motor B
const int BIN2=6;

const int PWMA=10;  // Controls speed for motor A
const int PWMB=11;  // Controls speed for motor B


void setup()
{
  int i;
  for(i=2;i<=6;i++)
    pinMode(i,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(13,OUTPUT);

  Serial.begin(9600);
  
}

/************************************************************************************************************
  Loop Function
*************************************************************************************************************/
/*
 * Valid Commands:
 *  "forward"   :   Moves car forward.
 *  "backward"  :   Moves car backward.
 *  "left"      :   Turns car clockwise (left).
 *  "right"     :   Turns car counter-clockwise (right).
 */
void loop()
{
  while(Serial.available()) {
    String cmd = Serial.readString();// read the incoming data as string
    Serial.print("Received Command: ");
    Serial.println(cmd);
    move(cmd);
    /*
    if(cmd == "forward"){
      forward();
    }else if(cmd == "backward"){
      backward();
    }else if(cmd == "left"){
      left();
    }else if(cmd == "right"){
      right();
    }else{
      Serial.print("CommandError: Invalid Command '");
      Serial.print(cmd);
      Serial.println("'");
    }
    */
  }
  delay(100);
}

/************************************************************************************************************
  Directional Functions
*************************************************************************************************************/
void motor_a(bool high){
  if(high){
    digitalWrite(AIN1,HIGH);
    digitalWrite(AIN2,LOW);
  }else{
    digitalWrite(AIN1,LOW);
    digitalWrite(AIN2,HIGH);
  }
  digitalWrite(PWMA,PWM_SPEED);
}

void motor_b(bool high){
  if(high){
    digitalWrite(BIN1,HIGH);
    digitalWrite(BIN2,LOW);
  }else{
    digitalWrite(BIN1,LOW);
    digitalWrite(BIN2,HIGH);
  }
  digitalWrite(PWMB,PWM_SPEED);
}

void move(String dir){
  digitalWrite(STBY,HIGH);
  if(dir == "forward"){
    motor_a(true);
    motor_b(true);
  }else if(dir == "backward"){
    motor_a(false);
    motor_b(false);
  }else if(dir == "left"){
    motor_a(true);
    motor_b(false);
  }else if(dir == "right"){
    motor_a(false);
    motor_b(true);
  }else{
    Serial.print("DirectionError: Invalid Direction '");
    Serial.print(dir);
    Serial.println("'");
  }
  
  delay(DELAY_TIME);
  digitalWrite(STBY,LOW);
  delay(DELAY_TIME);
}

void forward(){
  digitalWrite(STBY,HIGH);
  digitalWrite(AIN1,HIGH);
  digitalWrite(AIN2,LOW);
  digitalWrite(PWMA,200);
  digitalWrite(BIN1,HIGH);
  digitalWrite(BIN2,LOW);
  digitalWrite(PWMB,200);
  delay(1000);
  digitalWrite(STBY,LOW);
  delay(1000);
}
void backward(){
  digitalWrite(STBY,HIGH);
  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,HIGH);
  digitalWrite(PWMA,200);
  digitalWrite(BIN1,LOW);
  digitalWrite(BIN2,HIGH);
  digitalWrite(PWMB,200);
  delay(1000);
  digitalWrite(STBY,LOW);
  delay(1000);
}

void right(){
  digitalWrite(STBY,HIGH);
  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,HIGH);
  digitalWrite(PWMA,200);
  digitalWrite(BIN1,HIGH);
  digitalWrite(BIN2,LOW);
  digitalWrite(PWMB,200);
  delay(1000);
  digitalWrite(STBY,LOW);
  delay(1000);
}

void left(){
  digitalWrite(STBY,HIGH);
  digitalWrite(AIN1,HIGH);
  digitalWrite(AIN2,LOW);
  digitalWrite(PWMA,200);
  digitalWrite(BIN1,LOW);
  digitalWrite(BIN2,HIGH);
  digitalWrite(PWMB,200);
  delay(1000);
  digitalWrite(STBY,LOW);
  delay(1000);
}
