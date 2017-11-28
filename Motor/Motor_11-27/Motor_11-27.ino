int AIN1=2;
int AIN2=3;
int STBY=4;
int BIN1=5;
int BIN2=6;
int PWMA=10;
int PWMB=11;
int serialData=0;
int test=0;
void setup() {
  int i;
  for(i=2;i<=6;i++)
  pinMode(i,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
  Serial.begin(9600);

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

void reverse(){
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

void left(){
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

void right(){
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


void loop() {
  if (Serial.available() > 0)
  { 
     
    serialData=Serial.read();
    Serial.println(serialData);
    switch(serialData)
    {
     
    case 119:
      {
        Serial.println("forward");
        forward();
        break;   
        
      }
    case 115:
      {
        Serial.println("reverse");
        reverse();
        break;
      }
    case 97:
      {
        Serial.println("left");
        left();
        break;
      }
    case 100:
      {
        Serial.println("right");
        right();  
        break;
      }
    
    }
    Serial.println("end");
 }

  
  
  

}
