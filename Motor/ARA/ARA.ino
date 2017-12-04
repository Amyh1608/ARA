/************************************************************************************************************
  Initialization of Libraries
*************************************************************************************************************/
#include "Arduino.h"
#if !defined(SERIAL_PORT_MONITOR)
  #error "Arduino version not supported. Please update your IDE to the latest version."
#endif

#if defined(__SAMD21G18A__)
  // Shield Jumper on HW (for Zero, use Programming Port)
  #define port SERIAL_PORT_HARDWARE
  #define pcSerial SERIAL_PORT_MONITOR
#elif defined(SERIAL_PORT_USBVIRTUAL)
  // Shield Jumper on HW (for Leonardo and Due, use Native Port)
  #define port SERIAL_PORT_HARDWARE
  #define pcSerial SERIAL_PORT_USBVIRTUAL
#else
  // Shield Jumper on SW (using pins 12/13 or 8/9 as RX/TX)
  #include "SoftwareSerial.h"
  SoftwareSerial port(12, 13);
  #define pcSerial SERIAL_PORT_MONITOR
#endif

#include "EasyVR.h"

/************************************************************************************************************
  Initialization of Variables
  NOTE: motorCodes will be following:
        FORWARD = 400
        BACKWARD = 401
        LEFT = 402
        RIGHT = 403
*************************************************************************************************************/
EasyVR easyvr(port);

int AIN1=2;
int AIN2=3;
int STBY=4;
int BIN1=5;
int BIN2=6;
int PWMA=10;
int PWMB=11;
int serialData=0;
int motorCode = 0;
int currTime=0;
int Time=3001;
int freq=100;
int LED=13;

/************************************************************************************************************
  Groups and Commands
*************************************************************************************************************/
enum Groups
{
  GROUP_0  = 0,
};

enum Group0 
{
  G0_ARA = 0,
};

/************************************************************************************************************
  Grammar and Words
*************************************************************************************************************/
enum Wordsets
{
  SET_1  = -1,
  SET_2  = -2,
  SET_3  = -3,
};

enum Wordset1 
{
  S1_ACTION = 0,
  S1_MOVE = 1,
  S1_TURN = 2,
  S1_RUN = 3,
  S1_LOOK = 4,
  S1_ATTACK = 5,
  S1_STOP = 6,
  S1_HELLO = 7,
};

enum Wordset2 
{
  S2_LEFT = 0,
  S2_RIGHT = 1,
  S2_UP = 2,
  S2_DOWN = 3,
  S2_FORWARD = 4,
  S2_BACKWARD = 5,
};

enum Wordset3 
{
  S3_ZERO = 0,
  S3_ONE = 1,
  S3_TWO = 2,
  S3_THREE = 3,
  S3_FOUR = 4,
  S3_FIVE = 5,
  S3_SIX = 6,
  S3_SEVEN = 7,
  S3_EIGHT = 8,
  S3_NINE = 9,
  S3_TEN = 10,
};

// use negative group for wordsets
int8_t group, idx;

/************************************************************************************************************
  Set-Up Function
*************************************************************************************************************/
void setup()
{
  int i;
  for(i=2;i<=6;i++)
  pinMode(i,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(13,OUTPUT);
  
  // setup PC serial port
  pcSerial.begin(9600);
bridge:
  // bridge mode?
  int mode = easyvr.bridgeRequested(pcSerial);
  switch (mode)
  {
  case EasyVR::BRIDGE_NONE:
    // setup EasyVR serial port
    port.begin(9600);
    // run normally
    pcSerial.println(F("Bridge not requested, run normally"));
    pcSerial.println(F("---"));
    break;
    
  case EasyVR::BRIDGE_NORMAL:
    // setup EasyVR serial port (low speed)
    port.begin(9600);
    // soft-connect the two serial ports (PC and EasyVR)
    easyvr.bridgeLoop(pcSerial);
    // resume normally if aborted
    pcSerial.println(F("Bridge connection aborted"));
    pcSerial.println(F("---"));
    break;
    
  case EasyVR::BRIDGE_BOOT:
    // setup EasyVR serial port (high speed)
    port.begin(115200);
    pcSerial.end();
    pcSerial.begin(115200);
    // soft-connect the two serial ports (PC and EasyVR)
    easyvr.bridgeLoop(pcSerial);
    // resume normally if aborted
    pcSerial.println(F("Bridge connection aborted"));
    pcSerial.println(F("---"));
    break;
  }

  // initialize EasyVR  
  while (!easyvr.detect())
  {
    pcSerial.println(F("EasyVR not detected!"));
    for (int i = 0; i < 10; ++i)
    {
      if (pcSerial.read() == '?')
        goto bridge;
      delay(100);
    }
  }

  pcSerial.print(F("EasyVR detected, version "));
  pcSerial.print(easyvr.getID());

  if (easyvr.getID() < EasyVR::EASYVR3)
    easyvr.setPinOutput(EasyVR::IO1, LOW); // Shield 2.0 LED off

  if (easyvr.getID() < EasyVR::EASYVR)
    pcSerial.print(F(" = VRbot module"));
  else if (easyvr.getID() < EasyVR::EASYVR2)
    pcSerial.print(F(" = EasyVR module"));
  else if (easyvr.getID() < EasyVR::EASYVR3)
    pcSerial.print(F(" = EasyVR 2 module"));
  else
    pcSerial.print(F(" = EasyVR 3 module"));
  pcSerial.print(F(", FW Rev."));
  pcSerial.println(easyvr.getID() & 7);

  easyvr.setDelay(0); // speed-up replies

  easyvr.setTimeout(5);
  easyvr.setLanguage(0); //<-- same language set on EasyVR Commander when code was generated

  group = EasyVR::TRIGGER; //<-- start group (customize)
}

/************************************************************************************************************
  Loop Function
*************************************************************************************************************/
void loop()
{
  if (easyvr.getID() < EasyVR::EASYVR3)
    easyvr.setPinOutput(EasyVR::IO1, HIGH); // LED on (listening)

  if (group < 0) // SI wordset/grammar
  {
    pcSerial.print("Say a word in Wordset ");
    pcSerial.println(-group);
    easyvr.recognizeWord(-group);
  }
  else // SD group
  {
    pcSerial.print("Say a command in Group ");
    pcSerial.println(group);
    easyvr.recognizeCommand(group);
  }

  do
  {
    // allows Commander to request bridge on Zero (may interfere with user protocol)
    if (pcSerial.read() == '?')
    {
      setup();
      return;
    }
    //digitalWrite(LED,LOW);
    // <<-- can do some processing here, while the module is busy
  }
  while (!easyvr.hasFinished());
  
  if (easyvr.getID() < EasyVR::EASYVR3)
    easyvr.setPinOutput(EasyVR::IO1, LOW); // LED off

  idx = easyvr.getWord();
  if (idx == 0 && group == EasyVR::TRIGGER)
  {
    // beep
    easyvr.playSound(0, EasyVR::VOL_FULL);
    // print debug message
    pcSerial.println("Word: ROBOT");
    // write your action code here
    // group = GROUP_X\SET_X; <-- jump to another group or wordset
    return;
  }
  else if (idx >= 0)
  {
    // beep
    easyvr.playSound(0, EasyVR::VOL_FULL);
    // print debug message
    uint8_t flags = 0, num = 0;
    char name[32];
    pcSerial.print("Word: ");
    pcSerial.print(idx);
    if (easyvr.dumpGrammar(-group, flags, num))
    {
      for (uint8_t pos = 0; pos < num; ++pos)
      {
        if (!easyvr.getNextWordLabel(name))
          break;
        if (pos != idx)
          continue;
        pcSerial.print(F(" = "));
        pcSerial.println(name);
        break;
      }
    }
    // perform some action
    action();
    return;
  }
  idx = easyvr.getCommand();
  if (idx >= 0)
  {
    // beep
    easyvr.playSound(0, EasyVR::VOL_FULL);
    // print debug message
    uint8_t train = 0;
    char name[32];
    pcSerial.print("Command: ");
    pcSerial.print(idx);
    if (easyvr.dumpCommand(group, idx, name, train))
    {
      pcSerial.print(" = ");
      pcSerial.println(name);
    }
    else
      pcSerial.println();
    // perform some action
    action();
  }
  else // errors or timeout
  {
    if (easyvr.isTimeout())
      pcSerial.println("Timed out, try again...");
    int16_t err = easyvr.getError();
    if (err >= 0)
    {
      pcSerial.print("Error ");
      pcSerial.println(err, HEX);
    }
  }
}
/************************************************************************************************************
  Action Function
  Switches between the groups and where each of the actions will be coded into the program.
*************************************************************************************************************/
void action()
{
  switch (group)
  {
  case GROUP_0:
    switch (idx)
    {
    case G0_ARA:
      // write your action code here
      // group = GROUP_X\SET_X; <-- or jump to another group or wordset for composite commands
      for (int i=0;i<4;i++){
        freq=100;
        BLINK();
      }
      group = SET_2;
      break;
    }
    break;
  case SET_1:
    switch (idx)
    {
    case S1_ACTION:
      // write your action code here
      // group = GROUP_X\SET_X; <-- or jump to another group or wordset for composite commands
      break;
    case S1_MOVE:
      // write your action code here
      // group = GROUP_X\SET_X; <-- or jump to another group or wordset for composite commands
      break;
    case S1_TURN:
      // write your action code here
      // group = GROUP_X\SET_X; <-- or jump to another group or wordset for composite commands
      break;
    case S1_RUN:
      // write your action code here
      // group = GROUP_X\SET_X; <-- or jump to another group or wordset for composite commands
      break;
    case S1_LOOK:
      // write your action code here
      // group = GROUP_X\SET_X; <-- or jump to another group or wordset for composite commands
      break;
    case S1_ATTACK:
      // write your action code here
      // group = GROUP_X\SET_X; <-- or jump to another group or wordset for composite commands
      break;
    case S1_STOP:
      // write your action code here
      // group = GROUP_X\SET_X; <-- or jump to another group or wordset for composite commands
      break;
    case S1_HELLO:
      // write your action code here
      // group = GROUP_X\SET_X; <-- or jump to another group or wordset for composite commands
      break;
    }
    break;
  case SET_2:
    switch (idx)
    {
    case S2_LEFT:
      // write your action code here
      // group = GROUP_X\SET_X; <-- or jump to another group or wordset for composite commands
      lightLeft();
      left();
      motorCode = 402;
      group = GROUP_0;
      break;
    case S2_RIGHT:
      // write your action code here
      // group = GROUP_X\SET_X; <-- or jump to another group or wordset for composite commands
      lightRight();
      right();
      motorCode = 403;
      group = GROUP_0;
      break;
    //case S2_UP:
      // write your action code here
      // group = GROUP_X\SET_X; <-- or jump to another group or wordset for composite commands
      //break;
    //case S2_DOWN:
      // write your action code here
      // group = GROUP_X\SET_X; <-- or jump to another group or wordset for composite commands
      //break;
    case S2_FORWARD:
      // write your action code here
      // group = GROUP_X\SET_X; <-- or jump to another group or wordset for composite commands
      lightForward();
      forward();
      motorCode = 400;
      group = GROUP_0;
      break;
    case S2_BACKWARD:
      // write your action code here
      // group = GROUP_X\SET_X; <-- or jump to another group or wordset for composite commands
      motorCode = 401;
      lightReverse();
      reverse();
      group = GROUP_0;
      break;
    }
    break;
  case SET_3:
    switch (idx)
    {
    case S3_ZERO:
      // write your action code here
      // group = GROUP_X\SET_X; <-- or jump to another group or wordset for composite commands
      break;
    case S3_ONE:
      // write your action code here
      // group = GROUP_X\SET_X; <-- or jump to another group or wordset for composite commands
      break;
    case S3_TWO:
      // write your action code here
      // group = GROUP_X\SET_X; <-- or jump to another group or wordset for composite commands
      break;
    case S3_THREE:
      // write your action code here
      // group = GROUP_X\SET_X; <-- or jump to another group or wordset for composite commands
      break;
    case S3_FOUR:
      // write your action code here
      // group = GROUP_X\SET_X; <-- or jump to another group or wordset for composite commands
      break;
    case S3_FIVE:
      // write your action code here
      // group = GROUP_X\SET_X; <-- or jump to another group or wordset for composite commands
      break;
    case S3_SIX:
      // write your action code here
      // group = GROUP_X\SET_X; <-- or jump to another group or wordset for composite commands
      break;
    case S3_SEVEN:
      // write your action code here
      // group = GROUP_X\SET_X; <-- or jump to another group or wordset for composite commands
      break;
    case S3_EIGHT:
      // write your action code here
      // group = GROUP_X\SET_X; <-- or jump to another group or wordset for composite commands
      break;
    case S3_NINE:
      // write your action code here
      // group = GROUP_X\SET_X; <-- or jump to another group or wordset for composite commands
      break;
    case S3_TEN:
      // write your action code here
      // group = GROUP_X\SET_X; <-- or jump to another group or wordset for composite commands
      break;
    }
    break;
  }
}
/************************************************************************************************************
  Directional Functions
*************************************************************************************************************/

void BLINK(){
  digitalWrite(LED,HIGH);
  delay(freq);
  digitalWrite(LED,LOW);
  delay(freq);
}
void right(){
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
void lightRight(){
  for (int currTime=0;currTime<Time;currTime=currTime+freq){
    freq=2000;
    BLINK();
  }
}
void left(){
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

void lightLeft(){
  for (int currTime=0;currTime<Time;currTime=currTime+freq){
    freq=500;
    BLINK();
  }
}
void forward(){
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
void lightForward(){
  for (int currTime=0;currTime<Time;currTime=currTime+freq){
    freq=100;
    BLINK();
  }
}
void reverse(){
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
void lightReverse(){
  for (int currTime=0;currTime<Time;currTime=currTime+freq){
    freq=1000;
    BLINK();
  }
}
