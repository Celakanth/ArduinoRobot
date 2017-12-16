

#include <Servo.h>
#include <Wire.h>
#include <SoftwareSerial.h>

//#include <SPI.h> // Not actualy used but needed to compile

SoftwareSerial BT(12, 13);
// creates a "virtual" serial port/UART
// connect BT module TX to D10
// connect BT module RX to D11
// connect BT Vcc to 5V, GND to GND

/*-------------------------------------------------------------------------------------------------
   IR Remote controls
  ------------------------------------------------------------------------------------------------*/
/*
#define MY_PROTOCOL NEC
#define RIGHT_ARROW 0x807F0AF5   //Move several clockwise
#define LEFT_ARROW 0x807F8A75    //Move servo counterclockwise
#define SELECT_BUTTON 0x807FC837 //Center the servo
#define UP_ARROW 0x807F6897      //Increased number of degrees servo moves
#define DOWN_ARROW 0x807F58A7    //Decrease number of degrees servo moves
#define POWER 0x807F02FD
#define HOME 0x807F8877
#define RETURN 0x807F9867
#define VOLUP 0x807F6A95
#define VOLDOWN 0x807FEA15
#define SPOWER 0x807FAA55
#define ARMSELECT 0x807F32CD
//Pushing buttons 0-9 moves to fixed positions
#define BUTTON_0 0x807F807F //Pushing buttons 0-9 moves to fixed positions
#define BUTTON_1 0x807F728D // each 20 degrees greater
#define BUTTON_2 0x807FB04F
#define BUTTON_3 0x807F30CF
#define BUTTON_4 0x807F52AD
#define BUTTON_5 0x807F906F
#define BUTTON_6 0x807F10EF
#define BUTTON_7 0x807F629D
#define BUTTON_8 0x807FA05F
#define BUTTON_9 0x807F20DF
*/
#define trigPin1 9 //pin number 9 in arduino MEGA2560
#define echoPin1 8

int REC_PIN = 2;

boolean distanceTest = false;
boolean HeadOpen = false;
boolean DistinceTesting = false;
boolean rundistanceservo = false;
boolean spinServo = false;
int ServoPos = 0;
/*
IRrecv myReceiver(22);
IRdecode myDecoder;
*/
long duration, distance, UltraSensor1; //we'll use these variable to store and generate data
//IRsend mySender;

/*----------------------------------------------------------------------------------
   Servo Settings
   Pin 46 Right Arm
   Pin 44 Left Arm
   Pin 30 Left Shoulder
   Pin 32 Right Shoulder
  ---------------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------------
   Motor controls
  ---------------------------------------------------------------------------------*/
boolean Runmotors = false;
//unsigned int Buffer[RAWBUF];

Servo LeftServo;
Servo RightServo;

/*------------------------------------------------------------------------------
   Arm Controles
  -------------------------------------------------------------------------------*/

int LeftArmPos = 90;
int RightArmPos = 90;
int LeftShoulderPos = 90;
int RightShoulderPos = 90;

boolean isLeftArm = false;

char data;
String SerialData = "";
char a; // stores incoming character from other device
String allDevices[11] = {"Right Motor", "Left Motor", "Right Arm", "", "", "", "", "Left Shoulder", "Left Arm", "Right Shoulder"};
int currentUDPos[11];
int currentLRPos[11];
int currentDevice = 0;
int RunningMode = 0;

Servo myServos[11];
/*Servo LeftArm;
Servo LeftShoulder;
Servo RightArm;
Servo RightShoulder;*/
Servo DistanceServo;
Servo TurnServo;

void setup()
{
  Serial.begin(9600);

  BT.begin(9600);
  myServos[0].attach(28); //Left running motor
  myServos[1].attach(26); //Right running motor

  myServos[2].attach(42); //Right Arm Servo
  myServos[9].attach(43); //Right Shoulder Servo
  myServos[8].attach(53); //Left arm servo
  myServos[7].attach(52); //Left Shoulder Servo
  //RightServo.attach(28);
  //LeftServo.attach(26);
  //RightArm.attach(42);
  //LeftShoulder.attach(52);
  //RightShoulder.attach(43);
  //LeftArm.attach(53);
  TurnServo.attach(38);
  DistanceServo.attach(36);
  //myReceiver.enableIRIn();
  //myDecoder.UseExtnBuf(Buffer);

  pinMode(trigPin1, OUTPUT); // from where we will transmit the ultrasonic wave
  pinMode(echoPin1, INPUT);
  StopMotor();
  myServos[2].write(0);
  currentLRPos[5] = 90;
}
int ReadingByte = 0;
void loop()
{

  BlueThooth();
  serialCheck();
  // Drive each servo one at a time
  if (DistinceTesting)
  {
    DistinceCheck();
  }
  
  
}


void serialCheck()
{
  if(Serial.available() > 0){
    
    ReadingByte = Serial.read();
    Serial.print("I have read ");
    Serial.println(ReadingByte, DEC);

    switch(ReadingByte)
    {
      case 50:
        currentDevice = 2;
        currentUDPos[currentDevice] = 90;
        Serial.print("Setting Device ");
        Serial.println(data);
        break;
    }
  }  
  
}



void RunMotor()
{
  myServos[0].write(120); //Left running motor
  myServos[1].write(10); //Right running motor
}

void ReverseMotor()
{
  myServos[0].write(10); //Left running motor
  myServos[1].write(120); //Right running motor
}

void StopMotor()
{
  myServos[0].write(87);
  myServos[1].write(90);
}

void DistinceCheck()
{
  SonarSensor(trigPin1, echoPin1); // look bellow to find the difinition of the SonarSensor function
  UltraSensor1 = distance;         // store the distance in the first variable

  while (Serial.available())
  {
    delay(10);
    data = Serial.read();
    SerialData += data;
  }

  Serial.print("distance measured by the first sensor: ");
  Serial.print(UltraSensor1);
  Serial.println(" cm");

  SerialData = "";
  // make condition to control the LEDs
  if (UltraSensor1 <= 6) // if distance is less than 10 Cm turn the LED ON
  {
    StopMotor();
    DistinceTesting = false;
   
  }
  else // else turn the LED OFF
  {
    
    
  }
}

// SonarSensor function used to generate and read the ultrasonic wave
void SonarSensor(int trigPinSensor, int echoPinSensor) //it takes the trigPIN and the echoPIN
{
  //START SonarSensor FUNCTION
  //generate the ultrasonic wave
  //----------------------------------------------------------------------------------------------------------------------
  digitalWrite(trigPinSensor, LOW);  // put trigpin LOW
  delayMicroseconds(2);              // wait 2 microseconds
  digitalWrite(trigPinSensor, HIGH); // switch trigpin HIGH
  delayMicroseconds(10);             // wait 10 microseconds
  digitalWrite(trigPinSensor, LOW);  // turn it LOW again
  //----------------------------------------------------------------------------------------------------------------------

  //read the distance
  //----------------------------------------------------------------------------------------------------------------------
  duration = pulseIn(echoPinSensor, HIGH); //pulseIn funtion will return the time on how much the configured pin remain the level HIGH or LOW; in this case it will return how much time echoPinSensor stay HIGH
  distance = (duration / 2) / 29.1;        // first we have to divide the duration by two
} 
// END SonarSensor FUNCTION

String c = "";

void BlueThooth()
{

  if (BT.available())
  {
    data = BT.read();
    delay(100);
    //int charVal = data;
    //a = (BT.read());
    if (data == '1')
    {
     
        if(RunningMode == 0){
          RunMotor();
          RunningMode = 1;
          distanceTest = true;
        }
        else if(RunningMode == 1){
          ReverseMotor();
          RunningMode = 2;
          distanceTest = false;
        }
        else if (RunningMode == 2){
          StopMotor();
          RunningMode = 0;
          distanceTest = false;
        }
        Serial.print("Motors are running ");
        Serial.println(RunningMode);
    }
    if (data == '2')
    {
      currentDevice = 2;
      currentUDPos[currentDevice] = 90;
      Serial.print("Setting Device ");
      Serial.println(data);
      c = "";
    }
    if (data == '7')
    {
      currentDevice = 7;
      currentUDPos[currentDevice] = 90;
      Serial.print("Setting Device ");
      Serial.println(data);
      c = "";
    }
    if (data == '8')
    {
      currentDevice = 8;
      currentUDPos[currentDevice] = 90;
      Serial.print("Setting Device ");
      Serial.println(data);
      c = "";
    }
    if (data == '9')
    {
      currentDevice = 9;
      currentUDPos[currentDevice] = 90;
      Serial.print("Setting Device ");
      Serial.println(data);
      c = "";
    }

    if (data == '5')
    {
      currentLRPos[5] = currentLRPos[5] + 10;
      TurnServo.write(currentLRPos[5]);
    }
    if (data == '6')
    {
      
      currentLRPos[5] = currentLRPos[5] - 10;
      TurnServo.write(currentLRPos[5]);
    }
    if(data == '4'){
      currentUDPos[currentDevice] = currentUDPos[currentDevice] - 10;
      Serial.print("Rolling Servo on Pin ");
      Serial.println(currentDevice);
      myServos[currentDevice].write(currentUDPos[currentDevice]);
    }

    if (data == '3')
    {
      currentUDPos[currentDevice] = currentUDPos[currentDevice] + 10;
      Serial.print("Rolling Servo on Pin ");
      Serial.println(currentDevice);
      myServos[currentDevice].write(currentUDPos[currentDevice]);
    }

    Serial.println(data);
    //Serial.println(HeadOpen);
  }
}
