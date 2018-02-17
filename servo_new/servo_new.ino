

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
int RightHandPos = 0;
int LeftHandPos = 0;

boolean isLeftArm = false;

char data;
String SerialData = "";
char a; // stores incoming character from other device
String allDevices[11] = {"Left Motor","Right Motor", "Right Arm", "Right Hand", "Left Hand", "", "", "Left Shoulder", "Left Arm", "Right Shoulder"};
int currentUDPos[11];
int currentLRPos[11];
int currentDevice = 0;
int RunningMode = 0;
int LeftMotorStop = 87;
int RightMotorStop = 90;
bool iSPowerDown = false;

Servo myServos[11];

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
  myServos[3].attach(35); //Right Hand Servo
  myServos[4].attach(32); //Left Hand Servo
 
  TurnServo.attach(38);
  DistanceServo.attach(36);

  currentUDPos[0] = LeftMotorStop;
  currentUDPos[1] = RightMotorStop;

   currentUDPos[2] = 0;
   currentUDPos[9] = 90;
   currentUDPos[8] = 90;
   currentUDPos[7] = 90;
  

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
      //---------------Run Motors ----------------------
      case 102:
        if(currentUDPos[0] < 150 && !iSPowerDown){
          currentUDPos[0] = currentUDPos[0] + 10;
          currentUDPos[1] = currentUDPos[1] - 10;
          
        }
        else if(currentUDPos[0] > 87 && iSPowerDown) {
          currentUDPos[0] = currentUDPos[0] - 10;
          currentUDPos[1] = currentUDPos[1] + 10;
        }
        else if(currentUDPos[0] >= 150){
          iSPowerDown = true;
          currentUDPos[0] = currentUDPos[0] - 10;
          currentUDPos[1] = currentUDPos[1] + 10;
        }
        else if(currentUDPos[0] < 90){
          currentUDPos[0] = 87;
          currentUDPos[1] = 90;
          iSPowerDown = false;
        }
        Serial.print("Left Motor Foward at ");
        Serial.print(currentUDPos[0]);
        Serial.print(" Right Motor Foward at ");
        Serial.println(currentUDPos[1]);
        RunMotor(currentUDPos[0],currentUDPos[1]);
        Runmotors = true;
        DistinceTesting = true;
        break;
      case 98:
        
        if(currentUDPos[0] <=87 && currentUDPos[0] > 10 &&  !iSPowerDown){
          currentUDPos[0] = currentUDPos[0] - 10;
          currentUDPos[1] = currentUDPos[1] + 10;
        }
        else if(currentUDPos[0] < 80 && iSPowerDown) {
          currentUDPos[0] = currentUDPos[0] + 10;
          currentUDPos[1] = currentUDPos[1] - 10;
        }
        else if(currentUDPos[0] <= 10){
          iSPowerDown = true;
          currentUDPos[0] = currentUDPos[0] + 10;
          currentUDPos[1] = currentUDPos[1] - 10;
        }
        else if(currentUDPos[0] > 85){
          currentUDPos[0] = 87;
          currentUDPos[1] = 90;
          iSPowerDown = false;
        }
        Serial.print("Left Motor Backward at ");
        Serial.print(currentUDPos[0]);
        Serial.print(" Right Motor Backward at ");
        Serial.println(currentUDPos[1]);
        ReverseMotor(currentUDPos[0],currentUDPos[1]);
        Runmotors = true;
        DistinceTesting = false;
        break;
      case 115:
        StopMotor();
        iSPowerDown = false;
        currentUDPos[0] = 87;
        currentUDPos[1] = 90;
        Runmotors = false;
        DistinceTesting = false;
        break;
      case 107:
        exit(0);
        break;
      //---------------Servo Selection -----------------
      case 50:
        currentDevice = 2;
        Serial.print("Setting Device ");
        Serial.println(allDevices[2]);
        break;
      case 53:
        currentDevice = 7;
       Serial.print("Setting Device ");
        Serial.println(allDevices[7]);
      
        break;
      case 52:
        currentDevice = 8;
        Serial.print("Setting Device ");
        Serial.println(allDevices[8]);
        
        break;
      case 51:
        currentDevice = 9;
        Serial.print("Setting Device ");
        Serial.println(allDevices[9]);
        
        break;
      //--------------------End Servo Selection ---------------------

      //--------------------Set Servo pasition -------------------------
      case 117:
        currentUDPos[currentDevice] = currentUDPos[currentDevice] + 10;
        Serial.print("Rolling Servo on Device ");
        Serial.println(currentDevice);
        myServos[currentDevice].write(currentUDPos[currentDevice]);
        break;
      case 100:
        currentUDPos[currentDevice] = currentUDPos[currentDevice] - 10;
        Serial.print("Rolling Servo on Device ");
        Serial.println(currentDevice);
        myServos[currentDevice].write(currentUDPos[currentDevice]);
        break;

        case 108:
          Serial.print("Turning Left ");
          Serial.println(currentLRPos[5]);
          currentLRPos[5] = currentLRPos[5] + 10;
          TurnServo.write(currentLRPos[5]);
          break;
        case 114:
          Serial.print("Turning Right ");
          Serial.println(currentLRPos[5]);
          currentLRPos[5] = currentLRPos[5] - 10;
          TurnServo.write(currentLRPos[5]);
          break;

      //---------------------End Set pasition -----------------------
    }
  }  
  
}



void RunMotor(int LPower, int RPower)
{
  myServos[0].write(LPower); //Left running motor full foward = 120 all stop = 87
  myServos[1].write(RPower); //Right running motor full foward = 10 all stop = 90
}

void ReverseMotor(int LPower, int RPower)
{
  myServos[0].write(LPower); //Left running motor full reverse = 10
  myServos[1].write(RPower); //Right running motor full reverse = 120
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

  /*while (Serial.available())
  {
    delay(10);
    data = Serial.read();
    SerialData += data;
  }*/

  /*Serial.print("distance measured by the first sensor: ");
  Serial.print(UltraSensor1);
  Serial.println(" cm");*/

  delay(10);
  SerialData = "";
  // make condition to control the LEDs
  if (UltraSensor1 <= 6) // if distance is less than 10 Cm turn the LED ON
  {
    StopMotor();
    DistinceTesting = false;
    Serial.println("Too close stopping Motors");
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
          RunMotor(120,10);
          RunningMode = 1;
          distanceTest = true;
        }
        else if(RunningMode == 1){
          ReverseMotor(10,120);
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
