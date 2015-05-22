#include <Servo.h>
#include <SPI.h>
#include "RF24.h"

#include<Wire.h>
const int MPU=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

/*
  -Scripps Ranch High School Robotics Team-
  Quadcopter Wireless Receiving
  By Michael Yee && Duncan Klug
  Last Update : 5/21/2015
  Recieves data from Controller_ArduinoRegister
  Version Github
  ( ͡° ͜ʖ ͡°)
*/



Servo s, t, u, v;
bool on = true;
int val = 0; // Data received from the serial port
int lastMotorInXL = 0, lastMotorInXR = 0,
    lastMotorInYB = 0, lastMotorInYF = 0;
String x = "", y = "", z = "";
int xIndex = 0, yIndex = 0, zIndex = 0;
String nextLoop = "";
int decDir = 180; // for x - y changing position
bool motorIn = true, motorOn = true;
byte send;
byte addresses[][6] = {"1Node", "2Node"};
const uint64_t pipe = uint64_t(addresses);
int msg[15];
String theMessage = "";

//some more instance fields
double s1V;
double s3V;
double s2V;
double s4V;
double servo1Pos;
double servo2Pos;
double servo3Pos;
double servo4Pos;

int signalInX = 0, signalInY = 0, signalInZ = 0; // them data value instance fields
int motorInXL = 0, motorInXR = 0,
    motorInYF = 0, motorInYB = 0, motorInZ = 0;
int motorInX = 0, motorInY = 0;

int def = 0;

bool steerMode = true; // Are we steering, or should we be using the stablization code?

// EDIT WHICH PINS THE RADIO WILL BE ON
RF24 radio(9, 10);

void setup() {

  radio.begin();

  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);


  Serial.begin(115200); // open the serial port at 9600 bps

  radio.openWritingPipe(pipe);
  radio.openReadingPipe(1, pipe);

  //radio.openWritingPipe(addresses[1]);
  //radio.openReadingPipe(1, addresses[0]);
  //radio.openReadingPipe(1, addresses[0]);
  radio.startListening();

  /*
  pinMode(2, OUTPUT); // sets Pin 2 to Output [?]
  pinMode(3, OUTPUT); // sets Pin 3 to Output [?]
  pinMode(4, OUTPUT); // sets Pin 4 to Output [?]
  pinMode(5, OUTPUT); // sets Pin 5 to Output [?]

  s.write(0); // set Servo S to speed 0
  t.write(0); // set Servo T to speed 0
  u.write(0); // set Servo U to speed 0
  v.write(0); // set Servo V to speed 0
  */

  s.attach(2); // attaches Servo S to PIN 2
  t.attach(6); // attaches Servo T to PIN 3
  u.attach(4); // attaches Servo U to PIN 4
  v.attach(5); // attaches Servo V to PIN 5
  
  s.write(0); // set Servo S to speed 0
  t.write(0); // set Servo T to speed 0
  u.write(0); // set Servo U to speed 0
  v.write(0); // set Servo V to speed 0
  delay(3000);
}

void areWeSteering() // Are we steering? defined by the boolean SteerMode
{
    if (signalInX == def && signalInY == def)
    {
     steerMode = false;
    }
    else
    {
     steerMode = true;
    }
}

void stabilization()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L) 
  
  servo1Pos = map(AcX,0,20000,0,50);
  servo2Pos = map(AcX,0,-20000,0,50);
  servo3Pos = map(AcY,0,20000,0,50);
  servo4Pos = map(AcY,0,-20000,0,50);

  
  
     motorInXL += servo1Pos;
     motorInXR += servo2Pos;
     motorInYF += servo3Pos;
     motorInYB += servo4Pos;
     
}

void steering()
{
  // if Left Controller Stick is moved Left - signalInX < 0
  // if Left Controller Stick is moved Right - signalInX > 0
  // if Left Controller Stick is moved Up - signalInY > 0
  // if Left Controller Stick is moved Down - signalInY < 0

  if (signalInX < 0)
  {
    motorInXL = (signalInX * -1) - 5;
    motorInXR = signalInX * -1;
  }
  else if (signalInX > 0)
  {
    motorInXL = signalInX;
    motorInXR = signalInX - 5;
  }

  if (signalInY < 0)
  {
    motorInYB = (signalInY * -1) - 5;
    motorInYF = (signalInY * -1);
  }
  else if (signalInY > 0)
  {
    motorInYB = signalInY;
    motorInYF = signalInY - 5;
  }

  // if Right Controller Stick is moved Down - signalInZ = 0
  // This is to prevent a negative number which would decrease the speeds of all motors
  // May change it to allow this in the future (but just to a gradual decrease in altitude)

  if (signalInZ <= 0)
  {
    signalInZ = 0;
  }

  //deleted the throttle portion to fit the steering method



}


void throttle()
{
  if (signalInZ <= 0)
  {
    signalInZ = 0;
  }

  motorInXL += signalInZ;
  motorInXR += signalInZ;
  motorInYF += signalInZ;
  motorInYB += signalInZ;
}

void loop() //loops and runs the methods, writes servo values
{
  signalInX = 0, signalInY = 0, signalInZ = 0;
  motorInXL = 0, motorInXR = 0,
  motorInYF = 0, motorInYB = 0, motorInZ = 0;
  motorInX = 0, motorInY = 0;
if(motorOn)
{
  if (radio.available())
  {
    radio.read(msg, 1);

    char theChar = msg[0];

    //Serial.println(theChar); -- debug purposes
    
    if (theChar == ('S'))
    {
       //stabilization(); 
    }
    else if (theChar == ('K'))
    {
















      motorOn = false;
       lastMotorInXL = 0;
       lastMotorInYF = 0;
       lastMotorInXR = 0;
       lastMotorInYB = 0;
       
       s.write(0);
       t.write(0);
       u.write(0);
       v.write(0);
    }
    else if (theChar != ('C'))
    {
      //x.concat(theChar);  huh? why is this here....
      theMessage.concat(theChar);
      delay(2);
    }
    
    else
    {
      //Serial.println(theMessage); -- debug purposes

      xIndex = theMessage.indexOf("X");
      yIndex = theMessage.indexOf("Y");
      zIndex = theMessage.indexOf("Z");

      // theMessage should be X###Y###Z###
      x = theMessage.substring(xIndex + 1, yIndex);
      y = theMessage.substring(yIndex + 1, zIndex);
      z = theMessage.substring(zIndex + 1);

      signalInX = x.toInt() - 64;
      signalInY = y.toInt() - 64;
      signalInZ = z.toInt() - 64;




      //running the methods that edit the servo values
      steering();
      throttle();
      stabilization();

  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);


      //Capping the value output to 179 to prevent accidental unwanted calibration
      /*
      if (motorInXL >= 180)
        motorInXL = 179;
      if (motorInXR >= 180)
        motorInXR = 179;
      if (motorInYB >= 180)
        motorInYB = 179;
      if (motorInYF >= 180)
        motorInYF = 179;
        */
        // Capping the value output to 1 for testing
        if (motorInXL >= 10)
        motorInXL = 10;
      if (motorInXR >= 10)
        motorInXR = 10;
      if (motorInYB >= 10)
        motorInYB = 10;
      if (motorInYF >= 10)
        motorInYF = 10;  
        
        
      //Experimental spike protection
      if (fabs(motorInXL-lastMotorInXL)>30)
        motorInXL = lastMotorInXL;
      if (fabs(motorInXR-lastMotorInXR)>30)
        motorInXR = lastMotorInXR;
      if (fabs(motorInYF-lastMotorInYF)>30)
        motorInYF = lastMotorInYF;
      if (fabs(motorInYB-lastMotorInYB)>30)
        motorInYB = lastMotorInYB;

      //Writing them servo values to the servos
      s.write(motorInXL);
      t.write(motorInYF);
      u.write(motorInXR);
      v.write(motorInYB);

      //temporarily storing the values
      lastMotorInXL = motorInXL;
      lastMotorInXR = motorInXR;
      lastMotorInYF = motorInYF;
      lastMotorInYB = motorInYB;

      //Readout of what's being sent do the Servos
      Serial.print("MotorInXL: "); Serial.println(motorInXL);
      Serial.print("MotorInYF: "); Serial.println(motorInYF);
      Serial.print("MotorInXR: "); Serial.println(motorInXR);
      Serial.print("MotorInYB: "); Serial.println(motorInYB);

      theMessage = "";
    }

  }
  else //if no radio
  {
    //    Serial.println("No Radio.");
    //    Serial.println(lastMotorInXL);
    //    Serial.println(lastMotorInYF);
    //    Serial.println(lastMotorInXR);
    //    Serial.println(lastMotorInYB);
    //
    s.write(lastMotorInXL);
    t.write(lastMotorInYF);
    u.write(lastMotorInXR);
    v.write(lastMotorInYB);
  }
  }
}
