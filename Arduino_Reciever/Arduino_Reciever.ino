#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Servo.h>
#include <SPI.h> 
#include "RF24.h"

/*
  -Scripps Ranch High School Robotics Team-
  Quadcopter Wireless Receiving
  By Michael Yee && Duncan Klug
  Last Update : 5/13/2015
  Recieves data from Controller_ArduinoRegister
  Version Github
  ( ͡° ͜ʖ ͡°)
*/

// Assign a unique ID to this sensor at the same time
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

Servo s, t, u, v;
bool on = true;
int val = 0; // Data received from the serial port
int lastMotorInXL = 0, lastMotorInXR = 0,
    lastMotorInYB = 0, lastMotorInYF = 0;
String x = "", y = "", z = "";
int xIndex = 0, yIndex = 0, zIndex = 0;
String nextLoop = "";
int decDir = 180; // for x - y changing position
bool motorIn = true;
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
  t.attach(3); // attaches Servo T to PIN 3
  u.attach(4); // attaches Servo U to PIN 4
  v.attach(5); // attaches Servo V to PIN 5
}

void areWeSteering() // Are we steering? defined by the boolean SteerMode
{
//  if (signalInX == def && signalInY == def)
//  {
//   steerMode = false;
//  }
//  else
//  {
//   steerMode = true; 
//  }
}

void stabilization()
{
//   /* Get a new sensor event */ 
//  sensors_event_t event; 
//  accel.getEvent(&event);
// 
//  /* Display the results (acceleration is measured in m/s^2) */
// /* Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
//  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
//  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
// */
// 
//  int position;  //declares an integer with no value
//   int pval = 0; //pval -> previous value. For use with implementing into other code?
//   double s1V = 100*abs(event.acceleration.x);
//   double s3V = 100*abs(event.acceleration.x);
//   double s2V = 100*abs(event.acceleration.y);
//   double s4V = 100*abs(event.acceleration.y);
//   
//   double servo1Pos=map(s1V,0.0,1000.0,pval,180.0);
//   double servo2Pos=map(s2V,0.0,1000.0,pval,180.0);
//   double servo3Pos=map(s3V,0.0,1000.0,pval,180.0);
//   double servo4Pos=map(s4V,0.0,1000.0,pval,180.0); 
//   
//    motorInXL += 0;
//    motorInXR += 0;
//    motorInYF += 0;
//    motorInYB += 0;
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
  
  if (radio.available())
  {
    radio.read(msg, 1);

    char theChar = msg[0];
    
    //Serial.println(theChar); -- debug purposes
    
    if (theChar != ('C'))
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

//  if (steerMode)
//  {
//   steering(); 
//  }
//  else
//  {
//   stabilization(); 
//  }
  
  //Capping the value output to 179 to prevent accidental unwanted calibration
    if (motorInXL >= 180)
      motorInXL = 179;
    if (motorInXR >= 180)
      motorInXR = 179;
    if (motorInYB >= 180)
      motorInYB = 179;
    if (motorInYF >= 180)
      motorInYF = 179;
  
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
//  else //if no radio
//  {
//    Serial.println("No Radio.");
//    Serial.println(lastMotorInXL);
//    Serial.println(lastMotorInYF);
//    Serial.println(lastMotorInXR);
//    Serial.println(lastMotorInYB);
//    
//    s.write(lastMotorInXL);
//    t.write(lastMotorInYF);
//    u.write(lastMotorInXR);
//    v.write(lastMotorInYB);
//    
//  }
}
