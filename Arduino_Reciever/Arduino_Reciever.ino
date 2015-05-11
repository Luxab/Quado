#include <Servo.h>
#include <SPI.h>
#include "RF24.h"

/*
  -Scripps Ranch High School Robotics Team-
  Quadcopter Wireless Receiving
  By Michael Yee && Duncan Klug
  Last Update : 5/11/2015
  Recieves data from Controller_ArduinoRegister
  Version XX
  ( ͡° ͜ʖ ͡°)
*/

Servo s, t, u, v;
bool on = true;
int val = 0; // Data received from the serial port
int lastMotorInZ = 0, lastMotorInXL = 0, lastMotorInXR = 0,
    lastMotorInYB = 0, lastMotorInYF = 0;
int lenIn = 0;
String x = "", y = "", z = "";
int xIndex = 0, yIndex = 0, zIndex = 0, nLoopXIndex;
String nextLoop = "";
int decDir = 180; // for x - y changing position
bool motorIn = true;
byte send;
byte addresses[][6] = {"1Node", "2Node"};
const uint64_t pipe = uint64_t(addresses);
int msg[15];
String theMessage = "";

  int signalInX = 0, signalInY = 0, signalInZ = 0; // them data value instance fields
  int motorInXL = 0, motorInXR = 0,
      motorInYF = 0, motorInYB = 0, motorInZ = 0;
  int motorInX = 0, motorInY = 0;
  
  int def = 0;

bool SteerMode = false; // Are we steering, or should we be using the stablization code?

// EDIT WHICH PINS THE RADIO WILL BE ON
RF24 radio(9, 10);

void areWeSteering() // Are we steering? defined by the boolean SteerMode
{
  if (signalInX == def && signalInY == def && signalInZ == def)
  {
   SteerMode = false;
  }
  else
  {
   SteerMode = true; 
  }
}

void setup() {

  radio.begin();

  Serial.begin(115200); // open the serial port at 9600 bps
  //establishContact();
  // send a byte to establish contact until receiver responds
  //
  
   radio.openWritingPipe(pipe);
  radio.openReadingPipe(1, pipe);
  
  //radio.openWritingPipe(addresses[1]);
  //radio.openReadingPipe(1, addresses[0]);
  //radio.openReadingPipe(1, addresses[0]);
  radio.startListening();

  /*
  pinMode(9, OUTPUT); // sets Pin 9 to Output [?]
  pinMode(10, OUTPUT); // sets Pin 10 to Output [?]
  pinMode(11, OUTPUT); // sets Pin 11 to Output [?]
  pinMode(12, OUTPUT); // sets Pin 12 to Output [?]
  s.attach(9); // attaches Servo S to PIN 9
  t.attach(10); // attaches Servo T to PIN 10
  u.attach(11); // attaches Servo U to PIN 11
  v.attach(12); // attaches Servo V to PIN 12
  s.write(0); // set Servo S to speed 0
  t.write(0); // set Servo T to speed 0
  u.write(0); // set Servo U to speed 0
  v.write(0); // set Servo V to speed 0
  */


}

void loop() //does everything basically
{

  //String x = "";

  signalInX = 0, signalInY = 0, signalInZ = 0;
  motorInXL = 0, motorInXR = 0,
      motorInYF = 0, motorInYB = 0, motorInZ = 0;
  motorInX = 0, motorInY = 0;


  if (radio.available())
  {

    radio.read(msg, 1);

    char theChar = msg[0];
    
    //Serial.println(theChar);
    
    if (theChar != ('C'))
    {
      x.concat(theChar);
      theMessage.concat(theChar);
      delay(2);
    }
    else
    {
      //Serial.println(theMessage);
      // INSERT MOTOR CODE HERE?
      
    xIndex = theMessage.indexOf("X");
    yIndex = theMessage.indexOf("Y");
    zIndex = theMessage.indexOf("Z");
    /*
    No More Loop, Unnecessary
    nextLoop = theMessage.substring(zIndex);
    nLoopXIndex = nextLoop.indexOf("X");
    */

    // theMessage should be X###Y###Z###
    x = theMessage.substring(xIndex + 1, yIndex);
    y = theMessage.substring(yIndex + 1, zIndex);
    z = theMessage.substring(zIndex + 1);

    signalInX = x.toInt() - 64;
    signalInY = y.toInt() - 64;
    signalInZ = z.toInt() - 64;
    
    
    Serial.println(signalInX);
    Serial.println(signalInY);
    Serial.println(signalInZ);
    
    // DEFAULT SIGNALS :  X -64, Y 64, Z 64
    // DEFAULT MOTOR : XL-YF-XR-YB 59, 54, 64, 64

// I'm not sure what this if/else tree does, Michael can explain it
// also wouldn't this crash if signalInX or signalInY were == 0?

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
      motorInYF = signalInY - 10;
    }

    if (signalInZ <= 0)
    {
      signalInZ = 0;
    }
    else
    {
      //signalInZ = signalInZ;
    }

    motorInXL += signalInZ;
    motorInXR += signalInZ;
    motorInYF += signalInZ;
    motorInYB += signalInZ;

//Capping the value output to 179 to prevent accidental unwanted calibration
    if (motorInXL >= 180)
      motorInXL = 179;
    if (motorInXR >= 180)
      motorInXR = 179;
    if (motorInYB >= 180)
      motorInYB = 179;
    if (motorInYF >= 180)
      motorInYF = 179;
    
//Readout keeping us out of the dark
    Serial.print("MotorInXL: ");
    Serial.println(motorInXL);
    Serial.print("MotorInYF: ");
    Serial.println(motorInYF);
    Serial.print("MotorInXR: ");
    Serial.println(motorInXR);
    Serial.print("MotorInYB: ");
    Serial.println(motorInYB);
    
//Writing them servo values to the servos
    s.write(motorInXL);
    t.write(motorInYF);
    u.write(motorInXR);
    v.write(motorInYB);

    lastMotorInXL = motorInXL;
    lastMotorInXR = motorInXR;
    lastMotorInYF = motorInYF;
    lastMotorInYB = motorInYB;


    /*
    // begin writing outputs -- test
    radio.stopListening();
    String sendMessage = x+y+z;
    radio.openWritingPipe(pipe);
    for (int i = 0; i < sendMessage.length(); i ++)
    {
      int charToSend[1];
      charToSend[0] = sendMessage.charAt(i);
      radio.write(charToSend, 15);
    }
    msg[0] = 'C';  // sends a terminating string value
    radio.write(msg, 1);
    */
    //radio.powerDown();
    //delay(1);
    //radio.powerUp();

    // end writing
    
    theMessage = "";
    
    }
  }
  else // if no connection to the controller, then you have to do something I guess
  {
    Serial.println("No Radio.");
    Serial.println(lastMotorInXL);
    Serial.println(lastMotorInYF);
    Serial.println(lastMotorInXR);
    Serial.println(lastMotorInYB);
    
    s.write( lastMotorInXL);
    t.write( lastMotorInYF);
    u.write( lastMotorInXR);
    v.write( lastMotorInYB);
    
    
  }

}
