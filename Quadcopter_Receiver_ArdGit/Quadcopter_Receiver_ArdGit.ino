
#include <Servo.h>
#include <SPI.h>
#include "RF24.h"

/*
  -Scripps Ranch High School Robotics Team-
  Quadcopter Wireless Receiving
  By Michael Yee
  Last Update : 4/28/2015

  Goes hand-in-hand with Quadcopter_Controller_Register_ArdGit
  Version XX
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
const uint64_t pipe = 0xE8E8F0F0E1LL;
byte addresses[][6] = {"1Node", "2Node"};
int msg[15];

// EDIT WHICH PINS THE RADIO WILL BE ON
RF24 radio(9, 10);


void setup() {

  radio.begin();

  Serial.begin(4800); // open the serial port at 9600 bps
  //establishContact();
  // send a byte to establish contact until receiver responds
  //

  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);
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

void loop() {

  String theMessage = "";
  //char a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p;

  int signalInX = 0, signalInY = 0, signalInZ = 0;
  int motorInXL = 0, motorInXR = 0,
      motorInYF = 0, motorInYB = 0, motorInZ = 0;
  int motorInX = 0, motorInY = 0;


  if (radio.available())
  {

    for (int i = 0; i < 15; i ++)
    {
      radio.read(msg, 15);

      char theChar = msg[i];
      
      /*
      char a = msg[0];
      char b = msg[1];
      char c = msg[2];
      char d = msg[3];
      char e = msg[4];
      char f = msg[5];
      char g = msg[6];
      char h = msg[7];
      char p = msg[8];
      char j = msg[9];
      char k = msg[10];
      char l = msg[11];
      char m = msg[12];
      char n = msg[13];
      char o = msg[14];
      */
      
      
      if (theChar == ('C'))
      {
        break;

      }
      
      String x = String(theChar);
      theMessage += x;
      
      //theMessage = a+""+b+""+c+""+d+""+e+""+f+""+g+""+h+""+p+""+j+""+k+""+l+""+m+""+n+""+o;
      //theMessage = ""+a;
    }
    Serial.println(theMessage);
    
    //delay(2500);

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

    signalInX = x.toInt() * -1;
    signalInY = y.toInt();
    signalInZ = z.toInt();

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

    if (signalInZ < 0)
    {
      motorInZ = 0;
    }
    else
    {
      motorInZ = signalInZ;
    }

    motorInXL += signalInZ;
    motorInXR += signalInZ;
    motorInYF += signalInZ;
    motorInYB += signalInZ;

    if (motorInXL >= 180)
      motorInXL = 179;
    if (motorInXR >= 180)
      motorInXR = 179;
    if (motorInYB >= 180)
      motorInYB = 179;
    if (motorInYF >= 180)
      motorInYF = 179;

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


  }
  else
  {
    Serial.println("No Radio.");
  }

}



void establishContact() {
  while (Serial.available() <= 0) {
    Serial.println("A");   // send a capital A
    delay(50);
  }
}
