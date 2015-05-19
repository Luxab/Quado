#include <Servo.h>
#include <SPI.h>
#include "RF24.h"

/*
  -Scripps Ranch High School Robotics Team-
  Quadcopter Wireless Sending
  By Michael Yee
  Last Update : 5/4/2015

  Goes hand-in-hand with Quadcopter_Controller_RegisterGit
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
byte addresses[][6] = {"1Node", "2Node"};
const uint64_t pipe = uint64_t(addresses);
int msg[1];
String hold;

// EDIT WHICH PINS THE RADIO WILL BE ON
RF24 radio (9, 10);

void setup() {

  radio.begin();

  //initialize serial communications at a 9600 baud rate
  //Serial.begin(115200); // open the serial port at 9600 bps
  //Serial.begin(921600);
  Serial.begin(4000000);
  establishContact();
  // send a byte to establish contact until receiver responds
  Serial.setTimeout(1);

  radio.openWritingPipe(pipe);
  radio.openReadingPipe(1, pipe);

  //radio.openWritingPipe(addresses[0]);
  //radio.openReadingPipe(1,addresses[1]);

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

void loop()
{
  int signalInX = 0, signalInY = 0, signalInZ = 0;
  int motorInXL = 0, motorInXR = 0,
      motorInYF = 0, motorInYB = 0, motorInZ = 0;
  int motorInX = 0, motorInY = 0;



 //if (Serial.available() > 0)
  { // If data is available to read,

    //Serial.println("HOLD: ");
    //Serial.println(hold);
    
    
    //hold = "X123Y123Z123";
    hold = Serial.readString(); // read it and store it in val
    //Serial.println("HOLD" + hold);
    
    int indexOfC = hold.indexOf("C");
    hold = hold.substring(0, indexOfC);
    //hold = Serial.readString();
    String theMessage = hold;
    //Serial.println(theMessage);




    for (int i = 0; i < hold.length(); i ++)
    {
      char charToSend[1];
      charToSend[0] = theMessage.charAt(i);
      Serial.println(charToSend[0]); // do not comment out, seems to be needed?
      radio.write(charToSend, 15);
    }

    msg[0] = 'C';  // sends a terminating string value
    radio.write(msg, 1);
  }
  
  // begin reading -- test
  /*
    radio.openReadingPipe(1, pipe);
    radio.startListening();

    int messageRead[15];
    radio.read(messageRead, 15);

    String message = "";
    for (int i = 0; i < 15; i++)
    {
      message += "" + messageRead[i];
    }

    radio.stopListening();
  */
  // end reading

  //radio.powerDown();
  //delay(1);
  //radio.powerUp();
  /*
    Serial.println("RECEIEVED: ");
    Serial.print("" + message);
  */
}


void establishContact() {
  while (Serial.available() <= 0) {
    Serial.println("A");   // send a capital A
    delay(25);
  }
}
