#include <Servo.h>

/*
  -Scripps Ranch High School Robotics Team-
  Quadcopter_Motor_Test
  By Michael Yee
  4/22/2015

  Goes hand-in-hand with Quadcopter_Controller_RegisterV03
  Version 03
*/

Servo s, t, u, v;
int startRotorSpeed = 0; // change later
int maxRotorSpeed = 10; // change later, start low & slow
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

void setup() {

  //initialize serial communications at a 9600 baud rate
  Serial.begin(1000000); // open the serial port at 9600 bps
  establishContact();
  // send a byte to establish contact until receiver responds
  //

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

}

void loop()
{

  int signalInX = 0, signalInY = 0, signalInZ = 0;
  int motorInXL = 0, motorInXR = 0,
      motorInYF = 0, motorInYB = 0, motorInZ = 0;
  int motorInX = 0, motorInY = 0;


  if (Serial.available() > 0)
  { // If data is available to read,

    String hold = "";
    //val = Serial.read(); // read it and store it in val
    Serial.setTimeout(100);
    hold = Serial.readString();

    //Serial.println("VAL: " + hold);

    xIndex = hold.indexOf("X"); // ISSUE WITH indexOf()
    yIndex = hold.indexOf("Y");
    zIndex = hold.indexOf("Z");
    nextLoop = hold.substring(zIndex);
    nLoopXIndex = nextLoop.indexOf("X");
    /*
    Serial.print("X: ");
    Serial.println(xIndex);
    Serial.print("Y: ");
    Serial.println(yIndex);
    Serial.print("Z: ");
    Serial.println(zIndex);
    */
    x = hold.substring(xIndex + 1, yIndex);
    y = hold.substring(yIndex + 1, zIndex);
    z = hold.substring(zIndex + 1, nLoopXIndex + zIndex);
    /*
    Serial.print("X: ");
    Serial.println(x);
    Serial.print("Y: ");
    Serial.println(y);
    Serial.print("Z: ");
    Serial.println(z);
    */
    signalInX = (x.toInt() - 64) * -1; // subtract 64 because default
    signalInY = y.toInt() - 64; // read input value is 64
    signalInZ = z.toInt() - 64;

    Serial.print("XSIGNAL: ");
    Serial.println(signalInX);
    Serial.print("YSIGNAL: ");
    Serial.println(signalInY);
    Serial.print("ZSIGNAL: ");
    Serial.println(signalInZ);

    if (signalInX < 0)
    {
      motorInXL = (signalInX * -1) + 64 - 10;
      motorInXR = (signalInX * -1) + 64;
    }
    else if (signalInX > 0)
    {
      motorInXL = (signalInX) + 64 + 4;
      motorInXR = (signalInX) + 64 + 4 - 10;
    }

    if (signalInY < 0)
    {
      motorInYB = (signalInY * -1) + 64 + 4 - 10;
      motorInYF = (signalInY * -1) + 64 + 4;
    }
    if (signalInY > 0)
    {
      motorInYB = (signalInY) + 64;
      motorInYF = (signalInY) + 64 - 10;
    }

    if (signalInZ < 0)
    {
      motorInZ = 0;
    }
    else
    {
      motorInZ = signalInZ + 64;
    }
    
    if(motorInXL < 65)
      motorInXL = motorInZ;
    else
      motorInXL += signalInZ;
    if(motorInXR < 65)
      motorInXR = motorInZ;
    else
      motorInXR += signalInZ;
    if(motorInYF < 65)
      motorInYF = motorInZ;
    else
      motorInYF += signalInZ;
    if(motorInYB < 65)
      motorInYB = motorInZ;
    else
      motorInYB += signalInZ;


    Serial.print("XLMOTOR: ");
    Serial.println(motorInXL);
    Serial.print("XRMOTOR: ");
    Serial.println(motorInXR);
    Serial.print("YFMOTOR: ");
    Serial.println(motorInYF);
    Serial.print("YBMOTOR: ");
    Serial.println(motorInYB);
    Serial.print("ZMOTOR: ");
    Serial.println(motorInZ);
    
    
    // Assuming value 64 is the motor at rest !!!
    if ( motorIn )
    {
      if ( lastMotorInXL < 80 && motorInXL == 64)
      {
        lastMotorInXL -= 1;
        s.write(lastMotorInXL);
      }
      if ( lastMotorInXR < 80 && motorInXR == 64)
      {
        lastMotorInXR -= 1;
        u.write(lastMotorInXR);
      }
      if ( lastMotorInYF < 80 && motorInYF == 64)
      {
        lastMotorInYF -= 1;
        t.write(lastMotorInYF);
      }
      if ( lastMotorInYB < 80 && motorInYB == 64)
      {
        lastMotorInYB -= 1;
        v.write(lastMotorInYB);
      }
    }
    else
    {
      s.write(motorInXL);
      t.write(motorInYF);
      u.write(motorInXR);
      v.write(motorInYB);
      
    }


  }
  delay(3);
}



void establishContact() {
  while (Serial.available() <= 0) {
    Serial.println("A");   // send a capital A
    delay(50);
  }
}
