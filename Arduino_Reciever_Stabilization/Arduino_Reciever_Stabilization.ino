#include <Servo.h>
#include <SPI.h>
#include "RF24.h"
#include<Wire.h>

/*
  If I read the manual correctly, you MUST restrict roll or pitch; but not both.
  Not doing it results in infinite solutions and some math stuff I don't know.
  Meaning, pitch or roll has to be -90 to +90 while the other one is -180 to +180.
  In aerospace, they restrict pitch; in consumer hardawre (Microsoft and Android) they restrict roll.
  So to me it makes sense to restrict pitch.
*/
#define RESTRICT_PITCH // comment out to restrict roll to 90 deg instead

/*
  -Scripps Ranch High School Robotics Team-
  Quadcopter Wireless Receiving
  By Michael Yee && Duncan Klug
  Last Update : 5/21/2015
  Recieves data from Controller_ArduinoRegister
  Version Github
  ( ͡° ͜ʖ ͡°)
*/

/*
  Complementary filter and I2C rework by Elliot Yoon.
  This is completely untested as I have nothing to test it on, so please use caution.
  Lots of this code was taken from Kristian Lauszus from TKJ Electronics, under GNU GPL2.
  Web      :  http://www.tkjelectronics.com
  e-mail   :  kristianl@tkjelectronics.com
  Source   :  http://forum.arduino.cc/index.php?topic=58048.0
*/

double gyroXangle, gyroYangle; // angle calculated with gyro
double compAngleX, compAngleY; // angle calculated with complementary filter
double mPitch, mRoll;
double dt; // delta time

uint32_t timer;
uint8_t i2cData[14]; // buffer for I2C data

//const int MPU = 0x68; // I2C address of the MPU-6050
// below used to be int16_t
double AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; // IMU data

Servo s, t, u, v;
//bool on = true;
//int val = 0; // Data received from the serial port
int lastmotorInS = 0, lastmotorInU = 0,
    lastmotorInV = 0, lastmotorInT = 0;
String x = "", y = "", z = "";
int xIndex = 0, yIndex = 0, zIndex = 0;
String nextLoop = "";
//int decDir = 180; // for x - y changing position
//bool motorIn = true, motorOn = true;
bool motorOn = true;
//byte send;
byte addresses[][6] = {"1Node", "2Node"};
const uint64_t pipe = uint64_t(addresses);
int msg[15];
String theMessage = "";

int count = 0;

//some more instance fields
double s1V;
double s3V;
double s2V;
double s4V;
double tempStableS;
double tempStableU;
double tempStableT;
double tempStableV;

int signalInX = 0, signalInY = 0, signalInZ = 0; // them data value instance fields
int motorInS = 0, motorInU = 0,
    motorInT = 0, motorInV = 0, motorInZ = 0;
int motorInX = 0, motorInY = 0;

int def = 0;

bool steerMode = true; // Are we steering, or should we be using the stablization code?

// EDIT WHICH PINS THE RADIO WILL BE ON
RF24 radio(9, 10);

void setup() {

  radio.begin();
  Serial.begin(115200); // open the serial port at 9600 bps

  Wire.begin();
  /*
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  */
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(200); // wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  AcX = (i2cData[0] << 8) | i2cData[1];
  AcY = (i2cData[2] << 8) | i2cData[3];
  AcZ = (i2cData[4] << 8) | i2cData[5];

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(AcY, AcZ) * RAD_TO_DEG;
  double pitch = atan(-AcX / sqrt(AcY * AcY + AcZ * AcZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(AcY / sqrt(AcX * AcX + AcZ * AcZ)) * RAD_TO_DEG;
  double pitch = atan2(-AcX, AcZ) * RAD_TO_DEG;
#endif

  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();

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

/*
 updates instance fields with data from IMU
 should be called BEFORE ComplementaryFilter() is called
 currently called in stabilization()
*/
void updateIMU() {
  // update values
  while (i2cRead(0x3B, i2cData, 14));
  AcX = ((i2cData[0] << 8) | i2cData[1]);
  AcY = ((i2cData[2] << 8) | i2cData[3]);
  AcZ = ((i2cData[4] << 8) | i2cData[5]);
  Tmp = (i2cData[6] << 8) | i2cData[7];
  GyX = (i2cData[8] << 8) | i2cData[9];
  GyY = (i2cData[10] << 8) | i2cData[11];
  GyZ = (i2cData[12] << 8) | i2cData[13];

  dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
}

/*
  resets gyro angle if it drifts too much.
  should be called AFTER ComplementaryFilter() is called
  currently called in stabilization()
  returns true if gyro drift was fixed
*/
bool fixGyroDrift() {
  bool fixed = false;
  if (gyroXangle < -180 || gyroXangle > 180) {
    gyroXangle = compAngleX;
    fixed = true;
  }
  if (gyroYangle < -180 || gyroYangle > 180) {
    gyroYangle = compAngleY;
    fixed = true;
  }
  return fixed;
}

/*
  complementary filter stuff
  should be called AFTER updateIMU() is called
  currently called in stabalization()
*/
void ComplementaryFilter() {
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(AcY, AcZ) * RAD_TO_DEG;
  double pitch = atan(-AcX / sqrt(AcY * AcY + AcZ * AcZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(AcY / sqrt(AcX * AcX + AcZ * AcZ)) * RAD_TO_DEG;
  double pitch = atan2(-AcX, AcZ) * RAD_TO_DEG;
#endif

  // no clue what 131.0 is but okay
  double gyroXrate = GyX / 131.0; // Convert to deg/s
  double gyroYrate = GyY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  //if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
  if ((roll < -90 && compAngleX > 90) || (roll > 90 && compAngleX < -90)) {
    //kalmanX.setAngle(roll);
    compAngleX = roll;
    //kalAngleX = roll;
    gyroXangle = roll;
  } else {
    //kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  }

  //if (abs(kalAngleX) > 90)
  if (abs(compAngleX) > 90) {
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  }
  //kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  //if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
  if ((pitch < -90 && compAngleY > 90) || (pitch > 90 && compAngleY < -90)) {
    //kalmanY.setAngle(pitch);
    compAngleY = pitch;
    //kalAngleY = pitch;
    gyroYangle = pitch;
  } else {
    //kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  }

  //if (abs(kalAngleY) > 90)
  if (abs(compAngleY) > 90) {
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  }
  //kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll;
#endif

  //gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  //gyroYangle += gyroYrate * dt;
  mPitch = pitch;
  mRoll = roll;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  /*
    original equation values are supposed to be 0.98 and 0.02, respectively
    however, this person used 0.93 and 0.07; leaving it like this because there must be a reason why
    you can use any two values as long as they add up to 1.00
    original equation: angle = 0.98 * (angle + gyrData * dt) + 0.02 * (accData)
  */
  // commented out because this is called above in the preprocessor if statement blocks
  //compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  //compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
}

void areWeSteering() // Are we steering? defined by the boolean SteerMode
{
  /*
  if (signalInX == def && signalInY == def)
  {
    steerMode = false;
  }
  else
  {
    steerMode = true;
  }
  */
  steerMode = !(signalInX == def && signalInY == def); // cleaner way of doing it
}

void stabilization()
{
  /*
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  */

  /*
    order MUST be:
    updateIMU();
    ComplementaryFilter();
    fixGyroDrift();
  */
  updateIMU();
  ComplementaryFilter();
  fixGyroDrift();

  /*
  tempStableS = map(AcX, 5000, 20000, 0, 30);
  tempStableT = map(AcY, -5000, -20000, 0, 30);
  tempStableU = map(AcY, 5000, 20000, 0, 30);
  tempStableV = map(AcX, -5000, -20000, 0, 30);
  */

  // gain is sensitivity of stabilization
  // TODO: gain is set to arbitrary numbers, please determine values
  // http://technicaladventure.blogspot.com/2012/09/quadcopter-stabilization-control-system.html
  double pitchGain = 0.5;
  double rollGain = 0.5;

  /*
    s - ccw - 1
    t - cw - 2
    u - cw - 3
    v - ccw - 4
  */

  // TODO: correct which motors use what
  tempStableS = mRoll * rollGain;
  tempStableT = mPitch * pitchGain;
  tempStableU = mPitch * pitchGain;
  tempStableV = mRoll * rollGain;

  motorInS -= tempStableS;
  motorInT += tempStableT;
  motorInU -= tempStableU;
  motorInV += tempStableV;
}

void steering()
{
  // if Left Controller Stick is moved Left - signalInX < 0
  // if Left Controller Stick is moved Right - signalInX > 0
  // if Left Controller Stick is moved Up - signalInY > 0
  // if Left Controller Stick is moved Down - signalInY < 0

  if (signalInX < 0)
  {
    motorInS = (signalInX * -1) - 5;
    motorInU = signalInX * -1;
  }
  else if (signalInX > 0)
  {
    motorInS = signalInX;
    motorInU = signalInX - 5;
  }

  if (signalInY < 0)
  {
    motorInV = (signalInY * -1) - 5;
    motorInT = (signalInY * -1);
  }
  else if (signalInY > 0)
  {
    motorInV = signalInY;
    motorInT = signalInY - 5;
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
  motorInS += signalInZ;
  motorInU += signalInZ;
  motorInT += signalInZ;
  motorInV += signalInZ;
}

void loop() //loops and runs the methods, writes servo values
{

  // DEBUGGING PURPOSES
  /*
  stabilization();

  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp / 340.00 + 36.53); //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
  Serial.print(" | compAngleX = "); Serial.print(compAngleX);
  Serial.print(" | compAngleY = "); Serial.println(compAngleY);
  Serial.print(" | mRoll = " ); Serial.print(mRoll);
  Serial.print(" | mPitch = " ); Serial.println(mPitch);
  Serial.println(" ");

  Serial.print("tempStableS: "); Serial.println(tempStableS); 
  Serial.print("tempStableT: "); Serial.println(tempStableT);
  Serial.print("tempStableU: "); Serial.println(tempStableU); 
  Serial.print("tempStableV: "); Serial.println(tempStableV);
  
  Serial.print("motorInS: "); Serial.println(motorInS);
  Serial.print("motorInT: "); Serial.println(motorInT);
  Serial.print("motorInU: "); Serial.println(motorInU);
  Serial.print("motorInV: "); Serial.println(motorInV);

  delay(2000);
  */
  
  
  // MAIN CODE

    //  if(count<51)
    //  {
    //   count ++;
    //  }
    signalInX = 0, signalInY = 0, signalInZ = 0;
    motorInS = 0, motorInU = 0,
    motorInT = 0, motorInV = 0, motorInZ = 0;
    motorInX = 0, motorInY = 0;
    if (motorOn)
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
          lastmotorInS = 0;
          lastmotorInT = 0;
          lastmotorInU = 0;
          lastmotorInV = 0;

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



          // Capping the value output to 1 for testing
          if (motorInS >= 8)
            motorInS = 8;
          if (motorInU >= 8)
            motorInU = 8;
          if (motorInV >= 8)
            motorInV = 8;
          if (motorInT >= 8)
            motorInT = 8;

          //Moved stabilization() to AFTER the motor input cap
          stabilization();

          Serial.print("AcX = "); Serial.print(AcX);
          Serial.print(" | AcY = "); Serial.print(AcY);
          Serial.print(" | AcZ = "); Serial.print(AcZ);
          Serial.print(" | Tmp = "); Serial.print(Tmp / 340.00 + 36.53); //equation for temperature in degrees C from datasheet
          Serial.print(" | GyX = "); Serial.print(GyX);
          Serial.print(" | GyY = "); Serial.print(GyY);
          Serial.print(" | GyZ = "); Serial.println(GyZ);
          Serial.print(" | compAngleX = "); Serial.print(compAngleX);
          Serial.print(" | compAngleY = "); Serial.println(compAngleY);
          Serial.println(" ");

          //Capping the value output to 179 to prevent accidental unwanted calibration
          //Uncommented the cap
          if (motorInS >= 180)
            motorInS = 179;
          if (motorInU >= 180)
            motorInU = 179;
          if (motorInV >= 180)
            motorInV = 179;
          if (motorInT >= 180)
            motorInT = 179;


          //Experimental spike protection
          if (fabs(motorInS - lastmotorInS) > 30)
            motorInS = lastmotorInS;
          if (fabs(motorInU - lastmotorInU) > 30)
            motorInU = lastmotorInU;
          if (fabs(motorInT - lastmotorInT) > 30)
            motorInT = lastmotorInT;
          if (fabs(motorInV - lastmotorInV) > 30)
            motorInV = lastmotorInV;

          //Writing them servo values to the servos
          s.write(motorInS);
          t.write(motorInT);
          u.write(motorInU);
          v.write(motorInV);

          //temporarily storing the values
          lastmotorInS = motorInS;
          lastmotorInU = motorInU;
          lastmotorInT = motorInT;
          lastmotorInV = motorInV;

          //Readout of what's being sent do the Servos
          Serial.print("motorInS: "); Serial.println(motorInS);
          Serial.print("motorInT: "); Serial.println(motorInT);
          Serial.print("motorInU: "); Serial.println(motorInU);
          Serial.print("motorInV: "); Serial.println(motorInV);

          theMessage = "";
        }
      }
      else //if no radio
      {
        //    Serial.println("No Radio.");
        //    Serial.println(lastmotorInS);
        //    Serial.println(lastmotorInT);
        //    Serial.println(lastmotorInU);
        //    Serial.println(lastmotorInV);
        //
        s.write(lastmotorInS);
        t.write(lastmotorInT);
        u.write(lastmotorInU);
        v.write(lastmotorInV);
      }
    }
  
}
