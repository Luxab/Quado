#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

/*
  -Scripps Ranch High School Robotics Team-
  Quadcopter_Motor_Test
  plus Accel_Test
  By Michael Yee
  && Duncan Klug
  4/14/2015
  4/17/2015

  Testing Quadcopter flying up with controller, as well as accelerometer adjustment.
  Goes hand-in-hand with Quadcopter_Controller_Register.pde
  Version 02
*/

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

Servo s, t, u, v;
int startRotorSpeed = 0; // change later
int maxRotorSpeed = 10; // change later, start low & slow
bool on = true;
int val = 0; // Data received from the serial port
int motorIn = 0;
int lastMotorIn = 0;
int lenIn = 0;

void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup() {

  //initialize serial communications at a 9600 baud rate
  Serial.begin(500000); // open the serial port at 9600 bps
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

  //accelerometer stuff
  //Serial.begin(9600);
  Serial.println("Accelerometer Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();

}

void loop()
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  accel.getEvent(&event);
 
  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
  delay(100);
  
  if (Serial.available() > 0)
  { // If data is available to read,

    String hold = "";
    val = Serial.read(); // read it and store it in val
    hold += val;
    if (hold.equals("Z"))
    {
      while(hold.equals("Z"))
      {
        hold = "";
        val = Serial.read();
        hold += val;
      }
    //Serial.print("VAL: " + hold);
    Serial.println("VAL: " + hold + "   motorIn: " + motorIn); // prints data that is read, debug
      motorIn = hold.toInt();

      if (lastMotorIn > 80 && motorIn == 65)
      {
        s.write(lastMotorIn);
        t.write(lastMotorIn);
        u.write(lastMotorIn);
        v.write(lastMotorIn);
      }
      else
      {

        s.write(motorIn);
        t.write(motorIn);
        u.write(motorIn);
        v.write(motorIn);
        lastMotorIn = motorIn;
      }
    }

    /*
    else
    {
     Serial.println("NO VALUE RECEIEVED.");
     }
     */
  }

  delay(3);
}

void establishContact() {
  while (Serial.available() <= 0) {
    Serial.println("A");   // send a capital A
    delay(50);
  }
}
