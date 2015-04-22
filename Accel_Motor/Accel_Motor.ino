#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Servo.h>

/*
  Accelerometer motor adjustment
  Duncan Klug
  4-21-15
  ( ͡° ͜ʖ ͡°)
*/

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;


/*
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
*/

void setup(void) 
{
  servo1.attach(9);
  servo2.attach(10);
  servo3.attach(11);
  servo4.attach(12);
  
  Serial.begin(9600);
  Serial.println("Accelerometer Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  //displaySensorDetails();
}

void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  accel.getEvent(&event);
 
  /* Display the results (acceleration is measured in m/s^2) */
 /* Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
 */
 
  int position;  //declares an integer with no value
   int pval = 54; //pval -> previous value. For use with implementing into other code?
   double s1V = 100*abs(event.acceleration.x);
   double s3V = 100*abs(event.acceleration.x);
   double s2V = 100*abs(event.acceleration.y);
   double s4V = 100*abs(event.acceleration.y);
   
   double servo1Pos=map(s1V,0.0,1000.0,pval,180.0);
   double servo2Pos=map(s2V,0.0,1000.0,pval,180.0);
   double servo3Pos=map(s3V,0.0,1000.0,pval,180.0);
   double servo4Pos=map(s4V,0.0,1000.0,pval,180.0);
   
   if (servo1Pos<100 && servo3Pos<100)
   {
   //Serial.println(servoPos);
   servo1.write(servo1Pos);
   servo3.write(servo3Pos);
   }
   else
   {
   servo1.write(100);
   servo3.write(100);
   }
   
   if (servo2Pos<100 && servo4Pos<100)
   {
   //Serial.println(servoPos);
   servo2.write(servo2Pos);
   servo4.write(servo4Pos);
   }
   else
   {
   servo2.write(100);
   servo4.write(100);
   }
   Serial.println("");
    Serial.print("Servo 1: "); Serial.print(servo1Pos); 
    Serial.print("; Servo 2: "); Serial.print(servo2Pos); 
    Serial.print("; Servo 3: "); Serial.print(servo3Pos); 
    Serial.print("; Servo 4: "); Serial.print(servo4Pos);
  //delay(10);
}
