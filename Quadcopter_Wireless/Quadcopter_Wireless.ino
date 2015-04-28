#include <SPI.h>
#include "RF24.h"

bool radioNumber = false;

RF24 radio(7, 8); // pins 7 & 8

byte addresses[][6] = {"1Node", "2Node"};
// dont ask me why its named that

bool role = false;

void setup() {

  Serial.begin(1000000);
  Serial.println("Wireless Test");

  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  // PA level low to prevent power supply issues ???

  if (radioNumber)
  {
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1, addresses[0]);
  }
  else
  {
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1, addresses[1]);
  }

  // Start the radio listening for data
  radio.startListening();
}
//copy pasta ^


void loop() {

  // Ping Out //

  if (role == true)
  {
    radio.stopListening();

    Serial.println("Sending . . . ");

    int time = micros();

    // if the radio does not write at X Time with
    // this number of bytes
    // then print "Sending Failed . . ."
    if (!radio.write( &time, sizeof(int) ) )
    {
      Serial.println("Sending Failed . . .");
    }

    radio.startListening();

    int beganListeningAt = micros();
    boolean timeout = false;

    while (!radio.available())
    {
      // 200ms (200k microseconds)
      if (micros() - beganListeningAt > 200000) 
        timeout = true;
      break;
    }

    if (timeout)
    {
      Serial.println("Failed, response timed out.");
    }
    else
    {
      int gotTime;
      radio.read(&gotTime, sizeof(int) );
      time = micros();
      
      // Spew it
        Serial.print(("Sent "));
        Serial.print(time);
        Serial.print((", Got response "));
        Serial.print(gotTime);
        Serial.print((", Round-trip delay "));
        Serial.print(time-gotTime);
        Serial.println((" microseconds"));
      
    }
    
    delay(500); // try again later in 500ms
  }
  
  if(role == false)
  {
   int gotTime;
  
  if(radio.available())
 {
  while(radio.available())
  {
   radio.read(&gotTime, sizeof(int)); 
  }
  radio.stopListening();
  radio.write(&gotTime, sizeof(int));
  radio.startListening();
  Serial.print("Sent Response.");
  Serial.println(gotTime);
 } 
  }
  
  
}

