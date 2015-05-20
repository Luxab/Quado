import org.gamecontrolplus.gui.*;
import org.gamecontrolplus.*;
import net.java.games.input.*;
import processing.serial.*;
import java.awt.Font;
import g4p_controls.*;

/*
  -Scripps Ranch High School Robotics Team-
  Quadcopter_Motor_Test
  By Michael Yee
  4/17/2015

  Goes hand-in-hand with 
  Controller_ArduinoRegister
*/

float flyUp, flyX, flyY, rawY; // rawY is old for the Z-Axis
byte sendY, sendZ, sendX;
int val, flyUpZ, flyUpX, flyUpY;
Serial myPort;
String v;
boolean firstContact = false, emerStab = false, killMot = false;

// This one is essential
ControlIO control;
// You will need some of the following depending on the sketch.
//Font font = new Font("Monospaced", Font.PLAIN, 12);
// A ControlDevice might be a joystick, gamepad, mouse etc.
ControlDevice device;

// A device will have some combination of buttons, hats and sliders
ControlButton button, buttonKillMot;
ControlHat hat;
ControlSlider slider;
public void setup() {
  size(400, 400);
  String portName = Serial.list()[0];              //list serial ports, save the first one as portName
  myPort = new Serial(this, portName, 115200); 
  //myPort = new Serial(this, portName, 921600); 
  //myPort = new Serial(this, portName, 4000000); 
  control = ControlIO.getInstance(this);
  device = control.getDevice("Controller (XUSB Gamepad)");
  button = device.getButton("Button 0"); // can change
  buttonKillMot = device.getButton("Button 1");
  
  //device = control.getDevice("Mouse");

  if (device == null) {
    println("No suitable device configured");
    System.exit(-1); // End the program NOW!
  }
  
  //frameRate(30);
  frameRate(500);
  
}

// Poll for user input called from the draw() method.
public void getUserInput() {
  //px = map(device.getSlider("X-axis").getValue(), -1, 1, 0, width);
  flyUp = map(device.getSlider("Y Rotation").getValue()/1.725, -1, 1, 0, height);
  flyX = map(device.getSlider("X Axis").getValue()/1.725, -1, 1, 0, width);
  flyY = map(device.getSlider("Y Axis").getValue()/1.725,-1, 1, 0, height);
  // max speed at 179.94202

  
  // default is at 200, 0 is max(up), 400 min(down)

  // change default to -200, -400 is min, 0 is max
  flyUp *= -1; // change Y-Value
  flyX *= -1;
  flyY *= -1;
  
  // flyUp default value at 200
  
  if (flyUp < -198 && flyUp > -208)
  {
    flyUp = -203; // set tolerance, deadzone
  }
  
  flyUp += 267; 

  if (flyUp < 0)
  {
    flyUp = 0;
  }
  if (flyUp > 180)
  {
    flyUp = 180;
  }
  //sendZ = (byte)flyUp;


  if (flyX < -198 && flyX > -208)
  {
    flyX = -203; // set tolerance, deadzone
  }
  flyX += 267;


  if (flyX > 180)
  {
    flyX = 180;
  }
  //sendX = (byte)flyX;
  
  if (flyY < -198 && flyY > -208)
  {
    flyY = -203; // set tolerance, deadzone
  }
  flyY += 267; 

  if (flyY > 180)
  {
    flyY = 180;
  }
  //sendY = (byte)flyY;
  
  flyUpZ = (int)flyUp;
  flyUpX = (int)flyX;
  flyUpY = (int)flyY;
}

public void serialEvent( Serial myPort) {
  //put the incoming data into a String - 
  //the '\n' is our end delimiter indicating the end of a complete packet
  v = myPort.readStringUntil('\n');
  //make sure our data isn't empty before continuing
  if (v != null) {
    //trim whitespace and formatting characters (like carriage return)
    v = trim(v);
    println(v);

    //look for our 'A' string to start the handshake
    //if it's there, clear the buffer, and send a request for data
    if (firstContact == false) {
      if (v.equals("A")) {
        myPort.clear();
        firstContact = true;
        myPort.write("A");
        println("contact");
      }
    } 
    else 
    { //if we've already established contact, keep getting and parsing data
      
      //println("SENDING - sendX: " +sendX+ "  sendY: " +sendY+ "  sendZ : " + sendZ);
      //myPort.write(); // hopefully sends "A" or "B" but may send jibberish or numbers
      myPort.write("X"+flyUpX+"Y"+flyUpY+"Z"+flyUpZ); // sends X,Y,Z to Serial
      //println("SENT - sendX:    " +sendX+ "  sendY: " +sendY+ "  sendZ : " + sendZ);
      println("SENT - sendX: " +flyUpX+ " sendY: " +flyUpY+ " sendZ: " +flyUpZ);
      /*
      if (keyPressed == true)
      {
        System.exit(-1);
      }
      */
      // when you've parsed the data you have, ask for more:
      //myPort.write("A");
      
      if(button.pressed())
      {
       myPort.write("S"); // "S" sent
       println("SENT - emergencyStabilization"); 
      }
      if(buttonKillMot.pressed())
      {
       myPort.write("K"); // "K" sent
       println("SENT - killMot"); 
      }
      
      
    }
  }
}

public void draw() {
  getUserInput(); // Polling
  //println(frameRate);

  background(255, 255, 240);
  // Draw shadows
  fill(0, 0, 255, 32);
  noStroke();
  
  // Show position
  noStroke();
  fill(255, 64, 64, 64);
  ellipse(200-flyUpX+64, 200-flyUpY+64, 20, 20);
  
  
  noStroke();
  fill(200, 200, 200, 200);
  ellipse(200, 200-flyUp+64, 20, 20);
  
  if (keyPressed)
  {
    println("PROGRAM EMERGENCY CLOSE...");
    //System.exit(-1);
  }
}
