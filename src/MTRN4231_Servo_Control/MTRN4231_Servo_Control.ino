/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-servo-motor
 */

#include <Servo.h>

Servo servo1;  // create servo object to control a servo
Servo servo2;

int pos = 0;    // variable to store the servo position
int incomingByte = 0; // for incoming serial data

void setup() {
  Serial.begin(9600);
  Serial.println("Setting up Arduino");
  servo1.attach(9);  // attaches the servo on pin 9 to the servo object
  servo2.attach(10);
  servo1.write(45);
  servo2.write(45);
}

void loop() {

  if (Serial.available() > 0) {
    // read the incoming byte:
    String command = Serial.readString();  //read until timeout
     command.trim();   
    
    if (command == "open") {
      Serial.println("Opening Gripper");
      
      // in steps of 1 degree
      servo1.write(45);                   // tell servo to go to position in variable 'pos'
      servo2.write(45);
      delay(10);                          // waits 10ms for the servo to reach the position
      
    } else if (command == "close") {
      Serial.println("Closing Gripper");
      servo1.write(40);
      servo2.write(20);  
      delay(10);
    }
    else {
      Serial.print("Unknown Command Recieved:" ); Serial.println(command);
    }

  }
}
