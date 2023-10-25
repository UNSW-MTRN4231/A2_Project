/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-servo-motor
 */

#include <Servo.h>

Servo servo;  // create servo object to control a servo

int pos = 0;    // variable to store the servo position
int incomingByte = 0; // for incoming serial data

void setup() {
  Serial.begin(9600);
  servo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop() {

  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();

    if (incomingByte > 0) {
      Serial.print("HI");
      for (pos = 0; pos <= 90; pos += 1) { // rotate from 0 degrees to 180 degrees
        // in steps of 1 degree
        servo.write(pos);                   // tell servo to go to position in variable 'pos'
        delay(10);                          // waits 10ms for the servo to reach the position
      }

      for (pos = 90; pos >= 0; pos -= 1) { // rotate from 180 degrees to 0 degrees
        servo.write(pos);                   // tell servo to go to position in variable 'pos'
        delay(10);                          // waits 10ms for the servo to reach the position
      }
    }

    // say what you got

  }

  // WORKS

  // while (1) {

  // }
}
