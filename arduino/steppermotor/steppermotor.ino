/*
 Keyboard test

 For the Arduino Leonardo, Micro or Due

 Reads a byte from the serial port, sends a keystroke back.
 The sent keystroke is one higher than what's received, e.g.
 if you send a, you get b, send A you get B, and so forth.

 The circuit:
 * none

 created 21 Oct 2011
 modified 27 Mar 2012
 by Tom Igoe

 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/KeyboardSerial
 */

#include "Keyboard.h"
const int PUL=22; //define Pulse pin
const int DIR=2; //define Direction pin
const int ENA=3; //define Enable Pin
void setup() {
  // open the serial port:
  Serial.begin(9600);
  // initialize control over the keyboard:
  pinMode (PUL, OUTPUT);
  pinMode (DIR, OUTPUT );
  pinMode (ENA, OUTPUT);

}

void loop() {
  // check for incoming serial data:
  if (Serial.available() > 0) {
    // read incoming serial data:
    char inChar = Serial.read();
    // Type the next ASCII value from what you received:
    while (inChar != 'i'){
      for (int i=0; i<200; i++)    //Forward 6400 steps
      {
    digitalWrite(DIR,LOW);
    digitalWrite(ENA,HIGH);
    digitalWrite(PUL,HIGH);
    delayMicroseconds(1250);
    digitalWrite(PUL,LOW);
    delayMicroseconds(1250);
  }
  inChar = 'i';
      }
  }
}

