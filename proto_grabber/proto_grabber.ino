#include <Servo.h>

Servo myservo1; // grabber
Servo myservo2; // lifter

int pos1 = 10; // lifter starting angle
int pos2 = 90; // grabber starting angle
int crashswitchPin = 7;  // the input pin for the switch
int magPin = 9;  // the input pin for the magnetic sensor
int RLEDPin = 11; // the output pin for the Red LED
int GLEDPin = 12; // the output pin for the Green LED

void setup() {
  myservo1.attach(10); 
  myservo2.attach(4);
  Serial.begin(9600); 
  pinMode(crashswitchPin, INPUT);
  pinMode(magPin, INPUT);
  myservo1.write(pos1); // possibly not needed
  myservo2.write(pos2);
}

void loop() {
  int box-held = digitalRead(crashswitchPin); // read crash switch value
  int mag = digitalRead(magPin); // read magnetic sensor

  if ((pos2 > 0) && (box-held == LOW)) { // lifts box 90 degrees up
    pos2 -= 1;
    myservo2.write(pos2);
    delay(15);
  }

  if (pos1 < 100) { // grabber moves from 10 to 100 degrees
    pos1 += 1;
    myservo1.write(pos1);
    delay(30); // could be made faster
  }

  // turns on the light when the box is being lifted
  if ((89 >pos2 > 1) && (mag == LOW)) { // box is magnetic so light is green
    GLED == HIGH
    RLED == LOW
  }
  elif ((89 >pos2 > 1) && (mag == HIGH)) { // box isn't magnetic so light is red
    GLED == LOW
    RLED == HIGH
  }
  else { // lights are off when the box isn't being lifted
    GLED == LOW
    RLED == LOW
  }
}