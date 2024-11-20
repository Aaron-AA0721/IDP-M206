#include <Servo.h>

Servo myservo1; // grabber
Servo myservo2; // lifter

int pos1 = 10; // lifter starting angle
int pos2 = 90; // grabber starting angle
int crashswitchPin = 7;
int magPin = 9;

void setup() {
  myservo1.attach(12); 
  myservo2.attach(4);
  Serial.begin(9600); 
  pinMode(crashswitchPin, INPUT);
  myservo1.write(pos1); // possibly not needed
  myservo2.write(pos2);
}

void loop() {
  int val = digitalRead(crashswitchPin); // read input value

if ((pos2 > 0) && (val == LOW)) { // lifts box 90 degrees up
    pos2 -= 1;
    myservo2.write(pos2);
    delay(15); 
  }

  if (pos1 < 100) { // grabber moves from 10 to 100 degrees
    pos1 += 1;
    myservo1.write(pos1);
    delay(30); // could be made faster
  }

  if (val == LOW) {
    Serial.println("Box grabbed"); // or move on to next action
  }
}