#include <Servo.h>

Servo myservo1; // grabber
Servo myservo2; // lifter



int RLEDPin = 7; // the output pin for the Red LED
int GLEDPin = 8; // the output pin for the Green LED


int pos1 = 80; // grabber starting angle
int pos2 = 140; // lifter starting angle
int crashswitchPin = 10;
int magPin = 9;

void setup() {
  myservo1.attach(13); 
  myservo2.attach(12);

  Serial.begin(9600); 
  pinMode(crashswitchPin, INPUT);
  pinMode(magPin, INPUT);
  myservo1.write(pos1); // possibly not needed
  myservo2.write(pos2);
  delay(5000);
}
bool ok = 0;
void loop() {

  int box_held = digitalRead(crashswitchPin); // read crash switch value
  int mag = digitalRead(magPin); // read magnetic sensor

  // if ((pos2 > 0) && (box_held == LOW)) { // lifts box 90 degrees up
  //   pos2 -= 1;
  //   myservo2.write(pos2);
  //   delay(15);
  // }

  // if (pos1 < 100) { // grabber moves from 10 to 100 degrees
  //   pos1 += 1;
  //   myservo1.write(pos1);
  //   delay(30); // could be made faster
  // }

  // // turns on the light when the box is being lifted
  // if ((89 >pos2 && pos2 > 1) && (mag == LOW)) { // box is magnetic so light is green
  //   digitalWrite(RLEDPin, LOW);
  //   digitalWrite(GLEDPin, HIGH);
  //   // GLED == HIGH;
  //   // RLED == LOW;
  // }
  // else if ((89 >pos2 && pos2 > 1) && (mag == HIGH)) { // box isn't magnetic so light is red
  //   // GLED == LOW;
  //   // RLED == HIGH;
  //   digitalWrite(RLEDPin, HIGH);
  //   digitalWrite(GLEDPin, LOW);
  // }
  // else { // lights are off when the box isn't being lifted
  //   // GLED == LOW;
  //   // RLED == LOW;
  //   digitalWrite(RLEDPin, LOW);
  //   digitalWrite(GLEDPin, LOW);
  // }

  int val = digitalRead(crashswitchPin); // read input value
  if(!val) {ok = 1;Serial.println("good");}
  if(pos1<135 && !ok)pos1++;
  if(ok && pos2 > 90)pos2--;
  myservo1.write(pos1); // possibly not needed
  myservo2.write(pos2);
  
// if ((pos2 > 0) && (val == LOW)) { // lifts box 90 degrees up
//     pos2 -= 1;
//     myservo2.write(pos2);
//     delay(15); 
//   }

//   if (pos1 < 100) { // grabber moves from 10 to 100 degrees
//     pos1 += 1;
//     myservo1.write(pos1);
//     delay(30); // could be made faster
//   }

//   if (val == LOW) {
//     Serial.println("Box grabbed"); // or move on to next action
//   }

}