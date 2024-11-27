#include <Servo.h>

Servo myservo1; // grabber


int RLEDPin = 7; // the output pin for the Red LED
int GLEDPin = 8; // the output pin for the Green LED


int pos1 = 90; // grabber starting angle

int infraredPin = 8;
int magPin = 9;

void setup() {
  myservo1.attach(7);

  Serial.begin(9600); 
  pinMode(infraredPin, INPUT);
  pinMode(magPin, INPUT);
  myservo1.write(pos1); // possibly not needed
  delay(2000);
  Serial.println("start");
}
bool ok = 0;
void loop() {

  int box_found = digitalRead(infraredPin); // read crash switch value
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
  if(box_found){

  }
  //if(pos1>30)pos1--;
  myservo1.write(pos1); // possibly not needed
  
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