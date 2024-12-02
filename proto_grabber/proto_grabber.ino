#include <Servo.h>

Servo grabber; // grabber
Servo lifter; 

int RLEDPin = 7; // the output pin for the Red LED
int GLEDPin = 8; // the output pin for the Green LED


int pos1 = 25; // grabber starting angle
int pos2 = 105;
void Pick(){
  while(pos1<100){
    pos1++;
    grabber.write(pos1);
    delay(30);}
  
  
  Serial.println("down");
  pos2 = 110;
  lifter.write(pos2);
  delay(1000);
  while(pos1>25){
    pos1--;
    grabber.write(pos1);
    delay(30);}
  Serial.println("up");
  pos2 = 105;
  lifter.write(pos2);
  delay(1000);
}
void setup() {
  grabber.attach(7);
  lifter.attach(5);

  Serial.begin(9600); 
  grabber.write(pos1); // possibly not needed
  lifter.write(pos2);
  //delay(2000);
  Serial.println("start");
  Pick();
}
bool ok = 0;
void loop() {


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
  
  //grabber.write(pos1); // possibly not needed
  
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