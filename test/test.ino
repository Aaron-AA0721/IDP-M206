#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);

int ledPin = 13; // choose the pin for the LED
int magPin = 2; // choose the input pin
int magBox = 0; // variable for reading the pin status

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT); // declare LED as output
  pinMode(magPin, INPUT); // declare pushbutton as input
}
void loop(){
  magBox = digitalRead(inputPin); // read input value

}
void move(){

}