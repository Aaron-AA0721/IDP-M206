#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"

#define MAX_RANG (520)
#define ADC_SOLUTION (1023.0)
DFRobot_VL53L0X ToFSensor;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1); // left Motor on port M1
Adafruit_DCMotor *RightMotor = AFMS.getMotor(2); // right Motor on port M2
Adafruit_DCMotor *exMotor = AFMS.getMotor(3); // extra 18RPM motor

int RLEDPin = 11; // the output pin for the Red LED
int GLEDPin = 12; // the output pin for the Green LED
int BLEDPin = 13; // the output pin for the Blue LED
int ServoPin = 10; // the pin for servo
int LeftLineSensorPin = 7; //the pin for three line followers
int RightLineSensorPin = 8; 
int FrontLineSensorPin = 9;

int MagneticPin = 2; // the input pin for the magenetic sensor
int UltrasonicPin = A0; //the input pin of Ultrasonic Sensor


void setup() {
  Serial.begin(9600);

  Wire.begin();
  ToFSensor.begin(0x50);
  ToFSensor.setMode(ToFSensor.eContinuous,ToFSensor.eHigh);
  ToFSensor.start();

  pinMode(RLEDPin, OUTPUT); // declare LED as output
  pinMode(GLEDPin, OUTPUT);
  pinMode(BLEDPin, OUTPUT);
  pinMode(MagneticPin, INPUT); // declare mag pin as input
  pinMode(LeftLineSensorPin, INPUT);
  pinMode(RightLineSensorPin, INPUT);
  pinMode(FrontLineSensorPin, INPUT);

  myservo.attach(ServoPin)
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");
}
float UltraRead,UltraDistance;
float TofDistance;
int LeftLineRead,RightLineRead,FrontLineRead;
int MagRead = 0; // variable for reading the pin status
void loop(){
  MagRead = digitalRead(inputPin); // read input value
  UltraRead = analogRead(UltrasonicPin);
  UltraDistance = UltraRead * MAX_RANG / ADC_SOLUTION;
  TofDistance = TofSensor.getDistance();
  LeftLineRead =  digitalRead(LeftLineSensorPin);
  RightLineRead =  digitalRead(RightLineSensorPin);
  FrontLineRead =  digitalRead(FrontLineSensorPin);
}
void move(){
 //?
}