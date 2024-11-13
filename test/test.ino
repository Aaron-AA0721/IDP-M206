#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"

#define MAX_RANG (520)
#define ADC_SOLUTION (1023.0)
DFRobot_VL53L0X ToFSensor;
Servo myservo;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1); // left Motor on port M1
Adafruit_DCMotor *rightMotor = AFMS.getMotor(3); // right Motor on port M2
Adafruit_DCMotor *exMotor = AFMS.getMotor(2); // extra 18RPM motor

int RLEDPin = 11; // the output pin for the Red LED
int GLEDPin = 12; // the output pin for the Green LED
int BLEDPin = 13; // the output pin for the Blue LED
int ServoPin = 10; // the pin for servo
int LeftLineSensorPin = 7; //the pin for three line followers
int RightLineSensorPin = 8; 
int FrontLineSensorPin = 9;
int LeftLineBoundaryPin = 6;
int RightLineBoundaryPin = 5;

int MagneticPin = 2; // the input pin for the magenetic sensor
int UltrasonicPin = A0; //the input pin of Ultrasonic Sensor

int edges[12][4] = {{-1,-1,-1,1},{-1,2,0,3},{6,-1,-1,1},{4,1,-1,-1},{7,5,3,-1},{11,6,-1,4},{9,-1,2,5},{-1,8,4,-1},{-1,10,11,7},{-1,-1,6,10},{-1,9,-1,8},{8,-1,5,-1}};
//                   0            1          2           3           4         5           6          7           8             9             10          11
int distance[12][4] = {{20,-1,-1,-1},{-1,100,20,100},{80,-1,-1,100},{80,100,-1,-1},{80,100,80,-1},{40,100,-1,100},{80,-1,80,100},{-1,100,80,-1},{-1,40,40,100},{-1,-1,80,60},{-1,60,-1,40},{40,-1,40,-1}};
//                       0             1               2               3            4              5               6              7              8              9             10             11
bool BoxExists[12][5] = {{0,0,0,0,0},{1,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{1,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};
//                        0            1          2         3                 4           5       6           7             8         9         10            11

bool nodeTraveled[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
int disFromCurr [12] = {0,0,0,0,0,0,0,0,0,0,0,0};
int predecessor[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

bool BoxLoaded = 0;
bool BoxSensed = 0;

int currNode = 0;
int targetNode = 8;
int nextNode = 1;

int state = 0;
int direction = 0;
bool back = 0;

void setup() {
  Serial.begin(9600);
  currNode = 0;
  targetNode = 7;
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
  pinMode(LeftLineBoundaryPin, INPUT);
  pinMode(RightLineBoundaryPin, INPUT);
  myservo.attach(ServoPin);
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }}
  Serial.println("Motor Shield found.");
}
float UltraRead,UltraDistance;
float ToFDistance;
int LeftLineRead,RightLineRead,FrontLineRead,LeftBoundaryRead,RightBoundaryRead;
int MagRead = 0; // variable for reading the pin status

int inputBytes[10];
int inputBytePointer = 0;

void loop(){
  MagRead = digitalRead(MagneticPin); // read input value
  UltraRead = analogRead(UltrasonicPin);
  UltraDistance = UltraRead * MAX_RANG / ADC_SOLUTION;
  
  //Serial.println(UltraDistance,0);
  //Serial.println("cm");
  ToFDistance = ToFSensor.getDistance();
  LeftLineRead =  digitalRead(LeftLineSensorPin);
  RightLineRead =  digitalRead(RightLineSensorPin);
  FrontLineRead =  digitalRead(FrontLineSensorPin);
  //Serial.print("Front: ");
  //Serial.println(FrontLineRead);
  //Serial.print("Left: ");
  //Serial.println(LeftLineRead);
  //Serial.print("Right: ");
  //Serial.println(RightLineRead);
  LeftBoundaryRead = digitalRead(LeftLineBoundaryPin);
  RightBoundaryRead = digitalRead(RightLineBoundaryPin);
  digitalWrite(RLEDPin,HIGH);
  //Serial.println(FrontLineRead);
  delay(500);
  if (Serial.available() > 0)
    {
    // read the incoming byte:
    int incomingByte = 0;
    incomingByte = Serial.read();
    if(inputBytePointer<10) {
      inputBytes[inputBytePointer] = incomingByte;
      inputBytePointer++;
    // say what you got:
      Serial.print("I received: ");
      Serial.println(incomingByte, DEC);}
    }
  int next = PathFinding(currNode,targetNode);
  int tAngle;
  switch(state){
    case 0://moving
      tAngle = (direction+ back?2:0)%4;
      leftMotor->setSpeed((back^LeftBoundaryRead)?200:255);
      rightMotor->setSpeed((back^RightBoundaryRead)?200:255);
      leftMotor->run(back?FORWARD:BACKWARD);
      rightMotor->run(back?BACKWARD:FORWARD);
      if(FrontLineRead == (edges[node][(0+tAngle)%4] != -1) && LeftLineRead == (edges[node][(3+tAngle)%4] != -1) && RightLineRead == (edges[node][(1+tAngle)%4] != -1)){
        leftMotor->run(RELEASE);
        rightMotor->run(RELEASE);
        currNode = node;
        state = 1;
      }
      if(BoxSensed){
        leftMotor->run(RELEASE);
        rightMotor->run(RELEASE);
        state = 2;
      }
      break;
    case 1://rotating
      break;
    case 2://picking
      break;
    case 3://dropping
      break;
  }
  /*if(inputBytes[inputBytePointer-1]==10){
    int num = 0;
    inputBytePointer-=2;
    
    switch(state){
    case 0:
      Serial.print("Inputing for curr: ");
      for(int i=inputBytePointer;i>=0;i--){
        num+= (inputBytes[i]-'0')*pow(10,inputBytePointer-i);
      }
      currNode = num;
      state = 1;inputBytePointer=0;
      Serial.println(num);
      break;
    case 1:
      Serial.print("Waiting for des: ");
      for(int i=inputBytePointer;i>=0;i--){
        num+= (inputBytes[i]-'0')*pow(10,inputBytePointer-i);
      }
      targetNode = num;
      state = 2;inputBytePointer=0;
      Serial.println(num);
      break;
    case 2:
      Serial.print("Waiting for box loaded: ");
      for(int i=inputBytePointer;i>=0;i--){
        num+= (inputBytes[i]-'0')*pow(10,inputBytePointer-i);
      }
      BoxLoaded = num%2;
      Serial.println(num);
      state = 0;inputBytePointer=0;
      int next = PathFinding(currNode,targetNode);
      Serial.println(next);
      moveToNode(next,0);
      turn(1);
      break;
    default:
      break;
  }
  }*/
  
  
  
  
}

int PathFinding(int curr, int des){
  for(int i=0;i<12;i++){
    nodeTraveled[i]=0;
    disFromCurr[i]=1e9;
    predecessor[i]=curr;
  }
  nodeTraveled[curr]=1;
  disFromCurr[curr]=0;
  int initCurr = curr;
  int minD = 1e9;
  int nextCurr = curr;
  int index = -1;
  while(curr!=des){
    Serial.println(curr);
    nodeTraveled[curr]=1;
    minD = 1e9;
    for(int i=0;i<12;i++){
      index = intIndexInArray(i,curr);
      if(!nodeTraveled[i]){
        if(index!=-1){
          if(disFromCurr[i]>disFromCurr[curr]+edges[curr][index] && (!BoxLoaded || (!BoxExists[curr][index+1] && !BoxExists[i][0]))){
            disFromCurr[i]=disFromCurr[curr]+edges[curr][index];
            predecessor[i]=curr;
          }
        }
        if(disFromCurr[i]<minD){
          minD=disFromCurr[i];
          nextCurr = i;
        }
      }
    }
    curr = nextCurr;
  }
  while(predecessor[curr]!=initCurr){
    Serial.print("->");
    Serial.println(predecessor[curr]);
    curr = predecessor[curr];
  }
  Serial.println("Found next");
  Serial.println(curr);
  return curr;
}


int intIndexInArray(int goal, int curr){
  for(int i=0;i<4;i++){
    if(goal == edges[curr][i])return i;
  }
  return -1;
}

void moveToNode(int node, bool back){
  leftMotor->setSpeed(255);
  rightMotor->setSpeed(255);
  int tAngle = (direction+ back?2:0)%4;
  while(FrontLineRead != (edges[node][(0+tAngle)%4] != -1) || LeftLineRead != (edges[node][(3+tAngle)%4] != -1) || RightLineRead != (edges[node][(1+tAngle)%4] != -1)){
    leftMotor->run(back?FORWARD:BACKWARD);
    rightMotor->run(back?BACKWARD:FORWARD);
    LeftLineRead =  digitalRead(LeftLineSensorPin);
    RightLineRead =  digitalRead(RightLineSensorPin);
    FrontLineRead =  digitalRead(FrontLineSensorPin);
    LeftBoundaryRead = digitalRead(LeftLineBoundaryPin);
    RightBoundaryRead = digitalRead(RightLineBoundaryPin);
  }
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  currNode = node;
}
void turn(int angle){
  leftMotor->setSpeed(255);
  rightMotor->setSpeed(255);
  int tAngle = (direction + angle + 4)%4;
  bool back = angle>0;
  while(FrontLineRead != (edges[currNode][(0+tAngle)%4] != -1) || 
  LeftLineRead != (edges[currNode][(3+tAngle)%4] != -1) || 
  RightLineRead != (edges[currNode][(1+tAngle)%4] != -1) ||
  LeftBoundaryRead == 1 ||
  RightBoundaryRead == 1
  ){
    leftMotor->run(back?BACKWARD:FORWARD);
    rightMotor->run(back?BACKWARD:FORWARD);
    LeftLineRead =  digitalRead(LeftLineSensorPin);
    RightLineRead =  digitalRead(RightLineSensorPin);
    FrontLineRead =  digitalRead(FrontLineSensorPin);
    LeftBoundaryRead = digitalRead(LeftLineBoundaryPin);
    RightBoundaryRead = digitalRead(RightLineBoundaryPin);
  }
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  direction = tAngle;
}
