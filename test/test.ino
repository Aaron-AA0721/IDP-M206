#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"

#define MAX_RANG (520)
#define ADC_SOLUTION (1023.0)
DFRobot_VL53L0X ToFSensor;
Servo grabber; // grabber
Servo lifter; // lifter
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1); // left Motor on port M1
Adafruit_DCMotor *rightMotor = AFMS.getMotor(3); // right Motor on port M2
//Adafruit_DCMotor *exMotor = AFMS.getMotor(2); // extra 18RPM motor

int RLEDPin = A3; // the output pin for the Red LED
int GLEDPin = A1; // the output pin for the Green LED
int BLEDPin = A2; // the output pin for the Blue LED

int BlueState = 0; // stores state of Blue LED so it can flash

int grabberPin = 7; // the pin for the servos
int lifterPin = 5; 

//int crashswitchPin = 11;
int MagneticPin = 2; // the input pin for the magenetic sensor
int MagneticPin2 = 4;

int infraredPin = 8;
int LeftLineSensorPin = 12; // outside pins for line followers
int RightLineSensorPin = 9;
//int FrontLineSensorPin = 9;
int LeftLineBoundaryPin = 11; // inside pins for line followers
int RightLineBoundaryPin = 10;

int Lspeed,Rspeed;

int ButtonPin = 3; //GreenButton
int RedButtonPin = 6;

int UltrasonicPin = A0; //the input pin of Ultrasonic Sensor

int BoxDropped[2] = {0,0};

int edges[12][4] = {{1,-1,-1,-1},{-1,2,0,3},{6,-1,9999,1},{4,1,9999,-1},{7,5,3,-1},{11,6,-1,4},{9,-1,2,5},{-1,8,4,-1},{-1,10,11,7},{-1,-1,6,10},{-1,9,9999,8},{8,-1,5,9999}};
//                   0            1          2           3                 4            5           6          7           8             9             10          11
int distance[12][4] = {{20,-1,-1,-1},{-1,100,20,100},{80,-1,9999,100},{80,100,9999,-1},{80,100,80,-1},{40,100,-1,100},{80,-1,80,100},{-1,100,80,-1},{-1,40,40,100},{-1,-1,80,60},{-1,60,-1,40},{40,-1,40,-1}};
//                       0             1               2               3                4              5               6              7              8                  9             10             11
bool BoxExists[12][5] = {{0,0,0,0,0},{1,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{1,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};
//                        0            1          2         3                 4           5       6           7             8         9         10            11
int BoxPos[6] = {1,5,-1,-1,65,31}; // integer < 10 means a box at the node, integer <10 && >100 means a box on edge, if currBox>=4 Time of flight sensor will be on
bool BoxOffLine = 0;
//bool BoxCollected[6] = {0,0,0,0,0,0};
int CurrBox = 0;
int DesNodeSeq[5] = {1,-1,-1,-1,-1}; //the sequence of target nodes
int TarP = 0;//target pointer, the pointer(index) of the next target in DesNodeSeq

bool nodeTraveled[12] = {0,0,0,0,0,0,0,0,0,0,0,0};//temp arrays for path finding
int disFromCurr [12] = {0,0,0,0,0,0,0,0,0,0,0,0};
int predecessor[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

bool BoxLoaded = 0;
bool BoxMagnetic = 0;
bool BoxSensed = 0;
int BoxDelivered = 0;

int currNode = 0;
int targetNode = 8;
int nextNode = 1;
int tAngle;

int state = 0;
int direction = 0;
bool back = 0;

const int LongEdgeDistance = 75;

float UltraRead,UltraDistance;
float ToFDistance;
int LeftLineRead,RightLineRead,LeftBoundaryRead,RightBoundaryRead;
int crashswitchRead,infraredRead;
//int FrontLineRead;
int MagRead = 0; // variable for reading the pin status

int grabberAngle = 25;
int lifterAngle = 105;
bool inSlot = 0;
float MaxTofDistance = 0;
float InitTofDistance = 0;
bool PickedBoxOffline = 0;

int inputBytes[10];
int inputBytePointer = 0;//buffer for serial input data

bool reach = 0;
bool start = 0;
bool end = 0;

void MotorRun(int speedL,int speedR,uint8_t modeL,uint8_t modeR){ // drives the robot, updates the blue light accordingly
  leftMotor->setSpeed(speedL);
  rightMotor->setSpeed(speedR);
  leftMotor->run(modeL);
  rightMotor->run(modeR);
  if(modeL == RELEASE && modeR == RELEASE) digitalWrite(BLEDPin, LOW); // not, blue led off
  else digitalWrite(BLEDPin, BlueState?HIGH:LOW); // moving, blue led on
}

int PathFinding(int curr, int des){ // set current and destination, return next node, Dijkstra's algorithm
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
    //Serial.println(curr);
    nodeTraveled[curr]=1;
    minD = 1e9;
    for(int i=0;i<12;i++){
      index = IndexInArray(i,curr);
      if(!nodeTraveled[i]){
        if(index!=-1){
          if(disFromCurr[i]>disFromCurr[curr]+distance[curr][index] && (!BoxLoaded || (!BoxExists[curr][index+1] && !BoxExists[i][0]))){//if box loaded, edge/node with boxes will not be available
            disFromCurr[i]=disFromCurr[curr]+distance[curr][index];
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
    //Serial.print("->");
    //Serial.println(predecessor[curr]);
    curr = predecessor[curr];
  }
  //Serial.println("Found next");
  //Serial.println(curr);
  return curr;
}

// finds closest box
void BoxFinding(int cBox,int cNode){ // set current box to picked, find destination node,might be updated later but with limited boxes it seems easier to use if statements?
//could do another Dijkstra again, but as 6 boxes are fixed at nodes/ on edges, there is no need to do that
  while(BoxPos[cBox] == -1 && cBox<6){cBox++;BoxDelivered++;}
  if(cBox == 6){
    DesNodeSeq[0] = 0;
    for(int i=1;i<5;i++)DesNodeSeq[i] = -1;
    return;
  }
  if(BoxPos[cBox] < 10) {
    DesNodeSeq[0] = BoxPos[cBox];
    for(int i=1;i<5;i++)DesNodeSeq[i] = -1;
    return;
  }
  if(BoxPos[cBox] < 100) {
    DesNodeSeq[0] = BoxPos[cBox]%10;
    DesNodeSeq[1] = BoxPos[cBox]/10;
    for(int i=2;i<5;i++)DesNodeSeq[i] = -1;
    return;
  }
  return;
}

// if true, there should be a box ahead, if there should not be according to the box locations stores, PickBox() will not be called by infrared sensor signals
bool BoxAhead(int cBox,int cNode,int nNode){ 
  if(BoxPos[cBox] < 10) {
    return nNode == BoxPos[cBox];
  }
  if(BoxPos[cBox] < 100) {
    return (cNode == BoxPos[cBox]%10 && nNode == BoxPos[cBox]/10);
  }
  return false;
}
//after picking boxes, update box position array
void UpdateBox(int pickedBox){
  if(BoxPos[pickedBox] < 10) {
    BoxExists[BoxPos[0]][0] = 0;
    return;
  }
  if(BoxPos[pickedBox] < 100) {
    int a = BoxPos[pickedBox]%10;int b = BoxPos[pickedBox]/10;
    BoxExists[b][IndexInArray(a,b)] = BoxExists[a][IndexInArray(b,a)] = 0;
    return;
  }
}

int IndexInArray(int goal, int curr){
  for(int i=0;i<4;i++){
    if(goal == edges[curr][i])return i;
  }
  return -1;
}

void DropBox(){
  int LeftOrRighError = 0;
  tAngle = currNode == 10?2:3;
  bool turnDesPorN = ( (tAngle-direction)>0 ) ? ((tAngle-direction)>2?0:1) : ((tAngle-direction)<-2?1:0);
  reach = 0;
  // leftMotor->setSpeed(255);
  // rightMotor->setSpeed(255);
  Lspeed = !turnDesPorN?127:255;
  Rspeed = turnDesPorN?127:255;
  while(1){
    LeftBoundaryRead = digitalRead(LeftLineBoundaryPin);
    RightBoundaryRead = digitalRead(RightLineBoundaryPin);
    // leftMotor->run(turnDesPorN?BACKWARD:RELEASE);
    // rightMotor->run(turnDesPorN?RELEASE:FORWARD);
    MotorRun(Lspeed,Rspeed,turnDesPorN?BACKWARD:FORWARD,turnDesPorN?BACKWARD:FORWARD);
    if(!LeftBoundaryRead && !RightBoundaryRead)reach = 1;
    if(reach){
      if(LeftBoundaryRead && RightBoundaryRead){
        // leftMotor->run(RELEASE);
        // rightMotor->run(RELEASE);
        MotorRun(Lspeed,Rspeed,RELEASE,RELEASE);
        direction = tAngle;
        break;
        reach = 0;
        Serial.print("turned, now dir & next: ");
        Serial.print(nextNode);
        Serial.println(direction);
      }
    }
  }
  back = 0;
  reach = 0;
  int distanceD = 140 - BoxDropped[currNode == 10?0:1]*30;
  if(distanceD<60) distanceD = 60;
  for(int i=0;i< distanceD;i++){
    LeftBoundaryRead = digitalRead(LeftLineBoundaryPin);
    RightBoundaryRead = digitalRead(RightLineBoundaryPin);
    Lspeed = (((!back&&!RightBoundaryRead)||(back && LeftBoundaryRead)) )?50:255;
    Rspeed = (((!back&&!LeftBoundaryRead)||(back && RightBoundaryRead)) )?50:255;
    if((LeftBoundaryRead^!back)&&(RightBoundaryRead^!back)){
      Lspeed = Rspeed = 255;
    }
    // leftMotor->run(back?FORWARD:BACKWARD);
    // rightMotor->run(back?BACKWARD:FORWARD);
    MotorRun(Lspeed,Rspeed,back?FORWARD:BACKWARD,back?BACKWARD:FORWARD);
    delay(5);
  }
  MotorRun(Lspeed,Rspeed,RELEASE,RELEASE); //opens the grabber to drop the box
  lifterAngle = 95; 
  lifter.write(lifterAngle);
  grabberAngle = 90;
  grabber.write(grabberAngle);
  delay(300);
  lifterAngle = 115; 
  lifter.write(lifterAngle);
  delay(300);

  back = 1;
  LeftLineRead =  digitalRead(LeftLineSensorPin);
  RightLineRead =  digitalRead(RightLineSensorPin);
  Lspeed = Rspeed = 255;
  while(!LeftLineRead || !RightLineRead){
    LeftLineRead =  digitalRead(LeftLineSensorPin);
    RightLineRead =  digitalRead(RightLineSensorPin);
    LeftBoundaryRead = digitalRead(LeftLineBoundaryPin);
    RightBoundaryRead = digitalRead(RightLineBoundaryPin);
    if(LeftOrRighError == 0){
      if(!LeftBoundaryRead)LeftOrRighError == -1;
      if(!RightBoundaryRead)LeftOrRighError = 1;
    }
    /* Lspeed = (((!back&&!RightBoundaryRead)||(back && LeftBoundaryRead)) )?50:255;
    Rspeed = (((!back&&!LeftBoundaryRead)||(back && RightBoundaryRead)) )?50:255;
    if((LeftBoundaryRead^!back)&&(RightBoundaryRead^!back)){
      Lspeed = Rspeed = 255;
    }
    leftMotor->setSpeed((((!back&&!RightBoundaryRead)||(back && LeftBoundaryRead)) )||(LeftLineRead&&!RightLineRead))?50:255);
    rightMotor->setSpeed((((!back&&!LeftBoundaryRead)||(back && RightBoundaryRead)) )||(RightLineRead&&!LeftLineRead))?50:255);
    Lspeed = ( (!back&&!RightBoundaryRead) || (back && LeftBoundaryRead) || (LeftLineRead&&!RightLineRead) )?50:255;
    Rspeed =( (!back&&!LeftBoundaryRead) || (back && RightBoundaryRead) || (RightLineRead&&!LeftLineRead) )?50:255;
    Lspeed = (LeftLineRead&&!RightLineRead)?50:255;
    Rspeed = (RightLineRead&&!LeftLineRead)?50:255;
    if((LeftBoundaryRead^!back)&&(RightBoundaryRead^!back)){
      // leftMotor->setSpeed(255);
      // rightMotor->setSpeed(255);
      Lspeed = Rspeed = 255;
    }
    leftMotor->run(back?FORWARD:BACKWARD);
    rightMotor->run(back?BACKWARD:FORWARD); */
    MotorRun(Lspeed,Rspeed,FORWARD,BACKWARD);
  }
  
  /* while(LeftLineRead || RightLineRead){
    LeftLineRead =  digitalRead(LeftLineSensorPin);
    RightLineRead =  digitalRead(RightLineSensorPin);
    LeftBoundaryRead = digitalRead(LeftLineBoundaryPin);
    RightBoundaryRead = digitalRead(RightLineBoundaryPin);
    // Lspeed = (((!back&&!RightBoundaryRead)||(back && LeftBoundaryRead)) )?50:255;
    // Rspeed = (((!back&&!LeftBoundaryRead)||(back && RightBoundaryRead)) )?50:255;
    // if((LeftBoundaryRead^!back)&&(RightBoundaryRead^!back)){
    //   Lspeed = Rspeed = 255;
    // }
    // leftMotor->setSpeed((((!back&&!RightBoundaryRead)||(back && LeftBoundaryRead)) )||(LeftLineRead&&!RightLineRead))?50:255);
    // rightMotor->setSpeed((((!back&&!LeftBoundaryRead)||(back && RightBoundaryRead)) )||(RightLineRead&&!LeftLineRead))?50:255);
    // Lspeed = ( (!back&&!RightBoundaryRead) || (back && LeftBoundaryRead) || (LeftLineRead&&!RightLineRead) )?50:255;
    // Rspeed =( (!back&&!LeftBoundaryRead) || (back && RightBoundaryRead) || (RightLineRead&&!LeftLineRead) )?50:255;
    Lspeed = (!LeftLineRead&&RightLineRead)?50:255;
    Rspeed = (!RightLineRead&&LeftLineRead)?50:255;
    // if((LeftBoundaryRead^!back)&&(RightBoundaryRead^!back)){
    //   // leftMotor->setSpeed(255);
    //   // rightMotor->setSpeed(255);
    //   Lspeed = Rspeed = 255;
    // }
    // leftMotor->run(back?FORWARD:BACKWARD);
    // rightMotor->run(back?BACKWARD:FORWARD);
    MotorRun(Lspeed,Rspeed,FORWARD,BACKWARD);
    delay(10);
  } */
  MotorRun(Lspeed,Rspeed,RELEASE,RELEASE); // drives out backwards and closes the grabber
  grabberAngle = 25;
  grabber.write(grabberAngle);
  lifterAngle = 105;
  lifter.write(lifterAngle);
  delay(500);
  /* if(LeftOrRighError != 0){
    while(!LeftBoundaryRead || !RightBoundaryRead){
    LeftBoundaryRead = digitalRead(LeftLineBoundaryPin);
    RightBoundaryRead = digitalRead(RightLineBoundaryPin);
    Lspeed = LeftBoundaryRead?0:255;
    Rspeed = RightBoundaryRead?0:255;
    MotorRun(Lspeed,Rspeed,LeftOrRighError==1?FORWARD:BACKWARD,LeftOrRighError==1?FORWARD:BACKWARD);
  }
  } */
  
  //BoxPos[CurrBox] = 0;
  CurrBox++;
  BoxDelivered++;
  TarP = 0;
  BoxMagnetic = 0 ;
  PickedBoxOffline = 0;
  BoxDropped[currNode == 10?0:1]++;
  BoxFinding(CurrBox,currNode);
}

void PickBox(){
  //digitalWrite(BLEDPin, LOW); // not moving, blue led off    
      
  while(grabberAngle<90){
    MotorRun(Lspeed,Rspeed,RELEASE,RELEASE);
    grabberAngle++;
    grabber.write(grabberAngle);
    //Serial.println(grabberAngle);
    delay(5);
  }
  lifterAngle = 110;
  lifter.write(lifterAngle);
  /* delay(1000);
  for(int i=0;i<15;i++){
    LeftBoundaryRead = digitalRead(LeftLineBoundaryPin);
    RightBoundaryRead = digitalRead(RightLineBoundaryPin);
    Lspeed = RightBoundaryRead ? 255 : 50;
    Rspeed = LeftBoundaryRead ?255 : 50;
    if(!LeftBoundaryRead && !RightBoundaryRead){
      Lspeed = Rspeed = 255;
    }
    MotorRun(Lspeed,Rspeed,BACKWARD,FORWARD);
    delay(5);
  } */
  int counterI = 0;
  while(!infraredRead){
    counterI++;
    if(counterI>300)break;
    Serial.println(counterI);
    infraredRead = digitalRead(infraredPin);
    LeftBoundaryRead = digitalRead(LeftLineBoundaryPin);
    RightBoundaryRead = digitalRead(RightLineBoundaryPin);
    Lspeed = RightBoundaryRead ? 255 : 50;
    Rspeed = LeftBoundaryRead ?255 : 50;
    if(!LeftBoundaryRead && !RightBoundaryRead){
      Lspeed = Rspeed = 255;
    }
  // leftMotor->run(back?FORWARD:BACKWARD);
  // rightMotor->run(back?BACKWARD:FORWARD);
  MotorRun(Lspeed,Rspeed,BACKWARD,FORWARD);
  //Serial.println("moving towards box");
  }
  for(int i=0;i<35;i++){
    LeftBoundaryRead = digitalRead(LeftLineBoundaryPin);
    RightBoundaryRead = digitalRead(RightLineBoundaryPin);
    Lspeed = RightBoundaryRead ? 255 : 50;
    Rspeed = LeftBoundaryRead ?255 : 50;
    if(!LeftBoundaryRead && !RightBoundaryRead){
      Lspeed = Rspeed = 255;
    }
    MotorRun(Lspeed,Rspeed,BACKWARD,FORWARD);
    delay(5);
  }
  Lspeed = 110;
  Rspeed = 110;
  while(grabberAngle>25){
    grabberAngle--;
    grabber.write(grabberAngle);
    MotorRun(Lspeed,Rspeed,BACKWARD,BACKWARD);
    //Serial.println(grabberAngle);
    if(digitalRead(MagneticPin) || digitalRead(MagneticPin2))BoxMagnetic=1;
    delay(10);
  }
  lifterAngle = 105;
  lifter.write(lifterAngle);
  MotorRun(Lspeed,Rspeed,RELEASE,RELEASE);
  //delay(1000);
  Lspeed = 100;
  Rspeed = 100;
  for(int i=0;i<25;i++){
    MotorRun(Lspeed,Rspeed,FORWARD,FORWARD);
    delay(10);
  }
  for(int i=0;i<100;i++){
    if(digitalRead(MagneticPin) || digitalRead(MagneticPin2))BoxMagnetic=1;
    delay(5);
  }      
      
  //BoxLoaded = 1;
  PickedBoxOffline = 1;
  Serial.println("Finished");
}

bool OfflineFound = 0;

void PickBoxOffLine(){ // called when the offline boxes are sensed
  Serial.println(CurrBox);
  Serial.println("Found Box, turning");
  if(CurrBox == 5){
    for(int i=0;i<30;i++){
      Lspeed = RightBoundaryRead ? 255 : 50;
      Rspeed = LeftBoundaryRead ?255 : 50;
      MotorRun(Lspeed,Rspeed,BACKWARD,FORWARD);
      delay(10);
    }
  }
  MotorRun(Lspeed,Rspeed,RELEASE,RELEASE);
  //delay(500);
  tAngle = (currNode == 5)?2:0;
  bool turnDesPorN = ( (tAngle-direction)>0 ) ? ((tAngle-direction)>2?0:1) : ((tAngle-direction)<-2?1:0);
  reach = 0;
  // leftMotor->setSpeed(255);
  // rightMotor->setSpeed(255);
  Lspeed  = turnDesPorN ? 85:255;
  Rspeed = turnDesPorN ? 255:85;
  while(!RightLineRead){
    LeftBoundaryRead =  digitalRead(LeftLineBoundaryPin);
    RightBoundaryRead = digitalRead(RightLineBoundaryPin);
    LeftLineRead =  digitalRead(LeftLineSensorPin);
    RightLineRead =  digitalRead(RightLineSensorPin);
    // leftMotor->run(turnDesPorN?BACKWARD:RELEASE);
    // rightMotor->run(turnDesPorN?RELEASE:FORWARD);
    Lspeed  = turnDesPorN ? 85:(LeftLineRead?0:255);
    Rspeed = turnDesPorN ? (RightLineRead?0:255):85;
    MotorRun(Lspeed,Rspeed,turnDesPorN?BACKWARD:FORWARD,turnDesPorN?BACKWARD:FORWARD);
  }
  Serial.println("1");
  MotorRun(Lspeed,Rspeed,RELEASE,RELEASE);
  delay(300);
  /* while(!LeftLineRead || !RightLineRead){
     LeftBoundaryRead = digitalRead(LeftLineBoundaryPin);
     RightBoundaryRead = digitalRead(RightLineBoundaryPin);
     LeftLineRead =  digitalRead(LeftLineSensorPin);
     RightLineRead =  digitalRead(RightLineSensorPin);
     Lspeed = LeftLineRead?0:192;
     Rspeed = RightLineRead?0:192;
     // leftMotor->run(turnDesPorN?BACKWARD:RELEASE);
     // rightMotor->run(turnDesPorN?RELEASE:FORWARD);
     MotorRun(Lspeed,Rspeed,FORWARD,BACKWARD);
  }
  Serial.println("2");
  delay(1500); */
  back = 0;
  reach = 0;
  // uses the infrared sensor to find when the box is close enough to pick up
  infraredRead = digitalRead(infraredPin);
  for(int i=0;i < (currNode == 5 ? 45:72) ;i++){
    infraredRead = digitalRead(infraredPin); 
    Lspeed = 255;
    Rspeed = 255;
    // leftMotor->run(back?FORWARD:BACKWARD);
    // rightMotor->run(back?BACKWARD:FORWARD);
    if(!infraredRead)break;
    MotorRun(Lspeed,Rspeed,BACKWARD,FORWARD);
    delay(10);
  }

  MotorRun(Lspeed,Rspeed,RELEASE,RELEASE);
  delay(300);
  int infCounter = 0;
  if(!infraredRead){
    PickBox();
    OfflineFound = 1;
  }
  else{
    
    for(int i=0;i<70;i++){
      infraredRead = digitalRead(infraredPin);
      Lspeed = 180;
      Rspeed = 180;
      // leftMotor->run(back?FORWARD:BACKWARD);
      // rightMotor->run(back?BACKWARD:FORWARD);
      if(!infraredRead)infCounter++;
      if(infCounter>5)break;
      MotorRun(Lspeed,Rspeed,BACKWARD,BACKWARD);
      delay(10);
    }
    MotorRun(Lspeed,Rspeed,RELEASE,RELEASE);
    if(!infraredRead){
      PickBox();
      OfflineFound = 1;
    }
    else{
      infCounter = 0;
      for(int i=0;i<130;i++){
      infraredRead = digitalRead(infraredPin);
      Lspeed = 180;
      Rspeed = 180;
      // leftMotor->run(back?FORWARD:BACKWARD);
      // rightMotor->run(back?BACKWARD:FORWARD);
      if(!infraredRead)infCounter++;
      if(infCounter>5)break;
      MotorRun(Lspeed,Rspeed,FORWARD,FORWARD);
      delay(10);
      }
      MotorRun(Lspeed,Rspeed,RELEASE,RELEASE);
      if(!infraredRead){
        PickBox();
        OfflineFound = 1;
      }
    } 
  }
  delay(500);

  back = 1;
  LeftLineRead =  digitalRead(LeftLineSensorPin);
  RightLineRead =  digitalRead(RightLineSensorPin);
  while(!LeftLineRead || !RightLineRead){ // drives backwards until the line is sensed again
    LeftLineRead =  digitalRead(LeftLineSensorPin);
    RightLineRead =  digitalRead(RightLineSensorPin);
    LeftBoundaryRead = digitalRead(LeftLineBoundaryPin);
    RightBoundaryRead = digitalRead(RightLineBoundaryPin);
    /* Lspeed = (((!back&&!RightBoundaryRead)||(back && LeftBoundaryRead)) )?50:255;
    Rspeed = (((!back&&!LeftBoundaryRead)||(back && RightBoundaryRead)) )?50:255;
    if((LeftBoundaryRead^!back)&&(RightBoundaryRead^!back)){
      Lspeed = Rspeed = 255;
    }
    leftMotor->setSpeed((((!back&&!RightBoundaryRead)||(back && LeftBoundaryRead)) )||(LeftLineRead&&!RightLineRead))?50:255);
    rightMotor->setSpeed((((!back&&!LeftBoundaryRead)||(back && RightBoundaryRead)) )||(RightLineRead&&!LeftLineRead))?50:255);
    Lspeed = ( (!back&&!RightBoundaryRead) || (back && LeftBoundaryRead) || (LeftLineRead&&!RightLineRead) )?50:255;
    Rspeed =( (!back&&!LeftBoundaryRead) || (back && RightBoundaryRead) || (RightLineRead&&!LeftLineRead) )?50:255; */
    Lspeed = (LeftLineRead&&!RightLineRead)?0:255;
    Rspeed = (RightLineRead&&!LeftLineRead)?0:255;
    /* if((LeftBoundaryRead^!back)&&(RightBoundaryRead^!back)){
      // leftMotor->setSpeed(255);
      // rightMotor->setSpeed(255);
      Lspeed = Rspeed = 255;
    }
    // leftMotor->run(back?FORWARD:BACKWARD);
    // rightMotor->run(back?BACKWARD:FORWARD); */
    MotorRun(Lspeed,Rspeed,FORWARD,BACKWARD);
    delay(10);
  }
  MotorRun(Lspeed,Rspeed,RELEASE,RELEASE);
  Serial.println("4");
  /* while(LeftLineRead || RightLineRead){
    LeftLineRead =  digitalRead(LeftLineSensorPin);
    RightLineRead =  digitalRead(RightLineSensorPin);
    LeftBoundaryRead = digitalRead(LeftLineBoundaryPin);
    RightBoundaryRead = digitalRead(RightLineBoundaryPin);
    /* Lspeed = (((!back&&!RightBoundaryRead)||(back && LeftBoundaryRead)) )?50:255;
    Rspeed = (((!back&&!LeftBoundaryRead)||(back && RightBoundaryRead)) )?50:255;
    if((LeftBoundaryRead^!back)&&(RightBoundaryRead^!back)){
      Lspeed = Rspeed = 255;
    }
    leftMotor->setSpeed((((!back&&!RightBoundaryRead)||(back && LeftBoundaryRead)) )||(LeftLineRead&&!RightLineRead))?50:255);
    rightMotor->setSpeed((((!back&&!LeftBoundaryRead)||(back && RightBoundaryRead)) )||(RightLineRead&&!LeftLineRead))?50:255);
    Lspeed = ( (!back&&!RightBoundaryRead) || (back && LeftBoundaryRead) || (LeftLineRead&&!RightLineRead) )?50:255;
    Rspeed =( (!back&&!LeftBoundaryRead) || (back && RightBoundaryRead) || (RightLineRead&&!LeftLineRead) )?50:255; */
    Lspeed = (!LeftLineRead&&RightLineRead)?50:255;
    Rspeed = (!RightLineRead&&LeftLineRead)?50:255;
    /* if((LeftBoundaryRead^!back)&&(RightBoundaryRead^!back)){
      // leftMotor->setSpeed(255);
      // rightMotor->setSpeed(255);
      Lspeed = Rspeed = 255;
    }
    leftMotor->run(back?FORWARD:BACKWARD);
    rightMotor->run(back?BACKWARD:FORWARD); */
    MotorRun(Lspeed,Rspeed,FORWARD,BACKWARD);
    delay(10);
  // question
  // } */
  Serial.println("5");
  MotorRun(Lspeed,Rspeed,RELEASE,RELEASE);
  //delay(500);
  while(!LeftBoundaryRead || !RightBoundaryRead){
    LeftBoundaryRead = digitalRead(LeftLineBoundaryPin);
    RightBoundaryRead = digitalRead(RightLineBoundaryPin);
    // leftMotor->run(turnDesPorN?BACKWARD:RELEASE);
    // rightMotor->run(turnDesPorN?RELEASE:FORWARD);
    MotorRun(Lspeed,Rspeed,!turnDesPorN?BACKWARD:RELEASE,!turnDesPorN?RELEASE:FORWARD);
  }
  MotorRun(Lspeed,Rspeed,RELEASE,RELEASE);
  PickedBoxOffline = 1;
  back = 0;
  Serial.println("picked box offline");
}

/* void blue_flashing() { // makes Blue LED flash at 2Hz
  unsigned long currenttime = millis();
  if (currenttime - previoustime >= interval) { // check if 0.5s has passed
    previoustime = currenttime; // update the last time the LED flashed
    // if (blueState == LOW) {
    //   blueState = HIGH;
    // } else {
    //   blueState = LOW;
    // }
    blueState = !blueState;
    digitalWrite(BLEDPin, blueState);
  }
} */

void setup() {
  Serial.begin(9600);

  // timers for flashing Blue LED
  TCCR2A = 0x02; //setup Timer#2 as CTC
  TCCR2B = B111; //prescaler with clk/n
  OCR2A = 255;
  TIMSK2 = 0x02; //enable OCR2A interrupt
  sei();

  currNode = 0;
  targetNode = DesNodeSeq[TarP];
  nextNode = PathFinding(currNode,targetNode);

  Wire.begin();
  ToFSensor.begin(0x50);
  ToFSensor.setMode(ToFSensor.eContinuous,ToFSensor.eHigh);
  ToFSensor.start();

  pinMode(RLEDPin, OUTPUT); // declare LED as output
  pinMode(GLEDPin, OUTPUT);
  pinMode(BLEDPin, OUTPUT);
  pinMode(infraredPin, INPUT);
  pinMode(MagneticPin, INPUT); // declare mag pin as input
  pinMode(MagneticPin2, INPUT); // declare mag pin as input
  //pinMode(crashswitchPin,INPUT);
  pinMode(RedButtonPin,INPUT);
  pinMode(ButtonPin,INPUT);
  pinMode(LeftLineSensorPin, INPUT);
  pinMode(RightLineSensorPin, INPUT);
  //pinMode(FrontLineSensorPin, INPUT);
  pinMode(LeftLineBoundaryPin, INPUT);
  pinMode(RightLineBoundaryPin, INPUT);

  grabber.attach(grabberPin);
  lifter.attach(lifterPin);
  grabber.write(grabberAngle); // closes grabber so it doesn't collide with buildings
  lifter.write(lifterAngle);
  if(!AFMS.begin()) {         
    if(!AFMS.begin(1000)) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }}
  Serial.println("Motor Shield found.");
}
void(* resetFunc) (void) = 0;

// flashes Blue LED, called when moving
int counter = 0;
ISR(TIMER2_COMPA_vect) {
  counter++;
  if(counter==30) {
  counter=0;
  BlueState = !BlueState;
  }
}

int turncounter = 0;
int NumOfLineToPass = 0;
int NumOfLineCounter = 0;
float ToFReaings[5] = {0,1,2,3,4};
void loop(){
  if(BoxLoaded || PickedBoxOffline){
    if((digitalRead(MagneticPin)||digitalRead(MagneticPin2)))BoxMagnetic = 1;
    if(BoxMagnetic){ // turns on LED to show if box is magnetic
      digitalWrite(RLEDPin, HIGH);
      digitalWrite(GLEDPin, LOW);
    }
    else{
      digitalWrite(RLEDPin, LOW);
      digitalWrite(GLEDPin, HIGH);
    }
  }
  else{ // after box is dropped LED turns off
    digitalWrite(RLEDPin, LOW);
      digitalWrite(GLEDPin, LOW);
  }
  /* while(digitalRead(RedButtonPin)){
    continue;
  } */
  //Serial.println("Entering Loop");
  MagRead = digitalRead(MagneticPin); // read input values
  UltraRead = analogRead(UltrasonicPin);
  UltraDistance = UltraRead * MAX_RANG / ADC_SOLUTION;
  //crashswitchRead = digitalRead(crashswitchPin);
  BoxSensed = infraredRead && !digitalRead(infraredPin);
  infraredRead = digitalRead(infraredPin);
  
  //Serial.println(UltraDistance,0);
  //Serial.println("cm");
  ToFDistance = ToFSensor.getDistance();
  LeftLineRead =  digitalRead(LeftLineSensorPin);
  RightLineRead =  digitalRead(RightLineSensorPin);
  /* FrontLineRead =  digitalRead(FrontLineSensorPin);
  Serial.print("Front: ");
  Serial.println(FrontLineRead);
  Serial.print("Left: ");
  Serial.println(LeftLineRead);
  Serial.print("Right: ");
  Serial.println(RightLineRead); */
  LeftBoundaryRead = digitalRead(LeftLineBoundaryPin);
  RightBoundaryRead = digitalRead(RightLineBoundaryPin);
  //digitalWrite(RLEDPin,HIGH);
  //Serial.println(FrontLineRead);
  int buttonread = digitalRead(ButtonPin);
  //if(digitalRead(RedButtonPin))resetFunc();
  //Serial.println("Entering btnread");

  if(buttonread) start = 1;
  /* if (Serial.available() > 0) {
    Serial.println("Serial input");
    // read the incoming byte:
    int incomingByte = 0;
    incomingByte = Serial.read();
    start = 1;
    if(inputBytePointer<10) {
      inputBytes[inputBytePointer] = incomingByte;
      inputBytePointer++;
      // say what you got:
      Serial.print("I received: ");
      Serial.println(incomingByte, DEC);
    }
  } */
  //nextNode = PathFinding(currNode,targetNode);//curr = current node, targetNode = final destination, next= next node to reach
  //Serial.println(nextNode);

  if(buttonread){
    Serial.println(state);
  }

  //Serial.println("Entering switch");
  if(start && !end)
  switch(state) {
    case 0://moving
      //Serial.println("Entering move");
      NumOfLineCounter =0;
      //tAngle = (direction + (back?2:0))%4;
      tAngle = direction;
      Lspeed = RightBoundaryRead ? 255 : 50;
      Rspeed = LeftBoundaryRead ?255 : 50;
      if(!LeftBoundaryRead && !RightBoundaryRead){
        Lspeed = Rspeed = 255;
      }
      // leftMotor->run(back?FORWARD:BACKWARD);
      // rightMotor->run(back?BACKWARD:FORWARD);
      //Serial.println("Start move");
      MotorRun(Lspeed,Rspeed,BACKWARD,FORWARD);
      /* if((FrontLineRead == (edges[next][(0+tAngle)%4] != -1) && LeftLineRead == (edges[next][(3+tAngle)%4] != -1) && RightLineRead == (edges[next][(1+tAngle)%4] != -1) ) || reach){
        reach = 1;
        if(LeftBoundaryRead && RightBoundaryRead){
          leftMotor->run(RELEASE);
          rightMotor->run(RELEASE);
          currNode = next;
           state = 1;
          reach = 0;
        }  
      } */

      /* if(buttonread){
        Serial.println(nextNode);
        Serial.println(tAngle);
        Serial.println(direction);
        //Serial.print(FrontLineRead);
        Serial.print(LeftLineRead);
        Serial.print(RightLineRead);
        Serial.print(LeftBoundaryRead);
        Serial.print(RightBoundaryRead);
        Serial.print((edges[nextNode][(0+tAngle)%4]));
        Serial.print((edges[nextNode][(3+tAngle)%4]));
        Serial.println((edges[nextNode][(1+tAngle)%4]));
      } */
      
      if(!LeftLineRead && !RightLineRead && !reach)reach = 1;
      if(reach)turncounter++;
      // if traveling from node 5 to 6 look for the box off the line
      if(CurrBox==4 && currNode == 5 && nextNode == 6 && !BoxLoaded && !PickedBoxOffline){
        if(InitTofDistance == 0)InitTofDistance = ToFDistance;
        Serial.print("Variable_1:");
        Serial.println(ToFDistance);
        Serial.print("Init:");
        Serial.println(InitTofDistance);
        if(ToFDistance - InitTofDistance > 100)inSlot = 1;
        if(ToFDistance - InitTofDistance < 40) inSlot = 0;
        if(inSlot && !PickedBoxOffline){
          if(MaxTofDistance<ToFDistance)MaxTofDistance = ToFDistance;
          for(int i=0;i<4;i++)ToFReaings[i]=ToFReaings[i+1];
          ToFReaings[4] = ToFDistance;
          if(ToFReaings[0]>=ToFReaings[1]&&ToFReaings[1]>=ToFReaings[2]&&ToFReaings[2]<=ToFReaings[3]&&ToFReaings[3]<=ToFReaings[4] && MaxTofDistance - ToFReaings[2]>=30 ){
            PickBoxOffLine();
          }
        }
      } 
      
      // if traveling from node 1 to 3 look for the box off the line
      if(CurrBox==5 && currNode == 1 && nextNode == 3 && !BoxLoaded && !PickedBoxOffline){
        Serial.print("Variable_1:");
        Serial.println(ToFDistance);
        Serial.print("Init:");
        Serial.println(InitTofDistance);
        if(InitTofDistance == 0)InitTofDistance = ToFDistance;
        if(ToFDistance - InitTofDistance > 100)inSlot = 1;
        if(ToFDistance - InitTofDistance < 40) inSlot = 0;
        if(inSlot && !PickedBoxOffline){
          if(MaxTofDistance<ToFDistance)MaxTofDistance = ToFDistance;
          /* if(MaxTofDistance-ToFDistance>20){
            PickBoxOffLine();
          } */
          for(int i=0;i<4;i++)ToFReaings[i]=ToFReaings[i+1];
          ToFReaings[4] = ToFDistance;
          if(ToFReaings[0]>=ToFReaings[1]&&ToFReaings[1]>=ToFReaings[2]&&ToFReaings[2]<=ToFReaings[3]&&ToFReaings[3]<=ToFReaings[4] && MaxTofDistance - ToFReaings[2]>=40){
            PickBoxOffLine();
          }
        }
      }
      if(!infraredRead && !BoxLoaded && !PickedBoxOffline){
        // leftMotor->run(RELEASE);
        // rightMotor->run(RELEASE);
        if(BoxAhead(CurrBox,currNode,nextNode)){
          PickBox();
          Serial.println("picked");
        }
        
        //loop();
      }
      if(BoxMagnetic && targetNode == 10){
        targetNode = 11;
      }
      if(LeftBoundaryRead == (edges[nextNode][(0+tAngle)%4] != -1) && RightBoundaryRead== (edges[nextNode][(0+tAngle)%4] != -1) && LeftLineRead == (edges[nextNode][(3+tAngle)%4] != -1) && RightLineRead == (edges[nextNode][(1+tAngle)%4] != -1)  && reach && turncounter>50){
        Serial.print("counter: ");
        Serial.println(turncounter);
        turncounter = 0;
        //delay(100);
        reach = 0;
        InitTofDistance = 0;
        MaxTofDistance = 0;
        // leftMotor->run(RELEASE);
        // rightMotor->run(RELEASE);
        MotorRun(Lspeed,Rspeed,RELEASE,RELEASE);

        // when travelling from node 0 to 1, try to sense the box on the line
        if(currNode == 0 && nextNode == 1 && BoxPos[3]==-1) {
          UltraDistance = 1000;
          BoxPos[3] = UltraDistance<LongEdgeDistance?13:12; // if it is sensed it is on the left otherwise it is on the right
          BoxExists[1][UltraDistance<LongEdgeDistance?4:2] = BoxExists[UltraDistance<LongEdgeDistance?3:2][UltraDistance<LongEdgeDistance?2:4] = 1;
          Serial.println(UltraDistance,0);
          Serial.println("cm");
          Serial.println(UltraDistance<LongEdgeDistance?"box on 13":"box on 12");
        }
        // when travelling from node 11 to 5, try to sense the box on the line
        if(currNode == 11 && nextNode == 5 && BoxPos[2]==-1 ) {
          UltraDistance = 10;
          BoxPos[2] = UltraDistance<LongEdgeDistance?65:45; // if it is sensed it is on the right otherwise it is on the left
          BoxExists[5][UltraDistance<LongEdgeDistance?2:4] = BoxExists[UltraDistance<LongEdgeDistance?6:4][UltraDistance<LongEdgeDistance?4:2] = 1;
          Serial.println(UltraDistance,0);
          Serial.println("cm");
          Serial.println(UltraDistance<LongEdgeDistance?"box on 56":"box on 45");
          }
        /* find box along the line
        // if(currNode == 0 && nextNode == 1 && !BoxExists[1][2] && !BoxExists[1][4] && !BoxDelivered) BoxExists[1][UltraDistance<LongEdgeDistance?4:2] = BoxExists[UltraDistance<LongEdgeDistance?3:2][UltraDistance<LongEdgeDistance?2:4] = 1;
        // if(currNode == 2 && nextNode == 6 && !BoxExists[5][6] && !BoxDelivered) BoxExists[5][UltraDistance<LongEdgeDistance?2:4] = BoxExists[UltraDistance<LongEdgeDistance?6:4][UltraDistance<LongEdgeDistance?4:2] = 1;
        // if(currNode == 3 && nextNode == 4 && !BoxExists[5][6] && !BoxDelivered) BoxExists[5][UltraDistance<LongEdgeDistance?4:2] = BoxExists[UltraDistance<LongEdgeDistance?4:6][UltraDistance<LongEdgeDistance?2:4] = 1;
        */
        currNode = nextNode;
        state = 1;
        nextNode = PathFinding(currNode,targetNode); // find a path to the target node
        Serial.print("reach ");
        Serial.print(currNode);
        Serial.print(", angle: ");
        Serial.println(tAngle);
        Serial.println(PathFinding(currNode,targetNode));
        if(currNode == targetNode) {
          TarP++;
          if(TarP<5 && DesNodeSeq[TarP]!=-1) {
            targetNode = DesNodeSeq[TarP];
            nextNode = PathFinding(currNode,targetNode);
            state = 1;
            Serial.print("new des ");
            Serial.println(targetNode);}
          else { // when all boxes are delivered, return to start
            //state = 3; 
            if(BoxDelivered == 6){
              end = 1;
              MotorRun(255,255,BACKWARD,FORWARD);
              delay(1000);
              MotorRun(255,255,RELEASE,RELEASE);
              break;
            }
            if(BoxLoaded){
              // reach delivery area, drop
              // search for nearest box
              // update targetNode && newTargetNode
              DropBox();
              
              state = 1;
              Serial.print(LeftBoundaryRead);
              Serial.print(RightBoundaryRead);
              Serial.print(LeftLineRead);
              Serial.print(RightLineRead);
              BoxLoaded = 0;
              TarP = 0;
              targetNode = DesNodeSeq[0];
              nextNode = PathFinding(currNode,targetNode);
            }
            else { // if the target node is reached without sensing the desired box
              BoxLoaded = 1;
              TarP = 0;
              targetNode = BoxMagnetic?11:10;
              if(CurrBox>3 && !OfflineFound && false){//not used, no enough time for fixing the  function of skipping boxes
                CurrBox++;
                BoxFinding(CurrBox,currNode);
                BoxLoaded = 0;
                TarP = 0;
                targetNode = DesNodeSeq[0];
                nextNode = PathFinding(currNode,targetNode);
              }
              else{
                UpdateBox(CurrBox);
                DesNodeSeq[TarP+1] = targetNode;
                for(int i=1;i<5;i++)DesNodeSeq[i] = -1;
                nextNode = PathFinding(currNode,targetNode);
                state = 1;
              }
              
            }
            Serial.println("no target, find next");
            Serial.print("new targetNode: ");
            Serial.println(targetNode);
          }
        }
      } 
      //Serial.println("break 0");
      break;
    
    case 1: //rotating
      // calculates the angle needed to turn 
      tAngle = IndexInArray(nextNode,currNode)%4;
      back = 0;
      if(tAngle == direction){state = 0;break;}
      
      bool turnDesPorN = ( (tAngle-direction)>0 ) ? ((tAngle-direction)>2?0:1) : ((tAngle-direction)<-2?1:0);
      NumOfLineToPass = 0;

      bool UTurn = (tAngle-direction+4)%4 == 2;
      
      int tmp=((tAngle-direction)>0 ) ? ((tAngle-direction)>2?1:(tAngle-direction)) : ((tAngle-direction)<-2?1:(direction-tAngle));
      if(buttonread) {
        Serial.println(tAngle);
        Serial.println(direction);
        Serial.println(tmp);
      }
      
      for(int i=1;i<=tmp;i++){
        //Serial.print("dead?");
        if(edges[currNode][(direction+(turnDesPorN?1:-1)*i+4)%4] != -1) NumOfLineToPass++; // calculated how many times the line sensors should sense when turning
      }
      if(buttonread){
        Serial.print((tAngle-direction)>0 ) ? ((tAngle-direction)>2?1:(tAngle-direction)) : ((tAngle-direction)<-2?1:(direction-tAngle));
        Serial.print(NumOfLineToPass);
        Serial.print(": ");
        Serial.println(NumOfLineCounter);
      }
      
      // leftMotor->setSpeed(255);
      // rightMotor->setSpeed(255);
      Lspeed = (!turnDesPorN && !UTurn)?140:255;
      Rspeed = (turnDesPorN && !UTurn)?140:255;
      // leftMotor->run(turnDesPorN?BACKWARD:RELEASE);
      // rightMotor->run(turnDesPorN?RELEASE:FORWARD);
      MotorRun(Lspeed,Rspeed,turnDesPorN?BACKWARD:FORWARD,turnDesPorN?BACKWARD:FORWARD);
      if(buttonread){
        Serial.print(nextNode);
        Serial.println(tAngle);
      }
      // checks when sensors leave the line and how many other lines they cross to find angle rotated
      if(!LeftBoundaryRead && !RightBoundaryRead && !reach){
        reach = 1;
        turncounter = 0;}
      if(reach){
        turncounter++;
        Serial.println(turncounter);
        if(LeftBoundaryRead && RightBoundaryRead && turncounter>30){
          NumOfLineCounter++;
          reach = 0;
          if(NumOfLineToPass == NumOfLineCounter){ // stop after turning to the required line
            MotorRun(Lspeed,Rspeed,RELEASE,RELEASE);
            // leftMotor->run(RELEASE);
            // rightMotor->run(RELEASE);
            direction = tAngle;
            state = 0;
            reach = 0;
            Serial.print("turned, now dir & next: ");
            Serial.print(nextNode);
            Serial.println(direction);
          }
          /* while( LeftBoundaryRead && RightBoundaryRead){
            LeftBoundaryRead = digitalRead(LeftLineBoundaryPin);
            RightBoundaryRead = digitalRead(RightLineBoundaryPin);
            continue;
          } */
          
        }
      }
      //Serial.println("break 1");
      break;

    default:
      Serial.println("state: ????????");
      break;
  }
}