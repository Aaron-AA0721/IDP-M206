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

int RLEDPin = A4; // the output pin for the Red LED
int GLEDPin = A3; // the output pin for the Green LED
int BLEDPin = A2; // the output pin for the Blue LED

int BlueState = 0; // stores state of Blue LED so it can flash

// // int ServoPin1 = 10; // the pin for the servos
// // int ServoPin2 = 9; 
// int LeftLineSensorPin = 4; //the pin for three line followers
// int RightLineSensorPin = 5; 

int grabberPin = 13; // the pin for the servos
int lifterPin = 12; 
int crashswitchPin = 11;
int MagneticPin = 10; // the input pin for the magenetic sensor

int infraredPin = 9;
int LeftLineSensorPin = 3; //the pin for line followers
int RightLineSensorPin = 4; 
//int FrontLineSensorPin = 9;
int LeftLineBoundaryPin = 5;
int RightLineBoundaryPin = 6;

int Lspeed,Rspeed;

int ButtonPin = 2;

int UltrasonicPin = A0; //the input pin of Ultrasonic Sensor

int edges[12][4] = {{1,-1,-1,-1},{-1,2,0,3},{6,-1,9999,1},{4,1,9999,-1},{7,5,3,-1},{11,6,-1,4},{9,-1,2,5},{-1,8,4,-1},{-1,10,11,7},{-1,-1,6,10},{-1,9,9999,8},{8,-1,5,9999}};
//                   0            1          2           3                 4            5           6          7           8             9             10          11
int distance[12][4] = {{20,-1,-1,-1},{-1,100,20,100},{80,-1,9999,100},{80,100,9999,-1},{80,100,80,-1},{40,100,-1,100},{80,-1,80,100},{-1,100,80,-1},{-1,40,40,100},{-1,-1,80,60},{-1,60,-1,40},{40,-1,40,-1}};
//                       0             1               2               3            4              5               6              7              8                  9             10             11
bool BoxExists[12][5] = {{0,0,0,0,0},{1,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{1,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};
//                        0            1          2         3                 4           5       6           7             8         9         10            11
int BoxPos[6] = {1,5,-1,-1,-1,-1};
//bool BoxCollected[6] = {0,0,0,0,0,0};
int CurrBox = 0;
int DesNodeSeq[5] = {1,-1,-1,-1,-1}; //the sequence of target nodes
int TarP = 0;//target pointer, the pointer(index) of the next target in DesNodeSeq

bool nodeTraveled[12] = {0,0,0,0,0,0,0,0,0,0,0,0};//temp arrays for path finding
int disFromCurr [12] = {0,0,0,0,0,0,0,0,0,0,0,0};
int predecessor[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

bool BoxLoaded = 0;
bool BoxSensed = 0;
int BoxDelivered = 0;

int currNode = 0;
int targetNode = 8;
int nextNode = 1;
int tAngle;

int state = 0;
int direction = 0;
bool back = 0;

const int LongEdgeDistance = 80;

float UltraRead,UltraDistance;
float ToFDistance;
int LeftLineRead,RightLineRead,LeftBoundaryRead,RightBoundaryRead;
int crashswitchRead,infraredRead;
//int FrontLineRead;
int MagRead = 0; // variable for reading the pin status

int grabberAngle = 90;
int lifterAngle = 140;

int inputBytes[10];
int inputBytePointer = 0;
bool reach = 0;
bool start = 0;
bool end = 0;
void MotorRun(int speedL,int speedR,uint8_t modeL,uint8_t modeR){
  leftMotor->setSpeed(speedL);
  rightMotor->setSpeed(speedR);
  leftMotor->run(modeL);
  rightMotor->run(modeR);
  if(modeL == RELEASE && modeR == RELEASE) digitalWrite(BLEDPin, LOW); // not, blue led off
  else digitalWrite(BLEDPin, BlueState?HIGH:LOW); // moving, blue led on
}
int PathFinding(int curr, int des){ // set curr and des, return next node
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
          if(disFromCurr[i]>disFromCurr[curr]+distance[curr][index] && (!BoxLoaded || (!BoxExists[curr][index+1] && !BoxExists[i][0]))){
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
void BoxFinding(int cBox,int cNode){ // set curr box to picked, find des node,might be updated later but with limited boxes it seems easier to use if statements?
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
  tAngle = currNode == 10?2:3;
  bool turnDesPorN = ( (tAngle-direction)>0 ) ? ((tAngle-direction)>2?0:1) : ((tAngle-direction)<-2?1:0);
  reach = 0;
  // leftMotor->setSpeed(255);
  // rightMotor->setSpeed(255);
  Lspeed = Rspeed = 255;
  while(1){
    LeftBoundaryRead = digitalRead(LeftLineBoundaryPin);
    RightBoundaryRead = digitalRead(RightLineBoundaryPin);
    // leftMotor->run(turnDesPorN?BACKWARD:RELEASE);
    // rightMotor->run(turnDesPorN?RELEASE:FORWARD);
    MotorRun(Lspeed,Rspeed,turnDesPorN?BACKWARD:RELEASE,turnDesPorN?RELEASE:FORWARD);
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
  for(int i=0;i<100;i++){
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
  MotorRun(Lspeed,Rspeed,RELEASE,RELEASE);
  lifter.write(140);
  grabber.write(90);
  delay(1000);

  back = 1;
  LeftLineRead =  digitalRead(LeftLineSensorPin);
  RightLineRead =  digitalRead(RightLineSensorPin);
  while(!LeftLineRead || !RightLineRead){
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
    Lspeed = (LeftLineRead&&!RightLineRead)?50:255;
    Rspeed = (RightLineRead&&!LeftLineRead)?50:255;
    // if((LeftBoundaryRead^!back)&&(RightBoundaryRead^!back)){
    //   // leftMotor->setSpeed(255);
    //   // rightMotor->setSpeed(255);
    //   Lspeed = Rspeed = 255;
    // }
    // leftMotor->run(back?FORWARD:BACKWARD);
    // rightMotor->run(back?BACKWARD:FORWARD);
    MotorRun(Lspeed,Rspeed,FORWARD,BACKWARD);
    delay(10);
  }
  //BoxPos[CurrBox] = 0;
  CurrBox++;
  BoxDelivered++;
  TarP = 0;
  BoxFinding(CurrBox,currNode);
}
void PickBox(){
  BoxLoaded = 1;
  //todo
}
// void blue_flashing() { // makes Blue LED flash at 2Hz
//   unsigned long currenttime = millis();
//   if (currenttime - previoustime >= interval) { // check if 0.5s has passed
//     previoustime = currenttime; // update the last time the LED flashed
//     // if (blueState == LOW) {
//     //   blueState = HIGH;
//     // } else {
//     //   blueState = LOW;
//     // }
//     blueState = !blueState;
//     digitalWrite(BLEDPin, blueState);
//   }
// }
void setup() {
  Serial.begin(9600);

  TCCR2A = 0x02; //setup Timer#2 as CTC
  TCCR2B = B111; //prescaler with clk/n
  OCR2A = 255;
  TIMSK2 = 0x02; //enable OCR2A interrupt
  sei();

  currNode = 0;
  targetNode = DesNodeSeq[TarP];

  Wire.begin();
  ToFSensor.begin(0x50);
  ToFSensor.setMode(ToFSensor.eContinuous,ToFSensor.eHigh);
  ToFSensor.start();

  pinMode(RLEDPin, OUTPUT); // declare LED as output
  pinMode(GLEDPin, OUTPUT);
  pinMode(BLEDPin, OUTPUT);
  pinMode(infraredPin, INPUT);
  pinMode(MagneticPin, INPUT); // declare mag pin as input
  pinMode(crashswitchPin,INPUT);

  pinMode(ButtonPin,INPUT);
  pinMode(LeftLineSensorPin, INPUT);
  pinMode(RightLineSensorPin, INPUT);
  //pinMode(FrontLineSensorPin, INPUT);
  pinMode(LeftLineBoundaryPin, INPUT);
  pinMode(RightLineBoundaryPin, INPUT);

  grabber.attach(grabberPin);
  lifter.attach(lifterPin);
  grabber.write(grabberAngle);
  lifter.write(lifterAngle);

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }}
  Serial.println("Motor Shield found.");
}
int counter = 0;
ISR(TIMER2_COMPA_vect) {
  counter++;
  if (counter==30) {
  counter=0;
  BlueState = !BlueState;
  }
}
int NumOfLineToPass = 0;
int NumOfLineCounter = 0;
void loop(){
  MagRead = digitalRead(MagneticPin); // read input value
  UltraRead = analogRead(UltrasonicPin);
  UltraDistance = UltraRead * MAX_RANG / ADC_SOLUTION;
  crashswitchRead = digitalRead(crashswitchPin);
  //BoxSensed = infraredRead && !digitalRead(infraredPin);
  infraredRead = digitalRead(infraredPin);
  
  //Serial.println(UltraDistance,0);
  //Serial.println("cm");
  ToFDistance = ToFSensor.getDistance();
  LeftLineRead =  digitalRead(LeftLineSensorPin);
  RightLineRead =  digitalRead(RightLineSensorPin);
  //FrontLineRead =  digitalRead(FrontLineSensorPin);
  //Serial.print("Front: ");
  //Serial.println(FrontLineRead);
  //Serial.print("Left: ");
  //Serial.println(LeftLineRead);
  //Serial.print("Right: ");
  //Serial.println(RightLineRead);
  LeftBoundaryRead = digitalRead(LeftLineBoundaryPin);
  RightBoundaryRead = digitalRead(RightLineBoundaryPin);
  //digitalWrite(RLEDPin,HIGH);
  //Serial.println(FrontLineRead);
  int buttonread = digitalRead(ButtonPin);
  
  if(buttonread) start = 1;
  if (Serial.available() > 0)
    {
    // read the incoming byte:
    int incomingByte = 0;
    incomingByte = Serial.read();
    start = 1;
    if(inputBytePointer<10) {
      inputBytes[inputBytePointer] = incomingByte;
      inputBytePointer++;
    // say what you got:
      Serial.print("I received: ");
      Serial.println(incomingByte, DEC);}
    }
  nextNode = PathFinding(currNode,targetNode);//curr = current node, targetNode = final destination, next= next node to reach
  if(buttonread){
    Serial.println(state);
  }
  if(start && !end)
  switch(state){
    case 0://moving
      //blue_flashing(); // moving, Blue LED flashes
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
      MotorRun(Lspeed,Rspeed,back?FORWARD:BACKWARD,back?BACKWARD:FORWARD);
      // if((FrontLineRead == (edges[next][(0+tAngle)%4] != -1) && LeftLineRead == (edges[next][(3+tAngle)%4] != -1) && RightLineRead == (edges[next][(1+tAngle)%4] != -1) ) || reach){
      //   reach = 1;
      //   if(LeftBoundaryRead && RightBoundaryRead){
      //     leftMotor->run(RELEASE);
      //     rightMotor->run(RELEASE);
      //     currNode = next;
      //     state = 1;
      //     reach = 0;
      //   }
        
      // }
      if(buttonread){
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
      }
      
      
      if(LeftBoundaryRead == (edges[nextNode][(0+tAngle)%4] != -1) && RightBoundaryRead== (edges[nextNode][(0+tAngle)%4] != -1) && LeftLineRead == (edges[nextNode][(3+tAngle)%4] != -1) && RightLineRead == (edges[nextNode][(1+tAngle)%4] != -1) ){
        //delay(100);
        // leftMotor->run(RELEASE);
        // rightMotor->run(RELEASE);
        MotorRun(Lspeed,Rspeed,RELEASE,RELEASE);
        if(currNode == 0 && nextNode == 1 && BoxPos[2]==-1 && BoxDelivered==0) {
          BoxPos[2] = UltraDistance<LongEdgeDistance?13:12;
          BoxExists[1][UltraDistance<LongEdgeDistance?4:2] = BoxExists[UltraDistance<LongEdgeDistance?3:2][UltraDistance<LongEdgeDistance?2:4] = 1;
          Serial.println(UltraDistance,0);
          Serial.println("cm");
          Serial.println(UltraDistance<LongEdgeDistance?"box on 13":"box on 12");
        }
        if(currNode == 11 && nextNode == 5 && BoxPos[3]==-1 && BoxDelivered==1) {
          BoxPos[3] = UltraDistance<LongEdgeDistance?56:45;
          BoxExists[5][UltraDistance<LongEdgeDistance?2:4] = BoxExists[UltraDistance<LongEdgeDistance?6:4][UltraDistance<LongEdgeDistance?4:2] = 1;
          Serial.println(UltraDistance,0);
          Serial.println("cm");
          Serial.println(UltraDistance<LongEdgeDistance?"box on 56":"box on 45");
          }
        //find box along the line
        // if(currNode == 0 && nextNode == 1 && !BoxExists[1][2] && !BoxExists[1][4] && !BoxDelivered) BoxExists[1][UltraDistance<LongEdgeDistance?4:2] = BoxExists[UltraDistance<LongEdgeDistance?3:2][UltraDistance<LongEdgeDistance?2:4] = 1;
        // if(currNode == 2 && nextNode == 6 && !BoxExists[5][6] && !BoxDelivered) BoxExists[5][UltraDistance<LongEdgeDistance?2:4] = BoxExists[UltraDistance<LongEdgeDistance?6:4][UltraDistance<LongEdgeDistance?4:2] = 1;
        // if(currNode == 3 && nextNode == 4 && !BoxExists[5][6] && !BoxDelivered) BoxExists[5][UltraDistance<LongEdgeDistance?4:2] = BoxExists[UltraDistance<LongEdgeDistance?4:6][UltraDistance<LongEdgeDistance?2:4] = 1;
        currNode = nextNode;
        state = 1;
        Serial.print("reach ");
        Serial.print(currNode);
        Serial.print(", angle: ");
        Serial.println(tAngle);
        Serial.println(PathFinding(currNode,targetNode));
        if(currNode == targetNode) {
          TarP++;
          if(TarP<5 && DesNodeSeq[TarP]!=-1) {
            targetNode = DesNodeSeq[TarP];
            state = 1;
            Serial.print("new des ");
            Serial.println(targetNode);}
          else{
            //state = 3; 
            if(BoxDelivered == 6){
              end = 1;
              MotorRun(255,255,BACKWARD,FORWARD);
              delay(1000);
              MotorRun(255,255,RELEASE,RELEASE);
              break;}
            if(BoxLoaded){
              //reach delivery area, drop
              //search for nearest box
              //update targetNode && newTargetNode
                DropBox();
                state = 1;
                Serial.print(LeftBoundaryRead);
                Serial.print(RightBoundaryRead);
                Serial.print(LeftLineRead);
                Serial.print(RightLineRead);
                BoxLoaded = 0;
                TarP = 0;
                targetNode = DesNodeSeq[0];
              }
              else{ //no box but finished route?
                //do nothing
                BoxLoaded = 1;
                TarP = 0;
                targetNode = 11;
                Serial.print("new targetNode: ");
                Serial.println(targetNode);
                UpdateBox(CurrBox);
                DesNodeSeq[TarP+1] = targetNode;
                for(int i=1;i<5;i++)DesNodeSeq[i] = -1;
                state = 1;
              }
            Serial.println("no target, find next");
          }
          }
        }
      if(BoxSensed && !BoxLoaded){
        // leftMotor->run(RELEASE);
        // rightMotor->run(RELEASE);
        MotorRun(255,255,RELEASE,RELEASE);
        state = 2;
      }
      break;
    case 1://rotating
      //blue_flashing(); // moving, Blue LED flashes
      tAngle = IndexInArray(nextNode,currNode)%4;
      back = 0;
      if(tAngle == direction){state = 0;break;}
      
      bool turnDesPorN = ( (tAngle-direction)>0 ) ? ((tAngle-direction)>2?0:1) : ((tAngle-direction)<-2?1:0);
      NumOfLineToPass = 0;

      
      
      
      int tmp=((tAngle-direction)>0 ) ? ((tAngle-direction)>2?1:(tAngle-direction)) : ((tAngle-direction)<-2?1:(direction-tAngle));
      if(buttonread){
        Serial.println(tAngle);
        Serial.println(direction);
        Serial.println(tmp);
      }
      
      for(int i=1;i<=tmp;i++){
        //Serial.print("dead?");
        if(edges[currNode][(direction+(turnDesPorN?1:-1)*i+4)%4] != -1) NumOfLineToPass++;
      }
      if(buttonread){
        Serial.print((tAngle-direction)>0 ) ? ((tAngle-direction)>2?1:(tAngle-direction)) : ((tAngle-direction)<-2?1:(direction-tAngle));
        Serial.print(NumOfLineToPass);
        Serial.print(": ");
        Serial.println(NumOfLineCounter);
      }
      
      // leftMotor->setSpeed(255);
      // rightMotor->setSpeed(255);
      Lspeed = Rspeed = 255;
      // leftMotor->run(turnDesPorN?BACKWARD:RELEASE);
      // rightMotor->run(turnDesPorN?RELEASE:FORWARD);
      MotorRun(Lspeed,Rspeed,turnDesPorN?BACKWARD:RELEASE,turnDesPorN?RELEASE:FORWARD);
      if(buttonread){
        Serial.print(nextNode);
        Serial.println(tAngle);
      }
      if(!LeftBoundaryRead && !RightBoundaryRead)reach = 1;
      if(reach){
        if(LeftBoundaryRead && RightBoundaryRead){
          NumOfLineCounter++;
          reach = 0;
          if(NumOfLineToPass == NumOfLineCounter){
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
          // while( LeftBoundaryRead && RightBoundaryRead){
          //   LeftBoundaryRead = digitalRead(LeftLineBoundaryPin);
          //   RightBoundaryRead = digitalRead(RightLineBoundaryPin);
          //   continue;
          // }
          
        }
      }
      break;
    case 2://picking
      digitalWrite(BLEDPin, LOW); // not moving, blue led off
      if(!crashswitchRead && !BoxLoaded){
        Lspeed = Rspeed = 100;
        MotorRun(Lspeed,Rspeed,BACKWARD,FORWARD);
        if(grabberAngle<90){
          grabberAngle++;
          grabber.write(grabberAngle);
        }
        else BoxLoaded = 1;
      }
      else{
        BoxLoaded = 1;
        if(lifterAngle >90){
          lifterAngle--;
          lifter.write(lifterAngle);}
        else{
          // targetNode = random(0,2)?10:11;
          // Serial.print("new targetNode: ");
          // Serial.println(targetNode);
          UpdateBox(CurrBox);
          DesNodeSeq[TarP+1] = random(0,2)?10:11;
          for(int i=1;i<5;i++)DesNodeSeq[i] = -1;
          state = 0;
        }
      }
      //do something
      //update target node
      BoxLoaded = 1;
      state = 0;
      break;
    case 3://dropping && redirecting
      digitalWrite(BLEDPin, LOW); // not moving, blue led off
      //do something
      //targetNode = search for closest waste
      Serial.println("hih?");//state 3 not working?
      if(BoxLoaded){
      //reach delivery area, drop
      //search for nearest box
      //update targetNode && newTargetNode
        DropBox();
        state = 1;
        BoxLoaded = 0;
        TarP = 0;
      }
      else{ //no box but finished route?
        //do nothing
        BoxLoaded = 1;
        TarP = 0;
        targetNode = random(0,2)?10:11;
        Serial.print("new targetNode: ");
        Serial.println(targetNode);
        UpdateBox(CurrBox);
        DesNodeSeq[TarP+1] = targetNode;
        for(int i=1;i<5;i++)DesNodeSeq[i] = -1;
        state = 1;
      }
      break;
    default:
      Serial.println("??");
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




/*
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
}*/
