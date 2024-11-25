#include "DFRobot_VL53L0X.h"
float UltraRead,UltraDistance;
int UltrasonicPin = A0;
DFRobot_VL53L0X ToFSensor;
float ToFDistance;
#define MAX_RANG (520)
#define ADC_SOLUTION (1023.0)
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  ToFSensor.begin(0x50);
  ToFSensor.setMode(ToFSensor.eContinuous,ToFSensor.eHigh);
  ToFSensor.start();
}

void loop() {
  // put your main code here, to run repeatedly:
  UltraRead = analogRead(UltrasonicPin);
  UltraDistance = UltraRead *10* MAX_RANG / ADC_SOLUTION;
  // Serial.print(UltraDistance,0);
  // Serial.println(" U cm");
  ToFDistance = ToFSensor.getDistance();
  // Serial.println(ToFDistance);
  Serial.print("Variable_1:");
  Serial.print(UltraDistance);
  Serial.print(",");
  Serial.print("Variable_2:");
  Serial.println(ToFDistance);

  delay(100);
}
