float UltraRead,UltraDistance;
int UltrasonicPin = A0;
#define MAX_RANG (520)
#define ADC_SOLUTION (1023.0)
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  UltraRead = analogRead(UltrasonicPin);
  UltraDistance = UltraRead * MAX_RANG / ADC_SOLUTION;
  Serial.print(UltraDistance,0);
  Serial.println("cm");
  delay(100);
}
