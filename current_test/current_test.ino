#include <ACS712.h>

const int currentPin = A1; // Brush motor pins (IN1, IN2)
const int currentSensitivity = 66; 
double currentVal = 0;

void setup() {
  pinMode(currentPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  // We have the valeue in volts, offset of 0.5
  currentVal = analogRead(currentPin);
  currentVal = (currentVal/1024.0);
  currentVal -= 0.5;
  currentVal = currentVal/currentSensitivity;
  Serial.print("current Value: "); 
  Serial.print(currentVal);
  Serial.println(" [mA]");
  delay(1000);
}
