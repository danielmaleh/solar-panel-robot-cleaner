#include <Stepper.h>

Stepper mystep (5,2,3,4,5);
const int buttonPinC1 = 2; // End of travel button 2 on pin 3

void setup() {
  // put your setup code here, to run once:
  mystep.setSpeed(60);
  pinMode(buttonPinC1, INPUT_PULLUP);
  pinMode(A5, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A3, OUTPUT);
  Serial.begin(9600);
  bool buttonState1;
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(stepperMotorPins[0], HIGH);
  digitalWrite(stepperMotorPins[1], LOW);
  digitalWrite(stepperMotorPins[2], HIGH);
  digitalWrite(stepperMotorPins[3], LOW);

  buttonState1 = digitalRead(buttonPin1);

  mystep.step(5);

  float irValue = analogRead(irSensorPin); // Read value from IR sensor
  Serial.print("Button 1 State: "); Serial.println(buttonState1 ? "Released" : "Pressed");

  delay(500);
}
