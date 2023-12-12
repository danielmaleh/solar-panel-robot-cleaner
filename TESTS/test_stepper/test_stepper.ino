#include <Stepper.h>

const int stepsPerRevolution = 200; // change this to fit the number of steps per revolution for your motor
Stepper mystep(stepsPerRevolution, 2, 3, 4, 5);
const int irSensorPin = A0; // IR sensor pin, change as necessary
const int stepper_speed = 60; // To fix


void setup() {
  mystep.setSpeed(stepper_speed);
  Serial.begin(9600);
}

void loop() {
	mystep.step(5);
  delay(500);
}