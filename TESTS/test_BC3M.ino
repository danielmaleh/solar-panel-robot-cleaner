#include <def_const_func.h>

void setup() {
  initializeMotors();
  initializeButtonsAndSensors();
  initializeStepperMotor();
  Serial.begin(9600);
}

void loop() {
  // Example usage of the functions
  controlBrushMotor(true, MOTOR_SPEED_BRUSH); // Example values for direction and speed
  controlGearboxMotor(true, MOTOR_SPEED_GEAR); // Example values for direction and speed

  // Additional logic if needed
}