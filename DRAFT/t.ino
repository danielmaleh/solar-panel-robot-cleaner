#include <Stepper.h>

const int stepsPerRevolution = 200; // change this to fit the number of steps per revolution for your motor
Stepper mystep(stepsPerRevolution, 2, 3, 4, 5);
const int buttonPinC1 = 2; // End of travel button on pin 2
const int irSensorPin = A0; // IR sensor pin, change as necessary

void setup() {
  mystep.setSpeed(60);
  pinMode(buttonPinC1, INPUT_PULLUP);
  Serial.begin(9600);
}

void loop() {
  int buttonState1 = digitalRead(buttonPinC1);

  // Move the stepper motor only if the button is not pressed
  if (buttonState1 == HIGH) {
    mystep.step(5);
  }

  float irValue = analogRead(irSensorPin); // Read value from IR sensor
  Serial.print("Button State: ");
  Serial.println(buttonState1 == HIGH ? "Released" : "Pressed");

  delay(500);
}
