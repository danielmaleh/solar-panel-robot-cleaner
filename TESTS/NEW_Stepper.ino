//STEPPER - 20cm en 5 rev = 4 cm par rev.

// Define stepper motor connections and steps per revolution:
#define dirPin 2
#define stepPin 3
#define stepsPerRevolution 200

void setup() {
  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
}

void loop() {
  // Example usage of spinStepper function
  spinStepper(5, true, 500); // Spin 5 revolutions clockwise
  delay(1000);
  spinStepper(5, false, 500); // Spin 5 revolutions counterclockwise
  delay(1000);
}

// Function to control the stepper motor
void spinStepper(int revolutions, bool clockwise, int delayTime) {
  digitalWrite(dirPin, clockwise ? HIGH : LOW);

  for (int i = 0; i < revolutions * stepsPerRevolution; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delayTime);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delayTime);
  }
}
