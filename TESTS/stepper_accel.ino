// Define stepper motor connections and steps per revolution:
#define dirPin 2
#define stepPin 3
#define stepsPerRevolution 200

// Define acceleration parameters
const int startSpeed = 1000; // Higher value for slower start speed
const int endSpeed = 700;    // Lower value for faster end speed
const int accelerationSteps = 300; // Number of steps for acceleration and deceleration

void setup() {
  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
}

void smoothStepper(int steps, bool direction, int startSpeed, int endSpeed, int accelerationSteps) {
  digitalWrite(dirPin, direction ? HIGH : LOW);
  int stepDelay = startSpeed;
  int stepChange = (startSpeed - endSpeed) / accelerationSteps;

  // Accelerate
  for (int i = 0; i < accelerationSteps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
    stepDelay -= stepChange;
  }

  // Constant speed
  for (int i = accelerationSteps; i < steps - accelerationSteps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(endSpeed);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(endSpeed);
  }

  // Decelerate
  for (int i = steps - accelerationSteps; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
    stepDelay += stepChange;
  }
}

void loop() {
  // Spin the stepper motor 5 revolutions with acceleration/deceleration:
  smoothStepper(5 * stepsPerRevolution, true, startSpeed, endSpeed, accelerationSteps);

  delay(1000);

  // Spin the stepper motor 5 revolutions in the other direction:
  smoothStepper(5 * stepsPerRevolution, false, startSpeed, endSpeed, accelerationSteps);

  delay(1000);
}
