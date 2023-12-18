// Gearbox DC avec capteur courant 
const int motorPin1 = 5; 
const int motorPin2 = 4; 
const int motorPin3 = 12; 
const int motorSpeed = 220; // 130 6V ? 175  9V

// Brush DC avec capteur courant 
const int motorPin4 = 6; 
const int motorPin5 = 7; 


const int ButtonPin = A0; 

const int irSensorPin = A4;

const int currentPingear = A2; 
const int currentPinbrush = A5;
const float currentSensitivity = 0.185; 
float currentVal = 0;

// Define stepper motor connections and steps per revolution:
#define dirPin 2
#define stepPin 3
#define stepsPerRevolution 200

// Define acceleration parameters
const int startSpeed = 1000; // Higher value for slower start speed
const int endSpeed = 700;    // Lower value for faster end speed
const int accelerationSteps = 300; // Number of steps for acceleration and deceleration

void setup() {
  pinMode(motorPin1, OUTPUT); 
  pinMode(motorPin2, OUTPUT); 
  pinMode(motorPin3, OUTPUT); 

  pinMode(motorPin4, OUTPUT); 
  pinMode(motorPin5, OUTPUT); 

  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  pinMode(ButtonPin, INPUT);
  pinMode(irSensorPin, INPUT);
  Serial.begin(9600);
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

  float switch_val = analogRead(ButtonPin);

  float irValue = analogRead(irSensorPin);
  Serial.print("IR Value = ");
  Serial.println(irValue);

  if (irValue >= 450) {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);

    digitalWrite(motorPin4, LOW);
    digitalWrite(motorPin5, HIGH);
    currentVal = analogRead(currentPinbrush);
    currentVal = (2.5-(currentVal*(5.0/1024.0)))/currentSensitivity;
    Serial.print("Current value brush: "); 
    Serial.print(currentVal);
    Serial.println(" [A]");
  }
  else {
    digitalWrite(motorPin4, LOW);
    digitalWrite(motorPin5, LOW);
    if (switch_val != 0) {
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, HIGH);
      analogWrite(motorPin3, motorSpeed);
      currentVal = analogRead(currentPingear);
      currentVal = (2.5-(currentVal*(5.0/1024.0)))/currentSensitivity;
      Serial.print("Current value gear: "); 
      Serial.print(currentVal);
      Serial.println(" [A]");

    }
    else {
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, LOW);
      smoothStepper(5 * stepsPerRevolution, true, startSpeed, endSpeed, accelerationSteps);

      delay(1000);

      // Spin the stepper motor 5 revolutions in the other direction:
      smoothStepper(5 * stepsPerRevolution, false, startSpeed, endSpeed, accelerationSteps);

      delay(1000);
    }
  }
}