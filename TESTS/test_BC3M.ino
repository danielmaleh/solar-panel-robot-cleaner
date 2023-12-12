#include <Stepper.h>
#include <AccelStepper.h>

// Constants and Pin Definitions
#define MotorInterfaceType 4
#define STEPS_PER_REV 200
#define DEBOUNCE_DELAY 50 // Debounce delay in milliseconds
#define CURRENT_SENSITIVITY 0.185 

// Motor Pins and Speeds
const int gearboxMotorPins[] = {22, 23, 8}; // Motor Pin1, Pin2, SpeedPin
const int brushMotorPins[] = {24, 25, 9}; // Motor Pin4, Pin5, SpeedPin
const int stepperMotorPins[] = {26, 27, 28, 29}; // Motor Pin7, Pin8, Pin9, Pin10
const int motorSpeeds[] = {255, 178}; // Gearbox motor speed, Brush motor speed

// Button Pins
const int buttonPins[] = {A4, A5, A6};

// Current Sensor Pins
const int currentPins[] = {A1, A2};

// Global Variables for Button States and Debounce Times
float lastButtonVals[3] = {0};
unsigned long lastDebounceTimes[3] = {0};

// Stepper Motor Setup
AccelStepper stepperMotor = AccelStepper(MotorInterfaceType, stepperMotorPins[0], stepperMotorPins[1], stepperMotorPins[2], stepperMotorPins[3]);

void setup() {
  // Initialize Motor Pins
  for (int i = 0; i < 3; i++) {
    pinMode(gearboxMotorPins[i], OUTPUT);
    pinMode(brushMotorPins[i], OUTPUT);
  }
  stepperMotor.setMaxSpeed(1000);

  // Initialize Button and Current Sensor Pins
  for (int i = 0; i < 3; i++) {
    pinMode(buttonPins[i], INPUT);
    if (i < 2) pinMode(currentPins[i], INPUT); // Only two current sensor pins
  }

  Serial.begin(9600);
}

void loop() {
  // Example usage of the functions
  controlStepperMotor(400, 200); // Example values for position and speed
  controlBrushMotor(true, motorSpeeds[1]); // Example values for direction and speed
  controlGearboxMotor(true, motorSpeeds[0]); // Example values for direction and speed

  // Additional logic if needed
}

void checkSwitches() {
  for (int i = 0; i < 3; i++) {
    float currentVal = analogRead(buttonPins[i]);
    if (currentVal != lastButtonVals[i]) {
      lastDebounceTimes[i] = millis();
      if ((millis() - lastDebounceTimes[i]) > DEBOUNCE_DELAY) {
        lastButtonVals[i] = currentVal;
      }
    }
  }
}

void checkCurrent(int currentPin) {
  float currentVal = analogRead(currentPin);
  currentVal = (2.5 - (currentVal * (5.0 / 1024.0))) / CURRENT_SENSITIVITY;
  Serial.print("Current Value: ");
  Serial.print(currentVal, 5);
  Serial.println(" [A]");
}

// Function to control the stepper motor
void controlStepperMotor(int targetPosition, int speed) {
  stepperMotor.moveTo(targetPosition);
  stepperMotor.setSpeed(speed);
  while (stepperMotor.distanceToGo() != 0) {
    stepperMotor.runSpeed();
  }
}

// Function to control the brush DC motor
void controlBrushMotor(bool direction, int speed) {
  digitalWrite(brushMotorPins[0], direction ? HIGH : LOW);
  digitalWrite(brushMotorPins[1], direction ? LOW : HIGH);
  analogWrite(brushMotorPins[2], speed);
}

// Function to control the gearbox DC motor
void controlGearboxMotor(bool direction, int speed) {
  digitalWrite(gearboxMotorPins[0], direction ? HIGH : LOW);
  digitalWrite(gearboxMotorPins[1], direction ? LOW : HIGH);
  analogWrite(gearboxMotorPins[2], speed);
}