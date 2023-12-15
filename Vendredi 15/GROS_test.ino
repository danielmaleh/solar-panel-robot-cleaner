// Define stepper motor connections and steps per revolution:
#define dirPin 2
#define stepPin 3
#define stepsPerRevolution 200

// LED
const int ledred = 8; 
const int ledgreen = 9; 
const int ledblue = 11; 

// IR sensor
const int irSensorPin = A3; 

// DC Brush motor pin definition 
const int motorPin1 = 7; 
const int motorPin2 = 6; 
const int motorPin3 = 10; 
const int motorSpeed1 = 140; // 130 6V ? 175  9V

// Gearbox motor pin definition 
const int motorPin4 = 5; 
const int motorPin5 = 4; 
const int motorPin6 = 12; 
const int motorSpeed2 = 200;

const int ButtonPin1 = A0; 
const int ButtonPin2 = A1; 
const int ButtonPin3 = A2; 

void setup() {
  // Stepper Driver outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  // DC Brush motor driver output
  pinMode(motorPin1, OUTPUT); 
  pinMode(motorPin2, OUTPUT); 
  pinMode(motorPin3, OUTPUT); 

  // Gearbox motor driver output
  pinMode(motorPin4, OUTPUT); 
  pinMode(motorPin5, OUTPUT); 
  pinMode(motorPin6, OUTPUT); 

  // End switch
  pinMode(ButtonPin1, INPUT);
  pinMode(ButtonPin2, INPUT);
  pinMode(ButtonPin3, INPUT);

  //LED 
  pinMode(ledred, OUTPUT);
  pinMode(ledgreen, OUTPUT);

  Serial.begin(9600);
}

void loop() {

  float irValue = analogRead(irSensorPin);
  Serial.println("IR Sensor Value = ");
  Serial.println(irValue);

  digitalWrite(dirPin, HIGH);

  // Spin the stepper motor 5 revolutions fast:
  for (int i = 0; i < 5 * stepsPerRevolution; i++) {
    setColor(255, 0, 0);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);// shorter delay = higher speed 
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
  setColor(0, 0, 0);
  digitalWrite(stepPin, LOW);

  Serial.println("IR Sensor Value = ");
  Serial.println(irValue);

  float switch_val = analogRead(ButtonPin1);

  while (switch_val != 0) {
    setColor(0, 255, 0);
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    analogWrite(motorPin3, motorSpeed1);
    digitalWrite(motorPin4, LOW);
    digitalWrite(motorPin5, HIGH);
    analogWrite(motorPin6, motorSpeed2);
    switch_val = analogRead(ButtonPin1);
  }
  setColor(0, 0, 0);
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin4, LOW);
  digitalWrite(motorPin5, LOW);

  Serial.println("IR Sensor Value = ");
  Serial.println(irValue);
  
  switch_val = analogRead(ButtonPin2);

  while (switch_val != 0) {
    setColor(0, 0, 255);
    digitalWrite(motorPin4, HIGH);
    digitalWrite(motorPin5, LOW);
    analogWrite(motorPin6, motorSpeed2);
    switch_val = analogRead(ButtonPin2);
  }
  setColor(0, 0, 0);
  digitalWrite(motorPin4, LOW);
  digitalWrite(motorPin5, LOW);

  Serial.println("IR Sensor Value = ");
  Serial.println(irValue);

  // Set the spinning direction counterclockwise:
  digitalWrite(dirPin, LOW);
  switch_val = analogRead(ButtonPin3);
  while (switch_val != 0) {
    setColor(255, 0, 0);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
    switch_val = analogRead(ButtonPin3);
  }
  setColor(0, 0, 0);
  digitalWrite(stepPin, LOW);
}

void setColor(int red, int green, int blue) {
  analogWrite(ledred, red);
  analogWrite(ledgreen, green);
  analogWrite(ledblue, blue); 
}