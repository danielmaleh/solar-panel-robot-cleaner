#include <Stepper.h>
#include <AccelStepper.h>
#define MotorInterfaceType 4
//Motor driver pins
//Driver 1 : brush DC motor & gearbox DC motor + PWM, jumpers removed for PWM
//Gearbox motor
const int motorPin1 = 22; 
const int motorPin2 = 23; 
const int motorPin3 = 8; 
const int motorSpeed1 = 255; // (0-255)
//Brush motor
const int motorPin4 = 24; 
const int motorPin5 = 25; 
const int motorPin6 = 9; 
const int motorSpeed2 = 178; // 9V max for brush motor 

//Driver 2 : Stepper motor, jumpers kept in place 
const int motorPin7 = 26; 
const int motorPin8 = 27; 
const int motorPin9 = 28; 
const int motorPin10 = 29; 
const int steps_per_rev = 200;
//Stepper our_stepper(steps_per_rev, motorPin7, motorPin8, motorPin9, motorPin10);
AccelStepper our_stepper = AccelStepper(MotorInterfaceType, motorPin7, motorPin8, motorPin9, motorPin10);

//Current sensors (x3) Pins 
const int currentPin1 = A1; 
const int currentPin2 = A2;
const int currentPin3 = A3;
float currentVal1 = 0;
float currentVal2 = 0;
float currentVal3 = 0;
const float currentSensitivity = 0.185; 

//End switch buttons (x3) Pins
const int ButtonPin1 = A4; 
const int ButtonPin2= A5; 
const int ButtonPin3 = A6; 
float lastButtonVal1 = 0;
float lastButtonVal2 = 0;
float lastButtonVal3 = 0;
const int debounceDelay = 50; // Debounce delay in milliseconds
unsigned long lastDebounceTime1 = 0; // Timestamp to store last debounce time
unsigned long lastDebounceTime2 = 0;
unsigned long lastDebounceTime3 = 0;

void setup() {
  //Set motor pins as outputs
  //Gearbox
  pinMode(motorPin1, OUTPUT); 
  pinMode(motorPin2, OUTPUT); 
  pinMode(motorPin3, OUTPUT); 
  //Brush
  pinMode(motorPin4, OUTPUT); 
  pinMode(motorPin5, OUTPUT); 
  pinMode(motorPin6, OUTPUT); 
  //Stepper
  //our_stepper.setSpeed(60); //Rev per min 
  our_stepper.setMaxSpeed(1000);
  //Set button pins as inputs
  pinMode(ButtonPin1, INPUT);
  pinMode(ButtonPin2, INPUT);
  pinMode(ButtonPin3, INPUT);
  //Set current pins as inputs
  pinMode(currentPin1, INPUT);
  pinMode(currentPin2, INPUT);
  pinMode(currentPin3, INPUT);

  Serial.begin(9600);
}

void loop() {

  //Idea of this test loop : 
  //1) Set stepper motor for a certain time left
  //2) Set gearbox motor down  and brush motor on until bistable switch on 
  //3) Set gearbox motor up until chariot switch on 
  //4) Set stepper motor rigth until end course switch on 

  //Stepper motor control
  //our_stepper.step(steps_per_rev);
  //delay(3000);

  //Stepper position control 
  our_stepper.setCurrentPosition(0);
  // Run the motor forward at 200 steps/second until the motor reaches 400 steps (2 revolutions):
  while (our_stepper.currentPosition() != 400)  {
    our_stepper.setSpeed(200);
    our_stepper.runSpeed();
  }
  our_stepper.setSpeed(0); 
  our_stepper.runSpeed();
  our_stepper.setCurrentPosition(0);
  delay(1000); 
  while (lastButtonVal3 == 0) {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    analogWrite(motorPin3, motorSpeed1);
    digitalWrite(motorPin4, LOW);
    digitalWrite(motorPin5, HIGH);
    analogWrite(motorPin6, motorSpeed2);
    check_switches(); 
  }
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin4, LOW);
  digitalWrite(motorPin5, LOW);
  delay(1000); 
  while (lastButtonVal2 != 0) {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    analogWrite(motorPin3, motorSpeed1);
    check_switches(); 
  }
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  delay(1000); 
  // Getting a current value from current sensor
  currentVal1 = analogRead(currentPin1);
  currentVal1 = (2.5-(currentVal1*(5.0/1024.0)))/currentSensitivity;
  Serial.print("current Value1: "); 
  Serial.print(currentVal1, 5);
  Serial.println(" [A]");

  delay(3000); // Short delay for debounce and sensor stability 
}

void check_switches() {
  float currentButtonVal1 = analogRead(ButtonPin1);
  float currentButtonVal2 = analogRead(ButtonPin2);
  float currentButtonVal3 = analogRead(ButtonPin3);
  if (currentButtonVal1 != lastButtonVal1) {
    lastDebounceTime1 = millis();
  }
  if (currentButtonVal2 != lastButtonVal2) {
    lastDebounceTime2 = millis();
  }
  if (currentButtonVal3 != lastButtonVal3) {
    lastDebounceTime3 = millis();
  }
  if ((millis() - lastDebounceTime1) > debounceDelay) {
    if (currentButtonVal1 != lastButtonVal1) {
      lastButtonVal1 = currentButtonVal1; // released = 1023 and pressed = 0
    }
  }
  if ((millis() - lastDebounceTime2) > debounceDelay) {
    if (currentButtonVal2 != lastButtonVal2) {
      lastButtonVal2 = currentButtonVal2; 
    }
  }
  if ((millis() - lastDebounceTime3) > debounceDelay) {
    if (currentButtonVal3 != lastButtonVal3) {
      lastButtonVal3 = currentButtonVal3; 
    }
  }
}
