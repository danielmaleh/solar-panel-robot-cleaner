#include <Stepper.h>
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
Stepper our_stepper(steps_per_rev, motorPin7, motorPin8, motorPin9, motorPin10);

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
unsigned long lastDebounceTime = 0; // Timestamp to store last debounce time

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
  our_stepper.setSpeed(60); //Rev per min 
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

  // Command of a DC motor with PWM
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  analogWrite(motorPin3, motorSpeed);

  //Stepper motor control
  our_stepper.step(steps_per_rev);
  delay(500);
  our_stepper.step(-steps_per_rev);
  delay(500);

  // Getting a current value from current sensor
  currentVal1 = analogRead(currentPin1);
  currentVal1 = (2.5-(currentVal1*(5.0/1024.0)))/currentSensitivity;
  Serial.print("current Value1: "); 
  Serial.print(currentVal1, 5);
  Serial.println(" [A]");

  delay(3000); // Short delay for debounce and sensor stability

  // Reading an end switch state 
  float currentButtonVal = analogRead(ButtonPin1);
  if (currentButtonVal != lastButtonVal1) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (currentButtonVal != lastButtonVal1) {
      lastButtonVal1 = currentButtonVal;
      Serial.print("Button value: "); 
      Serial.println(lastButtonVal1, 5); // released = 1023 and pressed = 0
    }
  }
  delay(100); 
}