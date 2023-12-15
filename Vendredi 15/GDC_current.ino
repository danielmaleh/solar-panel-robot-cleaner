// Gearbox DC avec capteur courant 
const int motorPin1 = 5; 
const int motorPin2 = 4; 
const int motorPin3 = 12; 
const int motorSpeed = 220; // 130 6V ? 175  9V

const int ButtonPin = A0; 

const int currentPin = A2; 
const float currentSensitivity = 0.185; 
float currentVal = 0;

void setup() {
  pinMode(motorPin1, OUTPUT); 
  pinMode(motorPin2, OUTPUT); 
  pinMode(motorPin3, OUTPUT); 

  pinMode(ButtonPin, INPUT);
  Serial.begin(9600);
}

void loop() {

  float switch_val = analogRead(ButtonPin);

  if (switch_val == 0) {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    analogWrite(motorPin3, motorSpeed);
    currentVal = analogRead(currentPin);
    currentVal = (2.5-(currentVal*(5.0/1024.0)))/currentSensitivity;
    Serial.print("Current value: "); 
    Serial.print(currentVal);
    Serial.println(" [A]");

  }
  else {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
  }
}