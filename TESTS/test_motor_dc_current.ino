// Define pin connections & motor speed
const int motorPin1 = 9; // Motor connected to digital pin 9
const int motorPin2 = 10; // Motor connected to digital pin 10
const int motorPin3 = 11; // Motor connected to digital pin 11
const int motorSpeed = 200; // Adjust motor speed (0-255)
// PWM pin 
// 178,179 for 9V PWM 

const int currentPin = A8; // Brush motor pins (IN1, IN2)
const float currentSensitivity = 0.185; 
float currentVal = 0;

void setup() {
  pinMode(motorPin1, OUTPUT); // Set motor pin as output
  pinMode(motorPin2, OUTPUT); // Set motor pin as output
  pinMode(motorPin3, OUTPUT); // Set motor pin as output

  pinMode(currentPin, INPUT);
  Serial.begin(9600);
}

void loop() {

  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  analogWrite(motorPin3, motorSpeed);

  // We have the valeue in volts, offset of 0.5
  currentVal = analogRead(currentPin);
  // currentVal = (currentVal/1024.0);
  // currentVal = currentVal - 0.5;
  // currentVal = currentVal/currentSensitivity;
  currentVal = (2.5-(currentVal*(5.0/1024.0)))/currentSensitivity;
  Serial.print("current Value1: "); 
  Serial.print(currentVal, 5);
  Serial.println(" [A]");

  delay(3000); // Short delay for debounce and sensor stability

  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  analogWrite(motorPin3, motorSpeed);

  // We have the valeue in volts, offset of 0.5
  currentVal = analogRead(currentPin);
  // currentVal = (currentVal/1024.0);
  // currentVal -= 0.5;
  // currentVal = currentVal/currentSensitivity;
  currentVal = (2.5-(currentVal*(5.0/1024.0)))/currentSensitivity;
  Serial.print("current Value2: "); 
  Serial.print(currentVal);
  Serial.println(" [A]");

  delay(3000); // Short delay for debounce and sensor stability

}

