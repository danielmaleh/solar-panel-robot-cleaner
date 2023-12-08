// Define pin connections & motor speed
const int motorPin1 = 9; // Motor connected to digital pin 9
const int motorPin2 = 10; // Motor connected to digital pin 10
const int motorPin3 = 11; // Motor connected to digital pin 11
// const int buttonPin = 2; // Button connected to digital pin 2
// const int rainSensorPin = A0; // Raindrop sensor connected to analog pin A0
const int motorSpeed = 200; // Adjust motor speed (0-255)

const int currentPin = A8; // Brush motor pins (IN1, IN2)
const float currentSensitivity = 0.185; 
float currentVal = 0;

void setup() {
  pinMode(motorPin1, OUTPUT); // Set motor pin as output
  pinMode(motorPin2, OUTPUT); // Set motor pin as output
  pinMode(motorPin3, OUTPUT); // Set motor pin as output
  // pinMode(buttonPin, INPUT_PULLUP); // Set button pin as input with internal pull-up

  pinMode(currentPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  // int rainValue = analogRead(rainSensorPin); // Read value from rain sensor
  // //bool buttonState = digitalRead(buttonPin); // Read button state

  // // Print rain sensor value to serial monitor
  // Serial.print("Rain Sensor Value: ");
  // Serial.println(rainValue);

  // Control motor based on button state and rain sensor value
  // if (buttonState == LOW && rainValue < 300) { // Threshold for rainValue can be adjusted
  //   // Turn on motor if button is pressed and it's not raining
  //   analogWrite(motorPin, motorSpeed); 
  // } else {
  //   // Turn off motor otherwise
  //   analogWrite(motorPin, 0);
  // }

  
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

  delay(3000);

}

