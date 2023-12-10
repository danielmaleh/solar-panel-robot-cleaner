//#include <AFMotor.h> // Include Adafruit Motor shield library

AF_DCMotor dcMotor(4); // DC motor on M4
AF_Stepper stepperMotor(200, 2); // Stepper motor (200 steps, motor port #2)
const int irSensorPin = A0; // IR sensor on analog pin A0
const int buttonPin1 = 2; // End of travel button 1 on pin 2
const int buttonPin2 = 3; // End of travel button 2 on pin 3

int irValue = 0; // Variable for IR sensor reading
bool button1Pressed = false; // State of button 1
bool button2Pressed = false; // State of button 2

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud
  dcMotor.setSpeed(200); // Set DC motor speed
  stepperMotor.setSpeed(10); // Set stepper motor speed

  pinMode(buttonPin1, INPUT_PULLUP); // Set button 1 as input with pull-up
  pinMode(buttonPin2, INPUT_PULLUP); // Set button 2 as input with pull-up
}

void loop() {
  irValue = analogRead(irSensorPin); // Read IR sensor
  button1Pressed = digitalRead(buttonPin1) == LOW; // Read button 1 (LOW when pressed)
  button2Pressed = digitalRead(buttonPin2) == LOW; // Read button 2 (LOW when pressed)

  // Control motors based on sensor/button states
  if (irValue > 300 && !button1Pressed && !button2Pressed) {
    // Run motors only if IR sensor is triggered and neither button is pressed
    dcMotor.run(FORWARD);
    stepperMotor.step(100, FORWARD, SINGLE);
  } else {
    // Stop motors if either button is pressed or IR sensor is not triggered
    dcMotor.run(RELEASE);
    stepperMotor.release(); // Release stepper motor (if supported by shield)
  }

  delay(100); // Short delay to debounce buttons and reduce sensor reading noise
}

// unsigned long lastMotorUpdate = 0; // will store last time motors were updated
// const long motorInterval = 100; // interval at which to update motors (milliseconds)

// void loop() {
//   // Read IR sensor - This happens as fast as the loop cycless
//   irValue = analogRead(irSensorPin); // Read IR sensor
//   // Add your immediate IR sensor logic here

//   // Perform motor updates at a slower rate
//   unsigned long currentMillis = millis(); // grab current time
//   if (currentMillis - lastMotorUpdate >= motorInterval) {
//     lastMotorUpdate = currentMillis; // save the last time you updated the motors

//     // Check buttons at this slower rate
//     button1Pressed = digitalRead(buttonPin1) == LOW;
//     button2Pressed = digitalRead(buttonPin2) == LOW;

//     // Motor control logic (modify according to your needs)
//     if (!button1Pressed && !button2Pressed) {
//       // Run motors
//     } else {
//       // Stop motors
//     }
//   }

//   // Other less frequent tasks can go here
// }

