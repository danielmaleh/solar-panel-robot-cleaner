#include <NewPing.h> // Include NewPing library for ultrasonic sensor

// Motor Definitions
AF_DCMotor dcMotor1(4); // DC motor 1 on M4
AF_DCMotor dcMotor2(3); // DC motor 2 on M3
AF_Stepper stepperMotor(200, 2); // Stepper motor (200 steps, motor port #2)

// Pin Definitions
const int irSensorPin = A0; // IR sensor on analog pin A0
const int buttonPin1 = 2; // End of travel button 1 on pin 2
const int buttonPin2 = 3; // End of travel button 2 on pin 3
const int rainSensorPin = 4; // Raindrop sensor on digital pin 4
const int ultrasonicTrigPin = 6; // Ultrasonic sensor Trig pin
const int ultrasonicEchoPin = 7; // Ultrasonic sensor Echo pin
const int valveControlPin = 8; // Electronic valve control pin
const int ledPins[3] = {9, 10, 11}; // RGB LED pins (R, G, B)

// Ultrasonic Sensor Setup
NewPing sonar(ultrasonicTrigPin, ultrasonicEchoPin, 200); // NewPing object (max distance 200 cm)

// Debouncing Variables
unsigned long lastDebounceTime1 = 0, lastDebounceTime2 = 0; // Last debounce time for buttons
const long debounceDelay = 50; // Debounce time in milliseconds
bool lastButtonState1 = HIGH, lastButtonState2 = HIGH; // Previous state of buttons
bool buttonState1 = HIGH, buttonState2 = HIGH; // Current state of buttons

void setup() {
    // Motor Setup
    dcMotor1.setSpeed(200); // Set DC motor 1 speed
    dcMotor2.setSpeed(200); // Set DC motor 2 speed
    stepperMotor.setSpeed(10); // Set stepper motor speed

    // Pin Modes
    pinMode(buttonPin1, INPUT_PULLUP);
    pinMode(buttonPin2, INPUT_PULLUP);
    pinMode(rainSensorPin, INPUT);
    pinMode(valveControlPin, OUTPUT);
    for (int i = 0; i < 3; i++) pinMode(ledPins[i], OUTPUT);

    Serial.begin(9600); // Start serial communication
}

void loop() {
    // Read sensors
    int irValue = analogRead(irSensorPin); // Read IR sensor
    unsigned int distance = sonar.ping_cm(); // Read distance from ultrasonic sensor
    bool isRaining = digitalRead(rainSensorPin); // Read raindrop sensor

    // Debounce button 1
    bool reading1 = digitalRead(buttonPin1);
    if (reading1 != lastButtonState1) {
      lastDebounceTime1 = millis();
    }
    if ((millis() - lastDebounceTime1) > debounceDelay) {
      buttonState1 = reading1;
    }
    lastButtonState1 = reading1;

    // Debounce button 2
    bool reading2 = digitalRead(buttonPin2);
    if (reading2 != lastButtonState2) {
      lastDebounceTime2 = millis();
    }
    if ((millis() - lastDebounceTime2) > debounceDelay) {
      buttonState2 = reading2;
    }
    lastButtonState2 = reading2;

    // Motor and stepper control logic
    if (!isRaining && !buttonState1 && !buttonState2) {
      if (irValue > 300) { // Example threshold, adjust as needed
        dcMotor1.run(FORWARD);
        dcMotor2.run(FORWARD);
        stepperMotor.step(100, FORWARD, SINGLE);
      } else {
        dcMotor1.run(RELEASE);
        dcMotor2.run(RELEASE);
        stepperMotor.release(); // If supported by shield
      }
    } else {
      dcMotor1.run(RELEASE);
      dcMotor2.run(RELEASE);
      stepperMotor.release();
    }

    // Valve control logic based on ultrasonic distance
    digitalWrite(valveControlPin, distance < 10 ? HIGH : LOW); // Example condition

    // RGB LED control logic (example: turn red if raining)
    analogWrite(ledPins[0], isRaining ? 255 : 0); // Red
    analogWrite(ledPins[1], 0); // Green off
    analogWrite(ledPins[2], 0); // Blue off

    // Serial monitoring (optional)
    Serial.print("IR Sensor Value: "); Serial.println(irValue);
    Serial.print("Rain Sensor Status: "); Serial.println(isRaining ? "Raining" : "Not Raining");
    Serial.print("Ultrasonic Distance: "); Serial.println(distance);
    Serial.print("Button 1 State: "); Serial.println(buttonState1 ? "Released" : "Pressed");
    Serial.print("Button 2 State: "); Serial.println(buttonState2 ? "Released" : "Pressed");
}
