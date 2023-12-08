#include <Time.h> // Include Time library for time keeping

// Constants
const int cleaning_width = 30; // Define the effective cleaning width in cm
const unsigned long cleaning_period = 0; // Define the time between two cleanings in milliseconds
const int dist = 0; // Define the total width of all panels

float stepper_freq = 1/100; // Frequency of the stepper motor
float dc_brush_freq = 1/100; // Frequency of the brush motor
float dc_motor_freq = 1/100; // Frequency of the DC motor
float valve_freq = 1/1000; // Frequency of the valve
float rain_detector_freq = 1/300; // Frequency of the rain detector
float button_freq = 1/100; // Frequency of the button
float led_freq = 1/100; // Frequency of the LED
float ir_freq = 1; // Frequency of the IR sensor

// Pin Definitions
const int irSensorPin = A1; // IR sensor on analog pin A0
const int buttonPinR = A2; // End of travel button 1 on pin 2
const int buttonPinC1 = 2; // End of travel button 2 on pin 3
const int buttonPinC2 = 3; // End of travel button 2 on pin 3
const int rainSensorPin = A0; // Raindrop sensor on digital pin 4
const int valveControlPin = 1; // Electronic valve control pin
const int ledPins[3] = {A3, A4, A5}; // RGB LED pins (R, G, B)
const int stepperMotorPins[4] = {2, 3, 4, 5}; // Stepper motor pins (IN1, IN2, IN3, IN4)
const int dcMotorPins[3] = {5, 4, 7}; // DC motor pins (M1, M2, M3, M4)
const int brushMotorPins[3] = {6, 8, 9}; // Brush motor pins (IN1, IN2)

// Add other sensor and motor pins as per the schematic

// Debouncing Variables
unsigned long lastDebounceTime1 = 0, lastDebounceTime2 = 0; // Last debounce time for buttons
const long debounceDelay = 50; // Debounce time in milliseconds
bool lastButtonState1 = HIGH, lastButtonState2 = HIGH; // Previous state of buttons
bool buttonState1 = HIGH, buttonState2 = HIGH; // Current state of buttons

// Global variables
int step = dist / cleaning_width + (dist % cleaning_width != 0); // Calculate the step width
bool raining; // Boolean indicating if it is raining
unsigned long time; // Actual time
unsigned long cleaning_time; // Time of the last cleaning

// Enumeration for states
enum State {REST, WATER_FILLING, INITIAL_POSITION, FIRST_POSITION_CLEANING, CLEANING, UP_TRAVEL, TRANSLATION, RETURN_HOME, PROBLEM};
State STATE = REST;

// Setup and initialization
void setup() {

    // Pin Modes
    pinMode(buttonPinR, INPUT_PULLUP);
    pinMode(buttonPinC1, INPUT_PULLUP);
    pinMode(buttonPinC2, INPUT_PULLUP);
    pinMode(rainSensorPin, INPUT);
    pinMode(valveControlPin, OUTPUT);
    pinMode(irSensorPin, INPUT);
    for (int i = 0; i < 3; i++) pinMode(ledPins[i], OUTPUT);
    for (int i = 0; i < 4; i++) pinMode(stepperMotorPins[i], OUTPUT);
    for (int i = 0; i < 3; i++) pinMode(dcMotorPins[i], OUTPUT);
    for (int i = 0; i < 3; i++) pinMode(brushMotorPins[i], OUTPUT);

    Serial.begin(9600); // Start serial communication
}

// Main loop
void loop() {
    // Update the current time
    time = millis();

    // Check the states and perform actions based on the current state
    switch (STATE) {
        case REST:
            updateLEDs();
            checkAndDebounceButtonsR();
            checkAndDebounceButtonsC1();
            checkAndDebounceButtonsC2();
            checkRainDetector();
            if ((time >= cleaning_time + cleaning_period) || (raining && time - cleaning_time > cleaning_period / 2)) {
                STATE = INITIAL_POSITION;
                cleaning_time = time; // Update the last cleaning time
            }
        break;
        case WATER_FILLING:
            // if buttonC1=0 move upward with motor dc until buttonC1=1
            // if buttonC2=0 move sidways with motor stepper until buttonC2=1
            // if buttonC1=1 and buttonC2=1 fill water reservoir 
            // after filling done state = Initial_position
            if (buttonStateC1 == 0) {
                // move upward with motor dc until buttonC1=1
                // if buttonC2=0 move sidways with motor stepper until buttonC2=1
                // if buttonC1=1 and buttonC2=1 fill water reservoir 
            }
            else if (buttonStateC2 == 0) {
                // move sidways with motor stepper until buttonC2=1
                // if buttonC1=1 and buttonC2=1 fill water reservoir 
            }
            else if (buttonStateC1 == 1 && buttonStateC2 == 1) {
                // fill water reservoir 
            }
            else {
                // error
            }
            updateLEDs();
            checkAndDebounceButtonsR();
            checkAndDebounceButtonsC1();
            checkAndDebounceButtonsC2();
        break;
        case INITIAL_POSITION:
            // Logic to move to the initial position
            updateLEDs();
            checkAndDebounceButtonsC1();
            checkAndDebounceButtonsC2();
            // steps to 1st position and when finished state = FIRST_POSITION_CLEANING
            // either stay in function for whole time either do the loop.
            controlStepperMotor();
        break;
        case CLEANING:
            // Logic to clean the panels
            // go downward with dc motor until buttonR=1 
            // activate dc brush motor and valve 
            // when buttonR=1 stop dc motor and dc brush motor and valve and state = UP_TRAVELL
            updateLEDs();
            checkAndDebounceButtonsR();
            controlValve();
            controlDCMotor();
            controlBrushMotor();
        break;
        case UP_TRAVEL:
            // Logic to move up to the next position
            // go upward with dc motor until IR sensor detects the robot
            // nb_cycles = nb_cycles + 1
            // when IR sensor detects the robot stop dc motor 
            // if nb of cycles = N state = RETURN_HOME else state = TRANSLATION
            updateLEDs();
            checkAndDebounceButtonsC1();
            controlDCMotor();
            check_IR();
        break;
        case TRANSLATION:
            // Logic to translate to the next position
            // go sidways with stepper motor for step
            // when stepper motor finished move upwards with dc motor until buttonC1=1 and then state = CLEANING
            updateLEDs();
            checkAndDebounceButtonsC1();
            controlDCMotor();
            controlStepperMotor();
        break;
        case RETURN_HOME:
            // Logic to return to the initial position
            // go sidways back with stepper motor until buttonC2=1 
            // when buttonC2=1 move upwards with dc motor until buttonC1=1 and then state = REST
            updateLEDs();
            checkAndDebounceButtonsC1();
            checkAndDebounceButtonsC2();
            controlStepperMotor();
        break;
        case PROBLEM:
            // Logic to handle problems TO DO
            updateLEDs();
            controlDCMotor();
            controlBrushMotor();
            controlStepperMotor();
        break;
    }

    // Serial monitoring (optional)
    Serial.print("IR Sensor Value: "); Serial.println(irValue);
    Serial.print("Rain Sensor Status: "); Serial.println(isRaining ? "Raining" : "Not Raining");
    Serial.print("Button 1 State: "); Serial.println(buttonState1 ? "Released" : "Pressed");
    Serial.print("Button 2 State: "); Serial.println(buttonState2 ? "Released" : "Pressed");
}

// Define the sub-loops (functions) here based on the pseudocode structure
void updateLEDs() {
    static unsigned long last_time = 0;
    if (1/time-last_time >= led_freq) {
        // RGB LED control logic (example: turn red if raining)
        analogWrite(ledPins[0], isRaining ? 255 : 0); // Red
        analogWrite(ledPins[1], 0); // Green off
        analogWrite(ledPins[2], 0); // Blue off
        last_time = time;
    }
}

bool checkAndDebounceButtonsR() {
    static unsigned long last_time = 0;
    if (1/time-last_time >= button_freq) {
        // Debounce button 1
        bool reading1 = digitalRead(buttonPin1);
        if (reading1 != lastButtonState1) {
            lastDebounceTime1 = millis();
        }
        if ((millis() - lastDebounceTime1) > debounceDelay) {
            buttonState1 = reading1;
        }
        lastButtonState1 = reading1;
        return buttonState1;
    }
}

bool checkAndDebounceButtonsC1() {
    static unsigned long last_time = 0;
    if (1/time-last_time >= button_freq) {
        // Debounce button 1
        bool reading1 = digitalRead(buttonPin1);
        if (reading1 != lastButtonState1) {
            lastDebounceTime1 = millis();
        }
        if ((millis() - lastDebounceTime1) > debounceDelay) {
            buttonState1 = reading1;
        }
        lastButtonState1 = reading1;
        return buttonState1;
    }
}

bool checkAndDebounceButtonsC2() {
    static unsigned long last_time = 0;
    if (1/time-last_time >= button_freq) {
        // Debounce button 1
        bool reading1 = digitalRead(buttonPin1);
        if (reading1 != lastButtonState1) {
            lastDebounceTime1 = millis();
        }
        if ((millis() - lastDebounceTime1) > debounceDelay) {
            buttonState1 = reading1;
        }
        lastButtonState1 = reading1;
        return buttonState1;
    }
}

int checkRainDetector() {
    static unsigned long last_time = 0;
    if (1/time-last_time >= rain_detector_freq) {
        // Rain detector logic
        int rainValue = analogRead(rainSensorPin); // Read value from rain sensor
        last_time = time;
        return rainValue;
    }
}

void controlValve() {
    static unsigned long last_time = 0;
    if (1/time-last_time >= valve_freq) {
        // Valve control logic with opening and closing times logic
        digitalWrite(valveControlPin, distance < 10 ? HIGH : LOW); // Example condition
        last_time = time;
    }
}

void controlDCMotor(bool dir, int speed) {
    static unsigned long last_time = 0;
    if (1/time-last_time >= dc_motor_freq) {
        // DC motor logic add speed to pwm command
        if (dir) {
            digitalWrite(dcMotorPins[0], HIGH);
            digitalWrite(dcMotorPins[1], LOW);
            analogWrite(dcMotorPins[2], speed);
        } else {
            digitalWrite(dcMotorPins[0], LOW);
            digitalWrite(dcMotorPins[1], HIGH);
            analogWrite(dcMotorPins[2], speed);
        }
        last_time = time;
    }
}

void controlBrushMotor(bool dir, int speed) {
    static unsigned long last_time = 0;
    if (1/time-last_time >= dc_brush_freq) {
        // Brush motor logic add speed to pwm command
        if (dir) {
            digitalWrite(brushMotorPins[0], HIGH);
            digitalWrite(brushMotorPins[1], LOW);
            analogWrite(brushMotorPins[2], speed);
        } else {
            digitalWrite(brushMotorPins[0], LOW);
            digitalWrite(brushMotorPins[1], HIGH);
            analogWrite(brushMotorPins[2], speed);
        }
        last_time = time;
    }
}

void controlStepperMotor(bool dir) {
    static unsigned long last_time = 0;
    if (1/time-last_time >= stepper_freq) {
        // Motor and stepper control logic
        if (dir) {
            digitalWrite(stepperMotorPins[0], HIGH);
            digitalWrite(stepperMotorPins[1], LOW);
            digitalWrite(stepperMotorPins[2], HIGH);
            digitalWrite(stepperMotorPins[3], LOW);
        } else {
            digitalWrite(stepperMotorPins[0], LOW);
            digitalWrite(stepperMotorPins[1], HIGH);
            digitalWrite(stepperMotorPins[2], LOW);
            digitalWrite(stepperMotorPins[3], HIGH);
        }
        last_time = time;
    }
}

float check_IR() {
    static unsigned long last_time = 0;
    if (1/time-last_time >= ir_freq) {
        // IR logic
        float irValue = analogRead(irSensorPin); // Read value from IR sensor
        last_time = time;
        return irValue;
    }
}

// Other necessary functions 
// Integrate buttons interuptions.