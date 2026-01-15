/**
 * def_const_func.cpp - Function Implementations
 *
 * This file contains all function implementations for the Solar Panel
 * Cleaning Robot, including motor control, sensor reading, LED status,
 * and safety monitoring functions.
 */

#include <Arduino.h>
#include <Time.h>
#include <Servo.h>
#include "def_const_func.h"

// ============================================================================
// Global Variable Definitions
// ============================================================================

// Servo object for valve control
Servo myServo;

// System state
State currentState = REST;
int step = PANEL_WIDTH / CLEANING_WIDTH + (PANEL_WIDTH % CLEANING_WIDTH != 0);
int nb_cycles_counter = 0;
bool raining = false;
unsigned long currentTime;
unsigned long cleaningTime = 0;

// IR sensor
const float IR_PERIODE = 1.0;
bool IRseen = false;

// Rain sensor
const float RAIN_SENSOR_PERIODE = 1000.0;

// Motor configuration
const int MOTOR_SPEED_BRUSH = 200;
const int MOTOR_SPEED_GEAR = 200;
const float MOTOR_PERIODE = 100.0;

// Stepper motor speed profile
int stepperStartSpeed = 1500;       // Initial delay (microseconds)
int stepperEndSpeed = 1000;         // Final delay (microseconds)
int stepperAccelerationSteps = 500; // Steps to accelerate/decelerate

// LED timing
const float LED_PERIODE = 100.0;

// Current sensor timing
const float CURRENT_PERIODE = 500.0;

// Button timing and state
const float BUTTON_PERIODE = 30.0;
unsigned long lastDebounceTimeR = 0;
unsigned long lastDebounceTimeC1 = 0;
unsigned long lastDebounceTimeChome = 0;
bool lastButtonStateR = RELEASED;
bool lastButtonStateC1 = RELEASED;
bool lastButtonStateChome = CLICKED;
bool buttonStateR = RELEASED;
bool buttonStateC1 = RELEASED;
bool buttonStateChome = CLICKED;

// Valve timing
const float VALVE_PERIODE = 1000.0;

// ============================================================================
// Initialization Functions
// ============================================================================

/**
 * Initialize DC motor pins (gearbox and brush motors)
 */
void initializeMotors() {
    pinMode(GEARBOX_MOTOR_PIN1, OUTPUT);
    pinMode(GEARBOX_MOTOR_PIN2, OUTPUT);
    pinMode(BRUSH_MOTOR_PIN1, OUTPUT);
    pinMode(BRUSH_MOTOR_PIN2, OUTPUT);
}

/**
 * Initialize limit switch (button) pins
 */
void initializeButtons() {
    pinMode(BUTTON_PIN_R, INPUT);
    pinMode(BUTTON_PIN_C1, INPUT);
    pinMode(BUTTON_PIN_Chome, INPUT);
}

/**
 * Initialize RGB LED pins
 */
void initializeLEDPins() {
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(BLUE_LED_PIN, OUTPUT);
}

/**
 * Initialize rain sensor pin
 */
void initializeRainSensor() {
    pinMode(RAIN_SENSOR_PIN, INPUT);
}

/**
 * Initialize stepper motor control pins
 */
void initializeStepperMotor() {
    pinMode(STEPPER_DIR_PIN, OUTPUT);
    pinMode(STEPPER_STEP_PIN, OUTPUT);
}

/**
 * Initialize current sensor pins
 */
void initializeCurrentSensors() {
    pinMode(CURRENT_PIN_GEARBOX, INPUT);
    pinMode(CURRENT_PIN_BRUSH, INPUT);
}

// ============================================================================
// IR Sensor Functions
// ============================================================================

/**
 * Check IR sensor and update IRseen flag
 * Uses threshold-based detection for proximity sensing
 */
void checkIRSensor() {
    static unsigned long lastTime = 0;

    if (millis() - lastTime >= IR_PERIODE) {
        float irValue = analogRead(IR_SENSOR_PIN);

        if (irValue >= IR_THRESHOLD) {
            IRseen = true;
            Serial.println("Seen");
        } else {
            IRseen = false;
            Serial.println("Not Seen");
        }

        lastTime = millis();
    }
}

// ============================================================================
// Rain Sensor Functions
// ============================================================================

/**
 * Check rain sensor and log status
 * Lower values indicate rain (wet sensor)
 */
void checkRainSensor() {
    static unsigned long lastTime = 0;

    if (millis() - lastTime >= RAIN_SENSOR_PERIODE) {
        int rainValue = analogRead(RAIN_SENSOR_PIN);

        if (rainValue <= RAIN_THRESHOLD) {
            Serial.println("Raining");
        } else {
            Serial.println("Not Raining");
        }
        Serial.println(rainValue);

        lastTime = millis();
    }
}

// ============================================================================
// DC Motor Control Functions
// ============================================================================

/**
 * Control the brush motor direction
 * @param direction true for forward, false for reverse
 */
void controlBrushMotor(bool direction) {
    digitalWrite(BRUSH_MOTOR_PIN1, direction ? HIGH : LOW);
    digitalWrite(BRUSH_MOTOR_PIN2, direction ? LOW : HIGH);
}

/**
 * Control the gearbox motor direction
 * @param direction true for up, false for down
 */
void controlGearboxMotor(bool direction) {
    digitalWrite(GEARBOX_MOTOR_PIN1, direction ? HIGH : LOW);
    digitalWrite(GEARBOX_MOTOR_PIN2, direction ? LOW : HIGH);
}

// ============================================================================
// Stepper Motor Control Functions
// ============================================================================

/**
 * Control stepper motor with acceleration profile
 *
 * @param distance Number of revolutions to move
 * @param clockwise Direction (true = clockwise/right)
 * @param stepperStartSpeed Initial step delay in microseconds
 * @param stepperEndSpeed Final step delay in microseconds
 * @param stepperAccelerationSteps Number of steps for acceleration/deceleration
 */
void controlStepper(int distance, bool clockwise, int stepperStartSpeed,
                    int stepperEndSpeed, int stepperAccelerationSteps) {
    // Wake up stepper driver
    digitalWrite(STEPPER_SLEEP_PIN, HIGH);
    delayMicroseconds(2);

    // Calculate total steps
    int totalSteps = distance / CM_PER_REVOLUTION * STEPPER_STEPS_PER_REV;
    int stepDelay = stepperStartSpeed;
    int stepChange = (stepperStartSpeed - stepperEndSpeed) / stepperAccelerationSteps;

    // Set direction
    digitalWrite(STEPPER_DIR_PIN, clockwise ? HIGH : LOW);

    // Acceleration phase
    for (int i = 0; i < stepperAccelerationSteps && i < totalSteps; i++) {
        digitalWrite(STEPPER_STEP_PIN, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(STEPPER_STEP_PIN, LOW);
        delayMicroseconds(stepDelay);
        stepDelay -= stepChange;
    }

    // Constant speed phase
    for (int i = stepperAccelerationSteps; i < totalSteps - stepperAccelerationSteps; i++) {
        digitalWrite(STEPPER_STEP_PIN, HIGH);
        delayMicroseconds(stepperEndSpeed);
        digitalWrite(STEPPER_STEP_PIN, LOW);
        delayMicroseconds(stepperEndSpeed);
    }

    // Deceleration phase
    for (int i = totalSteps - stepperAccelerationSteps; i < totalSteps; i++) {
        digitalWrite(STEPPER_STEP_PIN, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(STEPPER_STEP_PIN, LOW);
        delayMicroseconds(stepDelay);
        stepDelay += stepChange;
    }

    // Put stepper driver to sleep
    digitalWrite(STEPPER_STEP_PIN, LOW);
    digitalWrite(STEPPER_SLEEP_PIN, LOW);
}

/**
 * Control stepper motor at constant speed (no acceleration)
 *
 * @param distance Number of revolutions to move
 * @param clockwise Direction (true = clockwise/right)
 */
void controlStepper(int distance, bool clockwise) {
    // Wake up stepper driver
    digitalWrite(STEPPER_SLEEP_PIN, HIGH);
    delayMicroseconds(2);

    // Calculate total steps (distance is in revolutions)
    int totalSteps = distance * STEPPER_STEPS_PER_REV;

    // Set direction
    digitalWrite(STEPPER_DIR_PIN, clockwise ? LOW : HIGH);

    // Move at constant speed
    for (int i = 0; i < totalSteps; i++) {
        digitalWrite(STEPPER_STEP_PIN, HIGH);
        delayMicroseconds(stepperEndSpeed);
        digitalWrite(STEPPER_STEP_PIN, LOW);
        delayMicroseconds(stepperEndSpeed);
    }

    // Put stepper driver to sleep
    digitalWrite(STEPPER_STEP_PIN, LOW);
    digitalWrite(STEPPER_SLEEP_PIN, LOW);
}

// ============================================================================
// LED Status Functions
// ============================================================================

/**
 * Update LED color based on current robot state
 * @param currentState Current operating state of the robot
 */
void updateLEDs(State currentState) {
    static unsigned long lastTime = 0;

    if (millis() - lastTime >= LED_PERIODE) {
        switch (currentState) {
            case REST:
                setColor(0, 0, 0, false);           // Off
                break;
            case WATER_FILLING:
                blueLED(true);                       // Blue flashing
                break;
            case INITIAL_POSITION:
                whiteLED(false);                     // White solid
                break;
            case CLEANING:
                greenLED(true);                      // Green flashing
                break;
            case UP_TRAVEL:
                greenLED(false);                     // Green solid
                break;
            case TRANSLATION:
                whiteLED(true);                      // White flashing
                break;
            case RETURN_HOME:
                redLED(false);                       // Red solid
                break;
            case PROBLEM:
                redLED(true);                        // Red flashing
                break;
            default:
                setColor(0, 0, 0, false);           // Off
                break;
        }
        lastTime = millis();
    }
}

/**
 * Set LED to red
 * @param flashing true for flashing, false for solid
 */
void redLED(bool flashing) {
    setColor(LED_INTENSITY, 0, 0, flashing);
}

/**
 * Set LED to green
 * @param flashing true for flashing, false for solid
 */
void greenLED(bool flashing) {
    setColor(0, LED_INTENSITY, 0, flashing);
}

/**
 * Set LED to blue
 * @param flashing true for flashing, false for solid
 */
void blueLED(bool flashing) {
    setColor(0, 0, LED_INTENSITY, flashing);
}

/**
 * Set LED to white
 * @param flashing true for flashing, false for solid
 */
void whiteLED(bool flashing) {
    setColor(LED_INTENSITY, LED_INTENSITY, LED_INTENSITY, flashing);
}

/**
 * Set RGB LED color with optional flashing
 *
 * @param red Red intensity (0-255)
 * @param green Green intensity (0-255)
 * @param blue Blue intensity (0-255)
 * @param flashing true for flashing effect, false for solid
 */
void setColor(int red, int green, int blue, bool flashing) {
    const int FLASH_DURATION = 500;  // Duration of each flash (ms)

    if (flashing) {
        for (int i = 0; i < 5; i++) {
            writeColor(red, green, blue);
            delay(FLASH_DURATION);
            writeColor(0, 0, 0);
            delay(FLASH_DURATION);
        }
    } else {
        writeColor(red, green, blue);
    }
}

/**
 * Write color values directly to LED pins
 *
 * @param red Red intensity (0-255)
 * @param green Green intensity (0-255)
 * @param blue Blue intensity (0-255)
 */
void writeColor(int red, int green, int blue) {
    analogWrite(RED_LED_PIN, red);
    analogWrite(GREEN_LED_PIN, green);
    analogWrite(BLUE_LED_PIN, blue);
}

// ============================================================================
// Current Monitoring Functions
// ============================================================================

/**
 * Check motor current and trigger PROBLEM state if threshold exceeded
 *
 * Uses ACS712 current sensor with the following calculation:
 * Current = (2.5V - SensorVoltage) / Sensitivity
 *
 * @param currentPin Analog pin connected to current sensor
 */
void checkCurrent(int currentPin) {
    static unsigned long lastTime = 0;

    if (millis() - lastTime >= CURRENT_PERIODE) {
        // Read and convert to current (Amps)
        float rawValue = analogRead(currentPin);
        float voltage = rawValue * (5.0 / 1024.0);
        float currentVal = (2.5 - voltage) / CURRENT_SENSITIVITY;
        currentVal = abs(currentVal);

        // Log current value
        Serial.print("Current Value: ");
        Serial.print(currentVal, 5);
        Serial.println(" [A]");

        // Check against appropriate threshold
        if (currentPin == CURRENT_PIN_GEARBOX) {
            if (currentVal >= CURRENT_THRESHOLD_GEARBOX) {
                Serial.println("Current threshold exceeded gearbox");
                currentState = PROBLEM;
            }
        } else if (currentPin == CURRENT_PIN_BRUSH) {
            if (currentVal >= CURRENT_THRESHOLD_BRUSH) {
                Serial.println("Current threshold exceeded brush");
                currentState = PROBLEM;
            }
        } else {
            Serial.println("Error: Current pin not defined");
        }

        lastTime = millis();
    }
}

// ============================================================================
// Button (Limit Switch) Functions
// ============================================================================

/**
 * Check and debounce robot limit switch (Button R)
 */
void checkButtonR() {
    static unsigned long lastTime = 0;

    if (millis() - lastTime >= BUTTON_PERIODE) {
        bool reading = digitalRead(BUTTON_PIN_R);

        if (reading != lastButtonStateR) {
            lastDebounceTimeR = millis();
        }

        if ((millis() - lastDebounceTimeR) > DEBOUNCE_DELAY) {
            if (reading != buttonStateR) {
                buttonStateR = reading;
                Serial.println("Button R changed");
            }
        }

        lastButtonStateR = reading;
        lastTime = millis();
    }
}

/**
 * Check and debounce home station limit switch (Button Chome)
 */
void checkButtonChome() {
    static unsigned long lastTime = 0;

    if (millis() - lastTime >= BUTTON_PERIODE) {
        bool reading = digitalRead(BUTTON_PIN_Chome);

        if (reading != lastButtonStateChome) {
            lastDebounceTimeChome = millis();
        }

        if ((millis() - lastDebounceTimeChome) > DEBOUNCE_DELAY) {
            if (reading != buttonStateChome) {
                buttonStateChome = reading;
                Serial.println("Button Chome changed");
            }
        }

        lastButtonStateChome = reading;
        lastTime = millis();
    }
}

/**
 * Check and debounce carriage limit switch (Button C1)
 */
void checkButtonC1() {
    static unsigned long lastTime = 0;

    if (millis() - lastTime >= BUTTON_PERIODE) {
        bool reading = digitalRead(BUTTON_PIN_C1);

        if (reading != lastButtonStateC1) {
            lastDebounceTimeC1 = millis();
        }

        if ((millis() - lastDebounceTimeC1) > DEBOUNCE_DELAY) {
            if (reading != buttonStateC1) {
                buttonStateC1 = reading;
                Serial.println("Button C1 changed");
            }
        }

        lastButtonStateC1 = reading;
        lastTime = millis();
    }
}

// ============================================================================
// Valve (Servo) Control Functions
// ============================================================================

/**
 * Control water valve with rate limiting
 * @param angle Target angle in degrees (0-180)
 */
void controlValve(int angle) {
    static unsigned long lastTime = 0;

    if (millis() - lastTime >= VALVE_PERIODE) {
        rotateServo(angle);
        lastTime = millis();
    }
}

/**
 * Rotate servo to specified angle
 * @param angle Target angle in degrees (0-180)
 */
void rotateServo(int angle) {
    if (angle >= 0 && angle <= 180) {
        // Map angle to pulse width in microseconds
        const int MIN_PULSE_WIDTH = 0;
        const int MAX_PULSE_WIDTH = 5000;
        int pulseWidth = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

        myServo.writeMicroseconds(pulseWidth);
        Serial.print("Angle: ");
        Serial.println(angle);
    } else {
        Serial.println("Angle out of range");
    }
}

// ============================================================================
// High-Level Motor Movement Functions
// ============================================================================

/**
 * Move robot in specified direction
 *
 * @param direction UP, DOWN, LEFT, or RIGHT
 * @param distanceOrSpeed For UP/DOWN: unused (speed is fixed)
 *                        For LEFT/RIGHT: number of stepper revolutions
 */
void moveMotor(MotorDirection direction, float distanceOrSpeed) {
    static unsigned long lastTime = 0;

    if (millis() - lastTime >= MOTOR_PERIODE) {
        switch (direction) {
            case UP:
                controlGearboxMotor(true);
                break;
            case DOWN:
                controlGearboxMotor(false);
                break;
            case LEFT:
                controlStepper(distanceOrSpeed, false);
                break;
            case RIGHT:
                controlStepper(distanceOrSpeed, true);
                break;
        }
        lastTime = millis();
    }
}

/**
 * Emergency stop - immediately halt all motors
 */
void stopAllMotors() {
    // Stop brush motor
    digitalWrite(BRUSH_MOTOR_PIN1, LOW);
    digitalWrite(BRUSH_MOTOR_PIN2, LOW);

    // Stop gearbox motor
    digitalWrite(GEARBOX_MOTOR_PIN1, LOW);
    digitalWrite(GEARBOX_MOTOR_PIN2, LOW);
}
