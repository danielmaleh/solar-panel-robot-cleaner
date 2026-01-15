/**
 * Solar Panel Cleaning Robot - Main Control Program
 *
 * This program controls an autonomous robot designed to clean solar panels.
 * The robot performs cleaning cycles by:
 *   1. Moving to an initial position
 *   2. Performing vertical cleaning passes with a rotating brush
 *   3. Translating horizontally between panel sections
 *   4. Returning to the home/docking station
 *
 * Hardware Components:
 *   - DC Gearbox Motor: Vertical movement (up/down)
 *   - DC Brush Motor: Rotating cleaning brush
 *   - Stepper Motor: Horizontal translation (left/right)
 *   - Servo Motor: Water valve control
 *   - IR Sensor: Position detection for docking
 *   - Rain Sensor: Weather detection
 *   - Limit Switches: End-of-travel detection
 *   - RGB LED: System status indication
 *
 * @author Daniel Abraham Elmaleh
 */

#include <Time.h>
#include "def_const_func.h"

// ============================================================================
// Global Variables
// ============================================================================

int cycleCount = 0;              // Current cleaning cycle counter
int irThreshold = 0;             // Calibrated IR sensor threshold
const int TOTAL_CLEANING_PASSES = 6;  // Number of horizontal passes to complete
const int INITIAL_TRANSLATION_REVS = 11;  // Revolutions to reach initial position
const int INTER_PASS_TRANSLATION_REVS = 7; // Revolutions between cleaning passes

// ============================================================================
// Setup
// ============================================================================

void setup() {
    Serial.begin(9600);
    Serial.println("Initializing Solar Panel Cleaning Robot...");

    initializeMotors();
    Serial.println("  Motors initialized");

    initializeButtons();
    Serial.println("  Buttons initialized");

    initializeLEDPins();
    Serial.println("  LEDs initialized");

    initializeRainSensor();
    Serial.println("  Rain sensor initialized");

    initializeStepperMotor();
    Serial.println("  Stepper motor initialized");

    initializeCurrentSensors();
    Serial.println("  Current sensors initialized");

    Serial.println("Setup complete!");
}

// ============================================================================
// Main Loop
// ============================================================================

void loop() {
    delay(2000);

    // First cycle: Calibrate IR and move to initial position
    if (cycleCount == 0) {
        performInitialSetup();
        performCleaningPass();
        cycleCount++;
    }
    // Last cycle: Perform final cleaning and return home
    else if (cycleCount == TOTAL_CLEANING_PASSES) {
        performFinalCleaningCycle();
        return;  // End program after returning home
    }
    // Intermediate cycles: Standard cleaning passes
    else {
        delay(1000);
        performTranslation(INTER_PASS_TRANSLATION_REVS);
        moveToTopOfPanel();
        performCleaningPass();
        cycleCount++;
        delay(1000);
    }
}

// ============================================================================
// Cleaning Operation Functions
// ============================================================================

/**
 * Performs initial setup including IR calibration and moving to start position
 */
void performInitialSetup() {
    calibrateIRSensor();
    delay(1000);

    // Move to initial cleaning position
    moveMotor(LEFT, INITIAL_TRANSLATION_REVS);
    stopAllMotors();
    delay(1000);

    moveToTopOfPanel();
}

/**
 * Performs a single cleaning pass (downward motion with brush active)
 */
void performCleaningPass() {
    moveMotor(DOWN, 0);
    controlBrushMotor(true);

    // Continue cleaning until bottom limit switch is triggered
    float buttonState = digitalRead(BUTTON_PIN_R);
    while (buttonState == 0) {
        moveMotor(DOWN, 0);
        controlBrushMotor(true);
        buttonState = digitalRead(BUTTON_PIN_R);
        Serial.println(buttonState);
    }

    // Stop motors
    digitalWrite(BRUSH_MOTOR_PIN1, LOW);
    digitalWrite(BRUSH_MOTOR_PIN2, LOW);
    delay(200);
    digitalWrite(GEARBOX_MOTOR_PIN1, LOW);
    digitalWrite(GEARBOX_MOTOR_PIN2, LOW);

    // Move back up using IR sensor
    returnToTopUsingIR();
}

/**
 * Moves the robot to the top of the panel using limit switch
 */
void moveToTopOfPanel() {
    float buttonState = digitalRead(BUTTON_PIN_R);
    while (buttonState == 1) {
        moveMotor(UP, 0);
        buttonState = digitalRead(BUTTON_PIN_R);
    }
    stopAllMotors();
    delay(1000);
}

/**
 * Returns to top position using IR sensor detection
 */
void returnToTopUsingIR() {
    float irValue = analogRead(IR_SENSOR_PIN);
    while (irValue < irThreshold) {
        moveMotor(UP, 0);
        irValue = analogRead(IR_SENSOR_PIN);
        Serial.println(irValue);
    }
    stopAllMotors();
}

/**
 * Performs horizontal translation using stepper motor
 * @param revolutions Number of motor revolutions
 */
void performTranslation(int revolutions) {
    moveMotor(LEFT, revolutions);
    stopAllMotors();
}

/**
 * Performs the final cleaning cycle and returns robot to home position
 */
void performFinalCleaningCycle() {
    // Move to final panel position
    float buttonState = digitalRead(BUTTON_PIN_C1);
    digitalWrite(STEPPER_SLEEP_PIN, HIGH);
    delayMicroseconds(2);
    digitalWrite(STEPPER_DIR_PIN, HIGH);

    while (buttonState == 1) {
        digitalWrite(STEPPER_STEP_PIN, HIGH);
        delayMicroseconds(1000);
        digitalWrite(STEPPER_STEP_PIN, LOW);
        delayMicroseconds(1000);
        buttonState = digitalRead(BUTTON_PIN_C1);
    }
    digitalWrite(STEPPER_SLEEP_PIN, LOW);
    delay(1000);

    // Perform final cleaning pass
    moveToTopOfPanel();
    performCleaningPass();
    delay(1000);

    // Return to home/docking station
    returnToHome();

    // Program complete - wait indefinitely
    delay(300000000);
}

/**
 * Returns the robot to the home/docking station
 */
void returnToHome() {
    float buttonState = digitalRead(BUTTON_PIN_Chome);
    digitalWrite(STEPPER_SLEEP_PIN, HIGH);
    delayMicroseconds(2);
    digitalWrite(STEPPER_DIR_PIN, LOW);

    while (buttonState == 1) {
        digitalWrite(STEPPER_STEP_PIN, HIGH);
        delayMicroseconds(1000);
        digitalWrite(STEPPER_STEP_PIN, LOW);
        delayMicroseconds(1000);
        buttonState = digitalRead(BUTTON_PIN_Chome);
    }
    digitalWrite(STEPPER_SLEEP_PIN, LOW);
}

// ============================================================================
// Calibration Functions
// ============================================================================

/**
 * Calibrates the IR sensor by averaging readings and setting threshold
 */
void calibrateIRSensor() {
    Serial.println("Calibrating IR sensor...");

    const int NUM_SAMPLES = 10;
    const int THRESHOLD_OFFSET = 25;
    int sum = 0;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        float irValue = analogRead(IR_SENSOR_PIN);
        Serial.print("  Sample ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(irValue);
        sum += irValue;
        delay(500);
    }

    irThreshold = (sum / NUM_SAMPLES) - THRESHOLD_OFFSET;
    Serial.print("IR threshold set to: ");
    Serial.println(irThreshold);
}
