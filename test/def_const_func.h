/**
 * def_const_func.h - Definitions, Constants, and Function Declarations
 *
 * This header file contains all hardware pin definitions, system constants,
 * and function declarations for the Solar Panel Cleaning Robot.
 *
 * Modules:
 *   - IR Sensor: Position detection
 *   - Rain Sensor: Weather monitoring
 *   - DC Brush Motor: Cleaning brush control
 *   - DC Gearbox Motor: Vertical movement
 *   - Stepper Motor: Horizontal translation
 *   - RGB LEDs: Status indication
 *   - Current Sensors: Motor fault detection
 *   - Limit Switches: End-of-travel detection
 *   - Servo Valve: Water dispensing control
 *
 * Robot States:
 *   REST, WATER_FILLING, INITIAL_POSITION, CLEANING,
 *   UP_TRAVEL, TRANSLATION, RETURN_HOME, PROBLEM
 */

#ifndef DEF_CONST_FUNC_H
#define DEF_CONST_FUNC_H

#include <Servo.h>

// ============================================================================
// System Configuration Constants
// ============================================================================

#define CLEANING_WIDTH              30      // Effective cleaning width (cm)
#define CLEANING_PERIOD             60000   // Time between cleanings (ms) - 1 minute
#define PANEL_WIDTH                 100     // Total width of one panel (cm)
#define DIST_TO_INITIAL_POSITION    45      // Distance to initial position (cm)
#define END_OF_CLEANING_DELAY       2000    // Delay after cleaning (ms)

// ============================================================================
// Pin Definitions - IR Sensor
// ============================================================================

#define IR_SENSOR_PIN               A4
#define IR_THRESHOLD                620     // Detection threshold value

// IR Sensor Reference Values:
//   < 20:      Nothing detected
//   150-160:   Object at ~30cm
//   250-280:   Object at ~20cm
//   460-470:   Object at ~10cm
//   > 600:     Object at < 5cm (may decrease if too close)

// ============================================================================
// Pin Definitions - Rain Sensor
// ============================================================================

#define RAIN_SENSOR_PIN             A15
#define RAIN_THRESHOLD              700     // Rain detection threshold

// ============================================================================
// Pin Definitions - DC Brush Motor
// ============================================================================

#define BRUSH_MOTOR_PIN1            6
#define BRUSH_MOTOR_PIN2            7

// ============================================================================
// Pin Definitions - DC Gearbox Motor
// ============================================================================

#define GEARBOX_MOTOR_PIN1          4
#define GEARBOX_MOTOR_PIN2          5
#define GEARBOX_MOTOR_SPEED_PIN     12

// ============================================================================
// Pin Definitions - Stepper Motor
// ============================================================================

#define STEPPER_DIR_PIN             2
#define STEPPER_STEP_PIN            3
#define STEPPER_SLEEP_PIN           28
#define STEPPER_STEPS_PER_REV       200     // Steps per revolution
#define CM_PER_REVOLUTION           2.0     // Distance per revolution (cm)

// ============================================================================
// Pin Definitions - RGB LED
// ============================================================================

#define RED_LED_PIN                 11
#define GREEN_LED_PIN               9
#define BLUE_LED_PIN                8
#define LED_INTENSITY               150     // PWM intensity (0-255)

// ============================================================================
// Pin Definitions - Current Sensors
// ============================================================================

#define CURRENT_PIN_GEARBOX         A14     // Current sensor for gearbox motor
#define CURRENT_PIN_BRUSH           A0      // Current sensor for brush motor
#define CURRENT_SENSITIVITY         0.185   // ACS712 sensitivity (V/A)
#define CURRENT_THRESHOLD_GEARBOX   1.5     // Fault threshold (A)
#define CURRENT_THRESHOLD_BRUSH     0.7     // Fault threshold (A)

// Alias for backwards compatibility
#define CURRENT_PIN1                CURRENT_PIN_GEARBOX
#define CURRENT_PIN2                CURRENT_PIN_BRUSH

// ============================================================================
// Pin Definitions - Limit Switches (Buttons)
// ============================================================================

#define BUTTON_PIN_R                22      // End-of-travel on robot
#define BUTTON_PIN_C1               24      // End-of-travel on carriage
#define BUTTON_PIN_Chome            26      // End-of-travel at home station
#define DEBOUNCE_DELAY              50      // Button debounce delay (ms)

// ============================================================================
// Pin Definitions - Servo Valve
// ============================================================================

#define SERVO_PIN                   13
#define VALVE_ANGLE_OPEN            180     // Valve open position (degrees)
#define VALVE_ANGLE_CLOSE           0       // Valve closed position (degrees)

// ============================================================================
// Enumerations
// ============================================================================

/**
 * Robot operating states
 */
enum State {
    REST,               // Idle, waiting for trigger
    WATER_FILLING,      // Filling water reservoir
    INITIAL_POSITION,   // Moving to start position
    CLEANING,           // Active cleaning in progress
    UP_TRAVEL,          // Returning to top of panel
    TRANSLATION,        // Horizontal movement
    RETURN_HOME,        // Returning to docking station
    PROBLEM             // Error state - all motors stopped
};

/**
 * Motor movement directions
 */
enum MotorDirection {
    UP,
    DOWN,
    LEFT,
    RIGHT
};

/**
 * Button states
 */
enum ButtonState {
    CLICKED,
    RELEASED
};

// ============================================================================
// Global Variable Declarations
// ============================================================================

// Servo object
extern Servo myServo;

// System state
extern State currentState;
extern int step;
extern int nb_cycles_counter;
extern bool raining;
extern unsigned long currentTime;
extern unsigned long cleaningTime;

// IR sensor
extern const float IR_PERIODE;
extern bool IRseen;

// Rain sensor
extern const float RAIN_SENSOR_PERIODE;

// Motors
extern const int MOTOR_SPEED_GEAR;
extern const float MOTOR_PERIODE;

// Stepper configuration
extern int stepperStartSpeed;
extern int stepperEndSpeed;
extern int stepperAccelerationSteps;

// LEDs
extern const float LED_PERIODE;

// Current sensors
extern const float CURRENT_PERIODE;

// Buttons - debouncing
extern const float BUTTON_PERIODE;
extern unsigned long lastDebounceTimeR;
extern unsigned long lastDebounceTimeC1;
extern unsigned long lastDebounceTimeChome;
extern bool lastButtonStateR;
extern bool lastButtonStateC1;
extern bool lastButtonStateChome;
extern bool buttonStateR;
extern bool buttonStateC1;
extern bool buttonStateChome;

// Valve
extern const float VALVE_PERIODE;

// ============================================================================
// Function Declarations - Initialization
// ============================================================================

void initializeMotors();
void initializeButtons();
void initializeLEDPins();
void initializeRainSensor();
void initializeStepperMotor();
void initializeCurrentSensors();

// ============================================================================
// Function Declarations - Sensors
// ============================================================================

void checkIRSensor();
void checkRainSensor();

// ============================================================================
// Function Declarations - Motor Control
// ============================================================================

void controlBrushMotor(bool direction);
void controlGearboxMotor(bool direction);
void controlStepper(int distance, bool clockwise, int startSpeed, int endSpeed, int accelSteps);
void controlStepper(int distance, bool clockwise);
void moveMotor(MotorDirection direction, float distanceOrSpeed);
void stopAllMotors();

// ============================================================================
// Function Declarations - LED Status
// ============================================================================

void updateLEDs(State currentState);
void redLED(bool flashing);
void greenLED(bool flashing);
void blueLED(bool flashing);
void whiteLED(bool flashing);
void setColor(int red, int green, int blue, bool flashing);
void writeColor(int red, int green, int blue);

// ============================================================================
// Function Declarations - Safety & Monitoring
// ============================================================================

void checkCurrent(int currentPin);
void checkButtonR();
void checkButtonC1();
void checkButtonChome();

// ============================================================================
// Function Declarations - Valve Control
// ============================================================================

void controlValve(int angle);
void rotateServo(int angle);

#endif // DEF_CONST_FUNC_H
