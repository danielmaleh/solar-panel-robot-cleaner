// def_const_func.h
// Header file for the main program. "def_const_func" stands for "definitions, constants and functions".
// Description: 
// This file contains all the constants and functions used in the main program.
// The functions are defined in the file 'def_const_func.cpp'.
// The constants are defined in the file 'def_const_func.h' and are used in the main program.
// We have the following modules: IR, rain, DC brush, DC gearbox, stepper, LEDs, current, buttons, valve (servo).
// this file is divided into: Constants, Global variables, Functions.
// The main program is divided into states. Each state has its own logic.
// The states are: REST, WATER_FILLING, INITIAL_POSITION, FIRST_POSITION_CLEANING, CLEANING, UP_TRAVEL, TRANSLATION, RETURN_HOME, PROBLEM.
// The main program is in the file 'main.ino'.


//------------------------------------GENERAL------------------------------------
#define CLEANING_WIDTH 30 // Effective cleaning width in cm
#define CLEANING_PERIOD 0 // Time between two cleanings in milliseconds
#define DIST 0 // Total width of all panels
#define DIST_TO_INITIAL_POSITION 0 // Distance to the initial position in cm
#define END_OF_CLEANING_DELAY 0 // Delay after cleaning in milliseconds, for the wheel to get down

// Global variables
extern int step;
extern bool raining; // Boolean indicating if it is raining
extern unsigned long currentTime; // Actual time
extern unsigned long cleaningTime; // Time of the last cleaning

// Enumeration for states
enum State { REST, WATER_FILLING, INITIAL_POSITION, CLEANING, UP_TRAVEL, TRANSLATION, RETURN_HOME, PROBLEM };
extern State currentState;

enum MotorDirection {
    UP, DOWN, LEFT, RIGHT
};


//------------------------------------IR------------------------------------
// GRADING:
// < 20 : nothing 
// 150-160 = at 30cm 
// 250-280 = at 20cm
// 460,470 = at 10cm
// >600 = < 5cm. Si trop proche la valeur rebaisse.

#define IR_THRESHOLD 500 
#define IR_SENSOR_PIN A4 

extern const float IR_PERIODE;
extern bool IRseen;


//------------------------------------RAIN------------------------------------
#define RAIN_THRESHOLD 700 
#define RAIN_SENSOR_PIN A0 

// Frequency of the rain detector
extern const float RAIN_SENSOR_PERIODE;


//------------------------------------DC_BRUSH------------------------------------
#define BRUSH_MOTOR_PIN1 24
#define BRUSH_MOTOR_PIN2 25
#define BRUSH_MOTOR_SPEED_PIN 9

extern const int MOTOR_SPEED_BRUSH;


//------------------------------------DC_GEAR------------------------------------
#define GEARBOX_MOTOR_PIN1 22
#define GEARBOX_MOTOR_PIN2 23
#define GEARBOX_MOTOR_SPEED_PIN 8

extern const int MOTOR_SPEED_GEAR;
extern const float MOTOR_PERIODE;


//------------------------------------STEPPER------------------------------------
// GRADING:
// 20cm in 5 rev. = 4 cm per rev.
#define CM_PER_REVOLUTION 4.0 // 4 cm per revolution
#define STEPPER_STEPS_PER_REVOLUTION 200 //remove

#define STEPPER_DIR_PIN 2
#define STEPPER_STEP_PIN 3


//------------------------------------LEDs------------------------------------
#define LED_INTENSITY 150 
#define RED_LED_PIN 11
#define GREEN_LED_PIN 10
#define BLUE_LED_PIN 9

extern const float LED_PERIODE;


//------------------------------------CURRENT------------------------------------
#define CURRENT_SENSITIVITY 0.185
#define CURRENT_THRESHOLD 0.5
#define CURRENT_PIN1 A0 // current sensor for brush motor
#define CURRENT_PIN2 A1 // current sensor for gearbox motor
#define CURRENT_PIN3 A3 // current sensor 1 for stepper motor
#define CURRENT_PIN4 A4 // current sensor 2 for stepper motor


extern const float CURRENT_PERIODE;


//------------------------------------BUTTONS------------------------------------
#define DEBOUNCE_DELAY 50 // Debounce delay in milliseconds

#define BUTTON_PIN_R A2 // End of travel button on robot
#define BUTTON_PIN_C1 2 // End of travel button on carrige
#define BUTTON_PIN_C2 3 // End of travel button on resting station

// Debouncing Variables
extern unsigned long lastDebounceTimeR, lastDebounceTimeC1, lastDebounceTimeC2;
extern bool lastButtonStateR, lastButtonStateC1, lastButtonStateC2;

extern bool buttonStateR, buttonStateC1, buttonStateC2;

extern const float BUTTON_PERIODE;


//------------------------------------VALVE (SERVO)------------------------------------
#define SERVO_PIN 9 // Define the pin connected to the servo
#define VALVE_ANGLE 90 // Angle of the valve in degrees
extern const float VALVE_PERIODE;


//====================================FUNCTION DECLARATIONS====================================
// Initialization functions
void initializeServo();
void initializeMotors();
void initializeButtons();
void initializeLEDPins();
void initializeRainSensor();
void initializeStepperMotor();
void initializeCurrentSensors();
void initializeSerialCommunication();


// State functions
//------------------------------------IR------------------------------------
void checkIRSensor();

//------------------------------------RAIN------------------------------------
void checkRainSensor();

//------------------------------------DC_BRUSH------------------------------------
void controlBrushMotor(bool direction, int speed);

//------------------------------------DC_GEAR------------------------------------
void controlGearboxMotor(bool direction, int speed);

//------------------------------------STEPPER------------------------------------
void controlStepper(int revolutions, bool clockwise);

//------------------------------------LEDs------------------------------------
void updateLEDs(State currentState);
void redLED(bool flashing);
void greenLED(bool flashing);
void blueLED(bool flashing);
void whiteLED(bool flashing);
void setColor(int red, int green, int blue, bool flashing);
void writeColor(int red, int green, int blue);

//------------------------------------CURRENT------------------------------------
void checkCurrent(int currentPin);

//------------------------------------BUTTONS------------------------------------
void checkButtonR();
void checkButtonC1();
void checkButtonC2();

//------------------------------------VALVE (Servo)------------------------------------
void controlValve(int angle);
void rotateServo(int angle);


//------------------------------------MOTORS MOUVEMENT------------------------------------
void moveMotor(MotorDirection direction, float distanceOrSpeed);


//------------------------------------STOP ALL MOTORS------------------------------------
void stopAllMotors();