// def_const_func.cpp
#include <Arduino.h>
#include "def_const_func.h"
#include <Time.h> 
#include <Servo.h>

//------------------------------------GOBAL DEFINITIONS------------------------------------
Servo myServo;  // Create a Servo object

int step = DIST / CLEANING_WIDTH + (DIST % CLEANING_WIDTH != 0); // In cm
int nb_cycles_counter = 0;
bool raining = false; // Boolean indicating if it is raining
bool IRseen = false; // Value of the IR sensor
unsigned long currentTime; // Actual time
unsigned long cleaningTime; // Time of the last cleaning
int stepperStartSpeed = 1000, stepperEndSpeed = 700, stepperAccelerationSteps = 300;
const float IR_PERIODE = 1.0; // Periode in milliseconds
const float RAIN_SENSOR_PERIODE = 1000.0; // Periode in milliseconds
const int MOTOR_SPEED_BRUSH = 200; // Brush motor speed
const int MOTOR_SPEED_GEAR = 200; // Gearbox motor speed
const float MOTOR_PERIODE = 100.0; // Periode in milliseconds
const float LED_PERIODE = 100.0; // Periode in milliseconds
const float CURRENT_PERIODE = 500.0; // Periode in milliseconds
const float BUTTON_PERIODE = 30.0; // Periode in milliseconds
const float VALVE_PERIODE = 1000.0; // Periode in milliseconds
bool buttonStateR = HIGH, buttonStateC1 = HIGH, buttonStateC2 = HIGH; // Current state of buttons. state is true when not pressed.

// Debouncing Variables
unsigned long lastDebounceTimeR = 0, lastDebounceTimeC1 = 0, lastDebounceTimeC2 = 0; // Last debounce time for buttons
bool lastButtonStateR = HIGH, lastButtonStateC1 = HIGH, lastButtonStateC2 = HIGH; // Previous state of buttons


State currentState = REST; // Initial state

//------------------------------------END GLOBAL DEFINITIONS------------------------------------

void initializeServo() {
    myServo.attach(SERVO_PIN);  // Attaches the servo on SERVO_PIN to the Servo object
}

void initializeMotors() {
    // Initialize gearbox motor pins
    pinMode(GEARBOX_MOTOR_PIN1, OUTPUT);
    pinMode(GEARBOX_MOTOR_PIN2, OUTPUT);
    pinMode(GEARBOX_MOTOR_SPEED_PIN, OUTPUT);

    // Initialize brush motor pins
    pinMode(BRUSH_MOTOR_PIN1, OUTPUT);
    pinMode(BRUSH_MOTOR_PIN2, OUTPUT);
    pinMode(BRUSH_MOTOR_SPEED_PIN, OUTPUT);
}

void initializeButtons() {
    pinMode(BUTTON_PIN_R, INPUT);
    pinMode(BUTTON_PIN_C1, INPUT);
    pinMode(BUTTON_PIN_C2, INPUT);
}

void initializeLEDPins() {
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(BLUE_LED_PIN, OUTPUT);
}

void initializeRainSensor() {
    pinMode(RAIN_SENSOR_PIN, INPUT);
}

void initializeStepperMotor() {
    pinMode(STEPPER_DIR_PIN, OUTPUT);
    pinMode(STEPPER_STEP_PIN, OUTPUT);
}

void initializeCurrentSensors() {
    pinMode(CURRENT_PIN1, INPUT);
    pinMode(CURRENT_PIN2, INPUT);
}

void initializeSerialCommunication() {
    Serial.begin(9600);
}


//------------------------------------IR------------------------------------
void checkIRSensor() {
    static unsigned long last_time = 0;
    if (millis() - last_time >= IR_PERIODE) {
        float irValue = analogRead(IR_SENSOR_PIN);
        if (irValue >= IR_THRESHOLD) {
            IRseen = true;
            Serial.println("Seen");
        } else {
            IRseen = false;
            Serial.println("Not Seen");
        }
        last_time = millis();
    }
}
//------------------------------------RAIN------------------------------------
void checkRainSensor() {
    static unsigned long last_time = 0;
    if (millis() - last_time >= RAIN_SENSOR_PERIODE) {
        int rainValue = analogRead(RAIN_SENSOR_PIN);
        if (rainValue <= RAIN_THRESHOLD) {
            Serial.println("Raining");
        } else {
            Serial.println("Not Raining");
        }
        Serial.println(rainValue);
        last_time = millis();
    }
}
//------------------------------------DC_BRUSH------------------------------------
void controlBrushMotor(bool direction, int speed) {
    digitalWrite(BRUSH_MOTOR_PIN1, direction ? HIGH : LOW);
    digitalWrite(BRUSH_MOTOR_PIN2, direction ? LOW : HIGH);
    analogWrite(BRUSH_MOTOR_SPEED_PIN, speed);

}
//------------------------------------DC_GEAR------------------------------------
void controlGearboxMotor(bool direction, int speed) {
    digitalWrite(GEARBOX_MOTOR_PIN1, direction ? HIGH : LOW);
    digitalWrite(GEARBOX_MOTOR_PIN2, direction ? LOW : HIGH);
    analogWrite(GEARBOX_MOTOR_SPEED_PIN, speed);

}
//------------------------------------STEPPER------------------------------------
void controlStepper(int distance, bool clockwise, int stepperStartSpeed, int stepperEndSpeed, int stepperAccelerationSteps) {
    digitalWrite(STEPPER_DIR_PIN, clockwise ? HIGH : LOW);

    int totalSteps = distance / CM_PER_REVOLUTION * STEPPER_STEPS_PER_REVOLUTION;
    int stepDelay = stepperStartSpeed;
    int stepChange = (stepperStartSpeed - stepperEndSpeed) / stepperAccelerationSteps;

    // Accelerate
    for (int i = 0; i < stepperAccelerationSteps && i < totalSteps; i++) {
        digitalWrite(STEPPER_STEP_PIN, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(STEPPER_STEP_PIN, LOW);
        delayMicroseconds(stepDelay);
        stepDelay -= stepChange;
    }

    // Constant speed
    for (int i = stepperAccelerationSteps; i < totalSteps - stepperAccelerationSteps; i++) {
        digitalWrite(STEPPER_STEP_PIN, HIGH);
        delayMicroseconds(stepperEndSpeed);
        digitalWrite(STEPPER_STEP_PIN, LOW);
        delayMicroseconds(stepperEndSpeed);
    }

    // Decelerate
    for (int i = totalSteps - stepperAccelerationSteps; i < totalSteps; i++) {
        digitalWrite(STEPPER_STEP_PIN, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(STEPPER_STEP_PIN, LOW);
        delayMicroseconds(stepDelay);
        stepDelay += stepChange;
    }
}

void controlStepper(int distance, bool clockwise) {
    digitalWrite(STEPPER_DIR_PIN, clockwise ? HIGH : LOW);

    int totalSteps = distance / CM_PER_REVOLUTION * STEPPER_STEPS_PER_REVOLUTION;

    // Constant speed
    for (int i = 0; i < totalSteps; i++) {
        digitalWrite(STEPPER_STEP_PIN, HIGH);
        delayMicroseconds(stepperEndSpeed);
        digitalWrite(STEPPER_STEP_PIN, LOW);
        delayMicroseconds(stepperEndSpeed);
    }
}
//------------------------------------LEDs------------------------------------
void updateLEDs(State currentState) {
    static unsigned long last_time = 0;
    if (millis() - last_time >= LED_PERIODE) {
        switch (currentState)
        {
        case REST:
            setColor(0,0,0,false);
            break;
        case WATER_FILLING:
            blueLED(true);
            break;
        case INITIAL_POSITION:
            whiteLED(false);
            break;
        case CLEANING:
            greenLED(true);
            break;
        case UP_TRAVEL:
            greenLED(false);
            break;
        case TRANSLATION:
            whiteLED(true);
            break;
        case RETURN_HOME:
            redLED(false);
            break;
        case PROBLEM:
            redLED(true);
            break;
        default:
            setColor(0,0,0,false);
            break;
        }
        last_time = millis();
    }
}

void redLED(bool flashing) {
    setColor(LED_INTENSITY, 0, 0, flashing);
}

void greenLED(bool flashing) {
    setColor(0, LED_INTENSITY, 0, flashing);
}

void blueLED(bool flashing) {
    setColor(0, 0, LED_INTENSITY, flashing);
}

void whiteLED(bool flashing) {
    setColor(LED_INTENSITY, LED_INTENSITY, LED_INTENSITY, flashing);
}

void setColor(int red, int green, int blue, bool flashing) {
    const int flashDuration = 500; // Duration of each flash
    if (flashing) {
        for (int i = 0; i < 5; i++) {
            writeColor(red, green, blue);
            delay(flashDuration);
            writeColor(0, 0, 0);
            delay(flashDuration);
        }
    } else {
        writeColor(red, green, blue);
    }
}

void writeColor(int red, int green, int blue) {
    analogWrite(RED_LED_PIN, red);
    analogWrite(GREEN_LED_PIN, green);
    analogWrite(BLUE_LED_PIN, blue);
}
//------------------------------------CURRENT------------------------------------
void checkCurrent(int currentPin) {
    static unsigned long last_time = 0;
    if (millis() - last_time >= CURRENT_PERIODE) {
        float currentVal = analogRead(currentPin);
        currentVal = (2.5 - (currentVal * (5.0 / 1024.0))) / CURRENT_SENSITIVITY;
        currentVal = abs(currentVal);
        Serial.print("Current Value: ");
        Serial.print(currentVal, 5);
        Serial.println(" [A]");
        if (currentPin == CURRENT_PIN1)
        {
            if (currentVal >= CURRENT_THRESHOLD_GEARBOX) {
                Serial.println("Current threshold exceeded gearbox");
                currentState = PROBLEM;
            }
        }
        else if (currentPin == CURRENT_PIN2)
        {
            if (currentVal >= CURRENT_THRESHOLD_BRUSH) {
                Serial.println("Current threshold exceeded brush");
                currentState = PROBLEM;
            }
        }
        else
        {
            Serial.println("Error: Current pin not defined");
        }
        last_time = millis();
    }
}
//------------------------------------BUTTONS------------------------------------
void checkButtonR() {
    static unsigned long last_time = 0;
    if (millis() - last_time >= BUTTON_PERIODE) {
        // Debounce button 1
        bool readingR = digitalRead(BUTTON_PIN_R);
        if (readingR != lastButtonStateR) {
            lastDebounceTimeR = millis();
        }
        if ((millis() - lastDebounceTimeR) > DEBOUNCE_DELAY) {
            if (readingR != buttonStateR) {
                buttonStateR = readingR;
                Serial.println("Button R changed");
            }
        }
        lastButtonStateR = readingR;
        last_time = millis();
    }
}

void checkButtonC2() {
    static unsigned long last_time = 0;
    if (millis() - last_time >= BUTTON_PERIODE) {
        // Debounce button 1
        bool readingC2 = digitalRead(BUTTON_PIN_C2);
        if (readingC2 != lastButtonStateC2) {
            lastDebounceTimeC2 = millis();
        }
        if ((millis() - lastDebounceTimeC2) > DEBOUNCE_DELAY) {
            if (readingC2 != buttonStateC2) {
                buttonStateC2 = readingC2;
                Serial.println("Button C2 changed");    
            }
        }
        lastButtonStateC2 = readingC2;
        last_time = millis();
    }
}

void checkButtonC1() {
    static unsigned long last_time = 0;
    if (millis() - last_time >= BUTTON_PERIODE) {
        // Debounce button 1
        bool readingC1 = digitalRead(BUTTON_PIN_C1);
        if (readingC1 != lastButtonStateC1) {
            lastDebounceTimeC1 = millis();
        }
        if ((millis() - lastDebounceTimeC1) > DEBOUNCE_DELAY) {
            if (readingC1 != buttonStateC1) {
                buttonStateC1 = readingC1;
                Serial.println("Button C1 changed");
            }
        }
        lastButtonStateC1 = readingC1;
        last_time = millis();
    }
}
//------------------------------------VALVE (Servo)------------------------------------
void controlValve(int angle) {
    static unsigned long last_time = 0;
    if (millis() - last_time >= VALVE_PERIODE) {
        rotateServo(angle);
        last_time = millis();
    }
}

void rotateServo(int angle) {
    // Convert angle to microseconds
    if (angle >= 0 && angle <= 180) {
        int pulseWidth = map(angle, 0, 180, 1000, 2000); // Map angle to microseconds
        myServo.writeMicroseconds(pulseWidth);
        Serial.print("Angle: ");
        Serial.println(angle);
    } else {
        Serial.println("Angle out of range");
    }
}
//------------------------------------MOTORS MOUVEMENT------------------------------------
void moveMotor(MotorDirection direction, float distanceOrSpeed) {
    static unsigned long last_time = 0;
    if (millis() - last_time >= MOTOR_PERIODE) {
        switch (direction) {
            case UP:
                controlGearboxMotor(true, distanceOrSpeed); // Fixed speed
                break;
            case DOWN:
                controlGearboxMotor(false, distanceOrSpeed); // Fixed speed
                break;
            case LEFT:
                controlStepper(distanceOrSpeed, false, stepperStartSpeed, stepperEndSpeed, stepperAccelerationSteps); // Assuming false is left 
                controlStepper(distanceOrSpeed, false); // Assuming false is left 
                break;
            case RIGHT:
                controlStepper(distanceOrSpeed, true, stepperStartSpeed, stepperEndSpeed, stepperAccelerationSteps); // Assuming true is right
                controlStepper(distanceOrSpeed, true); // Assuming true is right
                break;
        }
        last_time = millis();
    }
}
//------------------------------------STOP ALL MOTORS------------------------------------
void stopAllMotors() {
    // Stop DC brush motor
    digitalWrite(BRUSH_MOTOR_PIN1, LOW);
    digitalWrite(BRUSH_MOTOR_PIN2, LOW);
    analogWrite(BRUSH_MOTOR_SPEED_PIN, 0);

    // Stop Gearbox motor
    digitalWrite(GEARBOX_MOTOR_PIN1, LOW);
    digitalWrite(GEARBOX_MOTOR_PIN2, LOW);
    analogWrite(GEARBOX_MOTOR_SPEED_PIN, 0);

    // Stop Stepper motor (stop sending pulses)
    digitalWrite(STEPPER_STEP_PIN, LOW);
}
