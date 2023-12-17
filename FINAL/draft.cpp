#include "def_const_func.h"
#include <Time.h> 
#include <Servo.h>

enum MotorDirection {
    UP, DOWN, LEFT, RIGHT
};

//------------------------------------DC_GEAR------------------------------------
#define GEARBOX_MOTOR_PIN1 22
#define GEARBOX_MOTOR_PIN2 23
#define GEARBOX_MOTOR_SPEED_PIN 8

const int MOTOR_SPEED_GEAR = 255; // Gearbox motor speed
const float DC_MOTOR_PERIODE = 100.0; // Periode in milliseconds


//------------------------------------STEPPER------------------------------------
// GRADING:
// 20cm in 5 rev. = 4 cm per rev.
#define CM_PER_REVOLUTION 4.0 // 4 cm per revolution
#define STEPPER_STEPS_PER_REVOLUTION 200 //remove

#define STEPPER_DIR_PIN 2
#define STEPPER_STEP_PIN 3

const float STEPPER_PERIODE = 100.0; // Periode in milliseconds


void controlGearboxMotor(bool direction, int speed) {
    static unsigned long last_time = 0;
    if (millis() - last_time >= DC_MOTOR_PERIODE) {
        digitalWrite(GEARBOX_MOTOR_PIN1, direction ? HIGH : LOW);
        digitalWrite(GEARBOX_MOTOR_PIN2, direction ? LOW : HIGH);
        analogWrite(GEARBOX_MOTOR_SPEED_PIN, speed);
    }
}

void controlStepper(int distance, bool clockwise) {
    static unsigned long last_time = 0;
    if (millis() - last_time >= STEPPER_PERIODE) {
        digitalWrite(STEPPER_DIR_PIN, clockwise ? HIGH : LOW);

        int revolutions = distance / CM_PER_REVOLUTION;

        for (int i = 0; i < revolutions * STEPPER_STEPS_PER_REVOLUTION; i++) {
            digitalWrite(STEPPER_STEP_PIN, HIGH);
            delayMicroseconds(500);
            digitalWrite(STEPPER_STEP_PIN, LOW);
            delayMicroseconds(500);
        }
    }
}

void moveMotor(MotorDirection direction, float distanceOrSpeed) {
    switch (direction) {
        case UP:
            controlGearboxMotor(true, distanceOrSpeed); // Fixed speed
            break;
        case DOWN:
            controlGearboxMotor(false, distanceOrSpeed); // Fixed speed
            break;
        case LEFT:
            controlStepper(distanceOrSpeed, false); // Assuming false is left
            break;
        case RIGHT:
            controlStepper(distanceOrSpeed, true); // Assuming true is right
            break;
    }
}