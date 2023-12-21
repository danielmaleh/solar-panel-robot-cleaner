#include <Time.h> 
#include "def_const_func.h"

// Setup and initialization
void setup() {
    //void initializeSerialCommunication();
    Serial.begin(9600);
    Serial.println("SETUP8");
    myServo.attach(SERVO_PIN);
    Serial.println("SETUP1");
    void initializeMotors();
    Serial.println("SETUP2");
    void initializeButtons();
    Serial.println("SETUP3");
    void initializeLEDPins();
    Serial.println("SETUP4");
    void initializeRainSensor();
    Serial.println("SETUP5");
    void initializeStepperMotor();
    Serial.println("SETUP6");
    void initializeCurrentSensors();
    Serial.println("SETUP7");
}

// UP,
// WATER_FILLING,
// LEFT,
// PROBLEM,
// RIGHT,
// REST,
// DOWN,
// RETURN_HOME,
// BRUSH,
// TRANSLATION,
// VALVE,
// BUTTONR,
// BUTTONCHOME,
// BUTONC1,
// IR,
// RAIN,
// CURRENT_PIN1,
// CURRENT_PIN2

// Main loop
void loop() {
    // delay(5000);
    // Serial.println("LOOP");
    // moveMotor(UP, 250);
    // Serial.println("moveMotor1"); 
    // delay(1000);
    // stopAllMotors();
    // Serial.println("stopAllMotors1");
    // delay(1000);
    // updateLEDs(WATER_FILLING);
    // Serial.println("updateLEDs1");
    // delay(1000);
    // moveMotor(LEFT, 5);
    // Serial.println("moveMotor3");
    // delay(1000);
    // stopAllMotors();
    // Serial.println("stopAllMotors3");
    // delay(1000);
    // updateLEDs(PROBLEM);
    // Serial.println("updateLEDs2"); 
    // delay(1000);
    // moveMotor(RIGHT, 5);
    // Serial.println("moveMotor4");
    // delay(1000);
    // stopAllMotors();
    // Serial.println("stopAllMotors4");
    // delay(1000);
    // updateLEDs(REST);
    // Serial.println("updateLEDs3");
    // delay(1000);
    moveMotor(DOWN, 250);
    Serial.println("moveMotor2");
    delay(1000);
    stopAllMotors();
    Serial.println("stopAllMotors2");
    delay(1000);
    // updateLEDs(RETURN_HOME);
    // Serial.println("updateLEDs4");
    // delay(1000);
    // controlBrushMotor(true); // what is happening? what direction?
    // Serial.println("controlBrushMotor"); 
    // delay(1000);
    // updateLEDs(TRANSLATION);
    // Serial.println("updateLEDs5");
    // delay(1000);
    // controlValve(0);
    // Serial.println("controlValve");
    // delay(1000);
    // controlValve(180);
    // Serial.println("controlValve");
    // delay(1000);
    // checkButtonR();
    // Serial.println("checkButtonR");
    // delay(1000);
    // checkButtonC1();
    // Serial.println("checkButtonC1");
    // delay(1000);
    // checkButtonChome();
    // Serial.println("checkButtonChome");
    // delay(1000);
    checkIRSensor();
    Serial.println("checkIRSensor");
    delay(1000);
    // checkRainSensor();
    // Serial.println("checkRainSensor");
    // delay(1000);
    // checkCurrent(CURRENT_PIN1);
    // Serial.println("checkCurrent1");
    // delay(1000);
    // checkCurrent(CURRENT_PIN2);
    // Serial.println("checkCurrent2");
    // delay(1000);
}

