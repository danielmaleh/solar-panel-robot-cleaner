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

// Main loop
void loop() {
    Serial.println("LOOP");
    moveMotor(UP, 220);
    Serial.println("moveMotor1"); // do not work
    delay(1500);
    stopAllMotors();
    Serial.println("stopAllMotors1");
    delay(1500);
    // updateLEDs(WATER_FILLING);
    // Serial.println("updateLEDs1");
    // delay(1500);
    // moveMotor(LEFT, 5);
    // Serial.println("moveMotor3");
    // delay(1500);
    // stopAllMotors();
    // Serial.println("stopAllMotors3");
    // delay(1500);
    // updateLEDs(PROBLEM);
    // Serial.println("updateLEDs2"); 
    // delay(1500);
    // moveMotor(RIGHT, 5);
    // Serial.println("moveMotor4");
    // delay(1500);
    // stopAllMotors();
    // Serial.println("stopAllMotors4");
    // delay(1500);
    // updateLEDs(REST);
    // Serial.println("updateLEDs3");
    // delay(1500);
    // moveMotor(DOWN, 220);
    // Serial.println("moveMotor2");
    // delay(1500);
    // stopAllMotors();
    // Serial.println("stopAllMotors2");
    // delay(1500);
    // updateLEDs(RETURN_HOME);
    // Serial.println("updateLEDs4");
    // delay(1500);
    // controlBrushMotor(true, 230); // what is happening?
    // Serial.println("controlBrushMotor"); 
    // delay(1500);
    // updateLEDs(TRANSLATION);
    // Serial.println("updateLEDs5");
    // delay(1500);
    // controlValve(0);
    // Serial.println("controlValve");
    // delay(1500);
    // controlValve(180);
    // Serial.println("controlValve");
    // delay(1500);
    // checkButtonR();
    // Serial.println("checkButtonR");
    // delay(1500);
    // checkButtonC1();
    // Serial.println("checkButtonC1");
    // delay(1500);
    // checkButtonC2();
    // Serial.println("checkButtonC2");
    // delay(1500);
    // checkIRSensor();
    // Serial.println("checkIRSensor");
    // delay(1500);
    // checkRainSensor();
    // Serial.println("checkRainSensor");
    // delay(1500);
    // checkCurrent(CURRENT_PIN1);
    // Serial.println("checkCurrent1");
    // delay(1500);
    // checkCurrent(CURRENT_PIN2);
    // Serial.println("checkCurrent2");
    // delay(1500);
}

