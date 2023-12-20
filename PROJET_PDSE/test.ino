#include <Time.h> 
#include "def_const_func.h" 

// Setup and initialization
void setup() {
    //void initializeSerialCommunication();
    Serial.begin(9600);
    Serial.println("SETUP8");
    void initializeServo();
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
    moveMotor(UP, 5);
    Serial.println("moveMotor1");
    delay(2000);
    stopAllMotors();
    Serial.println("stopAllMotors1");
    delay(2000);
    moveMotor(DOWN, 5);
    Serial.println("moveMotor2");
    delay(2000);
    stopAllMotors();
    Serial.println("stopAllMotors2");
    delay(2000);
    moveMotor(LEFT, 5);
    Serial.println("moveMotor3");
    delay(2000);
    stopAllMotors();
    Serial.println("stopAllMotors3");
    delay(2000);
    moveMotor(RIGHT, 5);
    Serial.println("moveMotor4");
    delay(2000);
    stopAllMotors();
    Serial.println("stopAllMotors4");
    delay(2000);
    updateLEDs(WATER_FILLING);
    Serial.println("updateLEDs1");
    delay(2000);
    updateLEDs(PROBLEM);
    Serial.println("updateLEDs2");
    delay(2000);
    updateLEDs(REST);
    Serial.println("updateLEDs3");
    delay(2000);
    updateLEDs(RETURN_HOME);
    Serial.println("updateLEDs4");
    delay(2000);
    updateLEDs(TRANSLATION);
    Serial.println("updateLEDs5");
    delay(2000);
    checkButtonR();
    Serial.println("checkButtonR");
    delay(2000);
    checkButtonC1();
    Serial.println("checkButtonC1");
    delay(2000);
    checkButtonC2();
    Serial.println("checkButtonC2");
    delay(2000);
    checkIRSensor();
    Serial.println("checkIRSensor");
    delay(2000);
    checkRainSensor();
    Serial.println("checkRainSensor");
    delay(2000);
    checkCurrent(CURRENT_PIN1);
    Serial.println("checkCurrent1");
    delay(2000);
    checkCurrent(CURRENT_PIN2);
    Serial.println("checkCurrent2");
    delay(2000);
    controlBrushMotor(true, 5);
    Serial.println("controlBrushMotor");
    delay(2000);
    controlGearboxMotor(true, 5);
    Serial.println("controlGearboxMotor");
    delay(2000);
    controlStepper(5, true, 5, 5, 5);
    Serial.println("controlStepper1");
    delay(2000);
    controlStepper(5, true);
    Serial.println("controlStepper2");
    delay(2000);
    controlValve(90);
    Serial.println("controlValve");
    delay(2000);
    rotateServo(45);
    Serial.println("rotateServo");
    delay(2000);
}

