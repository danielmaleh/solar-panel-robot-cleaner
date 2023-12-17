#include "def_const_func.h"

void setup() {
    initializeServo();
    initializeButtons();
    initializeRainSensor();
    Serial.begin(9600);
}

void loop() {
    if (buttonStateR){
      controlValve(45);
    }
    controlValve(0);
    checkRainSensor();
    checkButtonR();
    Serial.println(buttonStateR);
    delay(100);
}
