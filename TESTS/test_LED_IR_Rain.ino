#include <def_const_func.h>

void setup() {
  initializeLEDPins();
  initializeSerialCommunication();
}

void loop() {
  checkIRSensor();
  checkRainSensor();

  // Example usage of setColor function
  redLED(false); // Solid Red
  delay(1000);

  greenLED(true); // Flashing Green
  delay(2000); // Increased delay to observe flashing
}