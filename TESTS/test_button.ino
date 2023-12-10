#include <Arduino.h>

const int ButtonPin = A2; // Button connected to pin A2
const int debounceDelay = 50; // Debounce delay in milliseconds

float lastButtonVal = 0;
unsigned long lastDebounceTime = 0; // Timestamp to store last debounce time

void setup() {
  pinMode(ButtonPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  float currentButtonVal = analogRead(ButtonPin);

  // Check if button state has changed
  if (currentButtonVal != lastButtonVal) {
    // Reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If the button state has not changed for a period longer than the debounce delay, consider the reading stable
    if (currentButtonVal != lastButtonVal) {
      lastButtonVal = currentButtonVal;

      // Print the stable button value
      Serial.print("Button value: "); 
      Serial.println(lastButtonVal, 5);
      // released = 1023 and pressed = 0.
    }
  }

  delay(100); // Short delay for sensor stability
}
