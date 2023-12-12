

const int rainSensorPin = A0; // Raindrop sensor connected to analog pin A0


void setup() {

}


void loop() {
  int rainValue = analogRead(rainSensorPin); // Read value from rain sensor

  // Print rain sensor value to serial monitor
  Serial.print("Rain Sensor Value: ");
  Serial.println(rainValue);
}