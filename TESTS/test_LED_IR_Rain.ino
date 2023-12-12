#define IR_THRESHOLD 500 // IR threshold value

const int irSensorPin = A0; // IR sensor pin
const int rainSensorPin = A1; // Raindrop sensor connected to analog pin A1

// Define the pins for the RGB LED
const int ledPins[] = {11, 10, 9}; // Red, Green, Blue

void setup() {
  for (int i = 0; i < 3; i++) {
    pinMode(ledPins[i], OUTPUT);
  }
  Serial.begin(9600);
}

void loop() {
  float irValue = analogRead(irSensorPin); // Read value from IR sensor

  // Check if the IR value crosses the threshold
  if (irValue >= IR_THRESHOLD) {
    Serial.println("Seen");
  } else {
    Serial.println("Not Seen");
  }

  int rainValue = analogRead(rainSensorPin); // Read value from rain sensor

  // Print rain sensor value to serial monitor
  Serial.print("Rain Sensor Value: ");
  Serial.println(rainValue);

  // Example usage of setColor function
  setColor(255, 0, 0, false); // Solid Red
  delay(1000);

  setColor(0, 255, 0, true); // Flashing Green
  delay(2000); // Increased delay to observe flashing

  setColor(0, 0, 255, false); // Solid Blue
  delay(1000);

  setColor(255, 255, 255, true); // Flashing White
  delay(2000); // Increased delay to observe flashing

  setColor(0, 0, 0, false); // Off
  delay(500);
}

void setColor(int red, int green, int blue, bool flashing) {
  const int flashDuration = 500; // Duration of each flash
  if (flashing) {
    for (int i = 0; i < 5; i++) { // Flash 5 times
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
  analogWrite(ledPins[0], red);
  analogWrite(ledPins[1], green);
  analogWrite(ledPins[2], blue);
}
