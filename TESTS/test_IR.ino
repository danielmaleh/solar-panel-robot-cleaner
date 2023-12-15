const int irSensorPin = A0; // IR sensor pin
const int threshold = 500; // Set this to your desired threshold value

void setup() {
  Serial.begin(9600);
}

void loop() {
  float irValue = analogRead(irSensorPin); // Read value from IR sensor

  // Check if the IR value crosses the threshold
  if (irValue >= threshold) {
    Serial.println("Seen");
  } else {
    Serial.println("Not Seen");
  }

  delay(500);
}
