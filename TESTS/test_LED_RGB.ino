// Define the pins for the RGB LED
const int redPin = 11;   // Red LED, connected to digital pin 11
const int greenPin = 10; // Green LED, connected to digital pin 10
const int bluePin = 9;   // Blue LED, connected to digital pin 9

void setup() {
  // Set the RGB LED pins as output
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
}

void loop() {
  // Red color
  setColor(255, 0, 0); // Red
  analogWrite(redPin, 200);
  delay(1000);

  // Green color
  setColor(0, 255, 0); // Green
  delay(1000);

  // Blue color
  setColor(0, 0, 255); // Blue
  delay(1000);

  // White (all colors)
  setColor(255, 255, 255); // White
  delay(1000);

  // Off (no color)
  setColor(0, 0, 0); // Off
  delay(1000);
}

void setColor(int red, int green, int blue) {
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
}
