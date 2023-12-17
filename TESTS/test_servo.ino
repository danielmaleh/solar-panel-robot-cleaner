#include <Servo.h>

#define SERVO_PIN 9 // Define the pin connected to the servo

Servo myServo;  // Create a Servo object

void initializeServo() {
    myServo.attach(SERVO_PIN);  // Attaches the servo on SERVO_PIN to the Servo object
}

void rotateServo(int angle) {
    // Check if the angle is within the servo's range
    if (angle >= 0 && angle <= 180) {
        myServo.write(angle);  // Tell servo to go to position in variable 'angle'
    } else {
        Serial.println("Angle out of range");
    }
}

void setup() {
    Serial.begin(9600);
    initializeServo();
    // Other initializations...
}

void loop() {
    // Example usage: Rotate servo to 90 degrees
    rotateServo(90);
    delay(1000);

    // Rotate back to 0 degrees
    rotateServo(0);
    delay(1000);

    // Additional logic...
}
