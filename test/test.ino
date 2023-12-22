#include <Time.h> 
#include "def_const_func.h"
int cont = 0; 
int ir_thresh = 0; 

void setup() {
    //void initializeSerialCommunication();
    Serial.begin(9600);
    Serial.println("SETUP8");
    //myServo.attach(SERVO_PIN);
    //controlValve(180);
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

void loop() {
  //FINAL DEMO
  delay(2000);
  //moveMotor(DOWN,0); 
  //delay(3000000000000); 
  if (cont == 0) { 
    calibrate_ir();
    delay(1000); 
    // INITIAL POSITION
    moveMotor(LEFT, 11);
    // 10 rev = 41 cm 
    // 43.5 cm de plateforme a première position 
    stopAllMotors();
    delay(1000); 
    float switchr = digitalRead(BUTTON_PIN_R);
    while (switchr == 1) {
      moveMotor(UP,0);
      switchr = digitalRead(BUTTON_PIN_R);
    }
    stopAllMotors();
    delay(1000);
    // CLEANING 
    moveMotor(DOWN,0);
    controlBrushMotor(true); 
    //controlValve(0);
    switchr = digitalRead(BUTTON_PIN_R);
    while (switchr == 0) {
      moveMotor(DOWN,0);
      controlBrushMotor(true);
      switchr = digitalRead(BUTTON_PIN_R);
      Serial.println(switchr);
    }
    digitalWrite(BRUSH_MOTOR_PIN1, LOW);
    digitalWrite(BRUSH_MOTOR_PIN2, LOW);
    //controlValve(180);
    delay(200);
    digitalWrite(GEARBOX_MOTOR_PIN1, LOW);
    digitalWrite(GEARBOX_MOTOR_PIN2, LOW);
    // UP TRAVEL
    float ir_value = analogRead(IR_SENSOR_PIN);
    while (ir_value < ir_thresh) {
      moveMotor(UP, 0);
      ir_value = analogRead(IR_SENSOR_PIN);
      Serial.println(ir_value);
    }
    stopAllMotors();
    cont = cont + 1; 
  }
  if (cont == 6) {
    // TRANSLATION TO LAST POSITION
    float switchr = digitalRead(BUTTON_PIN_C1);
    digitalWrite(STEPPER_SLEEP_PIN, HIGH);
    delayMicroseconds(2);
    digitalWrite(STEPPER_DIR_PIN, HIGH);
    while (switchr == 1) {
      digitalWrite(STEPPER_STEP_PIN, HIGH);
      delayMicroseconds(1000);
      digitalWrite(STEPPER_STEP_PIN, LOW);
      delayMicroseconds(1000);
      switchr = digitalRead(BUTTON_PIN_C1);
    }
    digitalWrite(STEPPER_SLEEP_PIN, LOW);
    delay(1000); 
    switchr = digitalRead(BUTTON_PIN_R);
    while (switchr == 1) {
      moveMotor(UP,0);
      switchr = digitalRead(BUTTON_PIN_R);
    }
    stopAllMotors();
    delay(1000); 
    // CLEANING
    moveMotor(DOWN,0);
    controlBrushMotor(true); 
    //controlValve(0);
    switchr = digitalRead(BUTTON_PIN_R);
    while (switchr == 0) {
      moveMotor(DOWN,0);
      controlBrushMotor(true);
      switchr = digitalRead(BUTTON_PIN_R);
      Serial.println(switchr);
    }
    digitalWrite(BRUSH_MOTOR_PIN1, LOW);
    digitalWrite(BRUSH_MOTOR_PIN2, LOW);
    //controlValve(180);
    delay(200);
    digitalWrite(GEARBOX_MOTOR_PIN1, LOW);
    digitalWrite(GEARBOX_MOTOR_PIN2, LOW);
    delay(1000); 
    // UP TRAVEL
    float ir_value = analogRead(IR_SENSOR_PIN);
    while (ir_value < ir_thresh) {
      moveMotor(UP, 0);
      ir_value = analogRead(IR_SENSOR_PIN);
      Serial.println(ir_value);
    }
    stopAllMotors();
    cont = cont + 1; 
    delay(1000);
    // GO BACK HOME 
    switchr = digitalRead(BUTTON_PIN_Chome);
    digitalWrite(STEPPER_SLEEP_PIN, HIGH);
    delayMicroseconds(2);
    digitalWrite(STEPPER_DIR_PIN, LOW);
    while (switchr == 1) {
      digitalWrite(STEPPER_STEP_PIN, HIGH);
      delayMicroseconds(1000);
      digitalWrite(STEPPER_STEP_PIN, LOW);
      delayMicroseconds(1000);
      switchr = digitalRead(BUTTON_PIN_Chome);
    }
    digitalWrite(STEPPER_SLEEP_PIN, LOW);
    // END PROGRAM
    delay(300000000); 
    exit; 
  }
  delay(1000);
  // TRANSLATION 
  moveMotor(LEFT, 7);
  stopAllMotors();
  delay(1000); 
  float switchr = digitalRead(BUTTON_PIN_R);
  while (switchr == 1) {
    moveMotor(UP,0);
    switchr = digitalRead(BUTTON_PIN_R);
  }
  stopAllMotors();
  delay(1000); 
  // CLEANING
  moveMotor(DOWN,0);
  controlBrushMotor(true);
  //controlValve(0); 
  switchr = digitalRead(BUTTON_PIN_R);
  while (switchr == 0) {
    moveMotor(DOWN,0);
    controlBrushMotor(true);
    switchr = digitalRead(BUTTON_PIN_R);
    Serial.println(switchr);
  }
  digitalWrite(BRUSH_MOTOR_PIN1, LOW);
  digitalWrite(BRUSH_MOTOR_PIN2, LOW);
  //controlValve(180);
  delay(200);
  digitalWrite(GEARBOX_MOTOR_PIN1, LOW);
  digitalWrite(GEARBOX_MOTOR_PIN2, LOW); 
  delay(1000); 
  // UP TRAVEL
  float ir_value = analogRead(IR_SENSOR_PIN);
  while (ir_value < ir_thresh) {
    moveMotor(UP, 0);
    ir_value = analogRead(IR_SENSOR_PIN);
    Serial.println(ir_value);
  }
  stopAllMotors();
  cont = cont + 1; 
  delay(1000);  
  // GO TO NEXT POSITION
  //END FINAL DEMO
}

void calibrate_ir () {
  int mean = 0; 
  for (int i = 0; i < 10; i++) {
    float ir_value = analogRead(IR_SENSOR_PIN);
    Serial.println(ir_value);
    mean = mean + ir_value; 
    Serial.println(mean); 
    delay(500);
  }
  ir_thresh = mean/10; 
  ir_thresh = ir_thresh - 25; 
  Serial.println(ir_thresh); 
} 

