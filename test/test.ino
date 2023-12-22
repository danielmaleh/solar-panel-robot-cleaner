#include <Time.h> 
#include "def_const_func.h"
int cont = 0; 
// Setup and initialization
void setup() {
    //void initializeSerialCommunication();
    Serial.begin(9600);
    Serial.println("SETUP8");
    myServo.attach(SERVO_PIN);
    controlValve(180);
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
  //TEST UTILE
/*   delay(2000); 
  moveMotor(DOWN,0);
  controlBrushMotor(true); 
  float switchr = digitalRead(BUTTON_PIN_R);
  while (switchr == 0) {
    moveMotor(DOWN,0);
    controlBrushMotor(true);
    switchr = digitalRead(BUTTON_PIN_R);
  }
  stopAllMotors();
  delay(4000); 
  float ir_value = analogRead(IR_SENSOR_PIN);
  while (ir_value < 600) {
    moveMotor(UP, 0);
    ir_value = analogRead(IR_SENSOR_PIN);
    Serial.println(ir_value);
  }
  stopAllMotors();
  delay(5000); 
  moveMotor(LEFT, 5);
  switchr = digitalRead(BUTTON_PIN_R);
  while (switchr == 1) {
    moveMotor(UP,0);
    switchr = digitalRead(BUTTON_PIN_R);
  }
  stopAllMotors();
  delay(30000);  */
  //FIN TEST UTILE 

  // IR AFFICHAGE
  /* float ir_value = analogRead(IR_SENSOR_PIN);
  Serial.println(ir_value);
  delay(500); */
  //FIN IR 


  //TEST ARRET BAS DU PANNEAU
/*   delay(2000); 
  //controlBrushMotor(true);
  //delay(4000); 
  float switchr = digitalRead(BUTTON_PIN_R);
  while (switchr == 0) {
    moveMotor(DOWN,0);
    switchr = digitalRead(BUTTON_PIN_R);
  }
  stopAllMotors();
  delay(30000);  */ 
  //FIN TEST

  // TEST BOUT A LAUTRE
  /* delay(2000); 
  float ir_value = analogRead(IR_SENSOR_PIN);
  Serial.println(ir_value);
  while (ir_value < 625) {
    moveMotor(UP, MOTOR_SPEED_GEAR);
    ir_value = analogRead(IR_SENSOR_PIN);
  }
  stopAllMotors();
  digitalWrite(STEPPER_SLEEP_PIN, HIGH);
  delayMicroseconds(2);
  float switchr = digitalRead(BUTTON_PIN_Chome);
  digitalWrite(STEPPER_DIR_PIN, LOW);
  while (switchr == 1) {
    digitalWrite(STEPPER_STEP_PIN, HIGH);
    delayMicroseconds(1000);
    digitalWrite(STEPPER_STEP_PIN, LOW);
    delayMicroseconds(1000);
    switchr = digitalRead(BUTTON_PIN_Chome);
  }
  delay(1000); 
  digitalWrite(STEPPER_DIR_PIN, HIGH);
  switchr = digitalRead(BUTTON_PIN_C1);
  while (switchr == 1) {
    digitalWrite(STEPPER_STEP_PIN, HIGH);
    delayMicroseconds(1000);
    digitalWrite(STEPPER_STEP_PIN, LOW);
    delayMicroseconds(1000);
    switchr = digitalRead(BUTTON_PIN_C1);
  }
  digitalWrite(STEPPER_SLEEP_PIN, LOW);
  delay(1000);  */
  // FIN BOUT A LAUTRE

  //FINAL DEMO
  delay(2000);
  //moveMotor(UP,0); 
  //delay(3000000); 
  if (cont == 0) {
    delay(2000); 
    moveMotor(LEFT, 11);
    // 10 rev = 41 cm 
    // 43.5 cm de plateforme a première position 
    stopAllMotors();
    delay(3000); 
    float switchr = digitalRead(BUTTON_PIN_R);
    while (switchr == 1) {
      moveMotor(UP,0);
      switchr = digitalRead(BUTTON_PIN_R);
    }
    delay(2000); 
    moveMotor(DOWN,0);
    controlBrushMotor(true); 
    controlValve(0);
    switchr = digitalRead(BUTTON_PIN_R);
    while (switchr == 0) {
      moveMotor(DOWN,0);
      controlBrushMotor(true);
      switchr = digitalRead(BUTTON_PIN_R);
      Serial.println(switchr);
    }
    controlValve(180);
    stopAllMotors();
    delay(4000); 
    float ir_value = analogRead(IR_SENSOR_PIN);
    while (ir_value < 600) {
      moveMotor(UP, 0);
      ir_value = analogRead(IR_SENSOR_PIN);
      Serial.println(ir_value);
    }
    stopAllMotors();
    cont = cont + 1; 
  }
  if (cont == 6) {
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
    delay(2000); 
    moveMotor(DOWN,0);
    controlBrushMotor(true); 
    controlValve(0);
    switchr = digitalRead(BUTTON_PIN_R);
    while (switchr == 0) {
      moveMotor(DOWN,0);
      controlBrushMotor(true);
      switchr = digitalRead(BUTTON_PIN_R);
      Serial.println(switchr);
    }
    stopAllMotors();
    controlValve(180);
    delay(4000); 
    float ir_value = analogRead(IR_SENSOR_PIN);
    while (ir_value < 600) {
      moveMotor(UP, 0);
      ir_value = analogRead(IR_SENSOR_PIN);
      Serial.println(ir_value);
    }
    stopAllMotors();
    cont = cont + 1; 
    delay(1000); 
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
    delay(300000000); 
    exit; 
  }
  delay(2000); 
  moveMotor(LEFT, 7);
  stopAllMotors();
  delay(3000); 
  float switchr = digitalRead(BUTTON_PIN_R);
  while (switchr == 1) {
    moveMotor(UP,0);
    switchr = digitalRead(BUTTON_PIN_R);
  }
  delay(2000); 
  moveMotor(DOWN,0);
  controlBrushMotor(true);
  controlValve(0); 
  switchr = digitalRead(BUTTON_PIN_R);
  while (switchr == 0) {
    moveMotor(DOWN,0);
    controlBrushMotor(true);
    switchr = digitalRead(BUTTON_PIN_R);
    Serial.println(switchr);
  }
  stopAllMotors();
  controlValve(180);
  delay(4000); 
  float ir_value = analogRead(IR_SENSOR_PIN);
  while (ir_value < 600) {
    moveMotor(UP, 0);
    ir_value = analogRead(IR_SENSOR_PIN);
    Serial.println(ir_value);
  }
  stopAllMotors();
  cont = cont + 1; 
  delay(2000);  
  //END FINAL DEMO

  /* delay(3000); 
  controlValve(0); // 0 ouverte
  delay(3000); 
  controlValve(180);// 180 fermée
  delay(3000); */
}

