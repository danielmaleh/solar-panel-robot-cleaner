#include <Time.h> 
#include "def_const_func.h" 

// Setup and initialization
void setup() {
    Serial.begin(9600);
    Serial.println("SETUP0");
    myServo.attach(SERVO_PIN);
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
    checkButtonChome();
    if (buttonStateChome == RELEASED) {
        currentState = RETURN_HOME;
    }
}

// Main loop
void loop() {
    if (currentState == PROBLEM) {
        updateLEDs(currentState);
        return;
    }

    if (buttonStateC1 == CLICKED){ // A voir si dernier netoyage apres ou avant clic
        Serial.println("IF17");
        stopAllMotors();
        currentState = RETURN_HOME;
        if (currentState != TRANSLATION) {
            Serial.println("IF0");
            currentState = PROBLEM;
        }
    }

    Serial.println("LOOP");
    // Update the current time
    currentTime = millis();
    Serial.println("1");
    updateLEDs(currentState);
    checkCurrent(CURRENT_PIN1);
    Serial.println("2");
    checkCurrent(CURRENT_PIN2);
    Serial.println("3");
    checkButtonR();
    Serial.println("4");
    checkButtonC1();
    Serial.println("5");
    checkButtonChome();
    Serial.println("6");
    checkIRSensor();
    Serial.println("7");

    // Check the states and perform actions based on the current state
    switch (currentState) {
        case REST:
            Serial.println("REST");
            checkRainSensor();
            if (buttonStateR == CLICKED) {
                Serial.println("IF1");
                currentState = PROBLEM;
            }
            if (IRseen == false || buttonStateChome == RELEASED) { 
                Serial.println("IF2");
                currentState = RETURN_HOME;
            }
            else if ((currentTime >= cleaningTime + CLEANING_PERIOD) || raining) {
                Serial.println("IF3");
                if (!raining) {
                    Serial.println("IF4");
                    currentState = WATER_FILLING;
                }
                else {
                    Serial.println("IF5");
                    currentState = INITIAL_POSITION;
                }
                cleaningTime = currentTime; // Update the last cleaning time
            }
            else {
                Serial.println("IF6");
                currentState = REST;
            }
        break;
        case WATER_FILLING:
            Serial.println("WATER_FILLING");
            // If not HOME then RETURN_HOME
            if (buttonStateR == CLICKED) {
                Serial.println("IF7");
                currentState = PROBLEM;
            }
            if (IRseen == false || buttonStateChome == RELEASED) {
                Serial.println("IF8");
                currentState = RETURN_HOME;
            }
            // after filling done state = Initial_position
            else {
                Serial.println("IF9");
                // fill water reservoir 
                delay(5000);
                currentState = INITIAL_POSITION;
            }
        break;
        case INITIAL_POSITION:
            Serial.println("INITIAL_POSITION");
            // Logic to move to the initial position
            // steps to 1st position and when finished state = CLEANING
            moveMotor(LEFT, DIST_TO_INITIAL_POSITION);
            stopAllMotors();
            currentState = CLEANING;
        break;
        case CLEANING:
            Serial.println("CLEANING");
            // Logic to clean the panels
            // If buttonR = 1  state = UP_TRAVEL, both for start and finish. delay for the finish.
            // Else go downward with dc motor and activate dc brush motor and valve (until buttonR=1).
            if (buttonStateR == RELEASED) {
                Serial.println("IF10");
                // Stop Brush motor
                digitalWrite(BRUSH_MOTOR_PIN1, LOW);
                digitalWrite(BRUSH_MOTOR_PIN2, LOW);
                controlValve(VALVE_ANGLE_CLOSE);
                delay(END_OF_CLEANING_DELAY);
                stopAllMotors();
                currentState = UP_TRAVEL;
            }
            else {
                Serial.println("IF11");
                moveMotor(DOWN, MOTOR_SPEED_GEAR);
                controlBrushMotor(HIGH);
                controlValve(VALVE_ANGLE_OPEN);
            }
        break;
        case UP_TRAVEL:
            Serial.println("UP_TRAVEL");
            // Logic to move up to the next position
            // go upward with dc motor until IR sensor detects the robot
            // nb_cycles = nb_cycles + 1
            // if nb of cycles = N state = RETURN_HOME else state = TRANSLATION
            if (buttonStateR == CLICKED) {
                Serial.println("IF12");
                currentState = PROBLEM;
            }
            else if (IRseen == false) {
                Serial.println("IF13");
                moveMotor(UP, MOTOR_SPEED_GEAR);
            }
            else {
                Serial.println("IF14");
                stopAllMotors();
                nb_cycles_counter = nb_cycles_counter + 1;
                if (nb_cycles_counter*step == DIST) {
                    Serial.println("IF15");
                    currentState = RETURN_HOME;
                    break;
                }
                currentState = TRANSLATION;
                stopAllMotors();
                moveMotor(LEFT, step);
            }
        break;
        case TRANSLATION:
            Serial.println("TRANSLATION");
            // Logic to translate to the next position
            // go sidways with stepper motor for step
            // when stepper motor finished move upwards with dc motor until buttonR=0 and then state = CLEANING
            if (buttonStateR == RELEASED) {
                Serial.println("IF16");
                moveMotor(UP, MOTOR_SPEED_GEAR);
            }
            else {
                Serial.println("IF18");
                stopAllMotors();
                currentState = CLEANING;
            }
        break;
        case RETURN_HOME:
            Serial.println("RETURN_HOME");
            // Logic to return to the initial position
            // if IRseen is false move upward with motor dc until IRseen is true
            // if buttonChome=1 move sidways with motor stepper until buttonChome=0
            if (buttonStateR == CLICKED) { // FIX THIS CONDITION WITHOUT PROBLEM STATE
                Serial.println("IF19");
                currentState = PROBLEM;
            }
            else if (IRseen == false) {
                Serial.println("IF20");
                moveMotor(UP, MOTOR_SPEED_GEAR);
            }
            if (IRseen == true) {
              moveMotor(UP, 200);
              delay(500); 
              while (buttonStateChome == RELEASED) {
                stopAllMotors(); 
                Serial.println("IF21");
                moveMotor(LEFT, step);
              }
              Serial.println("IF22");
              stopAllMotors();
              currentState = REST;
            }
        break;
        case PROBLEM:
            Serial.println("PROBLEM");
            // Logic to handle problems TO DO
            // stop everything
            stopAllMotors();
        break;
    }
}

