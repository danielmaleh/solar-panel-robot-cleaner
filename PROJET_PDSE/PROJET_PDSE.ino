#include <Time.h> 
#include <def_const_func.h> 

// Setup and initialization
void setup() {
    void initializeServo();
    void initializeMotors();
    void initializeButtons();
    void initializeLEDPins();
    void initializeRainSensor();
    void initializeStepperMotor();
    void initializeCurrentSensors();
    void initializeSerialCommunication();
}

// Main loop
void loop() {
    // Update the current time
    currentTime = millis();
    updateLEDs(currentState);
    checkCurrent(CURRENT_PIN1);
    checkCurrent(CURRENT_PIN2);
    checkButtonR();
    checkButtonC1();
    checkButtonC2();
    checkIRSensor();

    // Check the states and perform actions based on the current state
    switch (currentState) {
        case REST:
            checkRainSensor();
            if (buttonStateR == 0) {
                currentState = PROBLEM;
            }
            if (IRseen == false || buttonStateC1 == 0 || buttonStateC2 == 1) {
                currentState = RETURN_HOME;
            }
            else if ((currentTime >= cleaningTime + CLEANING_PERIOD) || (raining && currentTime - cleaningTime > CLEANING_PERIOD / 2)) {
                if (raining) {
                    currentState = WATER_FILLING;
                }
                else {
                    currentState = INITIAL_POSITION;
                }
                cleaningTime = currentTime; // Update the last cleaning time
            }
            else {
                currentState = REST;
            }
        break;
        case WATER_FILLING:
            // If not HOME then RETURN_HOME
            if (buttonStateR == 0) {
                currentState = PROBLEM;
            }
            if (IRseen == false || buttonStateC1 == 0 || buttonStateC2 == 1) {
                currentState = RETURN_HOME;
            }
            // after filling done state = Initial_position
            else {
                // fill water reservoir 
                delay(5000);
                currentState = INITIAL_POSITION;
            }
        break;
        case INITIAL_POSITION:
            // Logic to move to the initial position
            // steps to 1st position and when finished state = CLEANING
            moveMotor(LEFT, DIST_TO_INITIAL_POSITION);
            stopAllMotors();
            currentState = CLEANING;
        break;
        case CLEANING:
            // Logic to clean the panels
            // If buttonR = 1  state = UP_TRAVEL, both for start and finish. delay for the finish.
            // Else go downward with dc motor and activate dc brush motor and valve (until buttonR=1).
            if (buttonStateR == 1) {
                delay(END_OF_CLEANING_DELAY);
                stopAllMotors();
                currentState = UP_TRAVEL;
            }
            else {
                moveMotor(DOWN, MOTOR_SPEED_GEAR);
                controlBrushMotor(HIGH, MOTOR_SPEED_BRUSH);
                controlValve(VALVE_ANGLE);
            }
        break;
        case UP_TRAVEL:
            // Logic to move up to the next position
            // go upward with dc motor until IR sensor detects the robot
            // nb_cycles = nb_cycles + 1
            // if nb of cycles = N state = RETURN_HOME else state = TRANSLATION
            if (buttonStateR == 0) {
                currentState = PROBLEM;
            }
            else if (IRseen == false) {
                moveMotor(UP, MOTOR_SPEED_GEAR);
            }
            else {
                stopAllMotors();
                nb_cycles_counter = nb_cycles_counter + 1;
                if (nb_cycles_counter*step == DIST) {
                    currentState = RETURN_HOME;
                    break;
                }
                currentState = TRANSLATION;
            }
        break;
        case TRANSLATION:
            // Logic to translate to the next position
            // go sidways with stepper motor for step
            // when stepper motor finished move upwards with dc motor until buttonC1=0 and then state = CLEANING
            if (buttonStateR == 0) {
                currentState = PROBLEM;
            }
            else if (buttonStateC1 == 1) {
                moveMotor(LEFT, step);
                moveMotor(UP, MOTOR_SPEED_GEAR);
            }
            else {
                stopAllMotors();
                currentState = CLEANING;
            }
        break;
        case RETURN_HOME:
            // Logic to return to the initial position
            // if IRseen is false move upward with motor dc until IRseen is true
            // if buttonC2=1 move sidways with motor stepper until buttonC2=0
            if (buttonStateR == 0) {
                currentState = PROBLEM;
            }
            else if (IRseen == false) {
                moveMotor(UP, MOTOR_SPEED_GEAR);
            }
            else if (buttonStateC2 == 1) {
                moveMotor(RIGHT, step);
            }
            else {
                stopAllMotors();
                currentState = REST;
            }
        break;
        case PROBLEM:
            // Logic to handle problems TO DO
            // stop everything
            stopAllMotors();
        break;
    }
}