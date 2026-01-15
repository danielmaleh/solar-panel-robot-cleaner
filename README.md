# Solar Panel Cleaning Robot

An autonomous robotic system designed to clean solar panels efficiently and safely. The robot performs automated cleaning cycles using a rotating brush mechanism while traversing across solar panel arrays.

## Overview

This project implements a complete embedded control system for a solar panel cleaning robot built on an Arduino/ARM microcontroller platform. The robot autonomously navigates across solar panels, performing vertical cleaning passes with a rotating brush and horizontal translations between panel sections.

### Key Features

- **Autonomous Operation**: Fully automated cleaning cycles without human intervention
- **Multi-Axis Movement**: Vertical cleaning motion + horizontal panel traversal
- **Sensor Integration**: IR proximity, rain detection, and current monitoring
- **Safety Systems**: Motor fault detection via current sensing, limit switches for position control
- **Visual Feedback**: RGB LED status indication for all operating states

## Hardware Architecture

### Components

| Component | Description | Control |
|-----------|-------------|---------|
| DC Gearbox Motor | Vertical movement (up/down) | GPIO 4, 5 |
| DC Brush Motor | Rotating cleaning brush | GPIO 6, 7 |
| Stepper Motor | Horizontal translation (left/right) | GPIO 2, 3, 28 |
| Servo Motor | Water valve control | GPIO 13 |
| IR Sensor | Position detection for docking | A4 |
| Rain Sensor | Weather/precipitation detection | A15 |
| Current Sensors (x2) | Motor fault detection (ACS712) | A0, A14 |
| Limit Switches (x3) | End-of-travel detection | GPIO 22, 24, 26 |
| RGB LED | System status indication | GPIO 8, 9, 11 |

### Pin Configuration

```
Gearbox Motor:    PIN1=4, PIN2=5, SPEED=12
Brush Motor:      PIN1=6, PIN2=7
Stepper Motor:    DIR=2, STEP=3, SLEEP=28
Servo Valve:      PIN=13
IR Sensor:        A4
Rain Sensor:      A15
Current Sensors:  Gearbox=A14, Brush=A0
Limit Switches:   Robot=22, Carriage=24, Home=26
RGB LED:          R=11, G=9, B=8
```

## Software Architecture

### State Machine

The robot operates as a finite state machine with the following states:

| State | LED Color | Description |
|-------|-----------|-------------|
| REST | Off | Idle, waiting for cleaning trigger |
| WATER_FILLING | Blue (flashing) | Filling water reservoir |
| INITIAL_POSITION | White (solid) | Moving to starting position |
| CLEANING | Green (flashing) | Active cleaning in progress |
| UP_TRAVEL | Green (solid) | Returning to top of panel |
| TRANSLATION | White (flashing) | Horizontal movement between sections |
| RETURN_HOME | Red (solid) | Returning to docking station |
| PROBLEM | Red (flashing) | Error state - all motors stopped |

### Cleaning Cycle Sequence

1. **Calibration**: IR sensor threshold calibration on startup
2. **Initial Position**: Move to first panel cleaning position
3. **Cleaning Pass**: Descend with brush active until bottom limit
4. **Return to Top**: Ascend using IR sensor guidance
5. **Translation**: Move horizontally to next section
6. **Repeat**: Continue cleaning passes across panel width
7. **Return Home**: Navigate back to docking station

## File Structure

```
solar-panel-robot-cleaner/
├── test/                    # Main implementation
│   ├── test.ino             # Main program (state machine loop)
│   ├── def_const_func.h     # Constants and declarations
│   └── def_const_func.cpp   # Function implementations
├── TESTS/                   # Component test files
│   ├── test_servo.ino
│   ├── test_motor_dc_current.ino
│   ├── test_LED_RGB.ino
│   ├── test_stepper/
│   ├── test_button.ino
│   ├── test_current_sensor.ino
│   ├── test_rain_sensor.ino
│   └── test_IR.ino
├── libraries/               # External dependencies
│   ├── AccelStepper/        # Stepper motor acceleration
│   ├── Servo/               # Servo control
│   ├── Stepper/             # Basic stepper control
│   └── ACS712/              # Current sensor library
└── README.md
```

## Configuration

Key parameters can be adjusted in `def_const_func.h`:

```cpp
// Panel dimensions
#define CLEANING_WIDTH              30      // cm
#define PANEL_WIDTH                 100     // cm

// Sensor thresholds
#define IR_THRESHOLD                620
#define RAIN_THRESHOLD              700

// Current limits (fault detection)
#define CURRENT_THRESHOLD_GEARBOX   1.5     // Amps
#define CURRENT_THRESHOLD_BRUSH     0.7     // Amps

// Stepper motor
#define CM_PER_REVOLUTION           2.0     // cm/rev
#define STEPPER_STEPS_PER_REV       200
```

## Dependencies

- [Servo](https://www.arduino.cc/reference/en/libraries/servo/) - Standard Arduino servo library
- [AccelStepper](https://www.airspayce.com/mikem/arduino/AccelStepper/) - Advanced stepper motor control
- [Time](https://www.arduino.cc/reference/en/libraries/time/) - Timekeeping functionality

## Installation

1. Clone this repository
2. Open `test/test.ino` in Arduino IDE
3. Install required libraries via Library Manager
4. Select your board (Arduino Mega or compatible)
5. Upload to the microcontroller

## Usage

### Serial Monitor

Connect at 9600 baud to monitor robot status:
- Initialization progress
- Sensor readings (IR, current)
- State transitions
- Button events

### Manual Testing

Individual component tests are available in the `TESTS/` directory for:
- Motor functionality
- Sensor calibration
- Button debouncing
- LED patterns

## Safety Features

- **Current Monitoring**: Continuous monitoring of motor currents with automatic shutdown if thresholds exceeded
- **Limit Switches**: Physical end-of-travel detection prevents over-extension
- **Button Debouncing**: 50ms debounce delay prevents false triggers
- **PROBLEM State**: Immediate motor shutdown on fault detection

## Technical Specifications

- **Operating Voltage**: 5V logic, 12V motors
- **Current Sensor**: ACS712 (0.185 V/A sensitivity)
- **Stepper Resolution**: 200 steps/revolution
- **IR Sensor Range**: 5-30cm effective detection
- **Serial Baud Rate**: 9600

## License

This project was developed as part of an academic engineering project.

## Author

Daniel Abraham Elmaleh

---

*This robotic system was designed and implemented as a practical application of embedded systems, motor control, and sensor integration principles.*
