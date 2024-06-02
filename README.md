# RoboRacer-H7

## Project Overview
This Repo is based on the Portenta H7 development board. The robot is capable of self-navigation, obstacle detection and avoidance, as well as path tracking. This project utilizes various sensors and control systems to ensure high precision and responsive operation.

## Features
- **Ultrasonic Distance Measurement**: Real-time detection and calculation of obstacle distances.
- **Color Recognition**: Identifies different colors to assist in path tracking.
- **PID Control System**:
  - **Speed Control**: Precisely controls movement speed using a PID algorithm.
  - **Servo Control**: Precisely adjusts direction using a PID algorithm.
- **Modular Design**: Each function has a dedicated module, making it easy to extend and maintain.

## File Structure
- **include/**: Contains all header files, defining classes and functions.
  - **ultrasonicsensor.h**: Defines the `UltrasonicSensor` class for operating the ultrasonic sensor to measure distance.
  - **ColorSensors.h**: Defines functions and settings for handling multiple color sensor readings.
  - **PIDServoControl.h**: Provides an interface for PID control of the servo motor.
  - **PinDef.h**: Defines the pin configuration for all hardware components.
  - **PIDSpeedControl.h**: Provides PID-based speed control functionality.
- **src/**: Contains all source files implementing the defined interfaces.
  - **PIDSpeedControl.cpp**: Implements the functionality described in `PIDSpeedControl.h` for speed control.
  - **ultrasonicsensor.cpp**: Implements the class defined in `ultrasonicsensor.h` for operating the ultrasonic sensor.
  - **PIDServoControl.cpp**: Implements the PID servo control functionality described in `PIDServoControl.h`.
  - **ColorSensors.cpp**: Implements the code for handling color sensors as described in `ColorSensors.h`.
  - **main.cpp**: The main program, initializes hardware and integrates all module functionalities.

## System Requirements
- Arduino IDE
- Portenta H7 Development Board
- Required Libraries: Adafruit_APDS9960 library, Arduino core libraries

## Installation and Setup
### Environment Preparation
- Ensure the Arduino IDE is installed and supports the Portenta H7 development board.
- Install the required libraries via the Arduino Library Manager.

### Hardware Setup
- Connect all sensors and the power system to the Portenta H7 development board according to the `PinDef.h` file.

### Code Deployment
- Upload all files from the `src` and `include` folders to the Portenta H7 development board.
- Verify `main.cpp` to ensure correct startup configuration.

## Usage Instructions
Once the robot is started, it will automatically load the configuration and activate sensor monitoring. Through the serial port, users can view log outputs to understand the robot's status and activities.

## File Summary Table

| File Path                                      | Description                                                     |
|------------------------------------------------|-----------------------------------------------------------------|
| RoboRacer-H7/H7/include/ultrasonicsensor.h     | Defines the class and methods for operating the ultrasonic sensor. |
| RoboRacer-H7/H7/include/ColorSensors.h         | Defines the class and methods for reading and operating color sensors. |
| RoboRacer-H7/H7/include/PIDServoControl.h      | Defines the class and methods for PID control of the servo motor. |
| RoboRacer-H7/H7/include/PinDef.h               | Defines the pin configuration for different hardware devices.    |
| RoboRacer-H7/H7/include/PIDSpeedControl.h      | Defines the PID controller class and methods for speed control.  |
| RoboRacer-H7/H7/src/PIDSpeedControl.cpp        | Implements the PID speed control functions defined in `PIDSpeedControl.h`. |
| RoboRacer-H7/H7/src/ultrasonicsensor.cpp       | Implements the ultrasonic sensor operations defined in `ultrasonicsensor.h`. |
| RoboRacer-H7/H7/src/PIDServoControl.cpp        | Implements the PID servo control functions defined in `PIDServoControl.h`. |
| RoboRacer-H7/H7/src/ColorSensors.cpp           | Implements the operations and data reading for color sensors as defined in `ColorSensors.h`. |
| RoboRacer-H7/H7/src/main.cpp                   | Initializes and integrates all modules and functionalities; the main execution file. |
| RoboRacer-H7/OpenMV/find_lines.py              | Uses the OpenMV Cam to detect lines, calculate angular error from a target angle, and send the error value to an Arduino via UART. |

