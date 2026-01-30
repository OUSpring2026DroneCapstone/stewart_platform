# Stewart Platform Control System

This repository contains firmware for controlling a six-actuator Stewart platform using an Arduino Mega. The system computes actuator motion using position feedback, inverse kinematics, and quaternion-based rotations to move the platform smoothly in three-dimensional space.

---

## Capstone Computer Science Group

The Spring 2026 Capstone Computer Science Group consists of the following members:

- Taylor Wise
- Nicholas Louque
- Reese Booker
- Shaina Patel

## Hardware Overview

The platform consists of the following major components:

- Arduino Mega 2560 on the platform
- Six linear actuators
- Actuator position feedback
- Motor drivers
- External power supply for actuators
- Data transfer cable connecting the Arduino Mega to a laptop

The cable is used for both uploading code and sending commands to the platform.

---

## Software / Technology Overview

This project is developed using:

- Visual Studio Code
- PlatformIO IDE extension converted over from Arduino IDE
- Arduino framework
- Custom geometry and quaternion libraries

The primary coding languages are:

- C++ for calculations and controls
- Python for controller communication

Potential impelementation for the mini drones may include ROS2.

---

## Group Goals and Progress Plan

The goal of this project phase is to develop a fully functional six-degree-of-freedom (6-DOF) Stewart platform and its associated embedded controller. The focus of this phase is on reliable actuator control, accurate motion execution, and safe user interaction.

The group goals for the Stewart Platform are:
- Control actuator movement using an Xbox controller
- Provide calibration, centering, and emergency stop functionality
- Allow user control through serial commands
- Incorpoate a GUI for user-friendly interface

The progress plan for the platform is:

1. Establish Baseline
- Understand how to get existing software to run and current limitations

2. Make small modification to code
- Ensure all members know how to build and upload code onto the arduino
- Ensure the existing software is understandable enough to make modifications

3. Development
- Begin adding center, start, stop, etc. features through serial commands
- Ensure actuators stay within safe range of movements

4. Controller
- Establish connection between arduino and controller
- Move actuators based on controller movement

5. GUI
- Allow commands to be pressed through button rather than type

6. Refinement
- Improve any logic to ensure reliability of the platform
- Further refinements may be added as development continues

## Environment Setup (VS Code + PlatformIO)

### Install Visual Studio Code

Download and install Visual Studio Code from:

https://code.visualstudio.com/download

---

### Install PlatformIO

1. Open Visual Studio Code
2. Open the Extensions panel
3. Search for "PlatformIO IDE"
4. Install the extension
5. Restart VS Code when prompted

PlatformIO manages compilation, library dependencies, board configuration, uploading, and the serial monitor.

---

### Open the Project

1. Open Visual Studio Code
2. Open PlatformIO Home
3. Select "Open Project"
4. Choose the root directory of this repository

PlatformIO will automatically detect the project configuration.

---

## Hardware Connection

Connect a data transfer cable from your laptop to the Arduino Mega on the platform.

Ensure the cable supports data transfer. Charge-only cables will not work for uploading code or serial communication.

---

## Uploading Firmware

1. Connect the Arduino Mega to your laptop
2. In VS Code, click the PlatformIO Build button
2. In VS Code, click the PlatformIO Upload button after sucessful Build
3. Wait for the upload process to complete

---

## Serial Communication

The platform is controlled using serial commands.

To open the serial monitor:

1. Open PlatformIO
2. Build code and upload if wanting modifications
3. Click the Serial Monitor button

Commands must be followed by pressing Enter.

---

## Available Commands

- center  
  Moves the platform to its neutral position, then disables the motors

- start  
  Runs the predefined motion sequence

- stop  
  Immediately stops all motion and disables the motors

The platform must be centered before motion can be started.

---

## Program Structure

### setup()

- Initializes pins
- Starts serial communication
- Calculates baseline actuator length
- Runs calibration
- Disables motors for safety

---

### loop()

- Continuously checks for serial commands
- Executes motion only when start is requested

---

### checkSerialCommands()

- Parses serial input
- Handles start, stop, and center commands

---

### calibrate()

- Fully extends and retracts all actuators
- Records minimum and maximum sensor values
- Validates actuator feedback

---

### moveplat()

- Interpolates translation and rotation over time
- Computes desired actuator lengths
- Applies feedback control
- Monitors for emergency stop conditions

---

## Safety Behavior

- Motors are disabled by default
- stop overrides all motion immediately
- Motion speed is capped
- Position feedback is continuously monitored
- Motion is blocked unless the platform is centered

---

## Notes

- Motion sequences in loop() are examples and can be modified
- Geometry definitions and pin mappings are located in platform.h and pin_layout.h