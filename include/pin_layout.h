// Header file containing hardware configurations for the Arduino Due mapping to the linear actuators and motor drivers.
// Note that on the motor driver for actuators 4+5+6, M3 corresponds to actuator 4 and M1 to actuator 6
#pragma once
#include <Arduino.h>

// Direction pins
#define DIR_PIN_1 41
#define DIR_PIN_2 43
#define DIR_PIN_3 45
#define DIR_PIN_6 40
#define DIR_PIN_5 42
#define DIR_PIN_4 44

// PWM pins
#define PWM_PIN_1 2
#define PWM_PIN_2 3
#define PWM_PIN_3 4
#define PWM_PIN_6 5
#define PWM_PIN_5 6
#define PWM_PIN_4 7

// PA-14P potentiometer pins
#define POT_PIN_1 A1
#define POT_PIN_2 A2
#define POT_PIN_3 A3
#define POT_PIN_4 A4
#define POT_PIN_5 A5
#define POT_PIN_6 A6

// Enable for all motors (grouped as such due to different H-bridge controllers)
//Hi
#define ENABLE_MOTORS 52
#define ENABLE_MOTORS_2 25
