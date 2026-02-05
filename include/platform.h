// Header file containing software configurations, constants, and clarifying definitions for the Arduino.
//
# pragma once

#include "pin_layout.h"
#include <stdint.h>

// Platform parameters
#define NUM_MOTORS 6

// Actuator value bounds
#define MIN_POS 0
#define MAX_POS 1024
#define MIN_PWM 0
#define MAX_PWM 255

// Default platform calibration settings (average analog values at extrema for each actuator)
#define OFF_THRESHOLD 500  // ignore calibration if motors aren't powered (i.e. large reading difference from default is found)
int16_t ZERO_POS[NUM_MOTORS]  = { 45, 45, 45, 45, 45, 45 };
int16_t END_POS[NUM_MOTORS]   = { 875, 875, 875, 875, 875, 875 };

// Stroke and calibration limits (inches)
// Note: mapping currently treats the full range as 8". Use SAFE_MAX_INCHES
// to calibrate against a smaller, safer maximum without hard-stopping actuators.
#define STROKE_INCHES 8.0f
#define SAFE_MAX_INCHES 8.0f  // set your preferred safe max here

// Pin group arrays; each value corresponding to the actuator (see "pin_layout.h" for specific ins)
const uint8_t DIR_PINS[NUM_MOTORS] = { DIR_PIN_1, DIR_PIN_2, DIR_PIN_3, DIR_PIN_4, DIR_PIN_5, DIR_PIN_6 };
const uint8_t PWM_PINS[NUM_MOTORS] = { PWM_PIN_1, PWM_PIN_2, PWM_PIN_3, PWM_PIN_4, PWM_PIN_5, PWM_PIN_6 };
const uint8_t POT_PINS[NUM_MOTORS] = { POT_PIN_1, POT_PIN_2, POT_PIN_3, POT_PIN_4, POT_PIN_5, POT_PIN_6 };

// Movement parameters
#define RESET_DELAY 5000      // at full PWM, the actuator should fully extend/retract by 4s (6" stroke, 2.00"/s)
typedef enum _MotorDirection  // to clarify the direction in which actuators move
{
    RETRACT = 0,
    EXTEND  = 1
} MotorDirection;

// Serial configuration parameters
#define BAUD_RATE 115200  // baud rate for serial port (also needs to be set on host side)

// Serial input parameters
#define INPUT_TRIGGER 10   // at minimum, 6 numbers + 5 spaces
#define NUM_READINGS  255  // number of analog readings to average to acquire position

// Serial print/output parameters
// NOTE: feedback measurement settings: PRINT_INTERVAL = 10, ENABLE_PRINT_HEADERS = 0, only PRINT_CURRENT_POS
#define PRINT_INTERVAL          1000               // minimum time (ms) between printing serial info
#define ENABLE_PRINT            1                  // flag to enable printing variables (for debugging)
#define ENABLE_PRINT_HEADERS    ENABLE_PRINT && 1  // flag to enable variable headers (when not running analysis)
#define PRINT_DESIRED_POS       ENABLE_PRINT && 1  // print desired position values when printing serial info
#define PRINT_CURRENT_POS       ENABLE_PRINT && 1  // print current position values when printing serial info
#define PRINT_PWM               ENABLE_PRINT && 1  // print PWM values when printing serial info

//Declares the coordinates of the points of interest
//base_[x] correspond to the ball joints on the base and attached to actuator [x]
//origin corresponds to the origin in the CAD
const float base_1[3] = {4.8125, 11.464, 0};
const float base_2[3] = {11.98812, -5.18221723, 0};
const float base_3[3] = {5.17514616, -11.98104893, 0};
const float base_4[3] = {-base_1[0], base_1[1], base_1[2]};
const float base_5[3] = {-base_2[0], base_2[1], base_2[2]};
const float base_6[3] = {-base_3[0], base_3[1], base_3[2]};

// CHANGED TO CONST FLOAT*
const float* bases[6] = {base_1, base_2, base_3, base_4, base_5, base_6};

//plat_[x] correspond to the ball joints on the platform and attached to actuator [x]
//origin of plat_[x] is WRT to plat_0
//plat_0 is the middle of the platform, and the coordinates for plat_0 are WRT the origin at the base
const float plat_0[3] = {0, 0, 17.4775 - 0.625};
const float plat_1[3] = {0.75, 11.469, 0};
const float plat_2[3] = {9.00827306, -8.1549931, 0};
const float plat_3[3] = {7.94761289, -9.21565328, 0};
const float plat_4[3] = {-plat_1[0], plat_1[1], plat_1[2]};
const float plat_5[3] = {-plat_2[0], plat_2[1], plat_2[2]};
const float plat_6[3] = {-plat_3[0], plat_3[1], plat_3[2]};

const float* plats[6] = {plat_1, plat_2, plat_3, plat_4, plat_5, plat_6};

//TO DO: create a class for Actuator to make things neater
//class Actuator{
//  float base[3];
//  float top[3];
//  float top_center[3];
//};
//
//Actuator acts[6] = {
//  //actuator 1
//  {
//    {4.8125, 11.464, 0}, 
//    {0.75, 11.469, 0},
//    {0, 0, 17.4775 - 0.625}
//  },
//  //actuator 2
//  {
//    {11.98812, -5.18221723, 0},
//    {9.00827306, -8.1549931, 0},
//    {0, 0, 17.4775 - 0.625}
//  },
//  {
//    {5.17514616, -11.98104893, 0},
//    {7.94761289, -9.21565328, 0},
//    {0, 0, 17.4775 - 0.625}
//  },
//  {
//    {-4.8125, 11.464, 0}, 
//    {-0.75, 11.469, 0},
//    {0, 0, 17.4775 - 0.625}
//  },
//  {
//    {-11.98812, -5.18221723, 0},
//    {-9.00827306, -8.1549931, 0},
//    {0, 0, 17.4775 - 0.625}
//  },
//  {
//    {-5.17514616, -11.98104893, 0},
//    {-7.94761289, -9.21565328, 0},
//    {0, 0, 17.4775 - 0.625}
//  }
//};
