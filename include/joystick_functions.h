#pragma once

#include <Arduino.h>
#include "Quaternion.h"
#include "pin_layout.h"
#include "platform.h"
#include "joystick_frames.h"

// Forward declarations for globals defined in main.cpp
extern float zero_length;            // base actuator length
extern float T_cur[3];               // current position tracker
extern Quaternion R_cur;             // current rotation tracker

// Forward declaration for motion function defined in main.cpp
void moveplat(float duration, float length_min, float pos0[3], float pos1[3], Quaternion q0, Quaternion q1);

// Joystick frame handler
// Python sends: "J ax ay az azi alt yaw\n"
inline void handleJoystickFrames() {
  if (Serial.available() <= 0) return;
  if (Serial.peek() != 'J') return;

  Serial.read(); // consume 'J'
  float ax  = Serial.parseFloat();
  float ay  = Serial.parseFloat();
  (void)Serial.parseFloat();         // az (unused)
  float azi = Serial.parseFloat();   // enable flag
  (void)Serial.parseFloat();         // alt (unused)
  (void)Serial.parseFloat();         // yaw (unused)

  // Require enable flag
  if (azi < 0.5f) {
    Serial.println("No input - motors off");
    for (uint8_t m = 0; m < NUM_MOTORS; m++) analogWrite(PWM_PINS[m], 0);
    return;
  }

  // Deadband
  const float dead_t = 0.05f;

  // Quantize to -1/0/+1
  ax = (ax > dead_t) ? 1.0f : (ax < -dead_t ? -1.0f : 0.0f);
  ay = (ay > dead_t) ? 1.0f : (ay < -dead_t ? -1.0f : 0.0f);

  // Determine which position to go to
  float* target_pos = nullptr;
  String direction = "";

  if (ax > 0.5f) {
    target_pos = T_RIGHT;
    direction = "RIGHT";
  }
  else if (ax < -0.5f) {
    target_pos = T_LEFT;
    direction = "LEFT";
  }
  else if (ay > 0.5f) {
    target_pos = T_FORWARD;
    direction = "FORWARD";
  }
  else if (ay < -0.5f) {
    target_pos = T_BACK;
    direction = "BACK";
  }
  else {
    // No clear direction - stop motors
    for (uint8_t m = 0; m < NUM_MOTORS; m++) analogWrite(PWM_PINS[m], 0);
    return;
  }

  // Debug output
  Serial.print("Moving to: "); Serial.print(direction);
  Serial.print(" ("); Serial.print(target_pos[0]);
  Serial.print(", "); Serial.print(target_pos[1]);
  Serial.print(", "); Serial.print(target_pos[2]);
  Serial.println(")");

  // Lock rotation flat
  Quaternion q_flat = Quaternion(1, 0, 0, 0);

  // Move to the target position
  moveplat(1.0f, zero_length, T_cur, target_pos, q_flat, q_flat);

  // Update current position tracker
  T_cur[0] = target_pos[0];
  T_cur[1] = target_pos[1];
  T_cur[2] = target_pos[2];
  R_cur = q_flat;

  // Stop motors after reaching position
  for (uint8_t m = 0; m < NUM_MOTORS; m++) analogWrite(PWM_PINS[m], 0);
  
  Serial.println("Position reached - motors stopped");
}
