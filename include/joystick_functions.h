#pragma once

#include <Arduino.h>
#include "Quaternion.h"
#include <math.h>
#include "pin_layout.h"
#include "platform.h"
#include "joystick_frames.h"
// Stop helper provided by main.cpp
void stopMotors();

// Forward declarations for globals defined in main.cpp
extern float zero_length;            // base actuator length
extern float T_cur[3];               // current position tracker
extern Quaternion R_cur;             // current rotation tracker
extern unsigned long g_lastJFrameMs; // last time a joystick frame was received
extern bool joystick_input_active;   // tracks if joystick is currently providing input

// Timeout for joystick input (if no frame received in this time, button is released)
static const unsigned long JOYSTICK_TIMEOUT_MS = 200;

// Forward declaration for motion function defined in main.cpp
void moveplat(float duration, float length_min, float pos0[3], float pos1[3], Quaternion q0, Quaternion q1);
static const float Z_HOME = 2.0f;   // Lock platform height during joystick control

// Check for new joystick input and update the active flag (doesn't trigger movement)
inline void updateJoystickInputFlag() {
  // Check if we have a new joystick frame
  if (Serial.available() > 0 && Serial.peek() == 'J') {
    Serial.read(); // consume 'J'
    float ax  = Serial.parseFloat();
    float ay  = Serial.parseFloat();
    (void)Serial.parseFloat();         // az (unused)
    float azi = Serial.parseFloat();   // enable flag
    (void)Serial.parseFloat();         // alt (unused)
    (void)Serial.parseFloat();         // yaw (unused)

    // Update last frame timestamp
    g_lastJFrameMs = millis();

    if (azi < 0.5f) {
      joystick_input_active = false;
      return;
    }

    const float dead_t = 0.05f;
    float sx = (ax > dead_t) ? 1.0f : (ax < -dead_t ? -1.0f : 0.0f);
    float sy = (ay > dead_t) ? 1.0f : (ay < -dead_t ? -1.0f : 0.0f);

    if (sx == 0.0f && sy == 0.0f) {
      joystick_input_active = false;
    } else {
      joystick_input_active = true;
    }
  }

  // Check timeout - if no frame received recently, button is released
  if (millis() - g_lastJFrameMs > JOYSTICK_TIMEOUT_MS) {
    joystick_input_active = false;
  }
}

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

  // Update last frame timestamp
  g_lastJFrameMs = millis();

  if (azi < 0.5f) {
    joystick_input_active = false;
    stopMotors();
    return;
  }

  const float dead_t = 0.05f;
  float sx = (ax > dead_t) ? 1.0f : (ax < -dead_t ? -1.0f : 0.0f);
  float sy = (ay > dead_t) ? 1.0f : (ay < -dead_t ? -1.0f : 0.0f);

  if (sx == 0.0f && sy == 0.0f) {
    joystick_input_active = false;
    stopMotors();
    return;
  }

  joystick_input_active = true;

  float step = 0.25f;
  float dur  = 0.12f;

  float target[3] = { T_cur[0] + step * sx, T_cur[1] + step * sy, Z_HOME };
  target[0] = constrain(target[0], -3.0f, 3.0f);
  target[1] = constrain(target[1], -3.0f, 3.0f);

  Quaternion q_flat(1, 0, 0, 0);
  moveplat(dur, zero_length, T_cur, target, q_flat, q_flat);

  T_cur[0] = target[0];
  T_cur[1] = target[1];
  T_cur[2] = Z_HOME;     // ensure height stays fixed
  R_cur = q_flat;
}
