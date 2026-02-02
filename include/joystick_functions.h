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
    float alt = Serial.parseFloat();   // tilt forward/back (right stick Y)
    float yaw = Serial.parseFloat();   // tilt left/right (right stick X)

    // Update last frame timestamp
    g_lastJFrameMs = millis();

    if (azi < 0.5f) {
      joystick_input_active = false;
      return;
    }

    const float dead_t = 0.05f;
    const float dead_rot = 3.0f;  // deadzone for tilt direction

    float sx = (ax > dead_t) ? 1.0f : (ax < -dead_t ? -1.0f : 0.0f);
    float sy = (ay > dead_t) ? 1.0f : (ay < -dead_t ? -1.0f : 0.0f);

    // Determine tilt direction from right stick
    float tilt_x = (yaw > dead_rot) ? 1.0f : (yaw < -dead_rot ? -1.0f : 0.0f);
    float tilt_y = (alt > dead_rot) ? 1.0f : (alt < -dead_rot ? -1.0f : 0.0f);

    bool has_tilt = (tilt_x != 0.0f || tilt_y != 0.0f);

    if (sx == 0.0f && sy == 0.0f && !has_tilt) {
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
  float alt = Serial.parseFloat();   // tilt forward/back (right stick Y)
  float yaw = Serial.parseFloat();   // tilt left/right (right stick X)

  // Update last frame timestamp
  g_lastJFrameMs = millis();

  if (azi < 0.5f) {
    joystick_input_active = false;
    stopMotors();
    return;
  }

  const float dead_t = 0.05f;
  const float dead_rot = 3.0f;  // deadzone for tilt direction

  float sx = (ax > dead_t) ? 1.0f : (ax < -dead_t ? -1.0f : 0.0f);
  float sy = (ay > dead_t) ? 1.0f : (ay < -dead_t ? -1.0f : 0.0f);

  // Determine tilt direction from right stick
  float tilt_x = (yaw > dead_rot) ? 1.0f : (yaw < -dead_rot ? -1.0f : 0.0f);   // left/right
  float tilt_y = (alt > dead_rot) ? 1.0f : (alt < -dead_rot ? -1.0f : 0.0f);   // forward/back

  bool has_tilt = (tilt_x != 0.0f || tilt_y != 0.0f);

  if (sx == 0.0f && sy == 0.0f && !has_tilt) {
    joystick_input_active = false;
    stopMotors();
    return;
  }

  joystick_input_active = true;

  float step = 0.25f;
  float dur  = 0.12f;

  // Calculate target position
  float target[3] = { T_cur[0] + step * sx, T_cur[1] + step * sy, Z_HOME };
  target[0] = constrain(target[0], -3.0f, 3.0f);
  target[1] = constrain(target[1], -3.0f, 3.0f);

  // Simple directional tilt (10 degrees in cardinal directions)
  const float tilt_angle = 10.0f;  // degrees
  const float half_rad = tilt_angle * PI / 360.0f;  // half angle in radians
  const float c = cos(half_rad);
  const float s = sin(half_rad);

  Quaternion q_target;
  if (has_tilt) {
    if (tilt_y > 0.5f) {
      // Tilt FORWARD (rotate around X axis, positive)
      q_target = Quaternion(c, s, 0, 0);
    } else if (tilt_y < -0.5f) {
      // Tilt BACK (rotate around X axis, negative)
      q_target = Quaternion(c, -s, 0, 0);
    } else if (tilt_x > 0.5f) {
      // Tilt RIGHT (rotate around Y axis, positive)
      q_target = Quaternion(c, 0, s, 0);
    } else if (tilt_x < -0.5f) {
      // Tilt LEFT (rotate around Y axis, negative)
      q_target = Quaternion(c, 0, -s, 0);
    } else {
      q_target = Quaternion(1, 0, 0, 0);  // identity (no rotation)
    }
  } else {
    q_target = Quaternion(1, 0, 0, 0);  // identity (no rotation)
  }

  moveplat(dur, zero_length, T_cur, target, R_cur, q_target);

  T_cur[0] = target[0];
  T_cur[1] = target[1];
  T_cur[2] = Z_HOME;     // ensure height stays fixed
  R_cur = q_target;
}
