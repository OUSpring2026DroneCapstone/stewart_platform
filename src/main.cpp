/*
 * Platform Control System
 * Goal: Controls a multi-actuator platform using inverse kinematics.
 *
 * Features:
 * - Motor control (PWM + direction)
 * - Position tracking via potentiometers
 * - Preset motion sequences (demo, orbit, wave, etc.)
 * - Joystick control mode
 * - Calibration system for actuator limits
 * - LED feedback + fan control
 *
 * Core idea:
 * Desired platform position (T + rotation) -> actuator lengths -> motor commands
 *
 * Main loop handles:
 * - Serial commands
 * - Mode switching (idle/script/controller)
 * - Motion execution
 */

#include <Arduino.h>
#include <math.h>
#include <FastLED.h>
#include "pin_layout.h"
#include "Quaternion.h"
#include "platform.h"

#define NUM_LEDS  96

CRGB leds[NUM_LEDS];

// Actuator variables (PWM + direction per motor)
uint8_t pwm_cmd[NUM_MOTORS];
MotorDirection dir_cmd[NUM_MOTORS];

// Position variables aka real-world state from sensors (potentiometers)
int16_t pos[NUM_MOTORS];
int16_t input[NUM_MOTORS];
uint16_t desired_pos[NUM_MOTORS];

// Calibration variables aka target actuator positions computed from kinematics
int16_t end_readings[NUM_MOTORS];
int16_t zero_readings[NUM_MOTORS];
bool calibration_valid;

// Iterator/sum variables 
uint8_t motor;
uint8_t reading;
int32_t reading_sum;

// Quaternions / transforms 
Quaternion R0, R1, R2, R3;

float T0[3] = {0, 0, 0};
float T1[3] = {0, 0, 2};
float T2[3] = {1, -2, 3};
float TX[3] = {3, 0, 2};
float TY[3] = {0, 3, 2};
float TZ[3] = {0, 0, 5};

float zero_length = 0.0f;
float dur = 10.0f;

bool stop_requested = false;
bool centered = false;
bool joystick_input_active = false;
unsigned long g_lastJFrameMs = 0;
bool enable_move_stats = false;


// Discrete positions for D-pad control
#include "joystick_frames.h"
float T_CENTER[3] = {0, 0, 2};     // Home position
float T_FORWARD[3] = {0, 3.0, 2};   // D-pad UP
float T_BACK[3]    = {0, -3.0, 2};  // D-pad DOWN
float T_RIGHT[3]   = {3.0, 0, 2};   // D-pad RIGHT
float T_LEFT[3]    = {-3.0, 0, 2};  // D-pad LEFT

enum ControlMode : uint8_t {
  MODE_IDLE = 0,
  MODE_SCRIPT,
  MODE_CONTROLLER
};

ControlMode mode = MODE_IDLE;

enum PresetID : uint8_t {
  PRESET_NONE = 0,
  PRESET_DEMO,
  PRESET_FIGURE8,
  PRESET_ORBIT,
  PRESET_WAVE
};

static PresetID active_preset = PRESET_NONE;

float T_cur[3] = {0, 0, 2};
Quaternion R_cur = Quaternion(1, 0, 0, 0);

inline int getAverageReading(uint8_t motor);
inline float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
inline void moveAll(MotorDirection dir);
inline void calibrate();
inline void doCenter();
void lerp(const float pos0[3], const float pos1[3], float t, float T[3]);
inline void moveplat(float duration, float length_min, float pos0[3], float pos1[3], Quaternion q0, Quaternion q1);

void setMotorsEnabled(bool en);
void stopThisHoe();
void stopMotors();
void setFans(bool on);

void checkTextCommands();
void updateJoystickInputFlag();
// Joystick frame handler in header
#include "joystick_functions.h"

// Motors start / stop
void setMotorsEnabled(bool en) {
  // HIGH = disable, LOW = enable
  digitalWrite(ENABLE_MOTORS, en ? LOW : HIGH);

  #ifdef ENABLE_MOTORS_2
  digitalWrite(ENABLE_MOTORS_2, en ? LOW : HIGH);
  #endif
}

// Stop all motors by setting PWM to 0 (does not toggle enable pins)
void stopMotors() {
  for (uint8_t m = 0; m < NUM_MOTORS; m++) {
    analogWrite(PWM_PINS[m], 0);
  }
}

// Executes predefined motion patterns by chaining moveplat() calls.
// Each preset defines a trajectory in 3D space using waypoints.
void runPreset(PresetID p) {
  switch (p) {

    float Z_LOW;
    float Z_MID;
    float Z_HIGH;

    case PRESET_DEMO:
      // Simple demo sequence: center -> forward -> right -> left -> back -> up -> center 
      moveplat(dur, zero_length, T0, T1, R0, R0);
      moveplat(dur, zero_length, T1, TX, R0, R0);
      moveplat(dur, zero_length, TX, T1, R0, R0);
      moveplat(dur, zero_length, T1, TY, R0, R0);
      moveplat(dur, zero_length, TY, T1, R0, R0);
      moveplat(5.0f, zero_length, T1, TZ, R0, R0);
      break;

    case PRESET_FIGURE8:
    {
      // Figure-8 motion in XY plane with varying Z height
      Z_LOW  = 1.8f;   // lower at crossover
      Z_HIGH = 2.4f;   // higher at outer lobes
    
      float F8_1[3] = {  2.5f,  0.0f, Z_HIGH };
      float F8_2[3] = {  0.0f,  2.5f, Z_LOW  };
      float F8_3[3] = { -2.5f,  0.0f, Z_HIGH };
      float F8_4[3] = {  0.0f, -2.5f, Z_LOW  };
      float F8_5[3] = {  2.5f,  0.0f, Z_HIGH };
    
      moveplat(dur, zero_length, T_CENTER, F8_1, R0, R0);
      moveplat(dur, zero_length, F8_1, F8_2, R0, R0);
      moveplat(dur, zero_length, F8_2, F8_3, R0, R0);
      moveplat(dur, zero_length, F8_3, F8_4, R0, R0);
      moveplat(dur, zero_length, F8_4, F8_5, R0, R0);
      moveplat(dur, zero_length, F8_5, T_CENTER, R0, R0);
      break;
    }

    case PRESET_ORBIT:
    {
      // Orbit motion in a circle with 9 waypoints
      Z_LOW  = 1.8f;
      Z_MID  = 2.2f;
      Z_HIGH = 2.6f;

      float P1[3] = {  2.5f,  0.0f, Z_LOW  };
      float P2[3] = {  2.5f,  2.5f, Z_MID  };
      float P3[3] = {  0.0f,  2.5f, Z_HIGH };
      float P4[3] = { -2.5f,  2.5f, Z_MID  };
      float P5[3] = { -2.5f,  0.0f, Z_LOW  };
      float P6[3] = { -2.5f, -2.5f, Z_MID  };
      float P7[3] = {  0.0f, -2.5f, Z_HIGH };
      float P8[3] = {  2.5f, -2.5f, Z_MID  };
      float P9[3] = {  2.5f,  0.0f, Z_LOW  };

      moveplat(dur, zero_length, T_CENTER, P1, R0, R0);
      moveplat(dur, zero_length, P1, P2, R0, R0);
      moveplat(dur, zero_length, P2, P3, R0, R0);
      moveplat(dur, zero_length, P3, P4, R0, R0);
      moveplat(dur, zero_length, P4, P5, R0, R0);
      moveplat(dur, zero_length, P5, P6, R0, R0);
      moveplat(dur, zero_length, P6, P7, R0, R0);
      moveplat(dur, zero_length, P7, P8, R0, R0);
      moveplat(dur, zero_length, P8, P9, R0, R0);
      moveplat(dur, zero_length, P9, T_CENTER, R0, R0);

      break;
    }
      
    case PRESET_WAVE:
    {
        // Wave motion in a grid pattern with 8 waypoints, alternating Z height to create a "wave" effect
        // Forward -> Height increases as it reaches waypoint -> waves such that goes backward at lower point -> slowly climbs back up to backward waypoint
        Z_LOW  = 1.7f;
        Z_MID  = 2.0f;
        Z_HIGH = 2.5f;
      
        float P1[3] = {  2.5f,  0.0f, Z_HIGH };
        float P2[3] = { -2.5f,  0.0f, Z_LOW  };
        float P3[3] = {  2.5f,  0.0f, Z_HIGH };
        float P4[3] = {  0.0f,  0.0f, Z_MID  };
      
        float P5[3] = {  0.0f,  2.5f, Z_HIGH };
        float P6[3] = {  0.0f, -2.5f, Z_LOW  };
        float P7[3] = {  0.0f,  2.5f, Z_HIGH };
        float P8[3] = {  0.0f,  0.0f, Z_MID  };
      
        moveplat(dur, zero_length, T_CENTER, P1, R0, R0);
        moveplat(dur, zero_length, P1, P2, R0, R0);
        moveplat(dur, zero_length, P2, P3, R0, R0);
        moveplat(dur, zero_length, P3, P4, R0, R0);
      
        moveplat(dur, zero_length, P4, P5, R0, R0);
        moveplat(dur, zero_length, P5, P6, R0, R0);
        moveplat(dur, zero_length, P6, P7, R0, R0);
        moveplat(dur, zero_length, P7, P8, R0, R0);
      
        moveplat(dur, zero_length, P8, T_CENTER, R0, R0);

        break;
    }

    default:
      Serial.println("No preset selected");
      break;
  }
}

/*
 * System initialization:
 * - Configures all hardware pins (motors, sensors, fans)
 * - Initializes serial communication for commands
 * - Computes baseline actuator geometry (zero_length)
 * - Sets up rotation transforms
 * - Initializes LED strip and performs startup test
 */
void setup() {
  // Initialize serial interface for command input/output
  Serial.begin(BAUD_RATE);
  Serial.setTimeout(30);
  while (!Serial && millis() < 3000) {}

  // Configure motor control pins (direction + PWM) and potentiometer inputs
  for (motor = 0; motor < NUM_MOTORS; ++motor) {
    pinMode(DIR_PINS[motor], OUTPUT);
    digitalWrite(DIR_PINS[motor], LOW);

    pinMode(PWM_PINS[motor], OUTPUT);
    analogWrite(PWM_PINS[motor], 0);

    pinMode(POT_PINS[motor], INPUT);
  }

  // Disable motors by default for safety (HIGH = disabled)
  pinMode(ENABLE_MOTORS, OUTPUT);
  digitalWrite(ENABLE_MOTORS, HIGH);

  #ifdef ENABLE_MOTORS_2
  pinMode(ENABLE_MOTORS_2, OUTPUT);
  digitalWrite(ENABLE_MOTORS_2, HIGH);
  #endif

  // Compute baseline actuator length at home position (zero_length)
  // Used as reference for all motion calculations

  // zero_length
  for (int i = 0; i < 3; i++) {
    float d = plat_0[i] + plat_1[i] - base_1[i];
    zero_length += d * d;
  }
  zero_length = sqrt(zero_length);

  // Precompute rotation quaternions for motion control
  R0 = azi_alt_to_rot(0.0, 0.0);
  R1 = azi_alt_to_rot(0.0, 10.0);
  R2 = azi_alt_to_rot(90.0, 10.0);
  R3 = Quaternion(cos(PI/12), 0, 0, sin(PI/12));

  pinMode(FAN_PIN_1, OUTPUT);
  pinMode(FAN_PIN_2, OUTPUT);
  pinMode(FAN_TACH_1, INPUT_PULLUP);
  pinMode(FAN_TACH_2, INPUT_PULLUP);

  // Initialize cooling fans at full speed
  setFans(true);

  // Initialize LED strip and perform startup test (red flash)
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(80);

  // Startup test: flash red so we know the strip is alive
  fill_solid(leds, NUM_LEDS, CRGB::Red);
  FastLED.show();
  delay(500);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();

  // Print initial pot readings so we can verify sensors before moving
  Serial.println("POT READINGS (raw ADC):");
  for (uint8_t m = 0; m < NUM_MOTORS; m++) {
    int raw = getAverageReading(m);
    float ext_in = mapFloat(raw, ZERO_POS[m], END_POS[m], 0.0f, SAFE_MAX_INCHES);
    Serial.print("  M"); Serial.print(m + 1);
    Serial.print(": raw="); Serial.print(raw);
    Serial.print("  ext="); Serial.print(ext_in, 3); Serial.println("\"");
  }

  // System ready for commands via serial
  Serial.println("Ready. Commands: center | controller | start | stop | calibrate");
}

// Function to control cooling fans, turning on or off based on system state (e.g., on during motion, off when idle)
void setFans(bool on) {
  analogWrite(FAN_PIN_1, on ? 255 : 0);
  analogWrite(FAN_PIN_2, on ? 255 : 0);
}

// Updates LED strip with rainbow pattern. Called continuously in main loop for visual feedback.
void updateLEDs() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate < 20) return;  // ~50Hz
  lastUpdate = millis();

  static uint8_t hue = 0;
  hue += 4;  // speed of rainbow cycle

  fill_rainbow(leds, NUM_LEDS, hue, 255 / NUM_LEDS);
  FastLED.show();
}

 /*
 * Main control loop:
 * - Always listens for serial commands
 * - Runs joystick control if in controller mode
 * - Executes scripted presets if in script mode
 * - Updates LED feedback continuously
 */

void loop() {

  // Always watch for text commands
  checkTextCommands();

  // Always accept joystick frames, only moves if mode == MODE_CONTROLLER
  if (mode == MODE_CONTROLLER) {
    handleJoystickFrames();
  }

  // Scripted sequence
  if (mode == MODE_SCRIPT) {

    if (!centered) {
      doCenter();
    }

    setMotorsEnabled(true);

    Serial.println("Running preset");
    runPreset(active_preset);
  
    // clean shutdown
    for (uint8_t m = 0; m < NUM_MOTORS; m++) {
      analogWrite(PWM_PINS[m], 0);
    }
  
    setMotorsEnabled(false);
    mode = MODE_IDLE;
    active_preset = PRESET_NONE;
  
    Serial.println("Preset complete. Motors disabled.");
  }

  updateLEDs();
  delay(2);
}

inline void doCenter() {
    Serial.println("Centering: retracting all actuators...");
    stop_requested = false;
    setMotorsEnabled(true);

    for (uint8_t m = 0; m < NUM_MOTORS; m++) {
        digitalWrite(DIR_PINS[m], RETRACT);
        analogWrite(PWM_PINS[m], MAX_PWM);
    }

    delay(RESET_DELAY);

    for (uint8_t m = 0; m < NUM_MOTORS; m++) {
        analogWrite(PWM_PINS[m], 0);
    }

    setMotorsEnabled(false);
    centered = true;
    mode = MODE_IDLE;
    Serial.println("Centered. Motors disabled.");
}

// Text commands: center/start/controller/stop

// Parses incoming serial commands and updates system state.
// Commands include:
// - stop        -> halt all motion
// - center      -> return platform to home position
// - controller  -> enable joystick control
// - start <id>  -> run preset motion
// - calibrate   -> recalibrate actuator limits
void checkTextCommands() {
  static String buf = "";

  while (Serial.available()) {
    char c = Serial.peek();

    // If it's a joystick frame, don’t eat it here
    if (c == 'J') return;

    c = Serial.read();
    if (c == '\n' || c == '\r') {
      buf.trim();
      buf.toLowerCase();

      if (buf.length() == 0) { buf = ""; continue; }

      if (buf == "fantest") {
        Serial.println("Fan test: ramping pin 9 up then down...");
        pinMode(9, OUTPUT);
        for (int p = 0; p <= 255; p += 5) { analogWrite(9, p); delay(30); }
        for (int p = 255; p >= 0; p -= 5) { analogWrite(9, p); delay(30); }
        analogWrite(9, 0);
        Serial.println("Fan test done.");
      }
      else if (buf == "fans") {
        // Count tach pulses on both fans simultaneously over 500ms window
        // PC fans: 2 pulses per revolution
        Serial.println("FAN STATUS (measuring RPM for 500ms...):");
        uint32_t count1 = 0, count2 = 0;
        bool last1 = digitalRead(FAN_TACH_1);
        bool last2 = digitalRead(FAN_TACH_2);
        uint32_t t_end = millis() + 500;
        while (millis() < t_end) {
          bool cur1 = digitalRead(FAN_TACH_1);
          bool cur2 = digitalRead(FAN_TACH_2);
          if (last1 && !cur1) count1++;  // falling edge
          if (last2 && !cur2) count2++;
          last1 = cur1;
          last2 = cur2;
        }
        // RPM = (edges / 2 pulses_per_rev) / 0.5s * 60 = edges * 60
        uint32_t rpm1 = count1 * 60;
        uint32_t rpm2 = count2 * 60;
        Serial.print("  FAN_1 (pin 9,  tach pin 14): "); Serial.print(rpm1); Serial.println(" RPM");
        Serial.print("  FAN_2 (pin 10, tach pin 15): "); Serial.print(rpm2); Serial.println(" RPM");
      }
      else if (buf == "enable") {
        setMotorsEnabled(true);
        Serial.println("Motors enabled.");
      }
      else if (buf.startsWith("extend ")) {
        int n = buf.substring(7).toInt();
        if (n >= 1 && n <= NUM_MOTORS) {
          setMotorsEnabled(true);
          digitalWrite(DIR_PINS[n-1], EXTEND);
          analogWrite(PWM_PINS[n-1], 128);
          Serial.print("Extending M"); Serial.println(n);
        }
      }
      else if (buf.startsWith("retract ")) {
        int n = buf.substring(8).toInt();
        if (n >= 1 && n <= NUM_MOTORS) {
          setMotorsEnabled(true);
          digitalWrite(DIR_PINS[n-1], RETRACT);
          analogWrite(PWM_PINS[n-1], 128);
          Serial.print("Retracting M"); Serial.println(n);
        }
      }
      else if (buf == "mstop") {
        stopMotors();
        setMotorsEnabled(false);
        Serial.println("Motors stopped.");
      }
      else if (buf == "pots") {
        Serial.println("POT READINGS (raw ADC):");
        for (uint8_t m = 0; m < NUM_MOTORS; m++) {
          int raw = getAverageReading(m);
          float ext_in = mapFloat(raw, ZERO_POS[m], END_POS[m], 0.0f, SAFE_MAX_INCHES);
          Serial.print("  M"); Serial.print(m + 1);
          Serial.print(": raw="); Serial.print(raw);
          Serial.print("  ext="); Serial.print(ext_in, 3); Serial.println("\"");
        }
      }
      else if (buf == "stop") {
        stop_requested = true;
        mode = MODE_IDLE;
        active_preset = PRESET_NONE;
        centered = false;
        setMotorsEnabled(false);
      
        Serial.println("STOP received. All motion halted.");
      }
      else if (buf == "center") {
        doCenter();
      }
      else if (buf == "controller") {
        if (!centered) {
          Serial.println("Refusing: run 'center' first");
        } else {
          setMotorsEnabled(true);
          mode = MODE_CONTROLLER;
          Serial.println("Controller mode ON (J frames drive platform). Type 'stop' to exit.");
        }
      }
      else if (buf == "calibrate") {
        calibration_valid = true;
        Serial.println("Calibrating");
        calibrate();
        Serial.println("Calibrating Done");
      }
      else if (buf.startsWith("start")) {
        //doCenter();

        String arg = buf.substring(5);
        arg.trim();
      
        if (arg == "demo") {
          active_preset = PRESET_DEMO;
        }
        else if (arg == "figure8") {
          active_preset = PRESET_FIGURE8;
        }
        else if (arg == "orbit") {
          active_preset = PRESET_ORBIT;
        }
        else if (arg == "wave") {
          active_preset = PRESET_WAVE;
        }
        else {
          Serial.println("Unknown preset");
          buf = "";
          return;
        }
      
        setMotorsEnabled(true);
        mode = MODE_SCRIPT;
      
        Serial.print("Starting preset: ");
        Serial.println(arg);
      }
      else {
        Serial.print("Unknown command: ");
        Serial.println(buf);
      }

      buf = "";
    } else {
      buf += c;
    }
  }
}

// Joystick frames moved to include/joystick_functions.h


// Math computations
inline int getAverageReading(uint8_t motor)
{
    reading_sum = 0;
    for (reading = 0; reading < NUM_READINGS; ++reading) {
        reading_sum += analogRead(POT_PINS[motor]);
    }
    return reading_sum / NUM_READINGS;
}

inline float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline void moveAll(MotorDirection dir)
{
  for (motor = 0; motor < NUM_MOTORS; ++motor) {
    digitalWrite(DIR_PINS[motor], dir);
    analogWrite(PWM_PINS[motor], MAX_PWM);
  }
}

/*
 * Calibration routine:
 * 1. Extend all actuators to safe maximum length
 * 2. Record end readings (max extension)
 * 3. Retract to baseline and record zero readings
 * 4. Validate readings against thresholds
 * 5. Update calibration constants if valid
 */
inline void calibrate()
{
  // Extend to a configurable safe maximum instead of hard stop
  setMotorsEnabled(true);
  Serial.println("Extending to safe max");
  for (motor = 0; motor < NUM_MOTORS; ++motor) {
    digitalWrite(DIR_PINS[motor], EXTEND);
    analogWrite(PWM_PINS[motor], MAX_PWM);
  }

  bool all_done = false;
  unsigned long t0 = millis();
  while (!all_done && (millis() - t0) < RESET_DELAY) {
    all_done = true;
    for (motor = 0; motor < NUM_MOTORS; ++motor) {
      // If this motor already stopped, skip
      // Check current extension in inches using default calib anchors
      int16_t r = getAverageReading(motor);
      float ext_in = mapFloat(r, ZERO_POS[motor], END_POS[motor], 0.0f, SAFE_MAX_INCHES);
      if (ext_in >= SAFE_MAX_INCHES) {
        analogWrite(PWM_PINS[motor], 0);
        end_readings[motor] = r; // record safe-max reading per actuator
      } else {
        all_done = false;
      }
    }
    delay(5);
  }

  // Print reached extension per actuator at safe max
  Serial.println("<CALIB_EXTEND> Reached extensions (in)");
  for (uint8_t m = 0; m < NUM_MOTORS; m++) {
    float ext_in = mapFloat(end_readings[m], ZERO_POS[m], END_POS[m], 0.0f, SAFE_MAX_INCHES);
    Serial.print("  M"); Serial.print(m + 1);
    Serial.print(": "); Serial.println(ext_in, 3);
  }

  // Basic validation: ensure each actuator reached near safe max
  calibration_valid = true;
  for (motor = 0; motor < NUM_MOTORS; ++motor) {
    float ext_in = mapFloat(end_readings[motor], ZERO_POS[motor], END_POS[motor], 0.0f, SAFE_MAX_INCHES);
    if (ext_in < (SAFE_MAX_INCHES - 0.25f)) { // 0.25" tolerance
      calibration_valid = false;
      break;
    }
  }

  // Retract back toward baseline to capture zero_readings
  Serial.println("Retracting toward baseline");
  for (motor = 0; motor < NUM_MOTORS; ++motor) {
    digitalWrite(DIR_PINS[motor], RETRACT);
    analogWrite(PWM_PINS[motor], MAX_PWM);
  }

  t0 = millis();
  while ((millis() - t0) < RESET_DELAY) {
    bool all_zero = true;
    for (motor = 0; motor < NUM_MOTORS; ++motor) {
      int16_t r = getAverageReading(motor);
      float ext_in = mapFloat(r, ZERO_POS[motor], END_POS[motor], 0.0f, SAFE_MAX_INCHES);
      if (ext_in <= 0.1f) { // close to baseline
        analogWrite(PWM_PINS[motor], 0);
        zero_readings[motor] = r;
      } else {
        all_zero = false;
      }
    }
    if (all_zero) break;
    delay(5);
  }

  // Validate baseline
  if (calibration_valid) {
    for (motor = 0; motor < NUM_MOTORS; ++motor) {
      calibration_valid = (abs(zero_readings[motor] - ZERO_POS[motor]) < OFF_THRESHOLD);
      if (!calibration_valid) break;
    }
  }

  // Apply new anchors if valid: safe-max and baseline
  if (calibration_valid) {
    for (motor = 0; motor < NUM_MOTORS; ++motor) {
      END_POS[motor]  = end_readings[motor];
      ZERO_POS[motor] = zero_readings[motor];
    }

    // Optional post-calibration motion: sweep left and right
    Serial.println("Post-calibration: sweeping LEFT and RIGHT");
    stop_requested = false;
    setMotorsEnabled(true);
    enable_move_stats = true;
    // Use doubled lateral targets for sweep only
    float T_RIGHT_SWEEP[3] = { T_RIGHT[0] * 2.0f, T_RIGHT[1], T_RIGHT[2] };
    float T_LEFT_SWEEP[3]  = { T_LEFT[0]  * 2.0f, T_LEFT[1],  T_LEFT[2]  };
    moveplat(2.0f, zero_length, T_CENTER, T_RIGHT_SWEEP, R0, R0);
    moveplat(2.0f, zero_length, T_RIGHT_SWEEP, T_LEFT_SWEEP, R0, R0);
    moveplat(2.0f, zero_length, T_LEFT_SWEEP, T_CENTER, R0, R0);
    enable_move_stats = false;
  }

  // Stop all motion and disable motors for safety
  for (uint8_t m = 0; m < NUM_MOTORS; m++) {
    analogWrite(PWM_PINS[m], 0);
  }
  setMotorsEnabled(false);
}

void lerp(const float pos0[3], const float pos1[3], float t, float T[3]) {
  for (int i = 0; i < 3; i++) {
    T[i] = (1 - t) * pos0[i] + t * pos1[i];
  }
}

/*
 * Core motion function:
 * Interpolates platform position and rotation over time,
 * computes required actuator lengths using kinematics,
 * and applies closed-loop control (P controller) to motors.
 *
 * Steps:
 * - Interpolate position + rotation (lerp + slerp)
 * - Compute actuator lengths from geometry
 * - Compare with current readings
 * - Compute velocity command
 * - Convert to PWM + direction
 */
inline void moveplat(float duration, float length_min, float pos0[3], float pos1[3], Quaternion q0, Quaternion q1)
{
  int steps = (int)(duration * 10.0f);
  if (steps < 1) steps = 1;

  float Kp = 0.25f;

  float start_ext[NUM_MOTORS];
  
  float end_ext[NUM_MOTORS];
  float max_ext[NUM_MOTORS];
  for (uint8_t m = 0; m < NUM_MOTORS; m++) {
    start_ext[m] = 0.0f;
    end_ext[m]   = 0.0f;
    max_ext[m]   = -1e9f;
  }

  float prev_length_now[NUM_MOTORS];
  bool first_sample = true;

  static float vel_filter_state[NUM_MOTORS] = {0};


  for (int step = 0; step <= steps; step++) {

    checkTextCommands();

    if (stop_requested) {
      for (uint8_t m = 0; m < NUM_MOTORS; m++) {
        analogWrite(PWM_PINS[m], 0);
      }
      setMotorsEnabled(false);
      stop_requested = false;
      return;
    }

    // If in controller mode, check for new joystick input and stop if button released
    if (mode == MODE_CONTROLLER) {
      updateJoystickInputFlag();
      if (!joystick_input_active) {
        for (uint8_t m = 0; m < NUM_MOTORS; m++) {
          analogWrite(PWM_PINS[m], 0);
        }
        return;
      }
    }

    unsigned long start_time = millis();
    float t      = float(step) / steps;
    float t_next = float(step + 1) / steps;

    Quaternion rot_t    = slerp(q0, q1, t);
    Quaternion rot_next = slerp(q0, q1, t_next);

    float T_t[3], T_next[3];
    lerp(pos0, pos1, t, T_t);
    lerp(pos0, pos1, t_next, T_next);

    float pwm_local[NUM_MOTORS];
    float vel_local[NUM_MOTORS];   // store velocity per motor (commanded)
    float measured_vel_local[NUM_MOTORS];
    
    


    for (motor = 0; motor < NUM_MOTORS; ++motor) {
      const float* base = bases[motor];
      const float* plat = plats[motor];

      float rotated[3], rotated_next[3];
      rot_t.rotate(plat).toVector(rotated);
      rot_next.rotate(plat).toVector(rotated_next);

      float plat_t[3], plat_next[3];
      for (int i = 0; i < 3; i++) {
        plat_t[i]    = rotated[i]      + T_t[i]    + plat_0[i];
        plat_next[i] = rotated_next[i] + T_next[i] + plat_0[i];
      }

      float length_t = 0, length_next = 0;
      for (int i = 0; i < 3; i++) {
        float d      = plat_t[i]    - base[i];
        float d_next = plat_next[i] - base[i];
        length_t    += d * d;
        length_next += d_next * d_next;
      }
      length_t    = sqrt(length_t);
      length_next = sqrt(length_next);

      // Clamp desired lengths to safe stroke
      float max_len = length_min + SAFE_MAX_INCHES;
      if (length_t > max_len) length_t = max_len;
      if (length_next > max_len) length_next = max_len;

      float reading_now = getAverageReading(motor);
      float length_now = mapFloat(reading_now, ZERO_POS[motor], END_POS[motor], 0, SAFE_MAX_INCHES) + length_min;

      float measured_vel = 0.0f;

      if (!first_sample) {
        float dt = duration / steps;   // seconds per step
        measured_vel = (length_now - prev_length_now[motor]) / dt;
      }
      prev_length_now[motor] = length_now;

      float error = length_t - length_now;
      
      bool motors_enabled = (digitalRead(ENABLE_MOTORS) == LOW);
      
      float vel = (length_next - length_t + error * Kp) * steps / duration;
      
      if (!motors_enabled) {
        vel = 0.0f;
        measured_vel = 0.0f;
      }
      
      float alpha = 0.2f;   // tuning parameter
      vel_filter_state[motor] += alpha * (measured_vel - vel_filter_state[motor]);
      measured_vel = vel_filter_state[motor];

      measured_vel_local[motor] = measured_vel;
      vel_local[motor] = vel;

      // Clamp velocity range
      if (vel > 0.2f) vel = 0.2f; else if (vel < -0.2f) vel = -0.2f;

      // clamp for testing
      //if (vel > 0.75f) vel = 0.75f; else if (vel < -0.75f) vel = -0.75f;
      vel_local[motor] = vel;

      // Map magnitude to PWM, keep sign for direction
      int pwm_speed = (int)mapFloat(fabsf(vel), 0.0f, 2.0f, 0.0f, 255.0f);
      if (pwm_speed < 10) pwm_speed = 0; // lowered deadzone for debugging
      pwm_local[motor] = (vel >= 0.0f) ? pwm_speed : -pwm_speed;

      // Stats: track start, end, and max extension (inches)
      float ext_in = length_now - length_min;
      if (step == 0) start_ext[motor] = ext_in;
      if (ext_in > max_ext[motor]) max_ext[motor] = ext_in;
      if (step == steps) end_ext[motor] = ext_in;
    }

    for (motor = 0; motor < NUM_MOTORS; ++motor) {
        int p = (int)pwm_local[motor];
        if (p == 0) {
            analogWrite(PWM_PINS[motor], 0);
        } else {
            digitalWrite(DIR_PINS[motor], (p > 0) ? EXTEND : RETRACT);
            analogWrite(PWM_PINS[motor], abs(p));
        }
    }

    first_sample = false;

    bool motors_enabled = (digitalRead(ENABLE_MOTORS) == LOW);

    if (motors_enabled) {

      Serial.print("<CMD_VEL_PER_SEC> ");
      for (uint8_t m = 0; m < NUM_MOTORS; m++) {
        Serial.print(vel_local[m], 3);
        if (m < NUM_MOTORS - 1) Serial.print(", ");
      }
      Serial.println();

      Serial.print("<MEAS_VEL> ");
      for (uint8_t m = 0; m < NUM_MOTORS; m++) {
        Serial.print(measured_vel_local[m], 3);
        if (m < NUM_MOTORS - 1) Serial.print(", ");
      }
      Serial.println();

      Serial.print("<POS_IN> ");
      for (uint8_t m = 0; m < NUM_MOTORS; m++) {
          float reading_now = getAverageReading(m);
          float length_now = mapFloat(reading_now, ZERO_POS[m], END_POS[m], 0, SAFE_MAX_INCHES);
          Serial.print(length_now, 3);
          if (m < NUM_MOTORS - 1) Serial.print(", ");
      }
      Serial.println();

    }

    unsigned long target_delay = (unsigned long)((duration * 1000.0f) / (float)steps);
    while ((millis() - start_time) < target_delay) { }
  }

  // Print per-actuator move stats if enabled
  if (enable_move_stats) {
    Serial.println("<MOVE_STATS> Actuator extensions (in)");
    for (uint8_t m = 0; m < NUM_MOTORS; m++) {
      Serial.print("  M"); Serial.print(m + 1);
      Serial.print(": start="); Serial.print(start_ext[m], 3);
      Serial.print(", end=");   Serial.print(end_ext[m], 3);
      Serial.print(", max=");   Serial.println(max_ext[m], 3);
    }
  }
}