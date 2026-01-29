#include <Arduino.h>
#include <math.h>
#include  "pin_layout.h"
#include "Quaternion.h"
#include "platform.h"

// Actuator variables
uint8_t pwm[NUM_MOTORS];            // current PWM for each actuator
MotorDirection dir[NUM_MOTORS];     // current direction for each actuator (EXTEND or RETRACT)

// Position variables
int16_t pos[NUM_MOTORS];            // current position (measured by analog read) of each actuator
int16_t input[NUM_MOTORS];          // intermediate input retrieved from the serial buffer
uint16_t desired_pos[NUM_MOTORS];   // desired (user-inputted and validated) position of each actuator

// Calibration variables
int16_t end_readings[NUM_MOTORS];
int16_t zero_readings[NUM_MOTORS];

bool calibration_valid;

// Time variables (for printing)
unsigned long current_time;         // current time (in millis); used to measure difference from previous_time
unsigned long previous_time;        // last recorded time (in millis); measured from execution start or last print
// No watchdog timestamp; movement steps per frame

// Iterator/sum variables
uint8_t motor;                      // used to iterate through actuators by their indexing (0 to NUM_MOTORS - 1)
uint8_t reading;                    // used to iterate through analog reads (0 to NUM_READINGS - 1)
int32_t reading_sum;                // sum of multiple readings to be averaged for a final value

//Declare all quaternion rotations, with respect to (WRT) the home position
Quaternion R0;
Quaternion R1;
Quaternion R2;
Quaternion R3;

//Declare the transformations WRT the top of the platform when all actuators are retracted
float T0[3] = {0, 0, 0};
float T1[3] = {0, 0, 2};
float T2[3] = {1, -2, 3};
float TX[3] = {3, 0, 2};
float TY[3] = {0, 3, 2};
float TZ[3] = {0, 0, 5};

bool valid = false; // start disabled; no motion unless controller frames received
float zero_length = 0;
float dur;

// Added these
void calibrate();
void moveplat(float dur,
    float zero_length,
    float* T0,
    float* T1,
    Quaternion R0,
    Quaternion R1);
// Forward declare to allow use in loop() before definition
void moveAll(MotorDirection dir);


void setup()
{
    // Initialize serial immediately to avoid hanging at startup
    Serial.begin(BAUD_RATE);
    Serial.setTimeout(30);

    // Initialize pins
    for (motor = 0; motor < NUM_MOTORS; ++motor)
    {
        pinMode(DIR_PINS[motor], OUTPUT);
        digitalWrite(DIR_PINS[motor], LOW);

        pinMode(PWM_PINS[motor], OUTPUT);
        analogWrite(PWM_PINS[motor], 0);

        pinMode(POT_PINS[motor], INPUT);
    }

    // Initialize actuator enable switches (disabled by default)
    pinMode(ENABLE_MOTORS, OUTPUT);
    pinMode(ENABLE_MOTORS_2, OUTPUT);
    digitalWrite(ENABLE_MOTORS, HIGH);
    digitalWrite(ENABLE_MOTORS_2, HIGH);

    // For safety, ensure initial actuator speed variables are 0
    for (motor = 0; motor < NUM_MOTORS; ++motor)
    {
        pwm[motor] = 0;
    }

    // Serial already initialized above

    //find the length at 0" of extension
    //uses pythragorean thm and vector math
    for (int i = 0; i < 3; i++) {
      float d = plat_0[i] + plat_1[i] - base_1[i];
     
      zero_length += d * d;
    }
    
    zero_length = sqrt(zero_length);
    // Do not auto-calibrate or move at startup; controller-only movement
    calibration_valid = false;

    //calibrate_vel();

    //for azi_alt_to_rot, you declare an azimuthal angle (heading) and altitude angle (WRT to the horizon) of tilt, which gets converted into a quaternion rotation
    //azi_alt_to_rot(0.0, 10.0) signifies a tilt of 10 degrees WRT toward a heading of 0 degrees
    //more info in Quaternion library
    R0 = azi_alt_to_rot(0.0, 0.0);
    R1 = azi_alt_to_rot(0.0, 10.0);
    R2 = azi_alt_to_rot(90.0, 10.0);
    R3 = Quaternion(cos(PI/12), 0, 0, sin(PI/12));

    dur = 2;

    Serial.println("Ready. Controller control active.");
}

// Current pose for incremental control
static float T_cur[3] = {0, 0, 0};
static Quaternion R_cur = Quaternion(1, 0, 0, 0);
// Last commanded target to avoid re-triggering same motion repeatedly
// Removed last-command gating

void loop() {
    // Fast polling for commands / joystick frames
    if (Serial.available() > 0) {
        char c = Serial.peek();
        if (c == 'J') {
            Serial.read(); // consume 'J'
            // Expect 6 floats: ax ay az azi alt yaw followed by newline
            float ax = Serial.parseFloat();
            float ay = Serial.parseFloat();
            float az = Serial.parseFloat();
            float azi = Serial.parseFloat();
            float alt = Serial.parseFloat();
            float yaw = Serial.parseFloat();

            // process per frame

            // Echo received values for debugging
            Serial.print("<RX J> ax="); Serial.print(ax, 3);
            Serial.print(" ay="); Serial.print(ay, 3);
            Serial.print(" az="); Serial.print(az, 3);
            Serial.print(" azi="); Serial.print(azi, 1);
            Serial.print(" alt="); Serial.print(alt, 1);
            Serial.print(" yaw="); Serial.print(yaw, 1);
            Serial.print("\r\n");

            // Treat ax/ay/az as the only truth (simple D-pad + RB/LB mapping)
            const float dead = 0.05f;
            bool active = (fabsf(ax) > dead || fabsf(ay) > dead || fabsf(az) > dead);

            if (!active) {
                // Idle: no motion, hold drivers enabled with PWM=0
                valid = false;
                for (motor = 0; motor < NUM_MOTORS; ++motor) analogWrite(PWM_PINS[motor], 0);
                digitalWrite(ENABLE_MOTORS, LOW);
                digitalWrite(ENABLE_MOTORS_2, LOW);
            } else {
                // Quantize to -1, 0, +1 for clean single-axis steps
                ax = (ax > dead) ? 1.0f : (ax < -dead ? -1.0f : 0.0f);
                ay = (ay > dead) ? 1.0f : (ay < -dead ? -1.0f : 0.0f);
                az = (az > dead) ? 1.0f : (az < -dead ? -1.0f : 0.0f);
                // Map axes to inches and degrees
                // No D-pad flag logic; use ax/ay/az directly

                // Build orientation: ignore 'azi' for tilt to avoid unintended rotation
                Quaternion tilt = azi_alt_to_rot(0.0f, alt);
                float yaw_rad_half = (yaw * PI / 180.0f) * 0.5f;
                Quaternion q_yaw(cos(yaw_rad_half), 0, 0, sin(yaw_rad_half));
                Quaternion q_target = tilt * q_yaw;

                // Incremental stepping: add a small step to current target each frame
                const float STEP = 0.05f;   // 0.05 in per frame (~1.5 in/s @ 30Hz)
                float T_target[3] = {
                    T_cur[0] + ax * STEP,
                    T_cur[1] + ay * STEP,
                    T_cur[2] + az * STEP
                };

                // Safety clamps: limit workspace
                T_target[0] = constrain(T_target[0], -1.0f, 1.0f);
                T_target[1] = constrain(T_target[1], -1.0f, 1.0f);
                T_target[2] = constrain(T_target[2], -1.0f, 1.0f);

                // Allow movement only when button is held and inputs are active
                valid = true;
                digitalWrite(ENABLE_MOTORS, LOW);
                digitalWrite(ENABLE_MOTORS_2, LOW);

                // Move in very short increments per frame for fast stop on release
                moveplat(0.05f, zero_length, T_cur, T_target, R_cur, q_target);
                for (int i = 0; i < 3; ++i) T_cur[i] = T_target[i];
                R_cur = q_target;
            }
        } else if (c == 'S') {
            // Controller-only: ignore stop commands
            Serial.read();
        } else if (c == 'E') {
            // Controller-only: ignore enable commands
            Serial.read();
        } else if (c == 'D') {
            // Controller-only: ignore disable commands
            Serial.read();
        } else if (c == 'U') {
            // Ignore jog commands in controller-only mode
            Serial.read();
        } else if (c == 'R') {
            // Ignore jog commands in controller-only mode
            Serial.read();
        } else {
            // Consume and ignore whitespace/newlines and any unexpected bytes
            char u = Serial.read();
            (void)u; // no logging to avoid noise on line endings
        }
    }
}

//Gets the average values from the feedback pins
//Needed to smooth out the noise
//TO DO: see how changing NUM_READINGS affects the stability of the readings and the delay caused by taking a reading
inline int getAverageReading(uint8_t motor)
{
    reading_sum = 0;
    for (reading = 0; reading < NUM_READINGS; ++reading)
    {
        reading_sum += analogRead(POT_PINS[motor]);
    }
    return reading_sum / NUM_READINGS;
}

//Standard mapping function
inline float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//Moves all motors
inline void moveAll(MotorDirection dir)
{
    for (motor = 0; motor < NUM_MOTORS; ++motor)
    {
        digitalWrite(DIR_PINS[motor], dir);
        analogWrite(PWM_PINS[motor], MAX_PWM);
    }
}


inline void calibrate()
{
    // Extend all actuators
    moveAll(EXTEND);
    #if ENABLE_PRINT_HEADERS
    Serial.println("Extending");
    #endif
    delay(RESET_DELAY);

    // Stop the extension, get averaged analog readings
    for (motor = 0; motor < NUM_MOTORS; ++motor)
    {
        analogWrite(PWM_PINS[motor], 0);
        end_readings[motor] = getAverageReading(motor);

        // Check if the motors are powered (reading is valid)
        calibration_valid = (abs(end_readings[motor] - END_POS[motor]) < OFF_THRESHOLD);
        if (!calibration_valid)
        {
            break;
        }
    }
    // Retract all actuators
    moveAll(RETRACT);
    #if ENABLE_PRINT_HEADERS
    Serial.println("Retracting");
    #endif
    delay(RESET_DELAY);

    // Stop the retraction, get averaged analog readings
    if (calibration_valid)
    {
        for (motor = 0; motor < NUM_MOTORS; ++motor)
        {
            analogWrite(PWM_PINS[motor], 0);
            zero_readings[motor] = getAverageReading(motor);

            // Check if the motors are powered (reading is valid)
            calibration_valid = (abs(zero_readings[motor] - ZERO_POS[motor]) < OFF_THRESHOLD);
            if (!calibration_valid)
            {
                break;
            }
        }
    }

    // Set the new calibration values if found to be valid
    if (calibration_valid)
    {
        for (motor = 0; motor < NUM_MOTORS; ++motor)
        {
            END_POS[motor] = end_readings[motor];
            ZERO_POS[motor] = zero_readings[motor];
        }
    }

    // Print the calibration result (new max/min values, or a warning)
    #if ENABLE_PRINT_HEADERS
    {
        if (calibration_valid)
        {
            Serial.print("<COM> Finished calibration:\n");

            // Print minimum positions
            Serial.print("      MIN POS: ");
            for (motor = 0; motor < NUM_MOTORS; ++motor)
            {
                Serial.print(ZERO_POS[motor]);
                Serial.print(" ");
            }
            Serial.print("\n");

            // Print maximum positions
            Serial.print("      MAX POS: ");
            for (motor = 0; motor < NUM_MOTORS; ++motor)
            {
                Serial.print(END_POS[motor]);
                Serial.print(" ");
            }
            Serial.print("\n");

            // Print CR to end message block
            Serial.print("\r");
        }

        else
        {
            // Print an error message for calibration failure
            Serial.print("<COM> Failed calibration (using default values).\n");
            Serial.print("      Please verify that the actuators are powered on.\n\r");
        }
    }
    #endif // ENABLE_PRINT
}

//Function where allows you to linearly interpolate between two positions from t=0 to t=1
//Takes array T as an argument, and changes the values of T (instead of returning a new array like python)
void lerp(const float pos0[3], const float pos1[3], float t, float T[3]) {
  
  for (int i = 0; i < 3; i++) {
    T[i] = (1 - t) * pos0[i] + t * pos1[i];
  }
}

//Preliminary work to move the platform in a [flat or upward/downward] and [constant radius or expanding/contracting] spiral
//TO DO: use OOP to make a class that encompasses a variety of motions that moveplat() can take as an argument
//I tried to do this but I ran out of time
////void spiral(const float r0, const float rf, const float h, const float T_0[3], int steps, float T[3]) {
////  float r_t = t * rf + (1 - t) * r0;
////
////  for(t in steps; t<steps; t++){
////  T[0][step] = r_t * cos(2*PI*t);
////  T[1][step] = r_t * sin(2*PI*t);
////  T[2][step] = h * t;
////  }
////}

//moves the platform
inline void moveplat(float duration, float length_min, float pos0[3], float pos1[3], Quaternion q0, Quaternion q1){

    int steps = (int)(duration * 30); // smoothing (~30 Hz)
    if (steps < 1) steps = 1;
    float Kp = 0.0f; // remove proportional correction to eliminate jitter near target
    
    //keeps track of average error for iteration of Kp for PID
    float avg_error[NUM_MOTORS] = { 0, 0, 0, 0, 0, 0 };

    for (int step = 0; step < steps; ++step) {
        unsigned long start_time = millis();
        float t = float(step) / steps;
        // No watchdog here; loop drives short steps per frame
        float t_next = float(step+1) / steps;

        //slerp is similar to lerp, but interpolations between rotations instead of translations
        //more info in Quaternion library
        Quaternion rot_t = slerp(q0, q1, t);
        Quaternion rot_next = slerp(q0, q1, t_next);
  
        //Declare and define translations
        float T_t[3];
        float T_next[3];
        
        lerp(pos0, pos1, t, T_t);
        lerp(pos0, pos1, t_next, T_next);
  
        float pwm[6];

        //performs inverse kinematics on the motors to figure out the current and next positions
        //figures out the transformation and rotation for a given step, then applies to each point of the platform
        for (motor = 0; motor < NUM_MOTORS; ++motor) {
            // CHANGED TO CONST FLOAT*
            const float* base = bases[motor];
            const float* plat = plats[motor];
  
            float rotated[3];
            float rotated_next[3];
            
            rot_t.rotate(plat).toVector(rotated);
            rot_next.rotate(plat).toVector(rotated_next);
            
            float plat_t[3];
            float plat_next[3];
            
            for (int i = 0; i < 3; i++) {
                plat_t[i] = rotated[i] + T_t[i] + plat_0[i];
                plat_next[i] = rotated_next[i] + T_next[i] + plat_0[i];
            }
  
            float length_t = 0;
            float length_next = 0;

            //figures out the length of the actuator at the current and next step
            for (int i = 0; i < 3; i++) {
                float d = plat_t[i] - base[i];
                float d_next = plat_next[i] - base[i];
                
                length_t += d * d;
                length_next += d_next * d_next;
            }
            length_t = sqrt(length_t);
            length_next = sqrt(length_next);
            
            // Pure feedforward velocity based on IK only (ignore pot feedback until calibrated)
            float vel = (length_next - length_t) * steps / duration;

              
              //safety measure to stop the platform in case the actuators drift too far
//            if (abs(error) > .5){
//              Serial.print("Invalid Length: ");
//              Serial.print(length_t);
//              Serial.print(", ");
//              Serial.println(length_now);
//              //valid = false;
//            }
            
                        //cap speed to ~0.8"/s for much slower motion
                        if (fabsf(vel) > 0.3f){
                            vel = 0.3f * ((vel > 0)? 1:-1);
                        }

            //maps speed to a pwm value
            int pwm_speed = mapFloat(vel, 0, 0.3f, 0, 255);
            
            //sets lower cap for speed
            //motor will not move below this pwm
                        if (abs(pwm_speed) < 20){
                            pwm_speed = 0;
                        }
  
            pwm[motor] = pwm_speed;
        }
  
                if (!valid){
          for (motor = 0; motor < NUM_MOTORS; ++motor)
          {
                        pwm[motor] = 0;
          }
          break;
        }
        
        Serial.print('\n');
       
        // Sets the pwm values of the pins and this the speed of the motors
        for (motor = 0; motor < NUM_MOTORS; ++motor) {
            digitalWrite(DIR_PINS[motor], (pwm[motor]>0) ? EXTEND:RETRACT);
            analogWrite(PWM_PINS[motor], abs(pwm[motor]));
//            Serial.print(pwm[motor]);
//            if (motor < 5) Serial.print(", ");
        }
        //Serial.println("]");
                unsigned long target_delay = (unsigned long)((duration * 1000.0f) / (float)steps);
                unsigned long end_time = start_time + target_delay;
                //more accurate delay function
                while (millis() < end_time) {
                    // busy wait
                }
                //Serial.println(cur_time - start_time);
            }

    // Ensure motors stop at end of move
    for (motor = 0; motor < NUM_MOTORS; ++motor) {
            analogWrite(PWM_PINS[motor], 0);
    }

    Serial.print('\n');
//  Serial.print("Error Accumulated: ");
//  for (motor = 0; motor < NUM_MOTORS; ++motor)
//  {
//     analogWrite(PWM_PINS[motor], 0);
//     Serial.print(avg_error[motor], 6);
//     Serial.print(" ");
//  }
//  Serial.print('\n');
//  Serial.print('\n');
}
