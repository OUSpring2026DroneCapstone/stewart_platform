#include "platform.h"
#include "Quaternion.h"

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

bool valid = true;
float zero_length = 0;
float dur;

void setup()
{
    while (!Serial);

    // Initialize pins
    for (motor = 0; motor < NUM_MOTORS; ++motor)
    {
        pinMode(DIR_PINS[motor], OUTPUT);
        digitalWrite(DIR_PINS[motor], LOW);

        pinMode(PWM_PINS[motor], OUTPUT);
        analogWrite(PWM_PINS[motor], 0);

        pinMode(POT_PINS[motor], INPUT);
    }

    // Initialize actuator enable switches
    pinMode(ENABLE_MOTORS, OUTPUT);
    digitalWrite(ENABLE_MOTORS, LOW);

    // For safety, set initial actuator settings and speed to 0
    for (motor = 0; motor < NUM_MOTORS; ++motor)
    {
        pwm[motor] = MIN_PWM;
    }

    // Initialize serial communication
    Serial.begin(BAUD_RATE);

    //find the length at 0" of extension
    //uses pythragorean thm and vector math
    for (int i = 0; i < 3; i++) {
      float d = plat_0[i] + plat_1[i] - base_1[i];
     
      zero_length += d * d;
    }
    
    zero_length = sqrt(zero_length);
    Serial.println(zero_length);
    delay(1000);

    // Run calibration
    calibration_valid = true;
    delay(1000);
    Serial.println("Calibrating");
    calibrate();
    Serial.println("Calibrating Done");

    //calibrate_vel();

    //for azi_alt_to_rot, you declare an azimuthal angle (heading) and altitude angle (WRT to the horizon) of tilt, which gets converted into a quaternion rotation
    //azi_alt_to_rot(0.0, 10.0) signifies a tilt of 10 degrees WRT toward a heading of 0 degrees
    //more info in Quaternion library
    R0 = azi_alt_to_rot(0.0, 0.0);
    R1 = azi_alt_to_rot(0.0, 10.0);
    R2 = azi_alt_to_rot(90.0, 10.0);
    R3 = Quaternion(cos(PI/12), 0, 0, sin(PI/12));

    dur = 2;

    
    Serial.println("Up 2");
    moveplat(dur, zero_length, T0, T1, R0, R0);

    //when chaining motions like this, make sure that the initial translation and rotation for the current motion...
    //correspond to the final translation and rotation of the last motion
    //i.e. if you ended on T1 and R0 on the last move, start with T1 and R0 on the next
    Serial.println("Left");
    moveplat(dur, zero_length, T1, TX, R0, R0);
    Serial.println("Right");
    moveplat(dur, zero_length, TX, T1, R0, R0);

    Serial.println("Back");
    moveplat(dur, zero_length, T1, TY, R0, R0);
    Serial.println("Forth");
    moveplat(dur, zero_length, TY, T1, R0, R0);
    
    Serial.println("Up");
    moveplat(5, zero_length, T1, TZ, R0, R0);
    //moveplat(5, zero_length, TZ, T1, R0, R0);

    Serial.println("Roll Left");
    moveplat(dur, zero_length, TZ, TZ, R0, R1);
    Serial.println("Roll Right");
    moveplat(dur, zero_length, TZ, TZ, R1, R0);

    Serial.println("Pitch Down");
    moveplat(dur, zero_length, TZ, TZ, R0, R2);
    Serial.println("Pitch Up");
    moveplat(dur, zero_length, TZ, TZ, R2, R0);

    Serial.println("Yaw Left");
    moveplat(dur, zero_length, TZ, TZ, R0, R3);
    Serial.println("Yaw Right");
    moveplat(dur, zero_length, TZ, TZ, R3, R0);
      
    //Disables motors
    //Note that HIGH corresponds to disable
    digitalWrite(ENABLE_MOTORS, HIGH);
}

void loop() {
  //Do nothing
  delay(500);
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
    Serial.println("Extending");
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
    Serial.println("Retracting");
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

    int steps = duration * 10;
    float Kp = 0.25;
    
    //keeps track of average error for iteration of Kp for PID
    float avg_error[NUM_MOTORS] = { 0, 0, 0, 0, 0, 0 };

    for (int step = 0; step <= steps; step++) {
        long start_time = millis();
        float t = float(step) / steps;
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
            float* base = bases[motor];
            float* plat = plats[motor];
  
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
            
            //figure out the error between the ideal length (based off the inverse kinematics) vs. the actual length (based off potentiometer readings)th
            //"ideal length" is length_t, and actual is length_now
            float reading = getAverageReading(motor);
            float length_now = mapFloat(reading, ZERO_POS[motor], END_POS[motor], 0, 8) + length_min;
            float error = length_t - length_now;
            avg_error[motor] += abs(error) / steps;

            //figures out the velocity using a mix of feedforward (length_next - length_t)
            //and feedback/proportional control (error * Kp)
            //the feedforward terms gives a starting point for the velocity based off the inverse kinematics
            //the feedback term is used to speed up or slow down the actuator depending on if it is lagging behind or going too fast
            //TO DO: play around with the proportional coefficient (Kp) to see how it changes the response
            //based off experience, a higher Kp will result in jerkier motion, but less average error
            //Also TO DO: play around with step size (steps / duration) to see how things changes
            //smaller step sizes also seem to result in jerkier motion
            float vel = (length_next - length_t + error * Kp) * steps / duration;

              
              //safety measure to stop the platform in case the actuators drift too far
//            if (abs(error) > .5){
//              Serial.print("Invalid Length: ");
//              Serial.print(length_t);
//              Serial.print(", ");
//              Serial.println(length_now);
//              //valid = false;
//            }
            
            //caps the speed (as calculated above) at 2"/s, based off of the max speed of the actuator
            if (abs(vel) > 2){
              //Serial.print("Maximum Speed Exceeded: ");
              //Serial.println(vel);
              vel = 2 * ((vel > 0)? 1:-1);
              //valid = false;
            }

            //maps speed to a pwm value
            int pwm_speed = mapFloat(vel, 0, 2, 0, 255);
            
            //sets lower cap for speed
            //motor will not move below this pwm
            if (abs(pwm_speed) < 25){
              pwm_speed = 0;
            }
  
            pwm[motor] = pwm_speed;
        }
  
        if (!valid){
          for (motor = 0; motor < NUM_MOTORS; ++motor)
          {
            pwm[motor] = MIN_PWM;
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
        unsigned long cur_time = millis();
        int target_delay = duration/steps*1000;
        //more accurate delay function
        while ((cur_time - start_time) < target_delay){
          cur_time = millis();
        }
        //Serial.println(cur_time - start_time);
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
