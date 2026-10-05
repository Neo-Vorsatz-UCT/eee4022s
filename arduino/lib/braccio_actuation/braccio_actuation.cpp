/*
    Module to handle the actuation of the Braccio arm
    Implementation file
    Author: Neo Vorsatz
    Last updated: 26 July 2026
*/

/*
    This dragon protects the implementation. You have been warned!

                    -,,,__
                     \    ``~~--,,__                /   /
                     /              ``~~--,,_     //--//
          _,,,,-----,\              ,,,,---- >   (c  c)\
      ,;''            `\,,,,----''''   ,,-'''---/   /_ ;___        -,_
     ( ''---,;====;,----/             (-,,_____/  /'/ `;   '''''----\ `:.
     (                 '               `      (oo)/   ;~~~~~~~~~~~~~/--~
      `;_           ;    \            ;   \   `  ' ,,'
         ```-----...|     )___________|    )-----'''
Art by               \   /             \   \\
  Korrath            /  /,              `\   \\
                   ,'---\ \              ,---`,;,
                         ```

*/

// External includes
#include <Arduino.h>
#include <Servo.h>

// Internal includes
#include "braccio_actuation.h"

// CONSTANTS ================================

// Pins
#define SOFT_START_CONTROL_PIN 12

// Connectors
#define SLOT_1 11
#define SLOT_2 10
#define SLOT_3 9
#define SLOT_4 6
#define SLOT_5 3
#define SLOT_6 5
static const int SLOTS[] = {SLOT_1, SLOT_2, SLOT_3, SLOT_4, SLOT_5, SLOT_6};

// Initial target
/*
    Some values are intentionally exaggerated (out of bounds),
    to allow modification of bounds while still moving to the extrema.
*/
static const vector_t INIT_TARGET = {
    90.0f, // base
    0.0f, // joint1
    180.0f, // joint2
    0.0f, // joint3
    90.0f, // wrist
    180.0f // claw
};

// Bit-bang PWM
#define BITBANG_PWM_PERIOD 1000 // period of the PWM cycle for bit-banging, in microseconds
#define SOFT_START_PERIOD 20 // period of changes to PWM duty cycle, in milliseconds

// Mechanical bounds (come from original library)
#define BASE_MIN 0.0f
#define BASE_MAX 180.0f
#define JOINT1_MIN 15.0f
#define JOINT1_MAX 165.0f
#define JOINT2_MIN 0.0f
#define JOINT2_MAX 180.0f
#define JOINT3_MIN 0.0f
#define JOINT3_MAX 180.0f
#define WRIST_MIN 0.0f
#define WRIST_MAX 180.0f
#define CLAW_MIN 10.0f
#define CLAW_MAX 73.0f

// Blocking movement
#define MOVE_DELAY 10 // delay between steps, in milliseconds

// VARIABLES ================================

// Vectors describing the motion of the arm
static vector_t currentTarget; // target position
static vector_t currentPos; // position
static vector_t currentFilter; // output of 1st LPF, input of 2nd LPF

// Servo motors
static Servo baseServo;
static Servo joint1Servo;
static Servo joint2Servo;
static Servo joint3Servo;
static Servo wristServo;
static Servo clawServo;

// Low pass filter
static unsigned long prevTime; // previous time relevant to the filter, in milliseconds
static float filterTau = 0.333f; // time constant for one stage of LPF
/*
    By default, we want to reach 95% of the setpoint within 100ms.
    This can indirectly be reconfigured via `baSetFilterTau()` or `baSetFilterFreq()`.
    This requirement makes 3*tau=0.1 => tau=0.0333 (tau is the time constant).
*/

// HELPER FUNCTIONS ================================

/**
 * @brief Converts a joint angle into PWM width, in microseconds
 * 
 * @param angle The angle of the joint
 * 
 * @return The PWM width, in microseconds
 */
static inline int angleToPWM(float angle) {
    /*
        This conversion comes from the specifications of
        the SpringRC SR-311 and SpringRC SR-431 servo motors.
        A PWM width of 500 microseconds gives 0 degrees, and 2500 microseconds gives 180 degrees.
        This function linearly interpolates.
    */
    return 500+(int)(angle/180.f*(2500.0f-500.0f));
}

/**
 * @brief Sets the position of the arm, without checking bounds
 * 
 * @param position The position to set the arm to
 */
static void setPositionTrusted(vector_t position) {
    // Set position variable
    currentPos = position;

    // Set servo motors
    baseServo.writeMicroseconds(angleToPWM(position.base));
    joint1Servo.writeMicroseconds(angleToPWM(position.joint1));
    joint2Servo.writeMicroseconds(angleToPWM(position.joint2));
    joint3Servo.writeMicroseconds(angleToPWM(position.joint3));
    wristServo.writeMicroseconds(angleToPWM(position.wrist));
    clawServo.writeMicroseconds(angleToPWM(position.claw));
}

/**
 * @brief Applies a PWM cycle of length BITBANG_PWM_PERIOD to the soft start pin
 * 
 * @param onTime The time to keep the signal high, in microseconds
 */
static void bitBangPWM(unsigned int onTime) {
    // Bound on-time
    if (onTime>BITBANG_PWM_PERIOD) {onTime = BITBANG_PWM_PERIOD;}

    // Apply PWM
    digitalWrite(SOFT_START_CONTROL_PIN, HIGH);
	delayMicroseconds(onTime);
	digitalWrite(SOFT_START_CONTROL_PIN, LOW);
	delayMicroseconds(BITBANG_PWM_PERIOD-onTime); 
}

/**
 * @brief Updates an output using a simple exponential smoothing algorithm
 * 
 * @param output The address of the output that will be read and written to
 * @param input The new value to be applied to the output
 * @param alpha The alpha value
 */
static void expSmooth(float* output, float input, float alpha) {
    *output = (1-alpha)*(*output)+alpha*input;
}

// INITIALIZERS AND CREATION ================================

bool baInit(int softStart, const int conn[6]) {
    // Enable soft start pin
    if (softStart!=BA_DISABLE_SOFT_START) {
        pinMode(SOFT_START_CONTROL_PIN, OUTPUT);
        digitalWrite(SOFT_START_CONTROL_PIN, LOW);
    }

    // Initialize servos
    if (baseServo.attach(SLOTS[conn[0]], 500, 2500)==INVALID_SERVO) {return false;}
	if (joint1Servo.attach(SLOTS[conn[1]], 500, 2500)==INVALID_SERVO) {return false;}
	if (joint2Servo.attach(SLOTS[conn[2]], 500, 2500)==INVALID_SERVO) {return false;}
	if (joint3Servo.attach(SLOTS[conn[3]], 500, 2500)==INVALID_SERVO) {return false;}
	if (wristServo.attach(SLOTS[conn[4]], 500, 2500)==INVALID_SERVO) {return false;}
	if (clawServo.attach(SLOTS[conn[5]], 500, 2500)==INVALID_SERVO) {return false;}

    // Initialize vectors
    currentTarget = INIT_TARGET;
    currentFilter = INIT_TARGET;
    baSetPosition(INIT_TARGET);

    // Soft start
    /*
        This implementation uses the same idea as the original library; increasing duty cycle,
        but this implementation is gradual as opposed to 3-stage.
    */
    if (softStart!=BA_DISABLE_SOFT_START) {
        // Bound soft start
        if (softStart<BA_MIN_SOFT_START) {softStart = BA_MIN_SOFT_START;} // lower bound
        if (softStart>BA_MAX_SOFT_START) {softStart = BA_MAX_SOFT_START;} // upper bound
        // Progressively increase PWM duty cycle
        unsigned int onTime = 0;
        long int startTime;
        while (onTime<BITBANG_PWM_PERIOD) {
            onTime += softStart; // increase on-time
            startTime = millis(); // record start time
            while (millis()-startTime<SOFT_START_PERIOD) {
                bitBangPWM(onTime); // apply PWM until SOFT_START_PERIOD
            }
        }
        // Permanently set high
        digitalWrite(SOFT_START_CONTROL_PIN, HIGH);
    }

    // Mark timestamp
    prevTime = millis();

    // Indicate success
    return true;
}

vector_t baNewVector(float base, float joint1, float joint2, float joint3, float wrist, float claw) {
    vector_t vec = {base, joint1, joint2, joint3, wrist, claw};
    return vec;
}

// READING ================================

vector_t baGetTarget() {
    return currentTarget;
}

vector_t baGetPosition() {
    return currentPos;
}

float baGetError() {
    float totError = 0.0f;
    float error;

    // Add base error
    error = currentTarget.base-currentPos.base;
    totError += error*error;

    // Add joint1 error
    error = currentTarget.joint1-currentPos.joint1;
    totError += error*error;

    // Add joint2 error
    error = currentTarget.joint2-currentPos.joint2;
    totError += error*error;

    // Add joint3 error
    error = currentTarget.joint3-currentPos.joint3;
    totError += error*error;

    // Add wrist error
    error = currentTarget.wrist-currentPos.wrist;
    totError += error*error;

    // Add claw error
    error = currentTarget.claw-currentPos.claw;
    totError += error*error;

    // Return total error
    return totError;
}

bool baAtTarget(float tolerance) {
    return (baGetError()<tolerance);
}

// CONTROLLING ================================

bool baSetPosition(vector_t position) {
    // Bound position
    bool bounded = baBoundPosition(&position);

    // Set position
    setPositionTrusted(position);

    // Return whether the position was within the bounds or not
    return bounded;
}

bool baUpdateTarget(vector_t target) {
    // Bound position
    bool bounded = baBoundPosition(&target);

    // Set target
    currentTarget = target;

    // Return whether the target was within the bounds or not
    return bounded;
}

void baStep() {
    // Apply RC low-pass filter approximation to calculate alpha value
    unsigned long time = millis(); // get current time elapsed
    float deltaT = (time-prevTime)/1000; // calculate time passed, in seconds
    float alpha = deltaT/(filterTau+deltaT); // calculate alpha
    prevTime = time; // update previous time

    // Update 1st filter outputs using simple exponential smoothing
    expSmooth(&currentFilter.base, currentTarget.base, alpha);
    expSmooth(&currentFilter.joint1, currentTarget.joint1, alpha);
    expSmooth(&currentFilter.joint2, currentTarget.joint2, alpha);
    expSmooth(&currentFilter.joint3, currentTarget.joint3, alpha);
    expSmooth(&currentFilter.wrist, currentTarget.wrist, alpha);
    expSmooth(&currentFilter.claw, currentTarget.claw, alpha);

    // Update 2nd filter outputs using simple exponential smoothing
    expSmooth(&currentPos.base, currentFilter.base, alpha);
    expSmooth(&currentPos.joint1, currentFilter.joint1, alpha);
    expSmooth(&currentPos.joint2, currentFilter.joint2, alpha);
    expSmooth(&currentPos.joint3, currentFilter.joint3, alpha);
    expSmooth(&currentPos.wrist, currentFilter.wrist, alpha);
    expSmooth(&currentPos.claw, currentFilter.claw, alpha);

    // Set the servos while verifying bounds
    baSetPosition(currentPos); // note: this adjusts `currentPos` if outside bounds
}

bool baMove(vector_t target) {
    // Set the target
    bool bounded = baUpdateTarget(target);

    // Move to the target
    while (!baAtTarget(BA_POS_TOLERANCE)) {
        baStep();
        delay(MOVE_DELAY);
    }

    // Return whether the target was within the bounds or not
    return bounded;
}

void baSetFilterTau(float tau) {
    filterTau = tau;
}

void baSetFilterFreq(float freq) {
    filterTau = 1.0f/(2.0f*3.14f*freq);
}

// MANAGEMENT ================================

bool baTargetReachable(vector_t target) {
    // Check each bound
    if (target.base<BASE_MIN || target.base>BASE_MAX) {return false;}
    if (target.joint1<JOINT1_MIN || target.joint1>JOINT1_MAX) {return false;}
    if (target.joint2<JOINT2_MIN || target.joint2>JOINT2_MAX) {return false;}
    if (target.joint3<JOINT3_MIN || target.joint3>JOINT3_MAX) {return false;}
    if (target.wrist<WRIST_MIN || target.wrist>WRIST_MAX) {return false;}
    if (target.claw<CLAW_MIN || target.claw>CLAW_MAX) {return false;}

    // Within bounds
    return true;
}

bool baBoundPosition(vector_t *position) {
    // Assume within bounds
    bool bounded = true;

    // Check each bound (thicc stack of code)
    if (position->base<BASE_MIN) {position->base = BASE_MIN; bounded = false;}
    else if (position->base>BASE_MAX) {position->base = BASE_MAX; bounded = false;}
    if (position->joint1<JOINT1_MIN) {position->joint1 = JOINT1_MIN; bounded = false;}
    else if (position->joint1>JOINT1_MAX) {position->joint1 = JOINT1_MAX; bounded = false;}
    if (position->joint2<JOINT2_MIN) {position->joint2 = JOINT2_MIN; bounded = false;}
    else if (position->joint2>JOINT2_MAX) {position->joint2 = JOINT2_MAX; bounded = false;}
    if (position->joint3<JOINT3_MIN) {position->joint3 = JOINT3_MIN; bounded = false;}
    else if (position->joint3>JOINT3_MAX) {position->joint3 = JOINT3_MAX; bounded = false;}
    if (position->wrist<WRIST_MIN) {position->wrist = WRIST_MIN; bounded = false;}
    else if (position->wrist>WRIST_MAX) {position->wrist = WRIST_MAX; bounded = false;}
    if (position->claw<CLAW_MIN) {position->claw = CLAW_MIN; bounded = false;}
    else if (position->claw>CLAW_MAX) {position->claw = CLAW_MAX; bounded = false;}

    // Return result
    return bounded;
}
