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

// Deadband
#define CONTROL_I_LIMIT 20.0f // maximum force from integral
#define CONTROL_D_LIMIT 10.0f // maximum force from differential
#define DEAD_BAND 0.0f // deadband to elimate small forces from becoming unstable

// Movement
#define MOVE_STEP 1.0f // step size
#define MOVE_DELAY 10 // delay between steps, in milliseconds

// VARIABLES ================================

// Vectors describing the motion of the arm
static vector_t currentTarget; // target position
static vector_t currentPos; // position
static vector_t currentVel; // velocity
static vector_t prevError; // previous error
static vector_t integralError; // integral error

// Servo motors
static Servo baseServo;
static Servo joint1Servo;
static Servo joint2Servo;
static Servo joint3Servo;
static Servo wristServo;
static Servo clawServo;

// Software kinetics
static float controlP = 0.002f; // proportional control gain
static float controlI = 0.0f; // integral control gain
static float controlD = 0.12f; // differential control gain

// 0.01 0.0 0.17

// HELPER FUNCTIONS ================================

/**
 * @brief Sets the position of the arm, without checking bounds
 * 
 * @param position The position to set the arm to
 */
static void setPositionTrusted(vector_t position) {
    // Set position variable
    currentPos = position;

    // Set servo motors
    baseServo.write((int)position.base);
    joint1Servo.write((int)position.joint1);
    joint2Servo.write((int)position.joint2);
    joint3Servo.write((int)position.joint3);
    wristServo.write((int)position.wrist);
    clawServo.write((int)position.claw);
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
 * @brief Calculates the force for a joint
 * 
 * @param write The address to write the force to
 * @param target The target position of the joint
 * @param position The current position of the joint
 * @param integralError The address of the integral error
 * @param prevError The address of the previous error
 */
static void calcForce(float* write, float target, float position, float* integralError, float* prevError) {
    // Initialize
    float force;
    *write = 0.0f;

    // Calculate error
    float error = target-position;

    // Proportional control
    *write += controlP*error;

    // Integral control
    force = controlI*(*integralError);
    if (force<-CONTROL_I_LIMIT) {force = -CONTROL_I_LIMIT;}
    if (force>CONTROL_I_LIMIT) {force = CONTROL_I_LIMIT;}
    *write += force;

    // Differential control
    force = controlD*(error-*prevError);
    if (force<-CONTROL_D_LIMIT) {force = -CONTROL_D_LIMIT;}
    if (force>CONTROL_D_LIMIT) {force = CONTROL_D_LIMIT;}
    *write += force;

    // Update errors
    *integralError += error;
    *prevError = error;
}

// INITIALIZERS AND CREATION ================================

bool baInit(int softStart, const int conn[6]) {
    // Enable soft start pin
    if (softStart!=BA_DISABLE_SOFT_START) {
        pinMode(SOFT_START_CONTROL_PIN, OUTPUT);
        digitalWrite(SOFT_START_CONTROL_PIN, LOW);
    }

    // Initialize servos
    if (baseServo.attach(SLOTS[conn[0]])==INVALID_SERVO) {return false;}
	if (joint1Servo.attach(SLOTS[conn[1]])==INVALID_SERVO) {return false;}
	if (joint2Servo.attach(SLOTS[conn[2]])==INVALID_SERVO) {return false;}
	if (joint3Servo.attach(SLOTS[conn[3]])==INVALID_SERVO) {return false;}
	if (wristServo.attach(SLOTS[conn[4]])==INVALID_SERVO) {return false;}
	if (clawServo.attach(SLOTS[conn[5]])==INVALID_SERVO) {return false;}

    // Initialize vectors
    currentVel = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    prevError = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    integralError = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    currentTarget = INIT_TARGET;
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

vector_t baGetVelocity() {
    return currentVel;
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
    return (baGetError()<BA_POS_TOLERANCE);
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

void baStep(float step) {
    vector_t force;

    // Calculate base force
    calcForce(
        &force.base,
        currentTarget.base,
        currentPos.base,
        &integralError.base,
        &prevError.base
    );

    // Calculate joint1 force
    calcForce(
        &force.joint1,
        currentTarget.joint1,
        currentPos.joint1,
        &integralError.joint1,
        &prevError.joint1
    );

    // Calculate joint2 force
    calcForce(
        &force.joint2,
        currentTarget.joint2,
        currentPos.joint2,
        &integralError.joint2,
        &prevError.joint2
    );

    // Calculate joint3 force
    calcForce(
        &force.joint3,
        currentTarget.joint3,
        currentPos.joint3,
        &integralError.joint3,
        &prevError.joint3
    );

    // Calculate wrist force
    calcForce(
        &force.wrist,
        currentTarget.wrist,
        currentPos.wrist,
        &integralError.wrist,
        &prevError.wrist
    );

    // Calculate claw force
    calcForce(
        &force.claw,
        currentTarget.claw,
        currentPos.claw,
        &integralError.claw,
        &prevError.claw
    );
    
    // Adjust the velocity
    /*
        Force directly translates to acceleration (change in velocity).
        This assumes mass=1. To change the effective mass, change the controller gain.
        Halving the gain behaves like doubling the mass.
    */
    currentVel.base += step*force.base;
    currentVel.joint1 += step*force.joint1;
    currentVel.joint2 += step*force.joint2;
    currentVel.joint3 += step*force.joint3;
    currentVel.wrist += step*force.wrist;
    currentVel.claw += step*force.claw;

    // Adjust the position
    currentPos.base += step*currentVel.base;
    currentPos.joint1 += step*currentVel.joint1;
    currentPos.joint2 += step*currentVel.joint2;
    currentPos.joint3 += step*currentVel.joint3;
    currentPos.wrist += step*currentVel.wrist;
    currentPos.claw += step*currentVel.claw;

    // Set the servos while verifying bounds
    baSetPosition(currentPos); // note: this adjusts `currentPos` if outside bounds

    //#
    Serial.print("t: ");
    Serial.print(currentTarget.joint3);
    Serial.print(" | p: ");
    Serial.print(currentPos.joint3);
    Serial.print("\n");
}

bool baMove(vector_t target) {
    // Set the target
    bool bounded = baUpdateTarget(target);

    // Move to the target
    while (!baAtTarget(MOVE_STEP)) {
        baStep(MOVE_STEP);
        delay(MOVE_DELAY);
    }

    // Return whether the target was within the bounds or not
    return bounded;
}

void baSetControllerGain(float proportional, float integral, float differential) {
    controlP = (proportional>0.0f) ? proportional : 0.0f;
    controlI = (integral>0.0f) ? integral : 0.0f;
    controlD = (differential>0.0f) ? differential : 0.0f;
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
