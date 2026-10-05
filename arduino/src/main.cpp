/*
    Program to control the Arduino, which is responsible for actuation
    Author: Neo Vorsatz
    Last updated: 5 October 2026
*/

// #define RUN_TEST // Uncomment this to run the test script

// External includes
#include <Arduino.h>

// Internal includes
#include <braccio_actuation.h>

// GLOBAL VARIABLES ================================

vector_t pos;
bool serial;

// FUNCTION DECLARATIONS ================================

/**
 * @brief Panic function to turn on the builtin LED, and halt execution
 */
void panic();

/**
 * @brief Moves to a given setpoint using the blocking-movement version from `braccio_actuation`, then delays for 1 second
 * 
 * @param base The setpoint angle for the base
 * @param joint1 The setpoint angle for the first joint
 * @param joint2 The setpoint angle for the second joint
 * @param joint3 The setpoint angle for the third joint
 * @param write The setpoint angle for the wrist
 * @param claw The setpoint angle for the claw
 */
void move(int base, int joint1, int joint2, int joint3, int wrist, int claw);

/**
 * @brief Resets the position of the arm to the default/safe position
 */
void resetPos();

// MAIN ================================

#ifdef RUN_TEST

void setup() {
    // Initialize serial
    Serial.begin(9600);
    serial = (Serial) ? true : false;
    if (serial) {Serial.print("================================\n");}
    if (serial) {Serial.print("{ Program started\n");}
    if (serial) {Serial.print("> Waiting ...\n");}
    delay(1000*1); // wait 1 second

    // Initialize arm
    if (serial) {Serial.print("> ... Done waiting\n");}
    if (!baInit()) {
        if (serial) {Serial.print("! Initialisation failed\n");}
        if (serial) {Serial.print("} Program ended\n");}
        if (serial) {Serial.print("================================\n");}
        return;
    }
    if (serial) {Serial.print("> Braccio initialised\n");}

    // Wait
    delay(1000*1);

    // Test claw
    if (serial) {Serial.print("> Claw\n");}
    move(90, 0, 180, 0, 90, 0);
    resetPos();

    // Test wrist
    if (serial) {Serial.print("> Wrist\n");}
    move(90, 0, 180, 0, 0, 180);
    move(90, 0, 180, 0, 180, 180);
    resetPos();

    // Test joint3
    if (serial) {Serial.print("> Joint3\n");}
    move(90, 0, 180, 90, 90, 180);
    resetPos();

    // Test joint1
    if (serial) {Serial.print("> Joint1\n");}
    move(90, 90, 135, 0, 90, 180);

    // Test joint2
    if (serial) {Serial.print("> Joint2\n");}
    move(90, 90, 90, 0, 90, 180);

    // Test base
    if (serial) {Serial.print("> Base\n");}
    move(0, 90, 90, 0, 90, 180);
    move(180, 90, 90, 0, 90, 180);
    move(90, 90, 90, 0, 90, 180);
    resetPos();

    // End
    if (serial) {Serial.print("} Program ended\n");}
    if (serial) {Serial.print("================================\n");}
}

void loop() {
    // Nothing
}

#else

void setup() {
    // Initialize UART
    Serial.begin(115200);

    // Initialize arm
    if (!baInit()) {
        panic();
    }
}

void loop() {
    // Nothing
}

#endif

// FUNCTION DEFINITIONS ================================

void panic() {
    // Turn on LED to indicate error
    digitalWrite(LED_BUILTIN, HIGH);
    
    // Freeze the program
    while(true);
}

void move(float base, float joint1, float joint2, float joint3, float wrist, float claw) {
    pos.base = base;
    pos.joint1 = joint1;
    pos.joint2 = joint2;
    pos.joint3 = joint3;
    pos.wrist = wrist;
    pos.claw = claw;
    baMove(pos);
    delay(1000*1);
}

void resetPos() {
    move(90.0f, 0.0f, 180.0f, 0.0f, 90.0f, 180.0f);
}
