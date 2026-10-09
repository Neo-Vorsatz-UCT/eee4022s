/*
    Program to control the Arduino, which is responsible for actuation
    Author: Neo Vorsatz
    Last updated: 7 October 2026
*/

// #define RUN_TEST // Uncomment this to run the test script

// External includes
#include <Arduino.h>
#include <stdint.h>

// Internal includes
#include <uart_w_esp32.h>
#include <braccio_actuation.h>

// CONSTANTS ================================

// Braccio servo motor connectors
const int conn[6] = {BA_M1, BA_M2, BA_M3, BA_M4, BA_M6, BA_M5}; // my Braccio's wrist and claw motors are swapped

// Level shifter pin
#define LVL_SHIFT_PIN A0 // this pull is pulled low by a 10kR resistor, and must be pulled high

// Safety
#define TIMEOUT_SETPOINT 1000 // maximum time between valid setpoints before the Braccio moves to the safe position

// VARIABLES ================================

// Position control
vector_t pos; // position the Braccio is moving to

// Safety
unsigned long timeLastSetpoint; // timestamp of most recent valid setpoint, in milliseconds

// FUNCTION DECLARATIONS ================================

/**
 * @brief Panic function to turn on the builtin LED, and halt execution
 */
void panic();

/**
 * @brief Moves the Braccio to the "safe" position
 */
void safe();

/**
 * @brief Moves to a given setpoint using the blocking-movement version from `braccio_actuation`, then delays for 1 second
 * 
 * @param base The setpoint angle for the base
 * @param joint1 The setpoint angle for the first joint
 * @param joint2 The setpoint angle for the second joint
 * @param joint3 The setpoint angle for the third joint
 * @param wrist The setpoint angle for the wrist
 * @param claw The setpoint angle for the claw
 */
void testMove(float base, float joint1, float joint2, float joint3, float wrist, float claw);

/**
 * @brief Resets the position of the arm to the default position
 */
void testResetPos();

// MAIN ================================

#ifdef RUN_TEST

void setup() {
    // Initialize serial
    Serial.begin(115200);
    bool serial = (Serial) ? true : false;
    if (serial) {Serial.print("================================\n");}
    if (serial) {Serial.print("{ Program started\n");}
    if (serial) {Serial.print("> Waiting ...\n");}

    // Initialize builtin LED
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // Wait 1 second
    delay(1000);

    // Initialize arm
    if (serial) {Serial.print("> ... Done waiting\n");}
    if (!baInit(BA_DEFAULT_SOFT_START, conn)) {
        if (serial) {Serial.print("! Initialisation failed\n");}
        if (serial) {Serial.print("} Program ended\n");}
        if (serial) {Serial.print("================================\n");}
        return;
    }
    if (serial) {Serial.print("> Braccio initialised\n");}

    // Wait
    delay(1000);

    // Test claw
    if (serial) {Serial.print("> Claw\n");}
    testMove(90.0f, 0.0f, 180.0f, 0.0f, 90.0f, 0.0f);
    testResetPos();

    // Test wrist
    if (serial) {Serial.print("> Wrist\n");}
    testMove(90.0f, 0.0f, 180.0f, 0.0f, 0.0f, 180.0f);
    testMove(90.0f, 0.0f, 180.0f, 0.0f, 180.0f, 180.0f);
    testResetPos();

    // Test joint3
    if (serial) {Serial.print("> Joint3\n");}
    testMove(90.0f, 0.0f, 180.0f, 90.0f, 90.0f, 180.0f);
    testResetPos();

    // Test joint1
    if (serial) {Serial.print("> Joint1\n");}
    testMove(90.0f, 90.0f, 135.0f, 0.0f, 90.0f, 180.0f);

    // Test joint2
    if (serial) {Serial.print("> Joint2\n");}
    testMove(90.0f, 90.0f, 90.0f, 0.0f, 90.0f, 180.0f);

    // Test base
    if (serial) {Serial.print("> Base\n");}
    testMove(0.0f, 90.0f, 90.0f, 0.0f, 90.0f, 180.0f);
    testMove(180.0f, 90.0f, 90.0f, 0.0f, 90.0f, 180.0f);
    testMove(90.0f, 90.0f, 90.0f, 0.0f, 90.0f, 180.0f);
    testResetPos();

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
    initUART();

    // Initialize builtin LED
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // Wait 1 second
    delay(1000);

    // Initialize arm
    if (!baInit(BA_DEFAULT_SOFT_START, conn)) {
        panic();
    }

    // Initialize level shifter
    pinMode(LVL_SHIFT_PIN, OUTPUT);
    digitalWrite(LVL_SHIFT_PIN, HIGH);

    // Initialize setpoint control
    timeLastSetpoint = millis();
}

void loop() {
    // Read and update setpoints
    float buffer[6];
    if (newSetpoint(buffer)) {
        // Update setpoints
        pos.base = buffer[0];
        pos.joint1 = buffer[1];
        pos.joint2 = buffer[2];
        pos.joint3 = buffer[3];
        pos.wrist = buffer[4];
        pos.claw = buffer[5];
        baUpdateTarget(pos);

        // Update timestamp
        timeLastSetpoint = millis();
    }

    // Check for timeout of any new valid setpoints
    if ((millis()-timeLastSetpoint) > TIMEOUT_SETPOINT) {safe();}

    // Move the Braccio arm slightly
    baStep();
}

#endif

// FUNCTION DEFINITIONS ================================

void panic() {
    // Turn on LED to indicate error
    digitalWrite(LED_BUILTIN, HIGH);
    
    // Freeze the program
    while(true);
}

void safe() {
    pos.base = 90.0f;
    pos.joint1 = 0.0f;
    pos.joint2 = 180.0f;
    pos.joint3 = 0.0f;
    pos.wrist = 90.0f;
    pos.claw = 0.0f;
    baUpdateTarget(pos);
}

void testMove(float base, float joint1, float joint2, float joint3, float wrist, float claw) {
    pos.base = base;
    pos.joint1 = joint1;
    pos.joint2 = joint2;
    pos.joint3 = joint3;
    pos.wrist = wrist;
    pos.claw = claw;
    baMove(pos);
    delay(1000);
}

void testResetPos() {
    testMove(90.0f, 0.0f, 180.0f, 0.0f, 90.0f, 180.0f);
}
