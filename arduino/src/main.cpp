/*
    Program to control the Arduino, which is responsible for actuation
    Author: Neo Vorsatz
    Last updated: 26 July 2026
*/

#define RUN_TEST // Uncomment this to run the test script

// External includes
#include <Arduino.h>

// Internal includes
#include <braccio_actuation.h>

// GLOBAL VARIABLES ================================

vector_t pos;
bool serial;

// FUNCTION DECLARATIONS ================================

void move(int base, int joint1, int joint2, int joint3, int wrist, int claw);

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
    if (serial) {Serial.print("> Base\n");}move(90, 90, 90, 0, 90, 180);
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
    // Nothing
}

void loop() {
    // Nothing
}

#endif

// FUNCTION DEFINITIONS ================================

void move(int base, int joint1, int joint2, int joint3, int wrist, int claw) {
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
    move(90, 0, 180, 0, 90, 180);
}
