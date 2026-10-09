/*
    Module to handle the UART communication with the ESP32
    Implementation file
    Author: Neo Vorsatz
    Last updated: 5 October 2026
*/

// External includes
#include <Arduino.h>
#include <stdint.h>
#include <util/crc16.h> // CRC
#include <string.h>

// Internal includes
#include "uart_w_esp32.h"

// CONSTANTS ================================

// Header
const uint8_t UART_HEAD[2] = {0xC0, 0xDE}; // header of a packet, marking the start

// Lengths
#define SETPOINTS_LEN 6*sizeof(float) // number of bytes in the setpoints
#define BODY_LEN SETPOINTS_LEN+2 // number of bytes in the body

// VARIABLES ================================

// Reader state
static uint8_t state = 0; // 0: hunting for 1st byte of header | 1: expecting 2nd byte of header | 2: collecting body
static uint8_t body[BODY_LEN]; // buffer for setpoints + CRC
static uint8_t indexBody = 0; // index of next byte in the body

// HELPER FUNCTIONS ================================

/**
 * @brief Checks if an array of data corresponds to a CRC16 checksum
 * 
 * @param data Pointer to the array that supposedly generated the CRC
 * @param len The length of the array
 * @param crc The CRC16 checksum
 * 
 * @return `true` if the CRC matches, `false` otherwise
 * 
 * @note Uses CRC-16/GENIBUS - polynomial 0x1021, init 0xFFFF, MSB-first, final XOR 0xFFFF
 */
static bool checkCRC(const uint8_t *data, int len, uint16_t crc) {
    // CRC-16/GENIBUS = CRC-16/XMODEM algorithm seeded with 0xFFFF, with the result inverted
    uint16_t c = 0xFFFF;
    for (int i=0; i<len; i++) {
        c = _crc_xmodem_update(c, data[i]);
    }
    c ^= 0xFFFF;
    return (c == crc);
}

// API FUNCTIONS ================================

void initUART() {
    Serial.begin(250000);
}

bool newSetpoint(float *write) {
    // Assume no new setpoint was found yet
    bool foundNew = false;

    // While there are still bytes to read
    while (Serial.available() > 0) {
        // Read next byte
        uint8_t byte = (uint8_t)Serial.read();
 
        // If hunting for 1st byte of header
        if (state == 0) { 
            if (byte == UART_HEAD[0]) {state = 1;}
 
        // If expecting 2nd byte of header
        } else if (state == 1) {
            if (byte == UART_HEAD[1]) {
                state = 2;
                indexBody = 0;
            }
            else if (byte != UART_HEAD[0]) {
                // Reset the state, to hunt for the first byte of the header again
                state = 0;
            } // repeating the first byte stays in state 1
 
        // Extract body bytes
        } else {
            body[indexBody++] = byte;
            if (indexBody == BODY_LEN) { // body complete
                // Check and apply setpoint
                uint16_t crc = (uint16_t)body[SETPOINTS_LEN] | ((uint16_t)body[SETPOINTS_LEN+1]<<8); // assemble CRC16
                if (checkCRC(body, SETPOINTS_LEN, crc)) {
                    // New setpoint found
                    foundNew = true;
                    // Parse the bytes as floats (little-endian IEEE-754 float32)
                    memcpy(write, body, SETPOINTS_LEN);
                }

                // Resume hunting
                state = 0;
            }
        }
    }

    // Return whether or not a new setpoint was found
    return foundNew;
}
