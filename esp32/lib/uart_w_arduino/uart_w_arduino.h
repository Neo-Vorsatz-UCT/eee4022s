/*
    Module to handle the UART communication with the Arduino
    Header file
    Author: Neo Vorsatz
    Last updated: 3 October 2026
*/

#ifndef UART_W_ARDUINO_H
#define UART_W_ARDUINO_H

// FUNCTIONS ================================

/**
 * @brief Initializes UART
 */
void initUART();

/**
 * @brief Send a payload over UART
 * 
 * @param payload Pointer to the payload
 * @param len Length of the payload in bytes
 * 
 * @note The framing looks like:
 * HEAD_0 | HEAD_1 | len | payload | CRC16 (little-endian)
 */
void sendUART(const uint8_t *payload, size_t len);

#endif
