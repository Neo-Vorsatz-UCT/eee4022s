/*
    Module to handle the UART communication with the ESP32
    Header file
    Author: Neo Vorsatz
    Last updated: 5 October 2026
*/

#ifndef UART_W_ESP32_H
#define UART_W_ESP32_H

// External includes
#include <Arduino.h>
#include <Servo.h>

// FUNCTIONS ================================

/**
 * @brief Initializes UART
 */
void initUART();

/**
 * @brief Reads available bytes provided from UART, and writes a valid setpoint if found
 * 
 * @param write A buffer to which a new setpoint is written
 * 
 * @return `true` if a new setpoint was found, otherwise `false`
 */
bool newSetpoint(float *write);

#endif
