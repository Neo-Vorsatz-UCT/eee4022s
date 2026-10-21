/*
    Module to handle the UART communication with the Arduino
    Implementation file
    Author: Neo Vorsatz
    Last updated: 21 October 2026
*/

// External includes
#include "driver/uart.h" // UART
#include "esp_rom_crc.h" // LUT-accelerated CRC

// Internal includes
#include "uart_w_arduino.h"

// CONSTANTS ================================

#define UART_NUM UART_NUM_1 // chosen UART port on ESP32
#define UART_BAUD 640000 // baud rate for UART
#define UART_TX_PIN 12 // transmission pin for UART
#define UART_RX_PIN 13 // reception pin for UART
static const uint8_t UART_HEAD[2] = {0xC0, 0xDE}; // the start-marker

// FUNCTIONS ================================

void initUART() {
    // Start drivers
    uart_driver_install(UART_NUM, 256, 0, 0, NULL, 0);

    // Set configuration
    uart_config_t uart_conf = {
        .baud_rate = UART_BAUD, // baud rate
        .data_bits = UART_DATA_8_BITS, // data bits per frame
        .parity = UART_PARITY_DISABLE, // disable parity bit, because we have CRC
        .stop_bits = UART_STOP_BITS_1, // 1 stop bit
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // no flow control; if Arduino misses a byte then we accept the loss
        .source_clk = UART_SCLK_DEFAULT // default clock source
    };
    uart_param_config(UART_NUM, &uart_conf);
    uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, -1, -1);
}

void sendUART(const uint8_t *payload, size_t len) {
    // Create Circular Redundancy Check value
    uint16_t crc = esp_rom_crc16_be(0, payload, len);

    // Send bytes
    uart_write_bytes(UART_NUM, UART_HEAD, 2); // send header
    uart_write_bytes(UART_NUM, payload, len); // send payload
    uart_write_bytes(UART_NUM, (uint8_t*)(&crc), 2); // send CRC
}
