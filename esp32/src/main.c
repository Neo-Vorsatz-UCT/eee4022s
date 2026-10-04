/*
    Program to control the ESP32, which is responsible for networking
    Author: Neo Vorsatz
    Last updated: 5 October 2026
*/

// #define RUN_TEST // Uncomment this to run the test script

// External includes
#include <stdint.h> // integers
#include "freertos/FreeRTOS.h" // FreeRTOS
#include "freertos/task.h"

// Internal includes
#include <uart_w_arduino.h>
#include <local_wifi.h>
#include <server.h>

// CONSTANTS ================================

#define PAYLOAD_LEN 24 // length of payload (motor setpoints)

// FUNCTION DECLARATIONS ================================

static uint16_t crc16(const uint8_t *d, size_t n);

/**
 * @brief Initializes UART, Wi-Fi, and server behaviour
 */
static void init();

/**
 * @brief Continuously receives motor setpoints, and passes these to the Arduino
 */
static void taskDTLS(void *arg);

// MAIN ================================

#ifdef RUN_TEST

void app_main() {
    // TODO: Implement code for testing
}

#else

void app_main() {
    // Initialize
    init();

    // Create task for receiving packets over DTLS
    xTaskCreate(taskDTLS, "dtls", 10240, NULL, 10, NULL);
}

#endif

// FUNCTION DEFINITIONS ================================

static uint16_t crc16(const uint8_t *d, size_t n) {
    uint16_t c = 0xFFFF;
    while (n--) {
        c ^= ((uint16_t)*d++) << 8;
        for (int i=0; i<8; i++) {
            c = (c & 0x8000) ? (c << 1) ^ 0x1021 : c << 1;
        }
    }
    return c;
}

static void init() {
    initUART();
    initNetwork();
    initServer();
}

static void taskDTLS(void *arg) {
    // Create buffer for receiving payloads
    uint8_t buf[256];

    // Start session and handle packets
    while(1) {
        // Wait for a new session to successfully start
        newSession();

        // Handle packets
        uint16_t lastSeq = 0; // initialize latest sequence number
        while(1) {
            int ret = getPacket(buf, sizeof(buf), &lastSeq);
            if (ret<=0) {break;} // if session terminated, start a new one

            // Send payload over UART to Arduino
            if (ret-2==PAYLOAD_LEN) {sendUART(buf+2, PAYLOAD_LEN);}
        }
    }
}
