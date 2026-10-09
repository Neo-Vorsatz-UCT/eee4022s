/*
    Program to control the ESP32, which is responsible for networking
    Author: Neo Vorsatz
    Last updated: 8 October 2026
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
    // Initialize
    initUART();

    // Define setpoint
    union {
        float values[6];
        uint8_t bytes[6*sizeof(float)];
    } setpoint;
    for (int i = 0; i<6; i++) {
        setpoint.values[i] = 90.f;
    }
    setpoint.values[5] = 180.0f; // close the hand

    // Constantly send setpoint
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10)); // wait 10 milliseconds
        sendUART(setpoint.bytes, 6*sizeof(float));
    }
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
