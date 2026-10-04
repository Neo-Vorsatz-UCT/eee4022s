/*
    Module to handle the DTLS server start-up and communication
    Header file
    Author: Neo Vorsatz
    Last updated: 3 October 2026
    Built from the example at: https://github.com/Mbed-TLS/mbedtls/blob/development/programs/ssl/dtls_server.c
*/

#ifndef SERVER_H
#define SERVER_H

// External includes
#include <stdint.h> // integers

// FUNCTIONS ================================

/**
 * @brief Initializes DTLS server
 */
void initServer();

/**
 * @brief Starts a new session
 * 
 * @note This is blocking, until a client successfully starts a new session with this server
 */
void newSession();

/**
 * @brief Gets the next packet from the client
 * 
 * @param write A buffer to write the payload to
 * @param len The length of the buffer
 * @param lastSeq Address of last sequence number, used for sequencing packets
 * 
 * @return The length of the payload (in bytes), or <=0 is the session terminated
 * 
 * @note This is blocking, until the client sends a valid packet.
 * The last sequence number `lastSeq` just needs to be initialized, then this function will handle its value.
 */
int getPacket(uint8_t *write, unsigned long len, uint16_t *lastSeq);

#endif
