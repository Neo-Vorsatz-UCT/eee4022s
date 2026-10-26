/*
    Module to handle the DTLS server start-up and communication
    Implementation file
    Author: Neo Vorsatz
    Last updated: 26 October 2026
    Built from the example at: https://github.com/Mbed-TLS/mbedtls/blob/development/programs/ssl/dtls_server.c
*/

// External includes
#include "esp_log.h" // logging
#include <string.h> // strings
#include "psa/crypto.h" // PSA cryptography
#include "mbedtls/net_sockets.h" // DTLS networking
#include "mbedtls/ssl.h"
#include "mbedtls/ssl_cookie.h"
#include "mbedtls/timing.h"
#include "esp_timer.h" // benchmarking

// Internal includes
#include "server.h"

// CONSTANTS ================================

// Benchmarking
#define BENCHMARK

// PSK
static const unsigned char PSK[16] = {
    0xc7, 0x67, 0x6c, 0xa0, 0x68, 0xaf, 0xf2, 0x29, 0xe5, 0xac, 0x4e, 0x3f, 0x8e, 0x37, 0x21, 0x03
}; // hidden key generated with OpenSSL 3.6.4
static const char PSK_ID[] = "eee4022s-braccio-psk"; // identification for PSK

// Miscellaneous
#define PORT_STR "42069" // server port as a str
#define IDLE_MS 1000 // idle time until a session is closed, in milliseconds
static const char TAG[] = "dtls_rx"; // task tag

// VARIABLES ================================

// Contexts
static mbedtls_net_context listen_fd, client_fd;
static mbedtls_ssl_context ssl;
static mbedtls_ssl_config conf;
static mbedtls_ssl_cookie_ctx cookie;
static mbedtls_timing_delay_context timer;
static unsigned char client_ip[16];
static size_t cip_len;

// FUNCTIONS ================================

void initServer() {
    // Initialize PSA cryptography
    psa_status_t status = psa_crypto_init();
    if (status!=PSA_SUCCESS) {
        ESP_LOGE(TAG, "psa_crypto_init failed: %d", (int)status);
        abort();
    }

    // Initialize contexts - honestly, the documentation of this API isn't very descriptive. I'm just trusting this.
    mbedtls_net_init(&listen_fd); mbedtls_net_init(&client_fd);
    mbedtls_ssl_init(&ssl); mbedtls_ssl_config_init(&conf);
    mbedtls_ssl_cookie_init(&cookie);

    // Bind the application to listen for UDP traffic on the given port
    /*
        We add an explicit IPv4 wildcard, since lwIP may not have IPv6 enabled
    */
    mbedtls_net_bind(&listen_fd, "0.0.0.0", PORT_STR, MBEDTLS_NET_PROTO_UDP);

    // Configure DTLS protocol
    mbedtls_ssl_config_defaults(&conf, MBEDTLS_SSL_IS_SERVER, MBEDTLS_SSL_TRANSPORT_DATAGRAM, MBEDTLS_SSL_PRESET_DEFAULT);

    // Configure static PSK
    static const int suites[] = {MBEDTLS_TLS_PSK_WITH_AES_128_GCM_SHA256, 0};
    mbedtls_ssl_conf_ciphersuites(&conf, suites);
    mbedtls_ssl_conf_psk(&conf, PSK, sizeof PSK, (const unsigned char *)PSK_ID, strlen(PSK_ID));
    /*
        We don't use `mbedtls_ssl_conf_psk_cb()` because we're not checking identity of the client.
        Any client with the PSK is a valid client.
    */
    
    // Configure Hello Verification using cookies ~ yummy
    mbedtls_ssl_cookie_setup(&cookie);
    mbedtls_ssl_conf_dtls_cookies(&conf, mbedtls_ssl_cookie_write, mbedtls_ssl_cookie_check, &cookie);

    // Configure timeouts
    mbedtls_ssl_conf_handshake_timeout(&conf, 250, 2000);
    mbedtls_ssl_conf_read_timeout(&conf, IDLE_MS);

    // Finalise DTLS
    mbedtls_ssl_setup(&ssl, &conf);
    mbedtls_ssl_set_timer_cb(&ssl, &timer, mbedtls_timing_set_delay, mbedtls_timing_get_delay);
}

void newSession() {
    while (1) {
        // Reset connection and context
        mbedtls_net_free(&client_fd);
        mbedtls_ssl_session_reset(&ssl);

        // Blocks until a datagram arrives; "connects" the socket to the client
        if (mbedtls_net_accept(&listen_fd, &client_fd, client_ip, sizeof(client_ip), &cip_len)!=0) {continue;}

        // Save clients identification (for the cookie)
        mbedtls_ssl_set_client_transport_id(&ssl, client_ip, cip_len);

        // Set the basic input/output callbacks for read/write
        mbedtls_ssl_set_bio(&ssl, &client_fd, mbedtls_net_send, mbedtls_net_recv, mbedtls_net_recv_timeout);

        // Handshake while waiting for read/write
        int ret;
        do {ret = mbedtls_ssl_handshake(&ssl);}
        while (ret==MBEDTLS_ERR_SSL_WANT_READ || ret==MBEDTLS_ERR_SSL_WANT_WRITE);

        // Validate the handshake result
        if (ret==MBEDTLS_ERR_SSL_HELLO_VERIFY_REQUIRED) {continue;} // cookie was sent, and now the client must retry
        if (ret!=0) { // handshake failed
            ESP_LOGW(TAG, "handshake failed: -0x%04x", -ret);
            continue;
        }

        // A session has started
        ESP_LOGI(TAG, "session up"); // log that a session has started
        return;
    }
}

int getPacket(uint8_t *write, unsigned long len, uint16_t *lastSeq) {
    #ifdef BENCHMARK
    float bench_start = esp_timer_get_time()/1000.0f;
    float bench_delta = 0;
    #endif
    int ret;
    while (1) {
        // Read a packet from the session into the buffer
        ret = mbedtls_ssl_read(&ssl, write, len);
        if (ret==MBEDTLS_ERR_SSL_WANT_READ || ret==MBEDTLS_ERR_SSL_WANT_WRITE) { // if no data received
            #ifdef BENCHMARK
            bench_start = esp_timer_get_time()/1000.0f; // update start time
            #endif
            continue;
        }

        // Validate
        if (ret<=0) { // timeout/close/error
            ESP_LOGI(TAG, "session ended: %d", -ret);
            return ret;
        }
        if (ret!=26) {continue;} // check that the payload is the correct length

        // Check sequence number (assumes little endian)
        uint16_t seq = (write[1]<<8) | write[0]; // assemble sequence number
        if ((int16_t)(seq-*lastSeq)<=0) {continue;} // drop duplicate or old packets
        *lastSeq = seq;

        // Benchmarking
        #ifdef BENCHMARK
        bench_delta = esp_timer_get_time()/1000.0f - bench_start; // calculate decryption and verification latency
        ESP_LOGI(TAG, "latency: %f", bench_delta);
        #endif

        // Return length of payload
        return ret;
    }
}
