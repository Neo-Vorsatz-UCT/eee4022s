/*
    Module to handle the connection to a local Wi-Fi network
    Implementation file
    Author: Neo Vorsatz
    Last updated: 9 October 2026
*/

// External includes
#include <string.h> // strings
#include "esp_log.h" // logging
#include "freertos/FreeRTOS.h" // FreeRTOS
#include "freertos/event_groups.h"
#include "esp_wifi.h" // WiFi
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "mdns.h" // mDNS

// Internal includes
#include <local_wifi.h>
#include "wifi_credentials.h"

// CONSTANTS ================================

// Wi-Fi
#define WIFI_MAX_ATTEMPTS 5 // maximum reattempts to connect to the Wi-Fi network
static const char *WTAG = "wifi";
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

// mDNS
static const char MDNS_HOSTNAME[] = "eee4022s-braccio"; // mDNS host name (63 characters max)
static const char MDNS_SERVICE[] ="_actuation"; // mDNS service name (15 characters max)
#define PORT_INT 42069 // server port as an int

// VARIABLES ================================

static EventGroupHandle_t eventsWiFi;
static int numReattempts = 0;

// HELPER FUNCTIONS ================================

/**
 * @brief Event handler for Wi-Fi events
 */
static void eventHandlerWiFi(void *arg, esp_event_base_t base, int32_t id, void *data) {
    // If the event is to start
    if (base==WIFI_EVENT && id==WIFI_EVENT_STA_START) {
        esp_wifi_connect(); // attempt to connect
    
    // If the event is that Wi-Fi disconnected
    } else if (base==WIFI_EVENT && id==WIFI_EVENT_STA_DISCONNECTED) {
        if (numReattempts<WIFI_MAX_ATTEMPTS) {
            numReattempts++;
            ESP_LOGW(WTAG, "disconnected, retry %d", numReattempts);
            esp_wifi_connect(); // reattempt to connect
        } else {
            xEventGroupSetBits(eventsWiFi, WIFI_FAIL_BIT); // note failure to connect to Wi-Fi (ran out of attempts)
        }
    
    // If an IP address was received
    } else if (base==IP_EVENT && id==IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(WTAG, "got ip: " IPSTR, IP2STR(&ev->ip_info.ip));
        numReattempts = 0; // reset attempts
        xEventGroupSetBits(eventsWiFi, WIFI_CONNECTED_BIT); // note sucessful connection
    }
}

/**
 * @brief Initializes the WiFi connection
 * 
 * @note This is blocking
 */
static void initWiFi(void) {
    // Initialize event group
    eventsWiFi = xEventGroupCreate();

    // Non-Volatile Storage (NVS) is required by the Wi-Fi driver
    esp_err_t err = nvs_flash_init(); // initialize NVS
    if (err==ESP_ERR_NVS_NO_FREE_PAGES || err==ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase()); // erase NVS
        err = nvs_flash_init(); // retry intialisation
    }
    ESP_ERROR_CHECK(err);

    // Initialize network stack
    ESP_ERROR_CHECK(esp_netif_init());

    // Create default event loop running in background
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create the Wi-Fi station
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(netif, "eee4022s-esp32"); // set DHCP hostname

    // Configure Wi-Fi initialization to defaults
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));

    // Associate the event handler
    ESP_ERROR_CHECK( esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, eventHandlerWiFi, NULL, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, eventHandlerWiFi, NULL, NULL) );

    // Configure Wi-Fi
    wifi_config_t wc = {0};
    strncpy((char *)wc.sta.ssid, WIFI_SSID, sizeof(wc.sta.ssid)-1);
    strncpy((char *)wc.sta.password, WIFI_PASSWORD, sizeof(wc.sta.password)-1);
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK; // refuse weaker networks
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));

    // Start Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_start());

    // Disable WiFi power saving, because starting up full WiFi capabilities introduces significant latency
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    // Wait for Wi-Fi to connect or fail
    EventBits_t bits = xEventGroupWaitBits(eventsWiFi, WIFI_CONNECTED_BIT|WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    if ((bits & WIFI_CONNECTED_BIT)==0) { // if Wi-Fi not connected
        ESP_LOGE("wifi", "connecting to Wi-Fi failed");
        esp_restart(); // failed to connect to Wi-Fi, so restart EVERYTHING
    }
}

/**
 * @brief Initializes mDNS
 */
static void initMDNS() {
    // Initialize
    mdns_init();

    // Configure hostname and service
    mdns_hostname_set(MDNS_HOSTNAME);
    mdns_service_add(NULL, MDNS_SERVICE, "_udp", PORT_INT, NULL, 0);
}

// API FUNCTIONS ================================

void initNetwork() {
    initWiFi();
    initMDNS();
}
