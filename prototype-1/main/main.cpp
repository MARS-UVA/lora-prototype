#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ESP_HAL_H
#include <RadioLib.h>
#include "EspHal.h"

#include "esp_log.h"

// ============================================================
// USER CONFIGURATION
// ============================================================

// Give each ESP32 a different ID.
// ESP32 #1:
#define NODE_ID 1

// ESP32 #2:
// #define NODE_ID 2

// RFM95W frequency.
// MUST match the frequency of both RFM95W modules.
//
// For an Adafruit RFM95W 915 MHz module:
#define RF_FREQUENCY 915.0

// ============================================================
// RFM95W PIN CONNECTIONS
// ============================================================
// ESP #1 
// define SPI and radio pins
#define PIN_CS      10  // CS
#define PIN_SPIQ    13   // MISO
#define PIN_SPID    11   // MOSI
#define PIN_SPICLK  12   // SCLK
// #define PIN_RST     10  // RST
#define PIN_G0      9   // G0


// ESP #2 
// define SPI and radio pins
// #define PIN_CS      20  // CS
// #define PIN_SPIQ    2   // MISO
// #define PIN_SPID    7   // MOSI
// #define PIN_SPICLK  6   // SCLK
// #define PIN_RST     10  // RST
// #define PIN_G0      0   // G0

// PROPOSITION: Here we can use static allocations. Using the "new" keyword
// in CPP dynamically allocates to the heap, which is largely avoided in 
// embedded applications.

// RadioLib ESP-IDF HAL
//
// Consider stack allocations over dynamic allocations.
EspHal hal(PIN_SPICLK, PIN_SPIQ, PIN_SPID);

Module mod(&hal, PIN_CS, PIN_G0, RADIOLIB_NC);

// SX1276 / RFM95
SX1276 radio(&mod);

static const char* TAG = "RFM95";

// ============================================================
// PACKET BUFFER
// ============================================================

uint8_t receiveBuffer[256];

// ============================================================
// INITIALIZE RADIO
// ============================================================

bool initializeRadio(){
    ESP_LOGI(TAG, "Initializing RFM95W...");

    int state = radio.begin();

    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Radio initialization failed, code %d", state);
        return false;
    }

    ESP_LOGI(TAG, "Radio initialization successful");

    // Set frequency
    state = radio.setFrequency(RF_FREQUENCY);

    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Failed to set frequency, code %d", state);
        return false;
    }

    ESP_LOGI(TAG, "Frequency: %.1f MHz", RF_FREQUENCY);

    // --------------------------------------------------------
    // LoRa settings
    // --------------------------------------------------------
    //
    // These are approximately the same settings used by
    // the Adafruit RadioHead example:
    //
    // Bandwidth = 125 kHz
    // Coding rate = 4/5
    // Spreading factor = 7
    // CRC enabled
    //
    // RadioLib defaults are generally suitable, but we
    // explicitly configure them here so both radios match.

    state = radio.setBandwidth(125.0);

    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Failed to set bandwidth, code %d", state);
        return false;
    }

    state = radio.setSpreadingFactor(7);

    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Failed to set spreading factor, code %d", state);
        return false;
    }

    state = radio.setCodingRate(5);

    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Failed to set coding rate, code %d", state);
        return false;
    }

    // Enable CRC (Cyclic Redundancy Check): "an error-detecting code used to verify 
    // message integrity and ensure that data packets have not been corrupted during
    // wireless transmission"
    state = radio.setCRC(2);

    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Failed to enable CRC, code %d", state);
        return false;
    }

    // --------------------------------------------------------
    // Transmit power
    // --------------------------------------------------------
    //
    // RFM95W supports PA_BOOST output.
    // 20 dBm is a safer starting point than immediately using
    // the maximum 23 dBm.

    state = radio.setOutputPower(20);

    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "Failed to set TX power, code %d", state);
        return false;
    }

    ESP_LOGI(TAG, "Radio configuration complete");

    return true;
}

// ============================================================
// TRANSMIT
// ============================================================

void sendPacket(const char* message){
    ESP_LOGI(TAG, "TX: %s", message);

    int state = radio.transmit(message);

    if (state == RADIOLIB_ERR_NONE) {
        ESP_LOGI(TAG, "TX successful");
    }
    else {
        ESP_LOGE(TAG, "TX failed, code %d", state);
    }
}

// ============================================================
// RECEIVE
// ============================================================
void receivePacket(){
    int state = radio.receive(
        receiveBuffer,
        sizeof(receiveBuffer) - 1,
        3000 // timeout if nothing received after 3s
    );

    if (state == RADIOLIB_ERR_NONE) {

        size_t len = radio.getPacketLength();

        if (len >= sizeof(receiveBuffer)) {
            len = sizeof(receiveBuffer) - 1;
        }

        receiveBuffer[len] = '\0';

        ESP_LOGI(TAG, "RX: %s", (char*)receiveBuffer);

        // RSSI = Received Signal Strength Indicator measuered in dB
        float rssi = radio.getRSSI();
        ESP_LOGI(TAG, "RSSI: %.1f dBm", rssi);

        float snr = radio.getSNR();
        ESP_LOGI(TAG, "SNR: %.1f dB", snr);

    } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {

        // No packet received — this is normal.
        
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {

        ESP_LOGW(TAG, "CRC error");

    } else {

        ESP_LOGE(TAG, "RX failed, code %d", state);
    }
}

// ============================================================
// MAIN
// ============================================================

// extern "C" gives this function C-style Linkage
extern "C" void app_main(void){
    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "RFM95W LoRa node %d", NODE_ID);
    ESP_LOGI(TAG, "================================");

    // Initialize radio
    if (!initializeRadio()) {

        ESP_LOGE(TAG, "Radio initialization failed");

        while (true) {
            hal.delay(1000);
        }
    }

    ESP_LOGI(TAG, "Radio ready");

    // --------------------------------------------------------
    // Main loop
    // --------------------------------------------------------

    uint32_t packetNumber = 0;

    while (true) {
        if (NODE_ID == 1){
        // ----------------------------------------------------
        // Transmit a packet
        // ----------------------------------------------------

        // Testing simple character input from stdin
        // BE AWARE of chars left in stdin buffer if multiple chars entered before \n

        printf("Enter the singular char you want to transmit: ");

        char c = getchar();

        char message[100];

        snprintf(
            message,
            sizeof(message),
            "Hello from node %d (packet %lu): char = %c",
            NODE_ID,
            (unsigned long)packetNumber++,
            c
        );

        sendPacket(message);

        hal.delay(2000);

    } else {

        // ----------------------------------------------------
        // Listen for a reply
        // ----------------------------------------------------

        ESP_LOGI(TAG, "Listening for packets...");

        receivePacket();

        // Wait before transmitting again
        hal.delay(100);
    }
    }
}
