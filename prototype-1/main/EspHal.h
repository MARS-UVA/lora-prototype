#ifndef ESP_IDF_CUSTOM_HAL_H
#define ESP_IDF_CUSTOM_HAL_H

// ============================================================================
// RadioLib ESP-IDF HAL
// Compatible with ESP32-S3 and ESP32-C6
//
// Based on RadioLib's ESP-IDF HAL:
// https://github.com/jgromes/RadioLib/blob/master/src/hal/ESP-IDF/EspHal.h
//
// This version uses the ESP-IDF SPI master driver, so it does not directly
// access ESP32-family hardware registers.
//
// IMPORTANT:
// ESP_HAL_H must be defined BEFORE including RadioLib.h in main.cpp so that
// RadioLib does not automatically include its own EspHal.h.
// ============================================================================

#include <RadioLib.h>

// ============================================================================
// ESP-IDF
// ============================================================================

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

#include <inttypes.h>

// ============================================================================
// Arduino-style constants expected by RadioLibHal
// ============================================================================

#ifndef LOW
#define LOW 0
#endif

#ifndef HIGH
#define HIGH 1
#endif

#ifndef INPUT
#define INPUT GPIO_MODE_INPUT
#endif

#ifndef OUTPUT
#define OUTPUT GPIO_MODE_OUTPUT
#endif

#ifndef RISING
#define RISING GPIO_INTR_POSEDGE
#endif

#ifndef FALLING
#define FALLING GPIO_INTR_NEGEDGE
#endif

#ifndef NOP
#define NOP() asm volatile("nop")
#endif

// ============================================================================
// EspHal
// ============================================================================

class EspHal : public RadioLibHal {

public:

    // ------------------------------------------------------------------------
    // Constructor
    //
    // sck      = SPI clock GPIO
    // miso     = SPI MISO GPIO
    // mosi     = SPI MOSI GPIO
    // host     = SPI peripheral to use
    // clockHz  = SPI clock frequency
    //
    // SPI2_HOST works on both ESP32-S3 and ESP32-C6.
    // ------------------------------------------------------------------------

    EspHal(
        int8_t sck,
        int8_t miso,
        int8_t mosi,
        spi_host_device_t host = SPI2_HOST,
        uint32_t clockHz = 2000000
    )
        : RadioLibHal(
            GPIO_MODE_INPUT,
            GPIO_MODE_OUTPUT,
            LOW,
            HIGH,
            GPIO_INTR_POSEDGE,
            GPIO_INTR_NEGEDGE
        ),
        spiSCK(sck),
        spiMISO(miso),
        spiMOSI(mosi),
        spiHost(host),
        spiClockHz(clockHz),
        spiDevice(nullptr),
        busInitialized(false),
        deviceAdded(false),
        halInitialized(false)
    {
    }

    virtual ~EspHal() {
        term();
    }

    // ========================================================================
    // GPIO
    // ========================================================================

    void pinMode(
        uint32_t pin,
        uint32_t mode
    ) override {

        if (pin == RADIOLIB_NC) {
            return;
        }

        gpio_config_t config = {};

        config.pin_bit_mask = (1ULL << pin);

        // RadioLib passes GPIO_MODE_INPUT / GPIO_MODE_OUTPUT.
        config.mode =
            (mode == GPIO_MODE_OUTPUT)
                ? GPIO_MODE_OUTPUT
                : GPIO_MODE_INPUT;

        config.pull_up_en = GPIO_PULLUP_DISABLE;
        config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        config.intr_type = GPIO_INTR_DISABLE;

        gpio_config(&config);
    }

    void digitalWrite(
        uint32_t pin,
        uint32_t value
    ) override {

        if (pin == RADIOLIB_NC) {
            return;
        }

        gpio_set_level(
            (gpio_num_t)pin,
            value ? 1 : 0
        );
    }

    uint32_t digitalRead(
        uint32_t pin
    ) override {

        if (pin == RADIOLIB_NC) {
            return 0;
        }

        return gpio_get_level(
            (gpio_num_t)pin
        );
    }

    // ========================================================================
    // Interrupts
    // ========================================================================

    void attachInterrupt(
        uint32_t interruptNum,
        void (*interruptCb)(void),
        uint32_t mode
    ) override {

        if (interruptNum == RADIOLIB_NC) {
            return;
        }

        // The GPIO ISR service is installed once in init().
        gpio_set_intr_type(
            (gpio_num_t)interruptNum,
            (gpio_int_type_t)mode
        );

        gpio_isr_handler_add(
            (gpio_num_t)interruptNum,
            (gpio_isr_t)interruptCb,
            nullptr
        );
    }

    void detachInterrupt(
        uint32_t interruptNum
    ) override {

        if (interruptNum == RADIOLIB_NC) {
            return;
        }

        gpio_isr_handler_remove(
            (gpio_num_t)interruptNum
        );

        gpio_set_intr_type(
            (gpio_num_t)interruptNum,
            GPIO_INTR_DISABLE
        );
    }

    // ========================================================================
    // Timing
    // ========================================================================

    void delay(
        unsigned long ms
    ) override {

        if (ms == 0) {
            return;
        }

        // Use the same general approach as RadioLib's current ESP-IDF HAL:
        // sleep for most of the delay, then busy-wait the remainder.

        const uint64_t target_us =
            (uint64_t)esp_timer_get_time()
            + ((uint64_t)ms * 1000ULL);

        const TickType_t ticks =
            pdMS_TO_TICKS(ms);

        if (ticks > 1) {
            vTaskDelay(ticks - 1);
        }

        int64_t remaining_us =
            (int64_t)target_us
            - (int64_t)esp_timer_get_time();

        if (remaining_us > 0) {
            esp_rom_delay_us(
                (uint32_t)remaining_us
            );
        }
    }

    void delayMicroseconds(
        unsigned long us
    ) override {

        if (us == 0) {
            return;
        }

        esp_rom_delay_us(
            (uint32_t)us
        );
    }

    unsigned long millis() override {

        return (unsigned long)(
            esp_timer_get_time() / 1000ULL
        );
    }

    unsigned long micros() override {

        return (unsigned long)(
            esp_timer_get_time()
        );
    }

    long pulseIn(
        uint32_t pin,
        uint32_t state,
        unsigned long timeout
    ) override {

        if (pin == RADIOLIB_NC) {
            return 0;
        }

        unsigned long startMicros = micros();

        // Wait for pulse to start.
        while (digitalRead(pin) != state) {

            unsigned long currentMicros = micros();

            if (
                currentMicros - startMicros
                >= timeout
            ) {
                return 0;
            }
        }

        unsigned long pulseStart = micros();

        // Measure pulse duration.
        while (digitalRead(pin) == state) {

            unsigned long currentMicros = micros();

            if (
                currentMicros - pulseStart
                >= timeout
            ) {
                return 0;
            }
        }

        return (long)(
            micros() - pulseStart
        );
    }

    // ========================================================================
    // SPI initialization
    // ========================================================================

    void spiBegin() {

        if (deviceAdded) {
            return;
        }

        ESP_LOGI(
            "EspHal",
            "Initializing SPI host %d: SCK=%d MISO=%d MOSI=%d",
            spiHost,
            spiSCK,
            spiMISO,
            spiMOSI
        );

        // --------------------------------------------------------------------
        // Configure SPI bus
        // --------------------------------------------------------------------

        spi_bus_config_t busConfig = {};

        busConfig.mosi_io_num = spiMOSI;
        busConfig.miso_io_num = spiMISO;
        busConfig.sclk_io_num = spiSCK;

        busConfig.quadwp_io_num = -1;
        busConfig.quadhd_io_num = -1;

        // These fields are available on modern ESP-IDF versions and are
        // harmlessly initialized here.
        busConfig.data4_io_num = -1;
        busConfig.data5_io_num = -1;
        busConfig.data6_io_num = -1;
        busConfig.data7_io_num = -1;

        busConfig.max_transfer_sz =
            SOC_SPI_MAXIMUM_BUFFER_SIZE;

        busConfig.flags = 0;

        busConfig.isr_cpu_id =
            ESP_INTR_CPU_AFFINITY_AUTO;

        busConfig.intr_flags = 0;

        // DMA_CH_AUTO is what the current RadioLib ESP-IDF HAL uses.
        //
        // This is preferable to SPI_DMA_DISABLED because the latter limits
        // transfers to small buffers.
        esp_err_t err = spi_bus_initialize(
            spiHost,
            &busConfig,
            SPI_DMA_CH_AUTO
        );

        if (err == ESP_OK) {

            busInitialized = true;

            ESP_LOGI(
                "EspHal",
                "SPI bus initialized"
            );

        }
        else if (err == ESP_ERR_INVALID_STATE) {

            // Another component already initialized this SPI bus.
            //
            // This is allowed because the SPI bus may be shared.
            busInitialized = false;

            ESP_LOGI(
                "EspHal",
                "SPI bus already initialized"
            );

        }
        else {

            ESP_LOGE(
                "EspHal",
                "SPI bus initialization failed: %s",
                esp_err_to_name(err)
            );

            return;
        }

        // --------------------------------------------------------------------
        // Configure the RFM95W SPI device
        //
        // IMPORTANT:
        //
        // spics_io_num = -1
        //
        // RadioLib controls CS itself using digitalWrite().
        // --------------------------------------------------------------------

        spi_device_interface_config_t deviceConfig = {};

        deviceConfig.command_bits = 0;
        deviceConfig.address_bits = 0;
        deviceConfig.dummy_bits = 0;

        // SX1276 uses SPI mode 0.
        deviceConfig.mode = 0;

        deviceConfig.clock_source =
            SPI_CLK_SRC_DEFAULT;

        deviceConfig.duty_cycle_pos = 128;

        deviceConfig.cs_ena_pretrans = 0;
        deviceConfig.cs_ena_posttrans = 0;

        deviceConfig.clock_speed_hz =
            (int)spiClockHz;

        deviceConfig.input_delay_ns = 0;

        // IMPORTANT:
        // RadioLib controls CS.
        deviceConfig.spics_io_num = -1;

        deviceConfig.flags = 0;
        deviceConfig.queue_size = 1;

        deviceConfig.pre_cb = nullptr;
        deviceConfig.post_cb = nullptr;

        err = spi_bus_add_device(
            spiHost,
            &deviceConfig,
            &spiDevice
        );

        if (err != ESP_OK) {

            ESP_LOGE(
                "EspHal",
                "SPI device initialization failed: %s",
                esp_err_to_name(err)
            );

            return;
        }

        deviceAdded = true;

        ESP_LOGI(
            "EspHal",
            "SPI device added, clock=%" PRIu32 " Hz",
            spiClockHz
        );
    }

    // ========================================================================
    // SPI transaction begin
    // ========================================================================

    void spiBeginTransaction() {

        if (spiDevice != nullptr) {

            spi_device_acquire_bus(
                spiDevice,
                portMAX_DELAY
            );
        }
    }

    // ========================================================================
    // SPI transfer
    // ========================================================================

    void spiTransfer(
        uint8_t* out,
        size_t len,
        uint8_t* in
    ) {

        if (spiDevice == nullptr) {

            ESP_LOGE(
                "EspHal",
                "SPI device not initialized"
            );

            return;
        }

        if (len == 0) {
            return;
        }

        spi_transaction_t transaction = {};

        transaction.length =
            len * 8;

        transaction.tx_buffer =
            out;

        transaction.rx_buffer =
            in;

        esp_err_t err =
            spi_device_polling_transmit(
                spiDevice,
                &transaction
            );

        if (err != ESP_OK) {

            ESP_LOGE(
                "EspHal",
                "SPI transfer failed: %s",
                esp_err_to_name(err)
            );
        }
    }

    // ========================================================================
    // SPI transaction end
    // ========================================================================

    void spiEndTransaction() {

        if (spiDevice != nullptr) {

            spi_device_release_bus(
                spiDevice
            );
        }
    }

    // ========================================================================
    // SPI shutdown
    // ========================================================================

    void spiEnd() {

        if (
            deviceAdded &&
            spiDevice != nullptr
        ) {

            spi_bus_remove_device(
                spiDevice
            );

            spiDevice = nullptr;
            deviceAdded = false;

            ESP_LOGI(
                "EspHal",
                "SPI device removed"
            );
        }

        // Only free the bus if THIS HAL initialized it.
        if (busInitialized) {

            spi_bus_free(
                spiHost
            );

            busInitialized = false;

            ESP_LOGI(
                "EspHal",
                "SPI bus freed"
            );
        }
    }

    // ========================================================================
    // RadioLib HAL initialization
    // ========================================================================

    void init() override {

        if (halInitialized) {
            return;
        }

        spiBegin();

        // Install the GPIO ISR service once.
        esp_err_t err =
            gpio_install_isr_service(
                ESP_INTR_FLAG_IRAM
            );

        if (
            err != ESP_OK &&
            err != ESP_ERR_INVALID_STATE
        ) {

            ESP_LOGE(
                "EspHal",
                "GPIO ISR service installation failed: %s",
                esp_err_to_name(err)
            );
        }

        halInitialized = true;
    }

    // ========================================================================
    // RadioLib HAL termination
    // ========================================================================

    void term() override {

        if (!halInitialized) {
            return;
        }

        spiEnd();

        halInitialized = false;
    }

    // ========================================================================
    // Optional RadioLib functions
    // ========================================================================

    void tone(
        uint32_t pin,
        unsigned int frequency,
        RadioLibTime_t duration = 0
    ) override {

        // Not needed for the RFM95W.
        //
        // RadioLib requires the HAL interface to provide this function,
        // but your LoRa application does not use it.

        (void)pin;
        (void)frequency;
        (void)duration;
    }

    void noTone(
        uint32_t pin
    ) override {

        (void)pin;
    }

    void yield() override {

        taskYIELD();
    }

    void pullUpDown(
        uint32_t pin,
        bool enable,
        bool up
    ) override {

        if (pin == RADIOLIB_NC) {
            return;
        }

        if (enable) {

            gpio_set_pull_mode(
                (gpio_num_t)pin,
                up
                    ? GPIO_PULLUP_ONLY
                    : GPIO_PULLDOWN_ONLY
            );

        } else {

            gpio_set_pull_mode(
                (gpio_num_t)pin,
                GPIO_FLOATING
            );
        }
    }

private:

    // ========================================================================
    // SPI configuration
    // ========================================================================

    int8_t spiSCK;
    int8_t spiMISO;
    int8_t spiMOSI;

    spi_host_device_t spiHost;

    uint32_t spiClockHz;

    // ========================================================================
    // SPI state
    // ========================================================================

    spi_device_handle_t spiDevice;

    bool busInitialized;
    bool deviceAdded;
    bool halInitialized;
};

#endif // ESP_IDF_CUSTOM_HAL_H