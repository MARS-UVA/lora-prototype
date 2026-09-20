#include <stdio.h>
#include <stdint.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

#define SPICLK_PIN 12
#define SPIMISO_PIN 13
#define SPIMOSI_PIN 11
#define SPICS_PIN 10
#define G0_PIN 9

spi_device_handle_t rfm69;
uint8_t transmit_buffer[2] = {0x42, 0x00};
uint8_t receive_buffer[2];

spi_transaction_t transaction = {
    .length = 16,
    .tx_buffer = transmit_buffer,
    .rx_buffer = receive_buffer,
};


void spi_init(void){
    spi_bus_config_t busconfig = {
        .mosi_io_num = SPIMOSI_PIN,
        .miso_io_num = SPIMISO_PIN,
        .sclk_io_num = SPICLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_device_interface_config_t devconfig = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = SPICS_PIN,
        .queue_size = 1
    };

    spi_bus_initialize(SPI2_HOST, &busconfig, SPI_DMA_CH_AUTO);

    spi_bus_add_device(SPI2_HOST, &devconfig, &rfm69);
}

void app_main(void)
{
    spi_init();
    ESP_ERROR_CHECK(spi_device_transmit(rfm69, &transaction));
    printf("%02x \n", receive_buffer[0]);
    printf("%02x \n", receive_buffer[1]);
}
