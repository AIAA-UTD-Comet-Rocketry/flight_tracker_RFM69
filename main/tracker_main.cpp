/*
 * SPDX-FileCopyrightText: 2010-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"

#include <RadioLib.h>
#include "TinyGPS++.h"
#include "radiolib_esp32s3_hal.hpp" // include the hardware abstraction layer
#include "aprs.h"

// GPS UART Configuration
#define GPS_UART_NUM   UART_NUM_0
#define GPS_TX_PIN     21  // TX not needed (GPS is sending data)
#define GPS_RX_PIN     20  // ESP32 RX (Connect to GPS TX)
#define BUF_SIZE       1024

//Todo: CANbus UART Configuration

// RFM69 SPI Configuration
#define RFM69_MOSI    0
#define RFM69_MISO    1
#define RFM69_SCK     10

#define RFM69_SS      13
#define RFM69_IRQ     27
#define RFM69_RST     15

// NSS pin:   18
// DIO0 pin:  26
// NRST pin:  14
// DIO1 pin:  33

// Logging TAG
static const char *TAG = "ESP32-GPS-RFM69";

// Create a new instance of the HAL class
EspHal* hal = new EspHal(5, 19, 27);

// Radio module instance
RF69 radio = new Module(hal, 18, 26, 14, 33);

// TinyGPS++ instance
TinyGPSPlus gps;

// APRS codec instance
APRSPacket packet;

void radio_hal_Init() {
    radio.setFrequency(433.4);
    radio.setBitRate(1.55);
    uint8_t sw[] = {0xAA, 0xD5};
    radio.setSyncWord(sw, sizeof(sw));

    int state = radio.begin();
    if (state != RADIOLIB_ERR_NONE) {
    ESP_LOGI(TAG, "failed, code %d\n", state);
    while(true) {
      hal->delay(1000);
    }
  }
  ESP_LOGI(TAG, "success!\n");
}

// Function to Initialize GPS UART
void gps_uart_init() {
    uart_config_t uart_config = {};
    uart_config.baud_rate = 38400;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

    uart_driver_install(GPS_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(GPS_UART_NUM, &uart_config);
    uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}


void aprs_init() {
    packet.source = AX25Address::from_string("KK7SSP-11");
    packet.destination = AX25Address::from_string("APRS");
    packet.path = { AX25Address::from_string("WIDE1-1"), AX25Address::from_string("WIDE2-1") };
}


// Function to Initialize SPI for RFM69
void spi_radio_bus_init() {
    spi_bus_config_t buscfg = {};  // Initialize to 0
    buscfg.mosi_io_num = RFM69_MOSI;
    buscfg.miso_io_num = RFM69_MISO;
    buscfg.sclk_io_num = RFM69_SCK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "RFM69 SPI Initialized");
}


// Main Task for GPS Data Processing
void gps_task(void *pvParameters) {
    uint8_t data[BUF_SIZE];
    while (true) {
        int len = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                gps.encode(data[i]);
            }
        }

        // If new GPS location is available
        if (gps.location.isUpdated()) {
            char gps_buffer[64] = {};

            // Debug string
            snprintf(gps_buffer, sizeof(gps_buffer), "Lat: %.6f, Lon: %.6f",
                     gps.location.lat(), gps.location.lng());
            ESP_LOGI(TAG, "GPS: %s", gps_buffer);

            // Radio string
            packet.payload = snprintf(gps_buffer, sizeof(gps_buffer),
                                      "=%.2fN/%.2fW-Team 317", 
                                      gps.location.lat(), gps.location.lng());

            std::vector<uint8_t> APRSencoded = packet.encode();

            radio.transmit((uint8_t*)APRSencoded.data(), APRSencoded.size());

            /*
             * Without FIFO stitching or interrupt based packet management (ISR/DMA) there
             * is a limit to RFM69 radio frames containing data payloads less than 64 bytes.
             * Reduce comment field lengths or split transmission into multiple packets.
             */
            if (APRSencoded.size() >= 64)
                ESP_LOGI(TAG, "Radio FIFO error, check main file for comments");
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for stability
    }
}


// Radio task for testing transcievers
void radio_test(void *pvParameters) {
    // initialize
    ESP_LOGI(TAG, "[SX1276] Initializing ... ");
    int state = radio.begin();
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGI(TAG, "failed, code %d\n", state);
        while(true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    ESP_LOGI(TAG, "success!\n");

    while(1) {
        uint8_t testBuff[8] = {0x07, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
        ESP_LOGI(TAG, "[RFM69] Transmitting packet ... ");
        //state = radio.transmit("Hello World!");
        //transmit(const uint8_t* data, size_t len, uint8_t addr)
        state = radio.transmit(testBuff, 8);
        if (state == RADIOLIB_ERR_NONE) {
            // the packet was successfully transmitted
            ESP_LOGI(TAG, "success!");
        } else {
            ESP_LOGI(TAG, "failed, code %d\n", state);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }  
}


void chipIdEcho() {
    printf("Starting Chip Identification");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n\n", esp_get_minimum_free_heap_size());
}

extern "C" void app_main(void)
{
    printf("hello world");
    fflush(stdout);

    gps_uart_init();
    spi_radio_bus_init();
    radio_hal_Init(); // RFM69 connection

    xTaskCreate(radio_test, "radio_test", 2048, NULL, 5, NULL);
    xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);
}