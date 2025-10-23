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
#include "esp_wifi.h"
#include "math.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

#include <RadioLib.h>
#include "TinyGPS++.h"
#include "radiolib_esp32s3_hal.hpp" // include the hardware abstraction layer
#include "aprs.h"
#include "global_config.h"
#include "espnow_rx.h"

// GPS UART Configuration
#define GPS_UART_NUM   UART_NUM_1  // Changed from UART_NUM_0 to avoid conflict with console
#define GPS_TX_PIN     17  // TX not needed (GPS is sending data)
#define GPS_RX_PIN     18  // ESP32 RX (Connect to GPS TX)
#define BUF_SIZE       1024

// RFM69 SPI Configuration
#define RFM69_SCK     13 
#define RFM69_MISO    14 
#define RFM69_MOSI    11 
#define RFM69_CS      10 
#define RFM69_IRQ     46 // G0 Pin in Breakout board 
#define RFM69_RST     9 
#define RFM69_GPIO    19 // Not currently used 

// TODO: CANbus/TWAI UART Configuration
// TODO: Wifi setup
// TODO: LED status task

static const char *TAG = "ESP32-GPS-RFM69"; // Logging TAG
static QueueHandle_t espnow_q = NULL;

// Create a new instance of the HAL class
EspHal* hal = new EspHal(RFM69_SCK, RFM69_MISO, RFM69_MOSI);

// Radio module instance
RF69 radio = new Module(hal, RFM69_CS, RFM69_IRQ, RFM69_RST, RFM69_GPIO);

// TinyGPS++ instance
TinyGPSPlus gps;

// APRS codec instance
APRSPacket packet;

static bool radio_init() {
    ESP_LOGI(TAG, "[RFM69] Initializing...");
    int st = radio.begin();
    if (st != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "radio.begin() failed: %d", st);
        return false;
    }

    // Start with conservative/forgiving link params; align to ground RX later
    radio.setFrequency(RADIO_FREQ);          
    radio.setBitRate(BIT_RATE);            
    radio.setFrequencyDeviation(DEVIATION_FREQ);    
    radio.setRxBandwidth(RX_BANDWITH);      
    radio.setOOK(false);                 
    radio.setOutputPower(OUTPUT_PWR);
    radio.setSyncWord(sw, sizeof(sw));

    ESP_LOGI(TAG, "RFM69 ready");
    return true;
}

// Function to Initialize GPS UART
static void gps_uart_init() {
    ESP_LOGI(TAG, "Entered GPS function");
    uart_config_t uart_config = {};
    uart_config.baud_rate = GPS_BAUD_RATE;  
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

    uart_driver_install(GPS_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(GPS_UART_NUM, &uart_config);
    uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_LOGI(TAG, "GPS UART initialized at %d baud on UART%d", uart_config.baud_rate, GPS_UART_NUM);
}

static void aprs_init() {
    packet.source = AX25Address::from_string(CALLSIGN);
    packet.destination = AX25Address::from_string("APRS");
    packet.path = { AX25Address::from_string("WIDE1-1"), AX25Address::from_string("WIDE2-1") };
}

// Main Task for GPS Data Processing
void gps_task(void *pvParameters) {
    ESP_LOGI(TAG, "Entering GPS task");
    uint8_t data[BUF_SIZE];
    
    while (true) {
        int len = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        
        if (len > 0) {
            printf("\n=== GPS Data (Length: %d) ===\n", len);   
            for (int i = 0; i < len; i++) {
                gps.encode(data[i]); //Feed NMEA data to tinyGPS++
            }
        } else {
            printf("No GPS data received in last second\n");
        }

        if (gps.location.isUpdated()) {
            char gps_buffer[64] = {};
            snprintf(gps_buffer, sizeof(gps_buffer),
                    "Lat=%.6f, Lon=%.6f, Time=%02d:%02d:%02d, Sats=%ld",
                    gps.location.lat(), gps.location.lng(),
                    gps.time.hour(), gps.time.minute(), gps.time.second(),
                    gps.satellites.value());
            ESP_LOGI(TAG, "%s", gps_buffer);

            // Build compact APRS text (example; keep short!)
            char aprs_text[48] = {};
            snprintf(aprs_text, sizeof(aprs_text),
                    "=%.5fN/%.5fW Team%d",
                    fabs(gps.location.lat()), fabs(gps.location.lng()), IREC_TEAM_NUM);

            packet.payload = std::string(aprs_text);   // <-- assign string (FIX)
            std::vector<uint8_t> APRSencoded = packet.encode();

            /*
             * Without FIFO stitching or interrupt based packet management (ISR/DMA) there
             * is a limit to RFM69 radio frames containing data payloads less than 64 bytes.
             * Reduce comment field lengths or split transmission into multiple packets.
             */
            if (APRSencoded.size() < 64) {
                int st = radio.transmit(APRSencoded.data(), APRSencoded.size());  // enable once ready
                ESP_LOGI(TAG, "TX %s (len=%u)", st == RADIOLIB_ERR_NONE ? "ok" : "fail", (unsigned)APRSencoded.size());
            } else {
                ESP_LOGW(TAG, "APRS frame %uB exceeds RFM69 FIFO", (unsigned)APRSencoded.size());
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second delay
    }
}

// Radio task for testing transcievers
void radio_test(void *pvParameters) {

    while(1) {
        uint8_t testBuff[8] = {0x07, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
        ESP_LOGI(TAG, "[RFM69] Transmitting packet ... ");
        //state = radio.transmit("Hello World!");
        //transmit(const uint8_t* data, size_t len, uint8_t addr)
        int state = radio.transmit(testBuff, 8);
        if (state == RADIOLIB_ERR_NONE) {
            // the packet was successfully transmitted
            ESP_LOGI(TAG, "success!");
        } else {
            ESP_LOGI(TAG, "failed, code %d\n", state);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }  
}

static void chipIdEcho() {
    printf("\n=== Starting Chip Identification ===\n");
    fflush(stdout);
    
    // Add small delay to ensure output is flushed
    vTaskDelay(pdMS_TO_TICKS(100));

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
    fflush(stdout);

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    fflush(stdout);
    
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed\n");
        fflush(stdout);
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    fflush(stdout);

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());
    printf("=== Chip Identification Complete ===\n\n");
    fflush(stdout);
}

void payload_rx_task(void *pvParameters) {
  espnow_rx_frame_t f;
  while (1) {
    if (xQueueReceive(espnow_q, &f, pdMS_TO_TICKS(1000))) {
      // TODO: parse your telemetry payload in f.data[0..f.len-1]
      ESP_LOGI("telemetry", "from %02X:%02X:%02X:%02X:%02X:%02X len=%d",
               f.from[0],f.from[1],f.from[2],f.from[3],f.from[4],f.from[5], f.len);
      // Example: forward to APRS, log, or store
    }
  }
}

extern "C" void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    //esp_log_level_set("*", ESP_LOG_VERBOSE);
    esp_log_level_set("*", ESP_LOG_INFO);
    
    printf("\n\n=== ESP32 Flight Tracker Starting ===\n");
    fflush(stdout);
    
    //chipIdEcho();
    aprs_init();
    gps_uart_init();
    //radio_hal_Init(); // RFM69 connection
    if (!radio_init()) {
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    //xTaskCreate(radio_test, "radio_test", 2048, NULL, 5, NULL);
    xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);
    ESP_ERROR_CHECK(espnow_rx_start(&espnow_q)); // start receiver
    xTaskCreate(payload_rx_task, "payload_rx", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "App main completed, tasks started");
}