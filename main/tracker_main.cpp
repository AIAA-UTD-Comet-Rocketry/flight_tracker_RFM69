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
#include "math.h"

// GPS UART Configuration
#define GPS_UART_NUM   UART_NUM_1  // Changed from UART_NUM_0 to avoid conflict with console
#define GPS_TX_PIN     17  // TX not needed (GPS is sending data)
#define GPS_RX_PIN     18  // ESP32 RX (Connect to GPS TX)
#define BUF_SIZE       1024

// RFM69 SPI Configuration
#define RFM69_SCK     12    // SPI2 SCK
#define RFM69_MISO    13    // SPI2 MISO
#define RFM69_MOSI    11    // SPI2 MOSI
#define RFM69_CS      10    // NSS/CS (manual GPIO)
#define RFM69_IRQ     27    // DIO0 -> regular input GPIO (NOT a strapping pin)
#define RFM69_RST     15    // Reset line (regular output GPIO)
// #define RFM69_GPIO  15    // (remove/unused) conflicts with RST if kept


// TODO: CANbus UART Configuration
// TODO: Wifi setup

// Logging TAG
static const char *TAG = "ESP32-GPS-RFM69";

// Create a new instance of the HAL class
EspHal* hal = new EspHal(RFM69_SCK, RFM69_MISO, RFM69_MOSI);

// Radio module instance
RF69 radio = RF69(new Module(hal, RFM69_CS, RFM69_IRQ, RFM69_RST));

// TinyGPS++ instance
TinyGPSPlus gps;

// APRS codec instance
APRSPacket packet;

// void radio_hal_Init() {
//     radio.setFrequency(433.4);
//     radio.setBitRate(1.55);
//     uint8_t sw[] = {0xAA, 0xD5};
//     radio.setSyncWord(sw, sizeof(sw));

//     // initialize
//     ESP_LOGI(TAG, "[SX1276] Initializing ... ");
//     int state = radio.begin();
//     if (state != RADIOLIB_ERR_NONE) {
//         ESP_LOGI(TAG, "failed, code %d\n", state);
//         while(true) {
//             vTaskDelay(pdMS_TO_TICKS(1000));
//         }
//     }
//     ESP_LOGI(TAG, "success!\n");
// }

static bool radio_init() {
    ESP_LOGI(TAG, "[RFM69] Initializing...");
    int st = radio.begin();
    if (st != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "radio.begin() failed: %d", st);
        return false;
    }

    // Start with conservative/forgiving link params; align to your ground RX later
    radio.setFrequency(433.920);          // MHz
    radio.setBitRate(4800.0);             // bps
    radio.setFrequencyDeviation(50.0);    // kHz
    radio.setRxBandwidth(125.0);          // kHz
    radio.setOOK(false);                  // ensure FSK
    radio.setOutputPower(13);             // dBm (mind regs/antenna)

    uint8_t sw[] = {0x2D, 0xD4};          // sync word — must match receiver
    radio.setSyncWord(sw, sizeof(sw));

    ESP_LOGI(TAG, "RFM69 ready");
    return true;
}

// Function to Initialize GPS UART
void gps_uart_init() {
    ESP_LOGI(TAG, "Entered GPS function");
    uart_config_t uart_config = {};
    uart_config.baud_rate = 9600;  // NEO-6M default
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

    uart_driver_install(GPS_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(GPS_UART_NUM, &uart_config);
    uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_LOGI(TAG, "GPS UART initialized at %d baud on UART%d", uart_config.baud_rate, GPS_UART_NUM);
}


void aprs_init() {
    packet.source = AX25Address::from_string("KK7SSP-11");
    packet.destination = AX25Address::from_string("APRS");
    packet.path = { AX25Address::from_string("WIDE1-1"), AX25Address::from_string("WIDE2-1") };
}

// // Function to Initialize SPI for RFM69
// #include "driver/gpio.h"
void spi_radio_bus_init() {
  spi_bus_config_t b = {};
  b.mosi_io_num = RFM69_MOSI; 
  b.miso_io_num = RFM69_MISO; 
  b.sclk_io_num = RFM69_SCK;
  b.quadwp_io_num = -1; 
  b.quadhd_io_num = -1;
  b.max_transfer_sz = 4096;
    
  esp_err_t e = spi_bus_initialize(SPI2_HOST, &b, SPI_DMA_CH_AUTO);
  if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(e);
  gpio_config_t cs = { .pin_bit_mask = 1ULL<<RFM69_CS, .mode=GPIO_MODE_OUTPUT };
  ESP_ERROR_CHECK(gpio_config(&cs)); gpio_set_level(RFM69_CS, 1);
  gpio_config_t irq = { .pin_bit_mask = 1ULL<<RFM69_IRQ, .mode=GPIO_MODE_INPUT };
  ESP_ERROR_CHECK(gpio_config(&irq));
  gpio_config_t rst = { .pin_bit_mask = 1ULL<<RFM69_RST, .mode=GPIO_MODE_OUTPUT };
  ESP_ERROR_CHECK(gpio_config(&rst));

  gpio_set_level(RFM69_RST, 0); vTaskDelay(pdMS_TO_TICKS(1));
  gpio_set_level(RFM69_RST, 1); delay_us(200);
  gpio_set_level(RFM69_RST, 0); vTaskDelay(pdMS_TO_TICKS(5));
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
            // printf("GPS Parser Stats: Chars=%lu, Sentences=%lu, Failed=%lu\n", 
            //        gps.charsProcessed(), gps.sentencesWithFix(), gps.failedChecksum());
        } else {
            printf("No GPS data received in last second\n");
        }

        // Check if we have a valid location
        // if (gps.location.isUpdated()) {
        //     char gps_buffer[64] = {};

        //     snprintf(gps_buffer, sizeof(gps_buffer), 
        //              "Lat=%.6f, Lon=%.6f, Time=%d:%d:%d, Sats=%ld", 
        //              gps.location.lat(), gps.location.lng(), 
        //              gps.time.hour(), gps.time.minute(), gps.time.second(), gps.satellites.value());
        //     ESP_LOGI(TAG, "%s", gps_buffer);

        //     // Radio string
        //     packet.payload = snprintf(gps_buffer, sizeof(gps_buffer),
        //                               "=%.2fN/%.2fW-Team 317", 
        //                               gps.location.lat(), gps.location.lng());

        //     std::vector<uint8_t> APRSencoded = packet.encode();

        //     //radio.transmit((uint8_t*)APRSencoded.data(), APRSencoded.size());

        //     /*
        //      * Without FIFO stitching or interrupt based packet management (ISR/DMA) there
        //      * is a limit to RFM69 radio frames containing data payloads less than 64 bytes.
        //      * Reduce comment field lengths or split transmission into multiple packets.
        //      */
        //     if (APRSencoded.size() >= 64)
        //         ESP_LOGI(TAG, "Radio FIFO error, check main file for comments");
        // }
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
                    "=%.5fN/%.5fW Team317",
                    fabs(gps.location.lat()), fabs(gps.location.lng()));

            packet.payload = std::string(aprs_text);   // <-- assign string (FIX)
            std::vector<uint8_t> frame = packet.encode();

            if (frame.size() < 64) {
                // int st = radio.transmit(frame.data(), frame.size());  // enable once ready
                // ESP_LOGI(TAG, "TX %s (len=%u)", st == RADIOLIB_ERR_NONE ? "ok" : "fail", (unsigned)frame.size());
            } else {
                ESP_LOGW(TAG, "APRS frame %uB exceeds RFM69 FIFO", (unsigned)frame.size());
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

void chipIdEcho() {
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

extern "C" void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Set log level to verbose for debugging (changed to info for less jitter)
    //esp_log_level_set("*", ESP_LOG_VERBOSE);
    esp_log_level_set("*", ESP_LOG_INFO);
    
    printf("\n\n=== ESP32 Flight Tracker Starting ===\n");
    fflush(stdout);
    
    //chipIdEcho();

    gps_uart_init();

    //spi_radio_bus_init();
    //radio_hal_Init(); // RFM69 connection
    if (!radio_init()) {
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    //xTaskCreate(radio_test, "radio_test", 2048, NULL, 5, NULL);
    xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "App main completed, tasks started");
}
