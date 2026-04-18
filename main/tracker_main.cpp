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
#include "driver/twai.h"

#include <RadioLib.h>
#include "TinyGPS++.h"
#include "radiolib_esp32s3_hal.hpp" // include the hardware abstraction layer
#include "aprs.h"
#include "global_config.h"
#include "espnow_rx.h"
#include "led_status.h"
#include "canaerospace.h"

// TODO: Add MAX17048 Batt Mon. I2C driver

static const char *TAG = "ESP32-GPS-RFM69"; // Logging TAG
static QueueHandle_t espnow_q = NULL;
static SemaphoreHandle_t s_radio_mutex = NULL;

// Create a new instance of the HAL class
EspHal* hal = new EspHal(RFM69_SCK, RFM69_MISO, RFM69_MOSI);

// Radio module instance
RF69 radio = new Module(hal, RFM69_CS, RFM69_IRQ, RFM69_RST);

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
    radio.variablePacketLengthMode(RADIOLIB_RF69_MAX_PACKET_LENGTH);     
    //radio.variablePacketLengthMode(32);      
    radio.setBitRate(BIT_RATE);            
    radio.setFrequencyDeviation(DEVIATION_FREQ);    
    radio.setRxBandwidth(RX_BANDWITH);      
    radio.setOOK(false);                 
    radio.setOutputPower(OUTPUT_PWR);
    radio.setSyncWord(sw, sizeof(sw));
    radio.disableAES();
    radio.disableAddressFiltering();
    radio.setCrcFiltering(false);
    radio.setPreambleLength(PREAMBLE_LENGTH);
    radio.setDataShaping(RADIOLIB_SHAPING_NONE);
    radio.setEncoding(RADIOLIB_ENCODING_NRZ);

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

// Initialize CAN Bus 
static void can_bus_init() {
    ESP_LOGI(TAG, "[CAN] Entering CAN init");

    // Use NO_ACK for single-node bench tests (no second device to ACK).
    // When there are two nodes, switch to TWAI_MODE_NORMAL.
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)TWAI_TX_GPIO,
        (gpio_num_t)TWAI_RX_GPIO,
        TWAI_MODE_NORMAL  // change to TWAI_MODE_NORMAL when second node present
    );

    // Keep queues small and enable a few useful alerts
    g.tx_queue_len = 10;
    g.rx_queue_len = 10;
    g.alerts_enabled = TWAI_ALERT_RX_DATA | TWAI_ALERT_TX_SUCCESS |
                       TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED |
                       TWAI_ALERT_ERR_PASS | TWAI_ALERT_TX_FAILED;

    // 500 kbit/s is a good default; no need to hand-tune timing yet
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();

    // Accept everything to start; we can filter later
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install + start. ESP_ERROR_CHECK will log and abort on failure
    ESP_ERROR_CHECK(twai_driver_install(&g, &t, &f));
    ESP_ERROR_CHECK(twai_start());

    ESP_LOGI(TAG, "[CAN] Started @500k on TX=%d RX=%d (mode=%s)",
             TWAI_TX_GPIO, TWAI_RX_GPIO,
             (g.mode == TWAI_MODE_NO_ACK) ? "NO_ACK" : "NORMAL");
}

// CAN RX Task, listens for incoming CAN frames and logs their contents.
// Telemetry data from Flight Computer
static void can_rx_task(void *arg) {
    twai_message_t msg;

    while (1) {

        // Attempt to receive a CAN frame (wait up to 1 second)
        esp_err_t err = twai_receive(&msg, pdMS_TO_TICKS(1000));

        if (err == ESP_OK) {
            led_signal(LED_EVT_CAN_RX);

            ESP_LOGI("CAN-RX", "ID=0x%03X DLC=%d",
                     (unsigned)msg.identifier,
                     msg.data_length_code);

            if (!(msg.flags & TWAI_MSG_FLAG_RTR)) {
                char buf[3 * 8 + 1] = {0};
                for (int i = 0; i < msg.data_length_code && i < 8; i++) {
                    sprintf(buf + 3 * i, "%02X ", msg.data[i]);
                }
                ESP_LOGI("CAN-RX", "Data: %s", buf);
            }

            // Parse as CANaerospace and forward over RF
            canas_msg_t canas;
            if (canas_parse(&msg, &canas)) {
                char aprs_text[32];
                canas_format_aprs(&canas, aprs_text, sizeof(aprs_text));

                // Use a task-local packet to avoid racing with gps_task on the global `packet`
                APRSPacket can_pkt;
                can_pkt.source      = packet.source;
                can_pkt.destination = packet.destination;
                can_pkt.path        = packet.path;
                can_pkt.payload     = std::string(aprs_text);

                std::vector<uint8_t> encoded = can_pkt.encode();
                if (encoded.size() < 64) {
                    if (xSemaphoreTake(s_radio_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
                        int st = radio.transmit(encoded.data(), encoded.size());
                        led_signal(LED_EVT_RF_TX);
                        xSemaphoreGive(s_radio_mutex);
                        ESP_LOGI("CAN-RX", "RF TX %s payload=%s",
                                 st == RADIOLIB_ERR_NONE ? "ok" : "fail", aprs_text);
                    }
                }
            }
        }
        // Any other error (other than timeout) is logged to help diagnose issues.
        else if (err != ESP_ERR_TIMEOUT) {
            ESP_LOGW("CAN-RX", "Receive error: %s",
                     esp_err_to_name(err));
        }
    }
}

// Main Task for GPS Data Processing
void gps_task(void *pvParameters) {
    ESP_LOGI(TAG, "Entering GPS task");
    uint8_t data[BUF_SIZE];
    
    while (true) {
        int len = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        
        if (len > 0) {
            //printf("\n=== GPS Data (Length: %d) ===\n", len);   
            for (int i = 0; i < len; i++) {
                gps.encode(data[i]); //Feed NMEA data to tinyGPS++
            }
        } else {
            printf("No GPS data received in last second\n");
        }

        if (gps.location.isUpdated()) {
        //if (1) {
            led_signal(LED_EVT_GPS_TX);

            char gps_buffer[64] = {};
            snprintf(gps_buffer, sizeof(gps_buffer),
                    "Lat=%.6f, Lon=%.6f, Time=%02d:%02d:%02d, Sats=%ld",
                    gps.location.lat(), gps.location.lng(),
                    gps.time.hour(), gps.time.minute(), gps.time.second(),
                    gps.satellites.value());
            ESP_LOGI(TAG, "%s", gps_buffer);

            // Build compact APRS text
            char aprs_text[48] = {};
            snprintf(aprs_text, sizeof(aprs_text),
                    "=%.5fN/%.5fW Team%d",
                    fabs(gps.location.lat()), fabs(gps.location.lng()), IREC_TEAM_NUM);
            // snprintf(aprs_text, sizeof(aprs_text),
            //         "=%.5fN/%.5fW Team%d",
            //         -96.752381, 32.993008, IREC_TEAM_NUM);

            packet.payload = std::string(aprs_text);   // <-- assign string (FIX)
            std::vector<uint8_t> APRSencoded = packet.encode();

            /*
             * Without FIFO stitching or interrupt based packet management (ISR/DMA) there
             * is a limit to RFM69 radio frames containing data payloads less than 64 bytes.
             * Reduce comment field lengths or split transmission into multiple packets.
             */
            if (APRSencoded.size() < 64) {
                if (xSemaphoreTake(s_radio_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
                    int st = radio.transmit(APRSencoded.data(), APRSencoded.size());
                    led_signal(LED_EVT_RF_TX);
                    xSemaphoreGive(s_radio_mutex);
                    ESP_LOGI(TAG, "TX %s (len=%u)", st == RADIOLIB_ERR_NONE ? "ok" : "fail", (unsigned)APRSencoded.size());
                }
            } else {
                ESP_LOGW(TAG, "APRS frame %uB exceeds RFM69 FIFO", (unsigned)APRSencoded.size());
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second delay
    }
}

// CAN TX task for testing CAN bus transceiver
// Sends a counter frame every second on ID 0x100
// static void can_tx_task(void *arg) {
//     uint32_t counter = 0;

//     while (1) {
//         twai_message_t tx_msg = {};
//         tx_msg.identifier = 0x100;
//         tx_msg.data_length_code = 4;
//         tx_msg.data[0] = (counter >> 24) & 0xFF;
//         tx_msg.data[1] = (counter >> 16) & 0xFF;
//         tx_msg.data[2] = (counter >>  8) & 0xFF;
//         tx_msg.data[3] = (counter      ) & 0xFF;

//         esp_err_t err = twai_transmit(&tx_msg, pdMS_TO_TICKS(1000));
//         if (err == ESP_OK) {
//             ESP_LOGI("CAN-TX", "Sent ID=0x%03X counter=%lu",
//                      (unsigned)tx_msg.identifier, (unsigned long)counter);
//         } else {
//             ESP_LOGW("CAN-TX", "TX failed: %s", esp_err_to_name(err));
//         }

//         counter++;
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }

// Radio task for testing transcievers
void radio_test(void *pvParameters) {

    while(1) {
        uint8_t testBuff[8] = {0x07, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
        ESP_LOGI(TAG, "[RFM69] Transmitting packet ... ");
        //int state = radio.transmit("Hello World!!!");
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

void radio_rx_test(void *pvParameters) {
    uint8_t buf[64];
    size_t len = 0;

    ESP_LOGI(TAG, "[RX] Listening for packets...");

    while (1) {
        len = sizeof(buf);
        int state = radio.receive(buf, len, 0);

        if (state == RADIOLIB_ERR_NONE) {
            len = radio.getPacketLength();
            ESP_LOGI("RX-TEST", "Received %u bytes, RSSI=%.1f dBm",
                     (unsigned)len, radio.getRSSI());

            // Print hex dump
            char hex[3 * 64 + 1] = {0};
            for (size_t i = 0; i < len && i < 64; i++) {
                sprintf(hex + 3 * i, "%02X ", buf[i]);
            }
            ESP_LOGI("RX-TEST", "Hex : %s", hex);

            // Print as ASCII (non-printable → '.')
            char ascii[65] = {0};
            for (size_t i = 0; i < len && i < 64; i++) {
                ascii[i] = (buf[i] >= 0x20 && buf[i] <= 0x7E) ? (char)buf[i] : '.';
            }
            ESP_LOGI("RX-TEST", "Text: %s", ascii);

        } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
            // No packet within the timeout — just loop again
        } else {
            ESP_LOGW("RX-TEST", "Receive failed, code %d", state);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}   

void payload_rx_task(void *pvParameters) {
    espnow_rx_frame_t f;
    while (1) {
        if (xQueueReceive(espnow_q, &f, pdMS_TO_TICKS(1000))) {
            led_signal(LED_EVT_WIFI_RX);

            ESP_LOGI("Wifi-RX", "from %02X:%02X:%02X:%02X:%02X:%02X len=%d",
                    f.from[0],f.from[1],f.from[2],f.from[3],f.from[4],f.from[5], f.len);

            // Hex-encode raw CANAS payload with "W:" prefix (pass-through relay)
            char aprs_text[34] = {0};
            int pos = snprintf(aprs_text, sizeof(aprs_text), "W:");
            for (int i = 0; i < f.len && pos + 2 < (int)sizeof(aprs_text); i++) {
                pos += snprintf(aprs_text + pos, sizeof(aprs_text) - pos, "%02X", f.data[i]);
            }

            APRSPacket wifi_pkt;
            wifi_pkt.source      = packet.source;
            wifi_pkt.destination = packet.destination;
            wifi_pkt.path        = packet.path;
            wifi_pkt.payload     = std::string(aprs_text);

            std::vector<uint8_t> encoded = wifi_pkt.encode();
            if (encoded.size() < 64) {
                if (xSemaphoreTake(s_radio_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
                    int st = radio.transmit(encoded.data(), encoded.size());
                    led_signal(LED_EVT_RF_TX);
                    xSemaphoreGive(s_radio_mutex);
                    ESP_LOGI("Wifi-RX", "RF TX %s payload=%s",
                             st == RADIOLIB_ERR_NONE ? "ok" : "fail", aprs_text);
                }
            }
        }
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

extern "C" void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_log_level_set("*", ESP_LOG_INFO);

    printf("\n\n=== ESP32 Flight Tracker Starting ===\n");
    fflush(stdout);

    s_radio_mutex = xSemaphoreCreateMutex();

    // Bring up peripherals
    chipIdEcho();
    gps_uart_init();   // UART1 for GPS
    aprs_init();       // AX.25/APRS addresses, path, etc.
    can_bus_init(); // Intialize CAN / TWAI
    led_status_init(); // Initialize LED Array Driver

    // Intialize Radio (RFM69)
    if (!radio_init()) {
        led_signal(LED_EVT_ERROR);
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    //xTaskCreate(can_tx_task, "can_tx_task", 2048, NULL, 5, NULL);
    xTaskCreate(can_rx_task, "can_rx_task", 4096, NULL, 5, NULL);

    //xTaskCreate(radio_test, "radio_test", 2048, NULL, 5, NULL);
    xTaskCreate(gps_task,   "gps_task",   4096, NULL, 5, NULL);

    // Initalize and Start WiFi receiver task
    ESP_ERROR_CHECK(espnow_rx_start(&espnow_q));   // start ESP-NOW RX queue
    xTaskCreate(payload_rx_task, "payload_rx", 4096, NULL, 5, NULL);
    
    xTaskCreate(led_task, "led_task", 2048, NULL, 5, NULL); // Start LED task

    ESP_LOGI(TAG, "App main completed, tasks started");
}
