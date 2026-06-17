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

#include <RadioLib.h>
#include "TinyGPS++.h"
#include "radiolib_esp32s3_hal.hpp" // include the hardware abstraction layer
#include "aprs.h"
#include "global_config.h"
#include "espnow_rx.h"
#include "led_status.h"
#include "canaerospace.h"
#include "max17048.h"

static const char *TAG = "ESP32-GPS-RFM69"; // Logging TAG
static QueueHandle_t espnow_q = NULL;
static SemaphoreHandle_t s_radio_mutex = NULL;
static i2c_bus_handle_t i2c_bus = NULL;
static max17048_handle_t max17048 = NULL;
static volatile float g_batt_voltage = 0.0f;
static uint8_t  s_tlm_buf[24]  = {0};
static uint32_t s_tlm_received = 0;  // bitmask, bit N = chunk N received

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
    radio.setBitRate(BIT_RATE);            
    radio.setFrequencyDeviation(DEVIATION_FREQ);    
    radio.setRxBandwidth(RX_BANDWITH);      
    radio.setOOK(false);                 
    radio.setOutputPower(OUTPUT_PWR, true);
    radio.setSyncWord(sw, sizeof(sw));
    radio.disableAES();
    radio.disableAddressFiltering();
    radio.setCrcFiltering(true);
    radio.setPreambleLength(PREAMBLE_LENGTH);
    radio.setDataShaping(RADIOLIB_SHAPING_0_5); // Guassian shaping reduces spectral leakage into adjacent channels
    radio.setEncoding(RADIOLIB_ENCODING_NRZ); // No encoding: 1-> high, 0-> low

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

    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)TWAI_TX_GPIO,
        (gpio_num_t)TWAI_RX_GPIO,
        TWAI_MODE_NORMAL  // when second node present
    );

    // Keep queues small and enable a few useful alerts
    g.tx_queue_len = 10;
    g.rx_queue_len = 10;
    g.alerts_enabled = TWAI_ALERT_RX_DATA | TWAI_ALERT_TX_SUCCESS |
                       TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED |
                       TWAI_ALERT_ERR_PASS | TWAI_ALERT_TX_FAILED;

    // 250 kbit/s to match flight computer speed
    twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS();

    // Accept everything to start; we can filter later
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install + start. ESP_ERROR_CHECK will log and abort on failure
    ESP_ERROR_CHECK(twai_driver_install(&g, &t, &f));
    ESP_ERROR_CHECK(twai_start());

    ESP_LOGI(TAG, "[CAN] Started @250k on TX=%d RX=%d (mode=%s)",
             TWAI_TX_GPIO, TWAI_RX_GPIO,
             (g.mode == TWAI_MODE_NO_ACK) ? "NO_ACK" : "NORMAL");
}

static void max17048_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = { .clk_speed = 400 * 1000 },
        .clk_flags = 0,
    };
    i2c_bus = i2c_bus_create(I2C_NUM_0, &conf);
    max17048 = max17048_create(i2c_bus, MAX17048_I2C_ADDR_DEFAULT);
    if (0) {
        if (max17048_reset(max17048) == ESP_OK) {
            ESP_LOGI(TAG, "[BATT] max17048 reset success");
        }
        else {
            ESP_LOGI(TAG, "[BATT] max17048 fail to reset");
        }
    }


    ESP_LOGI(TAG, "[BATT] max17048 fuel guage intialized");
}


// Single cell fuel guage task to monitor voltage of lithium-ion battery powering the board
static void batt_monitor_task(void *pvParameters) {

    while (1) {
        // Get voltage and battery percentage
        float voltage = 0, percent = 0;
        max17048_get_cell_voltage(max17048, &voltage);
        max17048_get_cell_percent(max17048, &percent);

        g_batt_voltage = voltage;
        ESP_LOGI(TAG, "[BATT] Voltage:%.2fV, percent:%.2f%%", voltage, percent);
        
        vTaskDelay(pdMS_TO_TICKS(5000)); // 5 second delay
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

            // Build compact APRS text: GPS long, lat, team #, batt voltage
            char aprs_text[48] = {};
            int aprs_len = snprintf(aprs_text, sizeof(aprs_text),
                    "=%.5fN/%.5fWTeam%dV%.2f",
                    fabs(gps.location.lat()), fabs(gps.location.lng()), IREC_TEAM_NUM, g_batt_voltage);
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
                    xSemaphoreGive(s_radio_mutex);
                    if (st == RADIOLIB_ERR_NONE) {
                        // the packet was successfully transmitted
                        ESP_LOGI(TAG, "TX ok (len=%u)", (unsigned)APRSencoded.size());
                        led_signal(LED_EVT_RF_TX);
                    } else {
                        ESP_LOGI(TAG, "TX fail (len=%u) code=%d", (unsigned)APRSencoded.size(), st);
                    }
                }
            } else {
                ESP_LOGW(TAG, "APRS frame %uB exceeds RFM69 FIFO", (unsigned)APRSencoded.size());
            }
        }

        vTaskDelay(pdMS_TO_TICKS(GPS_APRS_PKT_DELAY)); 
    }
}

// CAN TX task for testing CAN bus transceiver
// Sends a counter frame every second on ID 0x100
static void can_tx_task(void *arg) {
    uint32_t counter = 0;

    while (1) {
        twai_message_t tx_msg = {};
        tx_msg.identifier = 0x100;
        tx_msg.data_length_code = 4;
        tx_msg.data[0] = (counter >> 24) & 0xFF;
        tx_msg.data[1] = (counter >> 16) & 0xFF;
        tx_msg.data[2] = (counter >>  8) & 0xFF;
        tx_msg.data[3] = (counter      ) & 0xFF;

        esp_err_t err = twai_transmit(&tx_msg, pdMS_TO_TICKS(1000));
        if (err == ESP_OK) {
            ESP_LOGI("CAN-TX", "Sent ID=0x%03X counter=%lu", (unsigned)tx_msg.identifier, (unsigned long)counter);
            led_signal(LED_EVT_CAN_TX);    
        } else {
            ESP_LOGW("CAN-TX", "TX failed: %s", esp_err_to_name(err));
        }

        counter++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// CAN RX Task, listens for incoming CAN frames and logs their contents.
// Telemetry data from Flight Computer
// Added Reassembly buffer for the 6-chunk telemetry packet
static void can_rx_task(void *pvParameters) {
    twai_message_t msg;

    while (1) {
        esp_err_t err = twai_receive(&msg, pdMS_TO_TICKS(1000));

        if (err != ESP_OK) {
            if (err != ESP_ERR_TIMEOUT) {
                ESP_LOGW("CAN-RX", "Receive error: %s", esp_err_to_name(err));
            }
            continue;
        }

        led_signal(LED_EVT_CAN_RX);

        if (msg.flags & TWAI_MSG_FLAG_RTR) continue;

        uint32_t id = msg.identifier;

        // Check if this is a telemetry chunk (IDs 1400-1405)
        if (id >= TLM_BASE_ID && id < TLM_BASE_ID + TLM_CHUNKS) {
            uint8_t chunk = id - TLM_BASE_ID;
            uint8_t offset = chunk * 4;

            // Copy the 4 data bytes (skip the 4-byte CANaerospace header)
            // CANaerospace layout: [node_id][dtc][svc_code][msg_code][data0..3]
            // data bytes start at index 4
            if (msg.data_length_code >= 8) {
                memcpy(&s_tlm_buf[offset], &msg.data[4], 4);
                s_tlm_received |= (1 << chunk);
            }

            // Check if all 6 chunks received
            if (s_tlm_received == TLM_ALL_CHUNKS) {
                s_tlm_received = 0;  // reset for next packet

                // Build RF packet: [0x01 type][24 bytes telemetry]
                uint8_t rf_buf[25];
                rf_buf[0] = 0x01;  // type = FC telemetry
                memcpy(&rf_buf[1], s_tlm_buf, 24);

                if (xSemaphoreTake(s_radio_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
                    int st = radio.transmit(rf_buf, sizeof(rf_buf));
                    led_signal(LED_EVT_RF_TX);
                    xSemaphoreGive(s_radio_mutex);

                    // Log decoded fields for debugging
                    can_tlm_packet_t *pkt = (can_tlm_packet_t *)s_tlm_buf;
                    ESP_LOGI("CAN-RX", "RF TX %s | Alt=%d ft Vel=%.1f fps Acc=%.2fg FSM=%d",
                             st == RADIOLIB_ERR_NONE ? "ok" : "fail",
                             pkt->altitude_ft,
                             pkt->vert_vel_fps_x10 / 10.0f,
                             pkt->accel_z_mg / 1000.0f,
                             pkt->fsm_state);
                }
            }
        }
        // Handle event frames (ID 1310) — still send individually
        else if (id == 1310 && msg.data_length_code >= 6) {
            uint8_t rf_buf[4];
            rf_buf[0] = 0x03;          // type = FC event
            rf_buf[1] = msg.data[0];   // node_id
            rf_buf[2] = msg.data[4];   // event_type
            rf_buf[3] = msg.data[5];   // event_data

            if (xSemaphoreTake(s_radio_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
                int st = radio.transmit(rf_buf, sizeof(rf_buf));
                led_signal(LED_EVT_RF_TX);
                xSemaphoreGive(s_radio_mutex);
                ESP_LOGI("CAN-RX", "Event RF TX %s type=0x%02X data=0x%02X",
                         st == RADIOLIB_ERR_NONE ? "ok" : "fail",
                         rf_buf[2], rf_buf[3]);
            }
        }
        // Log any other CAN frames for debugging
        else {
            ESP_LOGD("CAN-RX", "ID=0x%03X DLC=%d (unhandled)",
                     (unsigned)id, msg.data_length_code);
        }
    }
}

// Data is sent from WiFi
void payload_rx_task(void *pvParameters) {
    espnow_rx_frame_t f;
    while (1) {
        if (xQueueReceive(espnow_q, &f, pdMS_TO_TICKS(1000))) {
            led_signal(LED_EVT_WIFI_RX);

            ESP_LOGI("WIFI-RX", "from %02X:%02X:%02X:%02X:%02X:%02X len=%d",
                    f.from[0],f.from[1],f.from[2],f.from[3],f.from[4],f.from[5], f.len);
            ESP_LOGI("WIFI-RX", "Data: %s", f.data);

            // CANaerospace transmission
            uint8_t rf_buf[34];
            uint8_t pos = 0;
            rf_buf[pos++] = 0x02;  // type = WiFi payload
            memcpy(&rf_buf[pos], f.data, f.len);
            pos += f.len;

            if (xSemaphoreTake(s_radio_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
                int st = radio.transmit(rf_buf, pos);
                led_signal(LED_EVT_RF_TX);
                xSemaphoreGive(s_radio_mutex);
                ESP_LOGI("Wifi-RX", "RF TX %s len=%d",
                        st == RADIOLIB_ERR_NONE ? "ok" : "fail", pos);
            }
        }
    }
}

// Radio task for testing transcievers
void radio_test(void *pvParameters) {

    while(1) {
        uint8_t testBuff[8] = {0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02};
        ESP_LOGI(TAG, "[RFM69] Transmitting packet ... ");
        //int state = radio.transmit("Hello World!!!");
        //transmit(const uint8_t* data, size_t len, uint8_t addr)
        int state = radio.transmit(testBuff, 8);
        if (state == RADIOLIB_ERR_NONE) {
            // the packet was successfully transmitted
            ESP_LOGI(TAG, "success!");
            led_signal(LED_EVT_RF_TX);
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
    max17048_init(); // Initialize the MAX17048 sensor

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

    xTaskCreate(batt_monitor_task, "fuel_gauge", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "App main completed, tasks started");
}
