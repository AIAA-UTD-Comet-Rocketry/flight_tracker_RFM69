#ifndef GLOBAL_CONFIG_H
#define GLOBAL_CONFIG_H

/* BOARD GPIO PINOUT */
// GPS UART Pins
#define GPS_TX_PIN      18  // ESP32 RX (Connect to GPS TX)
#define GPS_RX_PIN      17  // TX not needed (GPS is sending data)

// RFM69 SPI Pins
#define RFM69_SCK       7 
#define RFM69_MISO      9 
#define RFM69_MOSI      8 
#define RFM69_CS        6 
#define RFM69_IRQ       4   // G0 Pin in Breakout board 
#define RFM69_RST       5 

// CAN/TWAI Pins
#define TWAI_TX_GPIO    38  // ESP32 -> Transceiver TXD
#define TWAI_RX_GPIO    37  // Transceiver RXD -> ESP32

// LED Status Pins
#define LED_GPS_PIN     34
#define LED_RF_PIN      35
#define LED_CANRX_PIN   33
#define LED_CANTX_PIN   26
#define LED_WIFI_PIN    36
#define LED_STATUS_PIN  21

// BMS I2C Pins
#define I2C_SDA         33
#define I2C_SCL         34


#define CALLSIGN            "KK7SSP-11"
#define IREC_TEAM_NUM       308

// Transceiver config:
static const uint8_t sw[] = {0x12, 0xAD};        // sync word — must match receiver

#define RADIO_FREQ          434.0   // MHz (Range from 431.0 - 510.0)
#define BIT_RATE            4.8     // kbps
#define DEVIATION_FREQ      5       // kHz
#define RX_BANDWITH         125.0   // kHz
#define OUTPUT_PWR          20      // dBM
#define PREAMBLE_LENGTH     32      // bits

// GPS config:
#define GPS_UART_NUM        UART_NUM_1  // Changed from UART_NUM_0 to avoid conflict with console
#define GPS_BAUD_RATE       9600    // NEO-6M default
#define BUF_SIZE            1024

// Wifi config:
static const uint8_t ESPNOW_SENDER_MAC[6] = {
     0xB0, 0x81, 0x84, 0x9B, 0xAB, 0x70 
    }; // Payload ESP32

#define ESPNOW_CHANNEL        6             // must match sender’s channel
#define ESPNOW_IFACE          WIFI_IF_STA   // WIFI_IF_STA or WIFI_IF_AP
#define ESPNOW_RX_QUEUE_LEN   16            // Queue sizing for received frames
#define ESPNOW_RX_MAX_LEN     32            // 32 bytes typical payload

#endif // global_config.h