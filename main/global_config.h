#ifndef GLOBAL_CONFIG_H
#define GLOBAL_CONFIG_H

#define CALLSIGN            "KK7SSP-11"
#define IREC_TEAM_NUM       317

// Transceiver config:
#define RADIO_FREQ          433.920 // MHz (Range from 431.0 - 510.0)
#define BIT_RATE            4.8     // kbps
#define DEVIATION_FREQ      50      // kHz
#define RX_BANDWITH         125.0   // kHz
#define OUTPUT_PWR          20      // dBM
static const uint8_t sw[] = {0x2D, 0xD4};        // sync word — must match receiver

// GPS config:
#define GPS_BAUD_RATE       9600    // NEO-6M default

// Wifi config:
#define ESPNOW_CHANNEL      6       // must match sender’s channel
#define ESPNOW_IFACE        WIFI_IF_STA       // WIFI_IF_STA or WIFI_IF_AP
static const uint8_t ESPNOW_SENDER_MAC[6] = { 0xB0, 0x81, 0x84, 0x9B, 0xAB, 0x70 }; // Payload ESP32
// Queue sizing for received frames
#define ESPNOW_RX_QUEUE_LEN   16
#define ESPNOW_RX_MAX_LEN     32               // 32 bytes typical payload

#endif // global_config.h