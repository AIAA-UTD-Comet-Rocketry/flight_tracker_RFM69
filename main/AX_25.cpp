#include <stdint.h>
#include <string.h>
#include <stdio.h>

#define FLAG 0x7E
#define CONTROL 0x03
#define PID 0xF0
#define ADDR_LEN 14
#define FCS_LEN 2
#define MAX_INFO_LEN 64

// AX.25 address encoding
void encode_ax25_addr(uint8_t *buf, const char *call, uint8_t ssid, int last, int command) {
    for (int i = 0; i < 6; i++) {
        buf[i] = (i < strlen(call)) ? call[i] << 1 : ' ' << 1;
    }
    buf[6] = 0x60 | ((ssid & 0x0F) << 1);
    if (command) buf[6] |= 0x80;
    if (last) buf[6] |= 0x01;
    else buf[6] &= 0xFE;
}

// CRC-16-CCITT (reversed poly)
uint16_t crc_ccitt(uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : (crc >> 1);
        }
    }
    return ~crc;
}

// Build AX.25 UI frame
size_t build_ax25_ui_frame(uint8_t *frame, const char *src, uint8_t src_ssid,
                           const char *dst, uint8_t dst_ssid,
                           double lat, double lon) {
    size_t i = 0;

    // Flag (start)
    frame[i++] = FLAG;

    // Address field (14 bytes: dest + source)
    uint8_t addr[ADDR_LEN];
    encode_ax25_addr(&addr[0], dst, dst_ssid, 0, 0);
    encode_ax25_addr(&addr[7], src, src_ssid, 1, 1);
    memcpy(&frame[i], addr, ADDR_LEN);
    i += ADDR_LEN;

    // Control field
    frame[i++] = CONTROL;

    // PID field
    frame[i++] = PID;

    // Information field (APRS-style GPS string)
    char info[MAX_INFO_LEN];
    snprintf(info, sizeof(info), "GPS: Lat=%.6f Lon=%.6f", lat, lon);
    size_t info_len = strlen(info);
    memcpy(&frame[i], info, info_len);
    i += info_len;

    // Frame Check Sequence (FCS) - CRC over everything between flags
    uint16_t crc = crc_ccitt(&frame[1], i - 1);  // skip the first FLAG
    frame[i++] = crc & 0xFF;
    frame[i++] = (crc >> 8) & 0xFF;

    // Flag (end)
    frame[i++] = FLAG;

    return i; // total frame length
}
