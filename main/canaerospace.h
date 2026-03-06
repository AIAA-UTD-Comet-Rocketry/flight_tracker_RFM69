#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "driver/twai.h"

/*
 * CANaerospace Data Type Codes (DTC)
 * Defined in the CANaerospace specification, section 3.
 */
typedef enum {
    CANAS_DTC_NODATA  = 0,   // No data
    CANAS_DTC_ERROR   = 1,   // Error (4-byte error code)
    CANAS_DTC_FLOAT   = 2,   // 32-bit IEEE 754 float
    CANAS_DTC_LONG    = 3,   // 32-bit signed integer
    CANAS_DTC_ULONG   = 4,   // 32-bit unsigned integer
    CANAS_DTC_BLONG   = 5,   // 32-bit bit field
    CANAS_DTC_SHORT   = 6,   // 16-bit signed integer
    CANAS_DTC_USHORT  = 7,   // 16-bit unsigned integer
    CANAS_DTC_BSHORT  = 8,   // 16-bit bit field
    CANAS_DTC_CHAR    = 9,   // 8-bit signed char
    CANAS_DTC_UCHAR   = 10,  // 8-bit unsigned char
    CANAS_DTC_BCHAR   = 11,  // 8-bit bit field
    CANAS_DTC_SHORT2  = 12,  // Two 16-bit signed integers
    CANAS_DTC_USHORT2 = 13,  // Two 16-bit unsigned integers
    CANAS_DTC_BSHORT2 = 14,  // Two 16-bit bit fields
    CANAS_DTC_CHAR4   = 15,  // Four 8-bit signed chars
    CANAS_DTC_UCHAR4  = 16,  // Four 8-bit unsigned chars
    CANAS_DTC_BCHAR4  = 17,  // Four 8-bit bit fields
    CANAS_DTC_CHAR2   = 18,  // Two 8-bit signed chars
    CANAS_DTC_UCHAR2  = 19,  // Two 8-bit unsigned chars
    CANAS_DTC_BCHAR2  = 20,  // Two 8-bit bit fields
} canas_dtc_t;

/*
 * Parsed CANaerospace message.
 *
 * CANaerospace frame layout (CAN data bytes):
 *   [0] node_id   — sender's node ID
 *   [1] dtc       — data type code (see canas_dtc_t)
 *   [2] svc_code  — service code (0 = normal data message)
 *   [3] msg_code  — rolling message counter
 *   [4..7]        — data payload (0–4 bytes, count in data_len)
 */
typedef struct {
    uint16_t msg_id;    // CAN message ID (11-bit, from frame identifier)
    uint8_t  node_id;   // Sender node ID
    uint8_t  dtc;       // Data Type Code
    uint8_t  svc_code;  // Service Code
    uint8_t  msg_code;  // Message counter
    uint8_t  data[4];   // Raw payload bytes
    uint8_t  data_len;  // Number of valid payload bytes (= DLC - 4)
} canas_msg_t;

/*
 * Parse a received TWAI frame into a CANaerospace message.
 * Returns false if the frame is an RTR frame or DLC < 4 (minimum CANaerospace size).
 */
bool canas_parse(const twai_message_t *frame, canas_msg_t *out);

/*
 * Format a parsed CANaerospace message as a compact ASCII string for use
 * as an APRS payload. Output format:
 *
 *   C:<msg_id_hex>,<node_id>,<dtc>,<msg_code>,<data_hex>
 *   Example: C:1A4,1,2,0,3F800000
 *
 * Maximum output length is 26 characters, which fits within the 34-byte
 * payload budget (64-byte RFM69 FIFO minus 30-byte AX.25 header).
 *
 * Returns the number of characters written (excluding null terminator),
 * or a negative value if buf_len is too small.
 */
int canas_format_aprs(const canas_msg_t *msg, char *buf, size_t buf_len);
