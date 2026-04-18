#include "canaerospace.h"
#include <string.h>
#include <stdio.h>

bool canas_parse(const twai_message_t *frame, canas_msg_t *out) {
    if (frame->flags & TWAI_MSG_FLAG_RTR) return false;  // RTR frames carry no data
    if (frame->data_length_code < 4)      return false;  // CANaerospace minimum is 4 bytes

    out->msg_id   = (uint16_t)frame->identifier;
    out->node_id  = frame->data[0];
    out->dtc      = frame->data[1];
    out->svc_code = frame->data[2];
    out->msg_code = frame->data[3];
    out->data_len = frame->data_length_code - 4;

    memcpy(out->data, &frame->data[4], out->data_len);

    return true;
}

int canas_format_aprs(const canas_msg_t *msg, char *buf, size_t buf_len) {
    // Build hex string from raw payload bytes (up to 4 bytes = 8 hex chars)
    char hex[9] = {0};
    for (int i = 0; i < msg->data_len && i < 4; i++) {
        snprintf(hex + 2 * i, 3, "%02X", msg->data[i]);
    }

    return snprintf(buf, buf_len, "C:%X,%u,%u,%u,%s",
                    msg->msg_id, msg->node_id, msg->dtc, msg->msg_code, hex);
}
