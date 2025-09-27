#include "aprs.h"
#include <cstring>
#include <stdexcept>
#include "esp_log.h"

// Logging TAG
static const char *TAG = "ESP32-APRS-CODEC";

AX25Address AX25Address::from_string(const std::string& str) {
    auto pos = str.find('-');
    AX25Address addr;
    if (pos == std::string::npos) {
        addr.callsign = str;
        addr.ssid = 0;
    } else {
        addr.callsign = str.substr(0, pos);
        addr.ssid = std::stoi(str.substr(pos + 1));
    }
    return addr;
}

std::string AX25Address::to_string() const {
    return callsign + (ssid ? "-" + std::to_string(ssid) : "");
}

void APRSPacket::encode_address(const AX25Address& addr, uint8_t* out, bool last) {
    // AX.25 address: 6 chars callsign (padded with spaces), 1 byte SSID
    for (size_t i = 0; i < 6; ++i) {
        uint8_t c = (i < addr.callsign.size()) ? addr.callsign[i] : ' ';
        out[i] = c << 1;
    }
    out[6] = 0x60 | ((addr.ssid & 0x0F) << 1);
    if (last) out[6] |= 0x01;  // set end-of-address bit
}

AX25Address APRSPacket::decode_address(const uint8_t* data) {
    AX25Address addr;
    addr.callsign.clear();
    for (int i = 0; i < 6; ++i) {
        char c = data[i] >> 1;
        if (c != ' ') addr.callsign += c;
    }
    addr.ssid = (data[6] >> 1) & 0x0F;
    return addr;
}

std::vector<uint8_t> APRSPacket::encode() const {
    std::vector<uint8_t> frame;
    frame.resize(7 * (2 + path.size()) + 2);  // addresses + control + PID
    size_t offset = 0;

    encode_address(destination, &frame[offset], false);
    offset += 7;
    encode_address(source, &frame[offset], path.empty());
    offset += 7;

    for (size_t i = 0; i < path.size(); ++i) {
        encode_address(path[i], &frame[offset], i == path.size() - 1);
        offset += 7;
    }

    frame[offset++] = 0x03;  // UI frame
    frame[offset++] = 0xF0;  // no layer 3 protocol

    frame.insert(frame.end(), payload.begin(), payload.end());
    return frame;
}

APRSPacket APRSPacket::decode(const std::vector<uint8_t>& data) {
    APRSPacket packet;
    if (data.size() < 16) {
        ESP_LOGE(TAG, "Frame too short");
        return packet;
    }

    size_t offset = 0;

    packet.destination = decode_address(&data[offset]);
    offset += 7;
    packet.source = decode_address(&data[offset]);
    offset += 7;

    while (!(data[offset - 1] & 0x01)) {  // check end-of-address bit
        packet.path.push_back(decode_address(&data[offset]));
        offset += 7;
        if (offset + 2 > data.size())
            ESP_LOGE(TAG, "Invalid address block");
    }

    // control field
    if (data[offset++] != 0x03 || data[offset++] != 0xF0)
        ESP_LOGE(TAG, "Not a UI frame");

    packet.payload = std::string(data.begin() + offset, data.end());
    return packet;
}
