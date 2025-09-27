#pragma once
#include <string>
#include <vector>

struct AX25Address {
    std::string callsign;
    uint8_t ssid;

    static AX25Address from_string(const std::string& str);
    std::string to_string() const;
};

class APRSPacket {
public:
    AX25Address destination;
    AX25Address source;
    std::vector<AX25Address> path;  // digipeaters
    std::string payload;

    // Encode to raw AX.25 UI frame (no HDLC framing here)
    std::vector<uint8_t> encode() const;

    // Decode from raw AX.25 UI frame (no HDLC framing here)
    static APRSPacket decode(const std::vector<uint8_t>& data);

private:
    static void encode_address(const AX25Address& addr, uint8_t* out, bool last);
    static AX25Address decode_address(const uint8_t* data);
};
