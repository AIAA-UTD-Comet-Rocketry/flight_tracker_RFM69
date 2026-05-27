| Supported Targets | ESP32-S3 |

## About
ESP32-S3 based GPS tracker and radio communication module for Comet Rocketry. Integrates an RFM69HCW transceiver at 434 MHz to transmit GPS position and CAN telemetry packets to a ground station. The board receives GPS coordinates via UART, flight computer (FC) telemetry over a CAN bus, and payload sensor data from a second ESP32 over ESP-NOW (Wi-Fi), then relays everything over RF using APRS/AX.25 and a custom binary packet format.

---

## How It Works

### Overview
The firmware runs five concurrent FreeRTOS tasks. All RF transmissions share a single mutex (`s_radio_mutex`) that serializes access to the RFM69 SPI bus — whichever task is ready to transmit acquires the mutex, sends its packet, then releases it.

### RTOS Tasks

| Task | Purpose |
|------|---------|
| `gps_task` | Reads NMEA sentences from the GPS module (UART1, 9600 baud), parses them with TinyGPS++, builds an APRS/AX.25 UI frame, and transmits via RFM69 every ~3 s when a new location fix arrives. The payload encodes latitude, longitude, team number (308), and live battery voltage. |
| `can_rx_task` | Listens on the CAN/TWAI bus for flight-computer telemetry (IDs 1400–1405, 6 chunks per frame). Once all 6 chunks are received it assembles a 25-byte RF packet and transmits it. Single-frame CAN events (ID 1310) are forwarded immediately as a 4-byte RF packet. |
| `payload_rx_task` | Dequeues ESP-NOW frames received from the payload ESP32 (filtered by MAC address) and forwards them over RF as type-`0x02` packets (up to 33 bytes). |
| `batt_monitor_task` | Polls the MAX17048 fuel-gauge IC over I2C every 5 s and writes the result to a shared global read by `gps_task` when building the next APRS packet. |
| `led_task` | Monitors a FreeRTOS event group and blinks per-subsystem status LEDs (GPS TX, RF TX, CAN RX/TX, Wi-Fi RX) plus a 1 Hz heartbeat on the board status LED. |

### RF Packet Types

| Packet | Description |
|--------|-------------|
| **APRS / AX.25** | GPS position report. Payload format: `=<lat>N/<lon>W Team308 V<voltage>`. Full AX.25 UI frame with callsign `KJ5LPG` and standard digipeater path `WIDE1-1,WIDE2-1`. Max 64 bytes to fit the RFM69 FIFO. |
| **Type `0x01`** | CAN telemetry: 1-byte type header + 24-byte struct containing timestamp, altitude (ft), vertical velocity, 3-axis acceleration (mG), pitch/roll/yaw (°), FSM state, pyro status, and status flags. |
| **Type `0x02`** | ESP-NOW payload forward: 1-byte type header + up to 32 bytes of raw payload data from the payload ESP32. |
| **Type `0x03`** | CAN event: 1-byte type header + node ID + event type + 1-byte data field. |

### RF Configuration

| Parameter | Value | Notes |
|-----------|-------|-------|
| Frequency | 434.0 MHz | UHF ISM / amateur band |
| Bit rate | 4.8 kbps | Slow and robust |
| FSK deviation | 5 kHz | Modulation index ≈ 2 |
| Output power | 20 dBm | RFM69HCW maximum (~100 mW) |
| Preamble | 32 bits | Aids receiver lock-on |
| Sync word | `0x10 0xAF` | Must match ground-station receiver |

---

## Prerequisites
- ESP-IDF VS Code Extension: [Installion Guide](https://docs.espressif.com/projects/vscode-esp-idf-extension/en/latest/installation.html)
- Git

## Steps to run the project
- Create new directory and run `git clone https://github.com/AIAA-UTD-Comet-Rocketry/flight_tracker_RFM69.git`
- [Open existing project](https://docs.espressif.com/projects/vscode-esp-idf-extension/en/latest/startproject.html#opening-an-existing-esp-idf-project)
- [Connect device](https://docs.espressif.com/projects/vscode-esp-idf-extension/en/latest/connectdevice.html)
- [Build project](https://docs.espressif.com/projects/vscode-esp-idf-extension/en/latest/buildproject.html)
- [Flash project](https://docs.espressif.com/projects/vscode-esp-idf-extension/en/latest/flashdevice.html)
- [Monitor project](https://docs.espressif.com/projects/vscode-esp-idf-extension/en/latest/monitoroutput.html)

## RFM69HCW Documentation
 - [Datasheet](https://cdn.sparkfun.com/datasheets/Wireless/General/RFM69HCW-V1.1.pdf)
 - [Radiolib](https://github.com/jgromes/RadioLib)
 - [Radiolib RFM69 default configuration](https://github.com/jgromes/RadioLib/wiki/Default-configuration#rf69sx1231)

#### Arduino Examples
- [Radiolib RFM69](https://github.com/jgromes/RadioLib/tree/master/examples/RF69)
- [RFM69 Reference](https://github.com/jlmurdoch/rfm69hcw_reference)

## ESP32 Help
- [ESPNOW Wifi Protocol](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/network/esp_now.html)
- [ESP32-S3 Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/index.html)
- [ESP32-S3 Datasheet](https://documentation.espressif.com/esp32-s3_datasheet_en.html)

For more information on structure and contents of ESP-IDF projects, please refer to Section [Build System](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html) of the ESP-IDF Programming Guide.

## GPS
- [GPS basics](https://learn.sparkfun.com/tutorials/gps-basics)
- [TinyGPS++ Library](https://github.com/mikalhart/TinyGPSPlus)
- [MAX-M10S Datasheet](https://content.u-blox.com/sites/default/files/MAX-M10S_DataSheet_UBX-20035208.pdf)

## CAN
- [ESP32 CAN/TWAI Guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/twai.html)
- [Beginner's guide to CAN bus](https://www.circuitbread.com/tutorials/understanding-can-a-beginners-guide-to-the-controller-area-network-protocol)
- [CAN Aerospace](https://files.stockflightsystems.com/_5_CANaerospace/CANaerospace_Presentation.pdf)

## PCB Design
- [PCB Design Guidelines](https://www.scs.stanford.edu/~zyedidia/docs/pcb/pcb_tutorial.pdf)

## Troubleshooting

* Program upload failure

    * Hardware connection is not correct: run `idf.py -p PORT monitor`, and reboot your board to see if there are any output logs.
    * The baud rate for downloading is too high: lower your baud rate in the `menuconfig` menu, and try again.

