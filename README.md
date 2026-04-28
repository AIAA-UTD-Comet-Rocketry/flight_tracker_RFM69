| Supported Targets | ESP32-S3 |

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

