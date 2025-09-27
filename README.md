| Supported Targets | ESP32-S3 |
.

## RFM69HCW Documentation
 - [Datasheet](https://cdn.sparkfun.com/datasheets/Wireless/General/RFM69HCW-V1.1.pdf)
 - NOTE: The following below is for the Si446x transceiver
 - [Programming Guide](https://www.silabs.com/documents/public/application-notes/AN633.pdf)
 - [Packet Handling](https://www.silabs.com/documents/public/application-notes/AN626.pdf)
 - More register guides in doc folder - goto "docs\EZRadioPRO_REVC2_API\EZRadioPRO_REVC2" and open index.html

## ESP32 Help

- [ESP32 Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/stable/get-started/index.html)
- [ESP32-S2 Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/get-started/index.html)

For more information on structure and contents of ESP-IDF projects, please refer to Section [Build System](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html) of the ESP-IDF Programming Guide.

## Troubleshooting

* Program upload failure

    * Hardware connection is not correct: run `idf.py -p PORT monitor`, and reboot your board to see if there are any output logs.
    * The baud rate for downloading is too high: lower your baud rate in the `menuconfig` menu, and try again.

