ESP32 Environment Reporter
==========================

This measures temperature and humidity using SHT3x, then broadcasts them using BLE.

## Prerequisites

* ESP32
* [ESP-IDF](https://idf.espressif.com/) v4.3.1+
* [esp-nimble-cpp](https://github.com/h2zero/esp-nimble-cpp)
* SHT3x sensor


## SDK Config

This app requires BLE (NimBLE) and C++ exceptions feature.

This works only as a BLE broadcaster, so the other roles can be disabled.

WiFi and SNTP configuration is required for system time synchronization.

* Component config
    * Bluetooth -> Enabled
    * Bluetooth controller
        * Bluetooth controller mode -> BLE only
    * Bluetooth Host -> NimBLE - BLEOnly
    * NimBLE Options
        * Enable BLE Peripheral role -> Disabled
        * Enable BLE Central role -> Disabled
        * Enable BLE Observer role -> Disabled
* Compiler options
    * Enable C++ exceptions -> Enabled
* Environment Reporter Configuration
    * WiFi SSID -> set SSID
    * WiFi Password -> set password
    * SNTP Server -> set NTP server
    * SHT3x I2C Address -> choose 0x44 or 0x45

A minimal configuration is included in this repository, see `sdkconfig.defaults`.


## Advertisement packet

This broadcasts a measurement result in PDU payload in advertising packet.
There is no need to connect to this device (the device is non-connectable).

Data format in Ad structure is described below (these fields are stored in this order):

```
Length      uint8_t  = 15;      // in bytes
AdType      uint8_t  = 0x16;    // Service Data
Service     UUID16   = 0x181A;  // Environmental Sensing Service
TimeStamp   uint64;             // Timestamp of measurement
Temperature uint16;             // 16bit raw sensor value
Humidity    uint16;             // 16bit raw sensor value
```

NOTE: they are stored in little endian.

The table below shows conversions from raw value to human-readable value.

|             |                                 |
| ----------- | ------------------------------- |
| Temperature | `-45.0 + 175.0 * (x / 65535.0)` |
| Humidity    | `100.0 * (x / 65535.0)`         |


## Interval of Advertising

After powering-on


## Acknowledgements

* Codes to establish connection to Wi-Fi AP is based on ESP-IDF example: https://github.com/espressif/esp-idf/tree/master/examples/wifi/getting_started/station
* Codes to synchronize system time with NTP is based on ESP-IDF example: https://github.com/espressif/esp-idf/tree/master/examples/protocols/sntp
* This is heaviliy depends on [esp-nimble-cpp](https://github.com/h2zero/esp-nimble-cpp) by h2zero.
