# Overview
This is the software for the ESP32, which was developed with the *Espressif IoT Development Framework* (ESP-IDF) version 6.1.0, using PlatformIO.

# Networking
Communication between the operator-side and ESP32 happens over the DTLS1.2 protocol. At the time of developing this project, mbedTLS 4.x only supported up to DTLS1.2. For authentication and encryption, pre-shared keys were used (visible in this repository).

# UART Communication
Communication between the ESP32 and Arduino happens over UART. The ESP32 sends the motor setpoints, along with a 16-bit header and CRC16, at 640,000 baud.

# Installation & Dependencies
I highly recommend using PlatformIO, because it will handle all of the dependencies.

# License
This directory and all its parts, where developed by me (not imported from a third party), are released under the same license as the entire project.
