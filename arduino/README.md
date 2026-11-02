# Overview
This is the software for the Arduino.

# UART Communication
Communication between the ESP32 and Arduino happens over UART. The ESP32 sends the motor setpoints, along with a 16-bit header and CRC16, at 640,000 baud.

# Actuation
This program re-implements the [Braccio library](https://github.com/arduino-libraries/Braccio) for Arduino, because the original library does not support cooperative computing and natural movement.

# Installation & Dependencies
I highly recommend using PlatformIO, because it will handle all of the dependencies.

# License
This directory and all its parts, where developed by me (not imported from a third party), are released under the same license as the entire project.
