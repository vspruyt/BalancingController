# BalancingController
Part of my balancing robot project.

Main controller, running on a Teensy 4.1.
The controller communicates with several peripherals:
1. AHRS subsystem: [FastAHRS](https://github.com/vspruyt/FastAhrs)
2. Adafruit BLE module for UART over Bluetooth communication with smartphone
3. Speed controller (which deals with the motor encoders)

Usage:
![alt text](https://github.com/vspruyt/BalancingController/raw/main/schema_pic.jpg "Example use")
