# demo_brushles

This project is a STM32-based brushless motor control prototype built around the STM32F407VGT6 microcontroller. It demonstrates the basic integration of a brushless ESC, an analog throttle input, an MPU6050 inertial sensor, UART telemetry, and a status LED.

## Overview

The firmware uses the STM32 MCU to:

- read a potentiometer or ADC input value,
- convert it to an ESC PWM pulse width,
- arm and drive a brushless ESC,
- read accelerometer and gyroscope data from the MPU6050,
- provide status information through UART,
- signal the system state using an RGB LED.

This project is intended as a compact embedded control testbench for drone or motor-control experiments.

## Hardware

The prototype is based on the following components:

- STM32F407VGT6
- MPU6050 IMU module
- Brushless ESC
- Brushless motor
- ADC input source
- RGB LED
- UART serial monitor

## Main features

- ADC-to-ESC conversion
- ESC start-up / arming sequence
- PWM signal generation for motor command
- I2C sensor acquisition from MPU6050
- UART debug output
- RGB LED status indication
- Safe pulse limits for ESC operation

## ESC command range

The firmware defines the following ESC output limits:

- Minimum pulse: 1000 us
- Maximum pulse: 1500 us

The ADC value is mapped into this range to control the motor command level.

## Project structure

```text
demo_brushles/
├── Core/
│   ├── Inc/
│   └── Src/
├── Drivers/
├── Debug/
├── .cproject
├── .mxproject
├── .project
├── demo_brushles.ioc
├── demo_brushles Debug.launch
├── STM32F407VGTX_FLASH.ld
├── STM32F407VGTX_RAM.ld
└── README.md
```

## Important files

- `Core/Src/main.c` : main firmware logic
- `Core/Src/mpu6050.c` : MPU6050 driver implementation
- `Core/Inc/mpu6050.h` : MPU6050 definitions and API
- `demo_brushles.ioc` : STM32CubeMX configuration

## Firmware behavior

The main application flow is:

1. Initialize peripherals
2. Start ADC conversion
3. Read ADC value
4. Convert ADC value to ESC pulse width
5. Arm the ESC safely
6. Read MPU6050 values
7. Send status to UART
8. Update the RGB LED according to the state

## Getting started

1. Open the project using STM32CubeIDE or another STM32-compatible IDE.
2. Load the `.ioc` file if you want to regenerate or modify the configuration.
3. Build the firmware.
4. Flash it to the STM32 board.
5. Connect the ESC, IMU, and power wiring correctly.
6. Monitor the serial output to verify startup and sensor data.

## Notes

- The ESC must be armed carefully according to the manufacturer instructions.
- The code is intended as a development and learning prototype.
- This project is designed to be extended for more advanced drone or closed-loop control logic.

## Summary

This project is a practical embedded control example for a brushless motor system based on STM32. It combines motor actuation, inertial sensing, and debugging interfaces in a compact and easy-to-modify firmware setup.
