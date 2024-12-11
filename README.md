# ADIS16470 Imu Driver for Host Machine

This repository contains the source code for the ADIS16470 IMU driver for the host machine. The driver is written in C++ 17 and uses the Linux Serial interface to communicate with the IMU.

## Features

- Support for the ADIS16470 IMU
- Support for reading sensor data (accelerometer, gyroscope, temperature(TODO))
- Support for configuring sensor settings (TODO)
- Support for reading sensor status and error codes (TODO)
- Support for reading sensor calibration data (TODO)
- Support calculating the rough time offset between the IMU and the host machine

## Installation

To install the driver, follow these steps:

1. Clone the repository: `git clone --recursive https://github.com/geniusdo/Imu-driver.git`
2. Create a build directory: `mkdir -p Imu-driver/build `
3. Enter the build directory: `cd Imu-driver/build`
4. Run the CMake command: `cmake ..`
5. Build the driver: `make`

## Run

To run the driver, just a simple command

`./imu_driver -d <device_name> -o <output_file> -t <seconds>`