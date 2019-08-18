# echelon-bike
Echelon hardware bike mock using the Nordic nRF52 embedded BLE platform.

## Dependencies
* nRF5 SDK (Required)
15.3.0 version as "nRF5_SDK_15.3.0"
* SEGGER Embedded Studio for ARM (Required)
  Tested with version 4.18
* GNU Toolchain (Optional, for Editing in Visual Studio Code)
gcc-arm-none-eabi-8-2019-q3 version as "gcc-arm-none-eabi-8-2019-q3"

## Compiling / Running
Project Folder Structure with Dependencies
* echelon-bike/ble_bike.c
* echelon-bike/ble_bike.h
* echelon-bike/main.c
* echelon-bike/pca10056/s140/sdk_config.h
* echelon-bike/pca10056/s140/ses/ble_bike_pca10056_s140.emProject
* nRF5_SDK_15.3.0/*
* gcc-arm-none-eabi-8-2019-q3/*

After downloading / correctly placing the dependencies and opening the project file in SEGGER Embedded Studio for ARM:
* Build > Build
* Target > Download
* Target > Verify (Optional)

## Supported Hardware
Tested against the Nordic nRF52840 Development Kit (PCA10056 / S140).

This should compile and run without modification for all nRF52840 chips, but requires porting project file for other chips in the nRF52 product line.

The project file for the nRF52840 is "pca10056/s140/ses/ble_bike_pca10056_s140.emProject".
