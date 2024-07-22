# VexUF Bootloader

## Overview

This repository introduces the bootloader for the VexUF project, designed for the STM32F401 microcontroller. The bootloader is responsible for loading the firmware `vexuf_fw.bin` from an SD card and flashing it to the application memory region of the MCU.

## Features

- **File Loading**: Reads the firmware file `vexuf_fw.bin` from the SD card.
- **Pin Check**: Only activates the bootloader if the `B1 (Boot1)` pin (PB2) is high.
- **Flash Erase and Write**: Erases the necessary flash sectors and writes the new firmware in 2KB chunks.
- **Auto Delete firmware file**: Once the flashing is complete successfully, The `vexuf_fw.bin` is deleted from the SD Card so VexUF doesn't keep flashing it again.
- **Application Jump**: Jumps to the application code after successful firmware flashing or if the B1 (Boot1) pin is low.

## Memory Map

```
- Bootloader:  0x08000000 - 0x08007FFF (32KB)
- Application: 0x08008000 - 0x0803FFFF (224KB)
- RAM:         0x20000000 - 0x2000FFFF (64KB)
```
## Usage

1. **Initial Setup**: Ensure the SD card is formatted with a FAT/FAT32 file system and the firmware file `vexuf_fw.bin` is placed in the root directory.
2. **Insert SD Card**: Insert the SD card in the SD card slot while VexUF is powered off.
3. **Pin Configuration**: Make sure that the B1 (Boot1) pin (PB2) is correctly configured to signal the bootloader activation.
4. **Bootloader Activation**: On reset/power-on, the bootloader will check the status of the B1 (Boot1) pin.
    - If the B1 (Boot1) pin is high and `vexuf_fw.bin` exists, the bootloader will start flashing the firmware.
    - If the B1 (Boot1) pin is low or `vexuf_fw.bin` does not exist, the bootloader will jump to the application.

## Indicators
1- When the `B1 (Boot1)` is shorted (set `HIGH`), the `Boot1 LED` will be turned on.
2- `SD Card LED` is turned on. The bootloader is accessing the SD card to find the `vexuf_fw.bin` file.
3- The bootloader start sequence: 
  - The following LEDS will sequentially and briefly blink: `Error LED`, `Warn LED`, `Info LED`.
4- If the `vexuf_fw.bin` file was found, and its size is less than 224KBs, the `Info LED` will blink five times indicating the start of the flashing process.
5- During flashing VexUF, the `Warn LED` will be flashing as it writes each chunk of the binary file to VexUF.
6- Once Flashing is complete, the `Info LED` will blink five times indicating the end of the flashing process.
7- The bootloader end sequence: 
  - The following LEDS will sequentially and briefly blink: `Info LED`, `Warn LED`, `Error LED`. 
8- `SD Card LED` is turned off. The bootloader is no longer accessing the SD card.

If the bootloader encounters an error, the `Error LED` will blink 5 times, and VexUF will jump to the application
