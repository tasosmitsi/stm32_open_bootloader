# stm32_open_bootloader

This is a simple bootloader for STM32F411CEUx microcontrollers. Purpose of that repository is the creation of a template bootloader that can be used to update the firmware of the microcontroller via different protocols. The bootloader is written in C and uses the STM32 HAL library. The bootloader should be completely agnostic to the protocol used to update the firmware, and to the firmware itself. 

## Building the bootloader and the firmware

Just open the project in VSCode and run the `ST-Link: Debug All` task. This will build the bootloader and the firmware, and flash them to the microcontroller. Given of course that you have the entire ST echosystem installed and the microcontroller is connected to the computer via ST-Link.