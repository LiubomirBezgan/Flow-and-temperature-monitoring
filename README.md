# Flow and temperature monitoring

FreeRTOS \ STM32F446RET6 \ STM32CubeIDE

![Figure 1.](https://github.com/LiubomirBezgan/Flow-and-temperature-monitoring/blob/main/device_list.png?raw=true, "Flow and temperature monitoring station mounted on two breadboards and supplied by a portable power bank")
*Figure 1.* A flow and temperature monitoring station mounted on two breadboards and supplied by a portable power bank.

_____
### DESCRIPTION & PURPOSE

The device serves to register liquid flow and temperature values, as well as ambient humidity and temperature. The measured results are recorded on an SD card. A measurement is initiated and stopped by pressing a button mounted on the NUCLEO-F446RE board. An LED, that is located on the breadboard, indicates that the measurement is ongoing. 
+ MCU Architecture: **Arm** 32-bit **Cortex-M4** CPU with DSP and FPU (STM32F446RET6)
+ OS: **FreeRTOS**
+ Peripheries:
  + **DS18B20** digital temperature sensor (**1-wire UART**),
  + **BME280** temperature, humidity and pressure sensor (**I2C**),
  + **SD card** reader (**FatFS**, **SPI**),
  + Liquid flow sensors (**timers**)

The solution was developed for practical use in the study [An Experimental and Numerical Investigation of a Heat Exchanger for Showers](https://www.mdpi.com/2883644).
![Figure 2.](https://github.com/LiubomirBezgan/Flow-and-temperature-monitoring/blob/main/scientific%20article_mdpi.png?raw=true, "Figure 2. An Experimental and Numerical Investigation of a Heat Exchanger for Showers")
*Figure 2.* The study "An Experimental and Numerical Investigation of a Heat Exchanger for Showers" where the device was used for experimental measurements.


