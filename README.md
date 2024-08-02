# F446RE-VL53L8CX

#### Serial monitoring
`minicom -D /dev/ttyACM0 -b 115200`

## TODO:

1. Redo with interrupts
2. redo with RTOS

## Relevant datasheets

[VL53L8CX](https://www.pololu.com/file/0J2029/vl53l8cx.pdf): See page 16 for SPI timing specification. Supported SPI modes are clock-polarity = 1 and clock-phase = 1. Be sure to change these in the `.ioc` file, as they are set to 0 by default.

VL53L8CX supports a maximum 3 Mhz SPI communication, so the SYSCLOCK frequency, APB1 prescaler, and SPI prescaler must be appropriately set on board the F446RE must be set accordingly. Use STM32CubeMX for proper setup.

Data is big-endian (VL53L8CX datasheet pg. 17)


### Useful reading:

[VL53L8CX Datasheet](https://www.st.com/resource/en/datasheet/vl53l8cx.pdf)

[DigiKey STM32 Nucleo SPI tutorial](https://www.digikey.com/en/maker/projects/getting-started-with-stm32-how-to-use-spi/09eab3dfe74c4d0391aaaa99b0a8ee17)

[More SPI Baudrate/SYSCLOCK information](https://community.st.com/t5/missing-articles/how-is-my-spi-s-baudrate-calculated-using-stm32cubemx/td-p/49592)

https://www.youtube.com/watch?v=eFKeNPJq50g&t=1s


um3109-a-guide-for-using-the-vl53l8cx-lowpower-highperformance-timeofflight-multizone-ranging-sensor-stmicroelectronics.pdf
