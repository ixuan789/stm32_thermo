# 一个温度计

作业存档。
> A thermometer using stm32+0.96oled, with UART Transmit.
> Using HAL Library with STM32CubeMX & CLion.

### Connection:

- Resister:
  RS -> ADC2_IN0 (PA0)
  
  NTC -> ADC1_IN1 (PA1)
  
- Button:
  S2 -> PA4 (Emoji)
  
  S3 -> PA5 (UART)
  
  S4 -> PA6 (Run Calibration)
  
- 0.96 OLED:
  PB3 -> CS
  
  PB4 -> DC
  
  PB5 -> RES
  
  PB6 -> MOSI (D1)
  
  PB7 -> SCLK (D0)
  
- UART1 Transmit.

### How to use
- Turn the RS to switch interface.
- Press button to switch Emoji and UART.
