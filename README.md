AFAIK there is no 2x16 I2C LCD example for libopencm3 using STM32F4, there is one with FreeRTOS for STM32F1 though.

Currently, the "main.c" file is an example which reads a 24cXX EEPROM's 16 bytes and prints it on the LCD. 

So, here you go!

Check the defines in `lcd.c` and change it accordingly if you want to, default mapping is:
| Function     | Port  | Pin    | Reset    | Alternate func | RCC Clock |
|--------------|-------|--------|----------|----------------|-----------|
| I2C SDA      | GPIOC | GPIO12 |          | GPIO_AF4       |RCC_GPIOC | 
| I2C SCL      | GPIOB | GPIO10 |          | GPIO_AF4       |RCC_GPIOB |
| I2C Instance | I2C2  |        | RST_I2C2 |                |RCC_I2C2 |