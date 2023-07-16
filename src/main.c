#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lcd.h"

static void i2c_setup(void) {
  rcc_periph_clock_enable(RCC_I2C1);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_reset_pulse(RST_I2C1);
  gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 | GPIO9);
  gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ,
                          GPIO8 | GPIO9);
  gpio_set_af(GPIOB, GPIO_AF4, GPIO8 | GPIO9);
  i2c_peripheral_disable(I2C1);
  i2c_reset(I2C1);
  i2c_set_standard_mode(I2C1);
  i2c_enable_ack(I2C1);
  i2c_set_speed(I2C1, i2c_speed_sm_100k, rcc_apb1_frequency / 1e6);
  i2c_peripheral_enable(I2C1);
}

int main(void) {
  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_180MHZ]);
  i2c_setup();
  uint8_t address = 0x01;
  uint8_t buffer[16];

  i2c_transfer7(I2C1, (uint8_t)0x50, &address, sizeof(address), buffer,
                sizeof(buffer));
  char *bufferptr = malloc(strlen(buffer));
  sprintf(bufferptr, "%s", buffer);

  lcd_init();
  lcd_backlight(true);
  lcd_write_string(bufferptr);
  lcd_display();

  free(bufferptr);

  while (1)
    ;
  return 0;
}