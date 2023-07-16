#include "lcd.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>

volatile uint32_t system_millis;
uint8_t _backlightval = LCD_BACKLIGHT;
uint8_t _displaycontrol;
uint8_t _displaymode;

#define PCF8574_ADDR (uint8_t)0x27

#define LCD_I2C_CLK RCC_I2C2
#define LCD_I2C_RST RST_I2C2
#define LCD_I2C I2C2

#define LCD_SCL_AF GPIO_AF4
#define LCD_SCL_CLK RCC_GPIOB
#define LCD_SCL_PORT GPIOB
#define LCD_SCL GPIO10

#define LCD_SDA_AF GPIO_AF4
#define LCD_SDA_CLK RCC_GPIOC
#define LCD_SDA_PORT GPIOC
#define LCD_SDA GPIO12

static void systick_setup(void) {
  systick_set_reload(180000);
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
  systick_interrupt_enable();
  systick_counter_enable();
}

/* Called when systick fires */
void sys_tick_handler(void) { system_millis++; }

/* sleep for delay milliseconds */
static void msleep(uint32_t delay) {
  uint32_t wake = system_millis + delay;
  while (wake > system_millis)
    ;
}

static void clock_setup(void) {
  rcc_periph_clock_enable(LCD_I2C_CLK);
  rcc_periph_clock_enable(LCD_SDA_CLK);
  rcc_periph_clock_enable(LCD_SCL_CLK);
}

static void i2c_setup(void) {
  rcc_periph_reset_pulse(LCD_I2C_RST);
  gpio_mode_setup(LCD_SCL_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, LCD_SCL);
  gpio_mode_setup(LCD_SDA_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, LCD_SDA);
  gpio_set_output_options(LCD_SCL_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ,
                          LCD_SCL);
  gpio_set_output_options(LCD_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ,
                          LCD_SDA);
  gpio_set_af(LCD_SCL_PORT, LCD_SCL_AF, LCD_SCL);
  gpio_set_af(LCD_SDA_PORT, LCD_SDA_AF, LCD_SDA);
  i2c_peripheral_disable(LCD_I2C);
  i2c_reset(LCD_I2C);
  i2c_set_standard_mode(LCD_I2C);
  i2c_enable_ack(LCD_I2C);
  i2c_set_speed(LCD_I2C, i2c_speed_sm_100k, rcc_apb1_frequency / 1e6);
  i2c_peripheral_enable(LCD_I2C);
}

static void lcd_write(uint8_t data) {
  uint8_t masked_data = data | _backlightval;
  i2c_transfer7(LCD_I2C, PCF8574_ADDR, &masked_data, sizeof(data), NULL, 0);
}

static void pulse_enable(uint8_t data) {
  lcd_write(data | En);
  msleep(1);
  lcd_write(data & ~En);
  msleep(1);
}

static void write4bits(uint8_t value) {
  lcd_write(value);
  pulse_enable(value);
}

static void send(uint8_t value, uint8_t mode) {
  uint8_t highnib = value & 0xf0;
  uint8_t lownib = (value << 4) & 0xf0;
  write4bits((highnib) | mode);
  write4bits((lownib) | mode);
}

static inline void command(uint8_t cmd) { send(cmd, 0); }
static inline size_t write(uint8_t val) {
  send(val, Rs);
  return 1;
}

static void clear(void) {
  command(LCD_CLEARDISPLAY);
  msleep(3);
}

static void home(void) {
  command(LCD_RETURNHOME);  // set cursor position to zero
  msleep(3);                // this command takes a long time!
}

static void lcd_send_data(char data) {
  char data_u, data_l;
  uint8_t data_t[4];
  data_u = (data & 0xf0);
  data_l = ((data << 4) & 0xf0);
  data_t[0] = data_u | 0x0D;  // en=1, rs=1
  data_t[1] = data_u | 0x09;  // en=0, rs=1
  data_t[2] = data_l | 0x0D;  // en=1, rs=1
  data_t[3] = data_l | 0x09;  // en=0, rs=1
  i2c_transfer7(LCD_I2C, PCF8574_ADDR, data_t, 4, NULL, 0);
}

void lcd_set_cursor(uint8_t col, uint8_t row) {
  int row_offsets[] = {0x00, 0x40, 0x14, 0x54};
  if (row > 2) {
    row = 1;  // we count rows starting w/0
  }
  command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void lcd_display(void) {
  _displaycontrol |= LCD_DISPLAYON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

void lcd_init(void) {
  clock_setup();
  systick_setup();
  i2c_setup();

  uint8_t masked_init = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;
  i2c_transfer7(LCD_I2C, PCF8574_ADDR, &masked_init, sizeof(masked_init), NULL,
                0);
  msleep(50);
  lcd_write(_backlightval);
  msleep(300);
  write4bits(0x03 << 4);
  msleep(5);
  write4bits(0x03 << 4);
  msleep(1);
  write4bits(0x03 << 4);
  msleep(1);

  // finally, set to 4-bit interface
  write4bits(0x02 << 4);

  // set # lines, font size, etc.
  command(LCD_FUNCTIONSET | masked_init);

  // turn the display on with no cursor or blinking default
  _displaycontrol = LCD_DISPLAYON | LCD_CURSORON | LCD_BLINKON;
  lcd_display();

  // clear it off
  clear();
  _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  command(LCD_ENTRYMODESET | _displaymode);
  home();
  lcd_display();
  msleep(2);
}

void lcd_backlight(bool status) {
  _backlightval = status ? LCD_BACKLIGHT : LCD_NOBACKLIGHT;
}

void lcd_write_string(char *str) {
  while (*str) {
    lcd_send_data(*str++);
  }
}