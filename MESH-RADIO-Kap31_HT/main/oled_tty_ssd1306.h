#pragma once
/**
 * oled_tty_ssd1306.h (ESP-IDF, header-only, C++ safe)
 *
 * Minimaler Text-Only Treiber für SSD1306 128x64 (I2C).
 *
 * Heltec WiFi LoRa 32 V3/V3.2 (ESP32-S3):
 *  - Vext (GPIO36 active LOW) muss an sein
 *  - OLED Reset (GPIO21 active LOW) vor Init
 *  - SDA=17, SCL=18, Addr=0x3C
 *
 * Schrift:
 *  - 6x8 ASCII (klassisch, gut lesbar)
 *
 * Zusatz-Helpers:
 *  - oled_tty_clear_line(page)          : löscht eine Display-Zeile (Page 0..7)
 *  - oled_tty_print_at(col,page,text)   : Cursor setzen + Text schreiben
 *  - oled_tty_printf_at(col,page,fmt,...) : printf an Position
 *  - oled_tty_write_line(page,text)     : Zeile löschen + Text schreiben (keine Reste)
 */

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern "C" {
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

#ifndef OLED_PORT
#define OLED_PORT I2C_NUM_0
#endif
#ifndef OLED_ADDR
#define OLED_ADDR 0x3C
#endif
#ifndef OLED_SDA
#define OLED_SDA 17
#endif
#ifndef OLED_SCL
#define OLED_SCL 18
#endif
#ifndef OLED_FREQ
#define OLED_FREQ 400000
#endif

#ifndef OLED_VEXT_GPIO
#define OLED_VEXT_GPIO -1
#endif
#ifndef OLED_VEXT_ON_LEVEL
#define OLED_VEXT_ON_LEVEL 0
#endif

#ifndef OLED_RST_GPIO
#define OLED_RST_GPIO -1
#endif

#define OLED_W     128
#define OLED_H     64
#define OLED_PAGES (OLED_H / 8)

static uint8_t _oled_col  = 0;
static uint8_t _oled_page = 0;

static inline void _oled_delay_ms(int ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

static inline void _oled_gpio_try_set_output(int gpio_num, int level) {
  if (gpio_num < 0) return;
  gpio_num_t p = (gpio_num_t)gpio_num;
  if (gpio_set_direction(p, GPIO_MODE_OUTPUT) != ESP_OK) return;
  gpio_set_level(p, level);
}

static inline void _oled_power_on_if_needed() {
  if (OLED_VEXT_GPIO >= 0) {
    _oled_gpio_try_set_output(OLED_VEXT_GPIO, OLED_VEXT_ON_LEVEL);
    _oled_delay_ms(50);
  }
}

static inline void _oled_reset_if_needed() {
  if (OLED_RST_GPIO >= 0) {
    // active LOW reset pulse
    _oled_gpio_try_set_output(OLED_RST_GPIO, 1);
    _oled_delay_ms(2);
    _oled_gpio_try_set_output(OLED_RST_GPIO, 0);
    _oled_delay_ms(10);
    _oled_gpio_try_set_output(OLED_RST_GPIO, 1);
    _oled_delay_ms(10);
  }
}

static inline esp_err_t _oled_i2c_write(uint8_t ctrl, const uint8_t* data, size_t n) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, ctrl, true);
  if (n) i2c_master_write(cmd, (uint8_t*)data, n, true);
  i2c_master_stop(cmd);
  esp_err_t e = i2c_master_cmd_begin(OLED_PORT, cmd, pdMS_TO_TICKS(200));
  i2c_cmd_link_delete(cmd);
  return e;
}

static inline esp_err_t _oled_cmd(uint8_t c) { return _oled_i2c_write(0x00, &c, 1); }

// 6x8 Font ASCII 32..127 (6 bytes per glyph)
static const uint8_t _f6x8[96][6] = {
  {0,0,0,0,0,0},{0,0,0x5F,0,0,0},{0,0x07,0,0x07,0,0},{0x14,0x7F,0x14,0x7F,0x14,0},
  {0x24,0x2A,0x7F,0x2A,0x12,0},{0x23,0x13,0x08,0x64,0x62,0},{0x36,0x49,0x55,0x22,0x50,0},{0,0x05,0x03,0,0,0},
  {0,0x1C,0x22,0x41,0,0},{0,0x41,0x22,0x1C,0,0},{0x14,0x08,0x3E,0x08,0x14,0},{0x08,0x08,0x3E,0x08,0x08,0},
  {0,0x50,0x30,0,0,0},{0x08,0x08,0x08,0x08,0x08,0},{0,0x60,0x60,0,0,0},{0x20,0x10,0x08,0x04,0x02,0},
  {0x3E,0x51,0x49,0x45,0x3E,0},{0,0x42,0x7F,0x40,0,0},{0x42,0x61,0x51,0x49,0x46,0},{0x21,0x41,0x45,0x4B,0x31,0},
  {0x18,0x14,0x12,0x7F,0x10,0},{0x27,0x45,0x45,0x45,0x39,0},{0x3C,0x4A,0x49,0x49,0x30,0},{0x01,0x71,0x09,0x05,0x03,0},
  {0x36,0x49,0x49,0x49,0x36,0},{0x06,0x49,0x49,0x29,0x1E,0},{0,0x36,0x36,0,0,0},{0,0x56,0x36,0,0,0},
  {0x08,0x14,0x22,0x41,0,0},{0x14,0x14,0x14,0x14,0x14,0},{0,0x41,0x22,0x14,0x08,0},{0x02,0x01,0x51,0x09,0x06,0},
  {0x32,0x49,0x79,0x41,0x3E,0},{0x7E,0x11,0x11,0x11,0x7E,0},{0x7F,0x49,0x49,0x49,0x36,0},{0x3E,0x41,0x41,0x41,0x22,0},
  {0x7F,0x41,0x41,0x22,0x1C,0},{0x7F,0x49,0x49,0x49,0x41,0},{0x7F,0x09,0x09,0x09,0x01,0},{0x3E,0x41,0x49,0x49,0x7A,0},
  {0x7F,0x08,0x08,0x08,0x7F,0},{0,0x41,0x7F,0x41,0,0},{0x20,0x40,0x41,0x3F,0x01,0},{0x7F,0x08,0x14,0x22,0x41,0},
  {0x7F,0x40,0x40,0x40,0x40,0},{0x7F,0x02,0x0C,0x02,0x7F,0},{0x7F,0x04,0x08,0x10,0x7F,0},{0x3E,0x41,0x41,0x41,0x3E,0},
  {0x7F,0x09,0x09,0x09,0x06,0},{0x3E,0x41,0x51,0x21,0x5E,0},{0x7F,0x09,0x19,0x29,0x46,0},{0x46,0x49,0x49,0x49,0x31,0},
  {0x01,0x01,0x7F,0x01,0x01,0},{0x3F,0x40,0x40,0x40,0x3F,0},{0x1F,0x20,0x40,0x20,0x1F,0},{0x7F,0x20,0x18,0x20,0x7F,0},
  {0x63,0x14,0x08,0x14,0x63,0},{0x03,0x04,0x78,0x04,0x03,0},{0x61,0x51,0x49,0x45,0x43,0},{0,0x7F,0x41,0x41,0,0},
  {0x02,0x04,0x08,0x10,0x20,0},{0,0x41,0x41,0x7F,0,0},{0x04,0x02,0x01,0x02,0x04,0},{0x40,0x40,0x40,0x40,0x40,0},
  {0,0x01,0x02,0x04,0,0},{0x20,0x54,0x54,0x54,0x78,0},{0x7F,0x48,0x44,0x44,0x38,0},{0x38,0x44,0x44,0x44,0x20,0},
  {0x38,0x44,0x44,0x48,0x7F,0},{0x38,0x54,0x54,0x54,0x18,0},{0x08,0x7E,0x09,0x01,0x02,0},{0x0C,0x52,0x52,0x52,0x3E,0},
  {0x7F,0x08,0x04,0x04,0x78,0},{0,0x44,0x7D,0x40,0,0},{0x20,0x40,0x44,0x3D,0,0},{0x7F,0x10,0x28,0x44,0,0},
  {0,0x41,0x7F,0x40,0,0},{0x7C,0x04,0x18,0x04,0x78,0},{0x7C,0x08,0x04,0x04,0x78,0},{0x38,0x44,0x44,0x44,0x38,0},
  {0x7C,0x14,0x14,0x14,0x08,0},{0x08,0x14,0x14,0x18,0x7C,0},{0x7C,0x08,0x04,0x04,0x08,0},{0x48,0x54,0x54,0x54,0x20,0},
  {0x04,0x3F,0x44,0x40,0x20,0},{0x3C,0x40,0x40,0x20,0x7C,0},{0x1C,0x20,0x40,0x20,0x1C,0},{0x3C,0x40,0x30,0x40,0x3C,0},
  {0x44,0x28,0x10,0x28,0x44,0},{0x0C,0x50,0x50,0x50,0x3C,0},{0x44,0x64,0x54,0x4C,0x44,0},{0,0x08,0x36,0x41,0,0},
  {0,0,0x7F,0,0,0},{0,0x41,0x36,0x08,0,0},{0x08,0x04,0x08,0x10,0x08,0},{0,0,0,0,0,0},
};

// -------------------------- Core API --------------------------

static inline esp_err_t oled_tty_init(void) {
  _oled_power_on_if_needed();

  i2c_config_t conf{};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = (gpio_num_t)OLED_SDA;
  conf.scl_io_num = (gpio_num_t)OLED_SCL;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = OLED_FREQ;
  conf.clk_flags = 0;

  esp_err_t e = i2c_param_config(OLED_PORT, &conf);
  if (e != ESP_OK) return e;

  e = i2c_driver_install(OLED_PORT, conf.mode, 0, 0, 0);
  if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) return e;

  _oled_reset_if_needed();

  // SSD1306 init
  _oled_cmd(0xAE);
  _oled_cmd(0xD5); _oled_cmd(0x80);
  _oled_cmd(0xA8); _oled_cmd(0x3F);
  _oled_cmd(0xD3); _oled_cmd(0x00);
  _oled_cmd(0x40);
  _oled_cmd(0x8D); _oled_cmd(0x14);
  _oled_cmd(0x20); _oled_cmd(0x00);
  _oled_cmd(0xA1);
  _oled_cmd(0xC8);
  _oled_cmd(0xDA); _oled_cmd(0x12);
  _oled_cmd(0x81); _oled_cmd(0xFF);
  _oled_cmd(0xD9); _oled_cmd(0xF1);
  _oled_cmd(0xDB); _oled_cmd(0x40);
  _oled_cmd(0xA4);
  _oled_cmd(0xA6);
  _oled_cmd(0xAF);

  _oled_col = 0;
  _oled_page = 0;
  return ESP_OK;
}

static inline void oled_tty_cursor(uint8_t col, uint8_t page) {
  if (col >= OLED_W) col = 0;
  if (page >= OLED_PAGES) page = 0;
  _oled_col = col; _oled_page = page;

  _oled_cmd(0xB0 | (_oled_page & 0x0F));
  _oled_cmd(0x00 | (_oled_col & 0x0F));
  _oled_cmd(0x10 | ((_oled_col >> 4) & 0x0F));
}

static inline void oled_tty_clear(void) {
  uint8_t z[16] = {0};
  for (uint8_t p = 0; p < OLED_PAGES; p++) {
    oled_tty_cursor(0, p);
    for (int i = 0; i < OLED_W; i += 16) _oled_i2c_write(0x40, z, sizeof(z));
  }
  oled_tty_cursor(0, 0);
}

static inline void oled_tty_putc(char c) {
  if (c == '\n') {
    _oled_col = 0;
    _oled_page = (uint8_t)((_oled_page + 1) % OLED_PAGES);
    oled_tty_cursor(_oled_col, _oled_page);
    return;
  }
  if (c < 32 || c > 127) c = '?';

  // 6 columns per glyph
  if (_oled_col + 6 >= OLED_W) {
    _oled_col = 0;
    _oled_page = (uint8_t)((_oled_page + 1) % OLED_PAGES);
    oled_tty_cursor(_oled_col, _oled_page);
  }

  _oled_i2c_write(0x40, _f6x8[(uint8_t)c - 32], 6);
  _oled_col += 6;
}

static inline void oled_tty_print(const char* s) { while (s && *s) oled_tty_putc(*s++); }

static inline void oled_tty_printf(const char* fmt, ...) {
  char b[128];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  oled_tty_print(b);
}

// -------------------------- Helpers --------------------------

/** Clear one text line (= one SSD1306 page). */
static inline void oled_tty_clear_line(uint8_t page) {
  if (page >= OLED_PAGES) return;
  oled_tty_cursor(0, page);

  uint8_t z[16] = {0};
  for (int i = 0; i < OLED_W; i += 16) _oled_i2c_write(0x40, z, sizeof(z));

  oled_tty_cursor(0, page);
}

/** Set cursor and print string (no clearing). */
static inline void oled_tty_print_at(uint8_t col, uint8_t page, const char* s) {
  oled_tty_cursor(col, page);
  oled_tty_print(s);
}

/** Set cursor and printf (no clearing). */
static inline void oled_tty_printf_at(uint8_t col, uint8_t page, const char* fmt, ...) {
  oled_tty_cursor(col, page);
  char b[128];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  oled_tty_print(b);
}

/**
 * Clear a line and write text from column 0.
 * This prevents leftover characters from older longer messages.
 */
static inline void oled_tty_write_line(uint8_t page, const char* s) {
  oled_tty_clear_line(page);
  oled_tty_print_at(0, page, s);
}