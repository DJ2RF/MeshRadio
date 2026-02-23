#pragma once
/**
 * oled_logger.h (ESP-IDF, header-only)
 * Mirrors ESP_LOG output to OLED using esp_log_set_vprintf.
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
}

#include "oled_tty_ssd1306.h"

static vprintf_like_t _oledlog_prev_vprintf = nullptr;
static SemaphoreHandle_t _oledlog_mutex = nullptr;
static bool _oledlog_strip_ansi = true;

#ifndef OLEDLOG_LINEBUF
#define OLEDLOG_LINEBUF 192
#endif

static char   _oledlog_linebuf[OLEDLOG_LINEBUF];
static size_t _oledlog_linepos = 0;

static inline void _oledlog_write_filtered(const char* s, size_t n) {
  for (size_t i = 0; i < n; i++) {
    char c = s[i];

    if (_oledlog_strip_ansi && c == '\x1b') { // ESC
      i++;
      if (i < n && s[i] == '[') {
        i++;
        while (i < n) {
          char x = s[i];
          if ((x >= 'A' && x <= 'Z') || (x >= 'a' && x <= 'z')) break;
          i++;
        }
      }
      continue;
    }

    if (c == '\r') continue;

    if (c == '\n') {
      if (_oledlog_linepos > 0) {
        _oledlog_linebuf[_oledlog_linepos] = 0;
        oled_tty_print(_oledlog_linebuf);
        _oledlog_linepos = 0;
      }
      oled_tty_putc('\n');
      continue;
    }

    if (_oledlog_linepos < (OLEDLOG_LINEBUF - 1)) {
      _oledlog_linebuf[_oledlog_linepos++] = c;
    } else {
      _oledlog_linebuf[_oledlog_linepos] = 0;
      oled_tty_print(_oledlog_linebuf);
      oled_tty_putc('\n');
      _oledlog_linepos = 0;
    }
  }
}

static inline int _oledlog_vprintf_hook(const char* fmt, va_list ap) {
  char tmp[256];
  va_list ap2;
  va_copy(ap2, ap);
  int n = vsnprintf(tmp, sizeof(tmp), fmt, ap2);
  va_end(ap2);

  int out = _oledlog_prev_vprintf ? _oledlog_prev_vprintf(fmt, ap) : vprintf(fmt, ap);

  if (_oledlog_mutex && xSemaphoreTake(_oledlog_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    size_t len = (n < 0) ? 0u : (size_t)((n < (int)sizeof(tmp)) ? n : (int)sizeof(tmp));
    _oledlog_write_filtered(tmp, len);
    xSemaphoreGive(_oledlog_mutex);
  }
  return out;
}

static inline void oled_logger_install(bool strip_ansi_colors) {
  _oledlog_strip_ansi = strip_ansi_colors;
  if (!_oledlog_mutex) _oledlog_mutex = xSemaphoreCreateMutex();
  _oledlog_prev_vprintf = esp_log_set_vprintf(&_oledlog_vprintf_hook);
}

static inline void oled_logger_uninstall(void) {
  if (_oledlog_prev_vprintf) {
    esp_log_set_vprintf(_oledlog_prev_vprintf);
    _oledlog_prev_vprintf = nullptr;
  }
}