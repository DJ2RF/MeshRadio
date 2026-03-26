#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MR_DISPLAY_WIDTH            128U
#define MR_DISPLAY_HEIGHT            64U
#define MR_DISPLAY_LINES              8U
#define MR_DISPLAY_CHARS_PER_LINE    21U

#define MR_DISPLAY_CTRL_SSD1306       1
#define MR_DISPLAY_CTRL_SH1106        2

#ifndef MR_DISPLAY_ENABLE
#define MR_DISPLAY_ENABLE 1
#endif

#ifndef MR_DISPLAY_I2C_ADDR
#define MR_DISPLAY_I2C_ADDR 0x3C
#endif

#ifndef MR_DISPLAY_I2C_HZ
#define MR_DISPLAY_I2C_HZ 400000U
#endif

#ifndef MR_DISPLAY_CONTRAST
#define MR_DISPLAY_CONTRAST 0x8F
#endif

typedef struct {
    bool initialized;
    bool found;
    uint8_t i2c_addr;
    uint8_t width;
    uint8_t height;
    uint8_t lines;
    uint8_t chars_per_line;
} mr_display_info_t;

esp_err_t mr_display_init(void);
esp_err_t mr_display_clear(void);
esp_err_t mr_display_fill(bool on);
esp_err_t mr_display_present(void);
esp_err_t mr_display_sleep(bool en);
esp_err_t mr_display_set_contrast(uint8_t contrast);
esp_err_t mr_display_flush(void);

void mr_display_power(bool on);
/*
 * line = 0..7 (8 Textzeilen bei 128x64 und 5x7 Font)
 * x    = Pixelposition, sinnvoll meist 0
 */
esp_err_t mr_display_draw_text(uint8_t x, uint8_t line, const char *text);
esp_err_t mr_display_printf(uint8_t x, uint8_t line, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

/* Kompatibel zum bisherigen 4-Zeilen-Aufruf: nutzt Zeilen 0/2/4/6 */
esp_err_t mr_display_show_boot(const char *line1, const char *line2);
esp_err_t mr_display_show_status(const char *title,
                                 const char *line1,
                                 const char *line2,
                                 const char *line3);

/* Neue echte 8-Zeilen-Ausgabe */
esp_err_t mr_display_show_status8(const char *line0,
                                  const char *line1,
                                  const char *line2,
                                  const char *line3,
                                  const char *line4,
                                  const char *line5,
                                  const char *line6,
                                  const char *line7);

const mr_display_info_t *mr_display_get_info(void);

#ifdef __cplusplus
}
#endif
