#include "incl/mr_display.h"
#include "incl/config_meshradio.h"
#include "incl/board_pins.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "incl/mr_i2c_bus.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "MR_DISPLAY";

#if (MR_BOARD_PRESET == MR_BOARD_HELTEC_V3)

#define MR_DISPLAY_BOARD_NAME    "HELTEC"
#define MR_DISPLAY_CONTROLLER    MR_DISPLAY_CTRL_SSD1306
#define MR_DISPLAY_PIN_SDA       17
#define MR_DISPLAY_PIN_SCL       18
#define MR_DISPLAY_PIN_RST       21
#define MR_DISPLAY_NEEDS_RESET   1
#define MR_DISPLAY_I2C_PORT      I2C_NUM_0

#elif (MR_BOARD_PRESET == MR_BOARD_LILYGO_SX1276)

#define MR_DISPLAY_BOARD_NAME    "LILYGO"
#define MR_DISPLAY_CONTROLLER    MR_DISPLAY_CTRL_SSD1306
#define MR_DISPLAY_PIN_SDA       PIN_I2C_SDA
#define MR_DISPLAY_PIN_SCL       PIN_I2C_SCL
#define MR_DISPLAY_PIN_RST       (-1)
#define MR_DISPLAY_NEEDS_RESET   0
#if defined(I2C_NUM_1)
#define MR_DISPLAY_I2C_PORT      I2C_NUM_1
#else
#define MR_DISPLAY_I2C_PORT      I2C_NUM_0
#endif

#elif (MR_BOARD_PRESET == MR_BOARD_TBEAM_V11_SX1276)

#define MR_DISPLAY_BOARD_NAME    "TBEAM_V11"
#define MR_DISPLAY_CONTROLLER    MR_DISPLAY_CTRL_SSD1306
#define MR_DISPLAY_PIN_SDA       PIN_I2C_SDA
#define MR_DISPLAY_PIN_SCL       PIN_I2C_SCL
#define MR_DISPLAY_PIN_RST       (-1)
#define MR_DISPLAY_NEEDS_RESET   0
#define MR_DISPLAY_I2C_PORT      I2C_NUM_0

#elif (MR_BOARD_PRESET == MR_BOARD_TBEAM_V12_AXP2101)

#define MR_DISPLAY_BOARD_NAME    "TBEAM_V12"
#define MR_DISPLAY_CONTROLLER    MR_DISPLAY_CTRL_SSD1306
#define MR_DISPLAY_PIN_SDA       PIN_I2C_SDA
#define MR_DISPLAY_PIN_SCL       PIN_I2C_SCL
#define MR_DISPLAY_PIN_RST       (-1)
#define MR_DISPLAY_NEEDS_RESET   0
#define MR_DISPLAY_I2C_PORT      I2C_NUM_0

#else
#error "Unknown MR_BOARD_PRESET for mr_display.c"
#endif

static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_oled = NULL;
static uint8_t s_fb[MR_DISPLAY_WIDTH * (MR_DISPLAY_HEIGHT / 8U)];
static mr_display_info_t s_info = {
    .initialized = false,
    .found = false,
    .i2c_addr = MR_DISPLAY_I2C_ADDR,
    .width = MR_DISPLAY_WIDTH,
    .height = MR_DISPLAY_HEIGHT,
    .lines = MR_DISPLAY_LINES,
    .chars_per_line = MR_DISPLAY_CHARS_PER_LINE,
};

static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00}, {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62}, {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00}, {0x14,0x08,0x3E,0x08,0x14}, {0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x60,0x60,0x00,0x00}, {0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00}, {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39}, {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}, {0x00,0x36,0x36,0x00,0x00}, {0x00,0x56,0x36,0x00,0x00},
    {0x08,0x14,0x22,0x41,0x00}, {0x14,0x14,0x14,0x14,0x14}, {0x00,0x41,0x22,0x14,0x08}, {0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E}, {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01}, {0x3E,0x41,0x49,0x49,0x7A},
    {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40}, {0x7F,0x02,0x0C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F}, {0x3F,0x40,0x38,0x40,0x3F},
    {0x63,0x14,0x08,0x14,0x63}, {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43}, {0x00,0x7F,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20}, {0x00,0x41,0x41,0x7F,0x00}, {0x04,0x02,0x01,0x02,0x04}, {0x40,0x40,0x40,0x40,0x40},
    {0x00,0x01,0x02,0x04,0x00}, {0x20,0x54,0x54,0x54,0x78}, {0x7F,0x48,0x44,0x44,0x38}, {0x38,0x44,0x44,0x44,0x20},
    {0x38,0x44,0x44,0x48,0x7F}, {0x38,0x54,0x54,0x54,0x18}, {0x08,0x7E,0x09,0x01,0x02}, {0x08,0x14,0x54,0x54,0x3C},
    {0x7F,0x08,0x04,0x04,0x78}, {0x00,0x44,0x7D,0x40,0x00}, {0x20,0x40,0x44,0x3D,0x00}, {0x7F,0x10,0x28,0x44,0x00},
    {0x00,0x41,0x7F,0x40,0x00}, {0x7C,0x04,0x18,0x04,0x78}, {0x7C,0x08,0x04,0x04,0x78}, {0x38,0x44,0x44,0x44,0x38},
    {0x7C,0x14,0x14,0x14,0x08}, {0x08,0x14,0x14,0x18,0x7C}, {0x7C,0x08,0x04,0x04,0x08}, {0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3F,0x44,0x40,0x20}, {0x3C,0x40,0x40,0x20,0x7C}, {0x1C,0x20,0x40,0x20,0x1C}, {0x3C,0x40,0x30,0x40,0x3C},
    {0x44,0x28,0x10,0x28,0x44}, {0x0C,0x50,0x50,0x50,0x3C}, {0x44,0x64,0x54,0x4C,0x44}, {0x00,0x08,0x36,0x41,0x00},
    {0x00,0x00,0x7F,0x00,0x00}, {0x00,0x41,0x36,0x08,0x00}, {0x08,0x04,0x08,0x10,0x08}
};

static esp_err_t mr_display_send_cmd(uint8_t cmd)
{
    if (s_oled == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint8_t buf[2] = { 0x00, cmd };
    return i2c_master_transmit(s_oled, buf, sizeof(buf), pdMS_TO_TICKS(100));
}

static esp_err_t mr_display_send_cmd_list(const uint8_t *cmds, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        const esp_err_t err = mr_display_send_cmd(cmds[i]);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

static esp_err_t mr_display_hw_reset(void)
{
#if MR_DISPLAY_NEEDS_RESET
    ESP_RETURN_ON_ERROR(gpio_reset_pin((gpio_num_t)MR_DISPLAY_PIN_RST), TAG, "gpio_reset_pin failed");
    ESP_RETURN_ON_ERROR(gpio_set_direction((gpio_num_t)MR_DISPLAY_PIN_RST, GPIO_MODE_OUTPUT), TAG, "gpio_set_direction failed");
    ESP_RETURN_ON_ERROR(gpio_set_level((gpio_num_t)MR_DISPLAY_PIN_RST, 1), TAG, "gpio_set_level failed");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(gpio_set_level((gpio_num_t)MR_DISPLAY_PIN_RST, 0), TAG, "gpio_set_level failed");
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_RETURN_ON_ERROR(gpio_set_level((gpio_num_t)MR_DISPLAY_PIN_RST, 1), TAG, "gpio_set_level failed");
    vTaskDelay(pdMS_TO_TICKS(20));
#endif
    return ESP_OK;
}

static esp_err_t mr_display_bus_init(void)
{
    if (s_i2c_bus != NULL) {
        return ESP_OK;
    }

    return mr_i2c_bus_get(MR_DISPLAY_I2C_PORT,
                          MR_DISPLAY_PIN_SDA,
                          MR_DISPLAY_PIN_SCL,
                          &s_i2c_bus);
}

static esp_err_t mr_display_dev_init(void)
{
    if (s_oled != NULL) {
        return ESP_OK;
    }

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MR_DISPLAY_I2C_ADDR,
        .scl_speed_hz = MR_DISPLAY_I2C_HZ,
    };

    return i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_oled);
}

static void mr_display_draw_char_raw(uint8_t x, uint8_t line, char c)
{
    if (line >= MR_DISPLAY_LINES || x >= MR_DISPLAY_WIDTH) {
        return;
    }

    uint8_t ch = (uint8_t)c;
    if (ch < 32U || ch > 126U) {
        ch = '?';
    }

    const uint8_t *glyph = font5x7[ch - 32U];
    const uint16_t base = (uint16_t)line * MR_DISPLAY_WIDTH + x;

    for (uint8_t i = 0; i < 5U; ++i) {
        if ((uint16_t)(x + i) < MR_DISPLAY_WIDTH) {
            s_fb[base + i] = glyph[i];
        }
    }
    if ((uint16_t)(x + 5U) < MR_DISPLAY_WIDTH) {
        s_fb[base + 5U] = 0x00;
    }
}

void mr_display_power(bool on)
{
#if (MR_BOARD_PRESET == MR_BOARD_HELTEC_V3)
#ifdef PIN_VEXT_CTRL
    gpio_num_t pin = (gpio_num_t)PIN_VEXT_CTRL;

    (void)gpio_reset_pin(pin);
    (void)gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    (void)gpio_set_level(pin, on ? 0 : 1);   // Heltec VEXT: LOW = ON
#else
    (void)on;
#endif
#else
    (void)on;
#endif
}

esp_err_t mr_display_present(void)
{
    return (s_info.initialized && s_info.found) ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t mr_display_clear(void)
{
    memset(s_fb, 0, sizeof(s_fb));
    return ESP_OK;
}

esp_err_t mr_display_fill(bool on)
{
    memset(s_fb, on ? 0xFF : 0x00, sizeof(s_fb));
    return mr_display_flush();
}

esp_err_t mr_display_set_contrast(uint8_t contrast)
{
    ESP_RETURN_ON_ERROR(mr_display_present(), TAG, "display not initialized");
    const uint8_t cmds[] = { 0x81, contrast };
    return mr_display_send_cmd_list(cmds, sizeof(cmds));
}

esp_err_t mr_display_sleep(bool en)
{
    ESP_RETURN_ON_ERROR(mr_display_present(), TAG, "display not initialized");
    return mr_display_send_cmd(en ? 0xAE : 0xAF);
}

esp_err_t mr_display_init(void)
{
    if (!MR_DISPLAY_ENABLE) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_LOGI(TAG,
             "init display board=%s ctrl=%d %ux%u port=%d sda=%d scl=%d addr=0x%02X",
             MR_DISPLAY_BOARD_NAME,
             MR_DISPLAY_CONTROLLER,
             MR_DISPLAY_WIDTH,
             MR_DISPLAY_HEIGHT,
             (int)MR_DISPLAY_I2C_PORT,
             (int)MR_DISPLAY_PIN_SDA,
             (int)MR_DISPLAY_PIN_SCL,
             MR_DISPLAY_I2C_ADDR);

    mr_display_power(true);
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_RETURN_ON_ERROR(mr_display_hw_reset(), TAG, "display reset failed");
    ESP_RETURN_ON_ERROR(mr_display_bus_init(), TAG, "i2c bus init failed");
    ESP_RETURN_ON_ERROR(mr_display_dev_init(), TAG, "i2c add device failed");

    esp_err_t err = i2c_master_probe(s_i2c_bus, MR_DISPLAY_I2C_ADDR, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        s_info.initialized = false;
        s_info.found = false;
        ESP_LOGW(TAG, "display not responding at 0x%02X: %s", MR_DISPLAY_I2C_ADDR, esp_err_to_name(err));
        return err;
    }

    static const uint8_t init_cmds[] = {
        0xAE,
        0xD5, 0x80,
        0xA8, 0x3F,
        0xD3, 0x00,
        0x40,
        0x8D, 0x14,
        0x20, 0x00,
#if (MR_BOARD_PRESET == MR_BOARD_HELTEC_V3)
        0xA1,
        0xC8,
#else
        0xA0,
        0xC0,
#endif
        0xDA, 0x12,
        0x81, MR_DISPLAY_CONTRAST,
        0xD9, 0xF1,
        0xDB, 0x40,
        0xA4,
        0xA6,
        0x2E,
        0xAF
    };

    ESP_RETURN_ON_ERROR(mr_display_send_cmd_list(init_cmds, sizeof(init_cmds)), TAG, "oled init failed");

    s_info.initialized = true;
    s_info.found = true;
    ESP_RETURN_ON_ERROR(mr_display_clear(), TAG, "clear failed");
    return mr_display_flush();
}

esp_err_t mr_display_flush(void)
{
    ESP_RETURN_ON_ERROR(mr_display_present(), TAG, "display not initialized");

    for (uint8_t page = 0; page < (MR_DISPLAY_HEIGHT / 8U); ++page) {
        uint8_t col_low = 0x00;
        uint8_t col_high = 0x10;
        if (MR_DISPLAY_CONTROLLER == MR_DISPLAY_CTRL_SH1106) {
            col_low = 0x02;
            col_high = 0x10;
        }

        const uint8_t page_cmds[] = {
            (uint8_t)(0xB0U + page),
            col_low,
            col_high
        };

        ESP_RETURN_ON_ERROR(mr_display_send_cmd_list(page_cmds, sizeof(page_cmds)), TAG, "page set failed");

        uint8_t tx[1 + MR_DISPLAY_WIDTH];
        tx[0] = 0x40;
        memcpy(&tx[1], &s_fb[(uint16_t)page * MR_DISPLAY_WIDTH], MR_DISPLAY_WIDTH);
        ESP_RETURN_ON_ERROR(i2c_master_transmit(s_oled, tx, sizeof(tx), pdMS_TO_TICKS(200)), TAG, "display transmit failed");
    }

    return ESP_OK;
}

esp_err_t mr_display_draw_text(uint8_t x, uint8_t line, const char *text)
{
    if (text == NULL) {
        return ESP_OK;
    }
    if (line >= MR_DISPLAY_LINES) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t cx = x;
    while ((*text != '\0') && (cx <= (MR_DISPLAY_WIDTH - 6U))) {
        mr_display_draw_char_raw(cx, line, *text++);
        cx = (uint8_t)(cx + 6U);
    }
    return ESP_OK;
}

esp_err_t mr_display_printf(uint8_t x, uint8_t line, const char *fmt, ...)
{
    if (fmt == NULL) {
        return ESP_OK;
    }

    char linebuf[MR_DISPLAY_CHARS_PER_LINE + 1U];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(linebuf, sizeof(linebuf), fmt, ap);
    va_end(ap);
    return mr_display_draw_text(x, line, linebuf);
}

esp_err_t mr_display_show_boot(const char *line1, const char *line2)
{
    ESP_RETURN_ON_ERROR(mr_display_clear(), TAG, "clear failed");
    ESP_RETURN_ON_ERROR(mr_display_draw_text(0, 0, "MESHRADIO"), TAG, "draw failed");
    ESP_RETURN_ON_ERROR(mr_display_draw_text(0, 2, line1 ? line1 : MR_DISPLAY_BOARD_NAME), TAG, "draw failed");
    ESP_RETURN_ON_ERROR(mr_display_draw_text(0, 4, line2 ? line2 : "DISPLAY OK"), TAG, "draw failed");
    return mr_display_flush();
}

esp_err_t mr_display_show_status(const char *title,
                                 const char *line1,
                                 const char *line2,
                                 const char *line3)
{
    ESP_RETURN_ON_ERROR(mr_display_clear(), TAG, "clear failed");
    ESP_RETURN_ON_ERROR(mr_display_draw_text(0, 0, title ? title : "STATUS"), TAG, "draw failed");
    ESP_RETURN_ON_ERROR(mr_display_draw_text(0, 2, line1 ? line1 : ""), TAG, "draw failed");
    ESP_RETURN_ON_ERROR(mr_display_draw_text(0, 4, line2 ? line2 : ""), TAG, "draw failed");
    ESP_RETURN_ON_ERROR(mr_display_draw_text(0, 6, line3 ? line3 : ""), TAG, "draw failed");
    return mr_display_flush();
}

esp_err_t mr_display_show_status8(const char *line0,
                                  const char *line1,
                                  const char *line2,
                                  const char *line3,
                                  const char *line4,
                                  const char *line5,
                                  const char *line6,
                                  const char *line7)
{
    ESP_RETURN_ON_ERROR(mr_display_clear(), TAG, "clear failed");
    ESP_RETURN_ON_ERROR(mr_display_draw_text(0, 0, line0 ? line0 : ""), TAG, "draw failed");
    ESP_RETURN_ON_ERROR(mr_display_draw_text(0, 1, line1 ? line1 : ""), TAG, "draw failed");
    ESP_RETURN_ON_ERROR(mr_display_draw_text(0, 2, line2 ? line2 : ""), TAG, "draw failed");
    ESP_RETURN_ON_ERROR(mr_display_draw_text(0, 3, line3 ? line3 : ""), TAG, "draw failed");
    ESP_RETURN_ON_ERROR(mr_display_draw_text(0, 4, line4 ? line4 : ""), TAG, "draw failed");
    ESP_RETURN_ON_ERROR(mr_display_draw_text(0, 5, line5 ? line5 : ""), TAG, "draw failed");
    ESP_RETURN_ON_ERROR(mr_display_draw_text(0, 6, line6 ? line6 : ""), TAG, "draw failed");
    ESP_RETURN_ON_ERROR(mr_display_draw_text(0, 7, line7 ? line7 : ""), TAG, "draw failed");
    return mr_display_flush();
}

const mr_display_info_t *mr_display_get_info(void)
{
    return &s_info;
}
