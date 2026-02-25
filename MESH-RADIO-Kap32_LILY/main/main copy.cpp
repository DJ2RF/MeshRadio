#include <stdio.h>
#include <string.h>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
}

#include "ssd1306.h"

// LILYGO T3 V1.6.1 OLED I2C Pins
#define I2C_SDA_PIN (gpio_num_t)21
#define I2C_SCL_PIN (gpio_num_t)22

static const char *TAG = "OLED_LILYGO";

// Helper: init SSD1306 with given I2C addr
static esp_err_t init_display(i2c_master_bus_handle_t bus, uint8_t addr, ssd1306_handle_t *out)
{
    ssd1306_config_t dev_cfg = {};
    dev_cfg.i2c_address = addr;                 // 0x3C (typisch), manchmal 0x3D
    dev_cfg.i2c_clock_speed = 400000;           // bei dir OK
    dev_cfg.panel_size = SSD1306_PANEL_128x64;
    dev_cfg.display_enabled = true;

    return ssd1306_init(bus, &dev_cfg, out);
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "I2C Bus-Initialisierung (new I2C master driver)...");

    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = I2C_NUM_0;
    bus_cfg.sda_io_num = I2C_SDA_PIN;
    bus_cfg.scl_io_num = I2C_SCL_PIN;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    ESP_LOGI(TAG, "SSD1306 init...");

    ssd1306_handle_t dev = nullptr;

    // Normalfall 0x3C, fallback 0x3D
    esp_err_t err = init_display(bus_handle, 0x3C, &dev);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Init @0x3C failed (%s), trying 0x3D...", esp_err_to_name(err));
        ESP_ERROR_CHECK(init_display(bus_handle, 0x3D, &dev));
    }

    ESP_LOGI(TAG, "Display bereit. Schreibe Status-Text...");

    // Bildschirm löschen
    ssd1306_clear_display(dev, false);

    // Textausgabe: diese Library sendet direkt via I2C (kein eigenes Page-Push nötig)
    ssd1306_display_text(dev, 0, "LILYGO T3 V1.6.1", false);
    ssd1306_display_text(dev, 1, "Status: Online", false);
    ssd1306_display_text(dev, 2, "OLED:ohne GPIO16", false);

    uint32_t seconds = 0;
    char str_buf[24];

    while (true) {
        snprintf(str_buf, sizeof(str_buf), "Sekunden: %lu", (unsigned long)seconds++);
        ssd1306_display_text(dev, 7, str_buf, false);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}