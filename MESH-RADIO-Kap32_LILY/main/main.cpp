/**
 * LILYGO T3 LoRa32 V1.6.1 OLED Demo
 *
 * OLED:
 *   SDA = GPIO21
 *   SCL = GPIO22
 *   RST = GPIO16 (active LOW)
 *   Addr = 0x3C
 */

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
}

// ---------- Board Config ----------
#define OLED_PORT      I2C_NUM_0
#define OLED_ADDR      0x3C
#define OLED_SDA       21
#define OLED_SCL       22
#define OLED_RST_GPIO  16
#define OLED_FREQ      100000

#include "oled_tty_ssd1306.h"

static const char* TAG = "OLED_T3";

static void oled_demo_screen()
{
    oled_tty_clear();

    oled_tty_write_line(0, "PAGE 0");
    oled_tty_write_line(1, "PAGE 1");
    oled_tty_write_line(2, "PAGE 2");
    oled_tty_write_line(3, "PAGE 3");
    oled_tty_write_line(4, "PAGE 4");
    oled_tty_write_line(5, "PAGE 5");
    oled_tty_write_line(6, "PAGE 6");
    oled_tty_write_line(7, "PAGE 7");
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Boot OLED demo (LILYGO T3 V1.6.1)...");

    esp_err_t err = oled_tty_init();
    ESP_LOGI(TAG, "oled_tty_init -> %s (%d)", esp_err_to_name(err), (int)err);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "OLED offline -> continuing without display.");
    }

    oled_demo_screen();

    int counter = 0;
    char b[20];

    while (true) {
        snprintf(b, sizeof(b), "CNT:%05d", counter++);
        oled_tty_write_line(7, b);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}