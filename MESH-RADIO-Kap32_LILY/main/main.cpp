// ============================================================
// MeshRadio – Embedded LoRa Mesh Engineering
// Source Header (Open Engineering Edition)
// ============================================================
//
// © 2026 Friedrich Riedhammer (DJ2RF)
// https://nerdverlag.com
// Contact: [fritz@nerdverlag.com](mailto:fritz@nerdverlag.com)
//
// ------------------------------------------------------------
// Project Philosophy
// ------------------------------------------------------------
// This project follows an Open Engineering approach.
//
// The sources are provided for learning, experimentation,
// research, and further development of embedded LoRa mesh
// systems.
//
// Goal:
// Understanding system architecture instead of using
// black-box solutions.
//
// Guiding principle:
// "profi-level stabil"
// Stability first. Deterministic design over features.
//
// ------------------------------------------------------------
// Usage & License Notes
// ------------------------------------------------------------
// - Educational and experimental use is encouraged.
// - Modification and extension are allowed.
// - Commercial usage or redistribution requires
//   prior permission from the author.
//
// ------------------------------------------------------------
// Additional Resources
// ------------------------------------------------------------
// All chapter sources and updates:
// https://nerdverlag.com
//
// ============================================================

/**
 * main.cpp — LILYGO T3 V1.6.1 OLED Test (ESP-IDF 5.x, U8g2, Software-I2C)
 *
 * Ziel:
 *  - SSD1306 OLED (128x64) mit kleiner Schrift ausgeben
 *  - KEIN GPIO16 (Reset) verwenden (bei deinen neuen Modulen problematisch)
 *  - I2C-Konflikte/ESP-IDF I2C new-driver Issues umgehen -> U8g2 Software-I2C nutzen
 *
 * Hardware:
 *  - OLED SDA = GPIO21
 *  - OLED SCL = GPIO22
 *  - OLED Address: meistens 0x3C (manchmal 0x3D)
 *
 * Build:
 *  - U8g2 als ESP-IDF component eingebunden (components/u8g2)
 *  - include: "u8g2.h"
 */

#include <stdio.h>
#include <string.h>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
}

#include "u8g2.h"

// --------- OLED / I2C Pins (T3 V1.6.1) ----------
#define OLED_SDA   GPIO_NUM_21
#define OLED_SCL   GPIO_NUM_22

// OLED I2C 7-bit Adresse. Falls dunkel bleibt: 0x3D testen.
#define OLED_ADDR  0x3C

static const char *TAG = "U8G2_SW_I2C";
static u8g2_t u8g2;

/**
 * U8g2 Callback: GPIO + Delay
 *
 * Für Software-I2C braucht U8g2:
 *  - Init der GPIOs (Open-Drain + Pullups)
 *  - Setzen von SDA/SCL Pegeln
 *  - Delay-Funktionen
 *
 * Wichtig: In C++ muss jede 'case' mit Variablen-Deklaration in {} Blöcken stehen,
 * sonst gibt es "jump to case label / crosses initialization" Fehler.
 */
static uint8_t u8x8_gpio_and_delay_esp32(u8x8_t *u8x8,
                                        uint8_t msg,
                                        uint8_t arg_int,
                                        void *arg_ptr)
{
    (void)u8x8;
    (void)arg_ptr;

    switch (msg) {

    // ---- Timing ----
    case U8X8_MSG_DELAY_MILLI:
        vTaskDelay(pdMS_TO_TICKS(arg_int));
        return 1;

    case U8X8_MSG_DELAY_10MICRO:
        esp_rom_delay_us(10 * arg_int);
        return 1;

    case U8X8_MSG_DELAY_100NANO:
        // "100ns" ist auf ESP32 nicht exakt möglich -> 1us reicht für SW-I2C
        esp_rom_delay_us(1);
        return 1;

    // ---- GPIO Init ----
    case U8X8_MSG_GPIO_AND_DELAY_INIT: {
        // SDA/SCL als Open-Drain Output mit Pullup
        gpio_config_t io;
        memset(&io, 0, sizeof(io));

        io.intr_type = GPIO_INTR_DISABLE;
        io.mode = GPIO_MODE_INPUT_OUTPUT_OD; // open drain
        io.pin_bit_mask = (1ULL << OLED_SDA) | (1ULL << OLED_SCL);
        io.pull_up_en = GPIO_PULLUP_ENABLE;
        io.pull_down_en = GPIO_PULLDOWN_DISABLE;

        gpio_config(&io);

        // Bus idle = High
        gpio_set_level(OLED_SDA, 1);
        gpio_set_level(OLED_SCL, 1);

        return 1;
    }

    // ---- SW-I2C Bitbang Leitungen ----
    case U8X8_MSG_GPIO_I2C_CLOCK:
        gpio_set_level(OLED_SCL, arg_int ? 1 : 0);
        return 1;

    case U8X8_MSG_GPIO_I2C_DATA:
        gpio_set_level(OLED_SDA, arg_int ? 1 : 0);
        return 1;

    default:
        // Nicht verwendete Signale ignorieren
        return 1;
    }
}

/**
 * app_main
 *
 * Setup:
 *  - U8g2 konfigurieren: SSD1306 128x64, Software I2C
 *  - Display initialisieren, Powersave aus
 * Loop:
 *  - Text in kleiner Schrift + Counter anzeigen
 */
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "U8g2 init: SSD1306 128x64, Software-I2C (no GPIO16 reset).");

    // U8g2 Setup: SSD1306, SW-I2C
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(
        &u8g2,
        U8G2_R0,
        u8x8_byte_sw_i2c,          // <--- U8g2 Software-I2C Byte-Funktion
        u8x8_gpio_and_delay_esp32  // <--- unser GPIO/Delay Callback
    );

    // U8g2 erwartet 8-bit I2C Adresse (7-bit << 1)
    u8x8_SetI2CAddress(&u8g2.u8x8, (OLED_ADDR << 1));

    // Display starten
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);   // Display ON

    ESP_LOGI(TAG, "Display init done. Drawing tiny text...");

    int cnt = 0;
    char line[32];

    while (true) {
        // Framebuffer löschen
        u8g2_ClearBuffer(&u8g2);

        // Sehr kleine Schrift
        u8g2_SetFont(&u8g2, u8g2_font_5x7_tf);
        //u8g2_SetFont(&u8g2, u8g2_font_4x6_tf);

        // Hinweis: y-Koordinate ist Baseline, nicht Top
        u8g2_DrawStr(&u8g2, 0,  6, "LILYGO T3 V1.6.1");
        u8g2_DrawStr(&u8g2, 0, 12, "U8g2 SW-I2C tiny font");
        u8g2_DrawStr(&u8g2, 0, 18, "GPIO16 NOT used");

        snprintf(line, sizeof(line), "CNT:%05d", cnt++);
        u8g2_DrawStr(&u8g2, 0, 30, line);

        // Buffer aufs Display senden
        u8g2_SendBuffer(&u8g2);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}