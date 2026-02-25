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
/**
 * Kapitel 31_HTHeltec WiFi LoRa 32 V3 / V3.2 (ESP32-S3) OLED Demo (MANUELL, 6x8)
 *
 * Interne OLED Verdrahtung:
 *   SDA = GPIO17
 *   SCL = GPIO18
 *   RST = GPIO21 (active LOW)
 *   VEXT= GPIO36 (active LOW -> LOW = ON)
 *   I2C Address = 0x3C
 *
 * Kein Logger Hook: Es wird NUR dann aufs OLED geschrieben,
 * wenn du oled_tty_* explizit aufrufst.
 */

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
}

// Board config (fix für Heltec V3/V3.2 S3)
#define OLED_PORT           I2C_NUM_0
#define OLED_ADDR           0x3C
#define OLED_SDA            17
#define OLED_SCL            18
#define OLED_RST_GPIO       21
#define OLED_VEXT_GPIO      36
#define OLED_VEXT_ON_LEVEL  0   // active LOW: 0 = ON

#include "oled_tty_ssd1306.h"

static const char* TAG = "OLED_MANUAL";

static void oled_demo_screen()
{
  oled_tty_clear();
  oled_tty_print("HELTEC V3 S3\n");
  oled_tty_print("MANUAL OLED 6x8\n");
  oled_tty_printf("I2C 0x%02X\n", OLED_ADDR);
  oled_tty_print("Ready...\n");
}

extern "C" void app_main(void)
{
  ESP_LOGI(TAG, "Boot manual OLED demo...");

  esp_err_t err = oled_tty_init();
  ESP_LOGI(TAG, "oled_tty_init -> %s (%d)", esp_err_to_name(err), (int)err);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "OLED init failed. Check: VEXT(GPIO36)=LOW, SDA17/SCL18, RST21, Addr 0x3C.");
    return;
  }

  oled_demo_screen();

  int counter = 0;
  
  //oled_tty_write_line(0, "Status: RUN");
  //oled_tty_printf_at(0, 7, "CNT:%05d", counter);

  while (true) {
    // Statuszeile unten aktualisieren (ohne Clear -> kein Flackern)
    oled_tty_cursor(0, 7);  // letzte Page
    oled_tty_printf("CNT:%05d", counter++);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}