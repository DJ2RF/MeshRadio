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
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_err.h"

/*
    =========================================================
    SX1276 LoRa RX Test (Polling)
    Board: TTGO T-Beam V1.1 (433 MHz)
    Chip:  SX1276
    =========================================================

    Ziel:
      - LoRa initialisieren
      - Dauerhaft auf Empfang schalten (RX Continuous)
      - Wenn ein Paket empfangen wird:
          * Payload-Länge lesen
          * FIFO auslesen
          * Text ausgeben
          * RSSI/SNR ausgeben

    Hinweis:
      - Kein Interrupt (DIO0) nötig
      - Wir pollen RegIrqFlags auf RxDone
*/

/* ===============================
   PIN MAPPING TTGO T-Beam V1.1
   =============================== */
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 27
#define PIN_NUM_CLK  5
#define PIN_NUM_CS   18
#define PIN_NUM_RST  23

#define LORA_SPI_HOST VSPI_HOST

static const char *TAG = "LORA_RX";
static spi_device_handle_t lora_spi = NULL;

/* ===============================
   Register (SX1276 LoRa Mode)
   =============================== */
#define REG_FIFO                0x00
#define REG_OP_MODE             0x01
#define REG_FRF_MSB             0x06
#define REG_FRF_MID             0x07
#define REG_FRF_LSB             0x08
#define REG_IRQ_FLAGS           0x12
#define REG_RX_NB_BYTES         0x13
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_FIFO_ADDR_PTR       0x0D
#define REG_FIFO_RX_BASE_ADDR   0x0F
#define REG_MODEM_CONFIG_1      0x1D
#define REG_MODEM_CONFIG_2      0x1E
#define REG_MODEM_CONFIG_3      0x26
#define REG_PKT_RSSI_VALUE      0x1A
#define REG_PKT_SNR_VALUE       0x1B
#define REG_VERSION             0x42

/* IRQ Bits */
#define IRQ_RX_DONE             0x40
#define IRQ_PAYLOAD_CRC_ERROR   0x20

/* ===============================
   SPI init
   =============================== */
static esp_err_t lora_spi_init(void)
{
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };

    esp_err_t err = spi_bus_initialize(LORA_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
    };

    return spi_bus_add_device(LORA_SPI_HOST, &devcfg, &lora_spi);
}

/* ===============================
   Register access
   =============================== */
static esp_err_t lora_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), value };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx };
    return spi_device_transmit(lora_spi, &t);
}

static esp_err_t lora_read_reg(uint8_t reg, uint8_t *value)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), 0x00 };
    uint8_t rx[2] = { 0 };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx, .rx_buffer = rx };

    esp_err_t err = spi_device_transmit(lora_spi, &t);
    if (err == ESP_OK) *value = rx[1];
    return err;
}

/* ===============================
   Reset
   =============================== */
static void lora_reset(void)
{
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

/* ===============================
   Helpers
   =============================== */
static void lora_clear_irqs(void)
{
    lora_write_reg(REG_IRQ_FLAGS, 0xFF);
}

static void lora_enter_lora_mode_sleep(void)
{
    // Sleep + LoRa
    lora_write_reg(REG_OP_MODE, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void lora_set_frequency_433(void)
{
    uint32_t frf = 0x6C8000; // 433 MHz
    lora_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    lora_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
    lora_write_reg(REG_FRF_LSB, (uint8_t)(frf));
}

static void lora_set_modem_defaults(void)
{
    // BW 125 kHz, CR 4/5, Explicit Header
    lora_write_reg(REG_MODEM_CONFIG_1, 0x72);

    // SF7, CRC On
    lora_write_reg(REG_MODEM_CONFIG_2, 0x74);

    // AGC Auto On
    lora_write_reg(REG_MODEM_CONFIG_3, 0x04);
}

static void lora_set_standby(void)
{
    // LoRa + Standby
    lora_write_reg(REG_OP_MODE, 0x81);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void lora_rx_continuous(void)
{
    // RX base address typischerweise 0x00
    lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0x00);

    // FIFO pointer auf RX base
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x00);

    // RX Continuous: LoRa + RXCONT
    lora_write_reg(REG_OP_MODE, 0x85);
}

/* FIFO auslesen */
static void lora_fifo_read(uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        lora_read_reg(REG_FIFO, &buf[i]);
    }
}

/*
    RSSI/SNR:
    - SNR ist signed (2's complement), Wert/4 = dB
    - RSSI ist abhängig vom Band. Für 433 MHz verwendet man typischerweise:
        RSSI[dBm] = -157 + RegPktRssiValue
*/
static void lora_print_rssi_snr(void)
{
    uint8_t rssi_raw = 0;
    uint8_t snr_raw  = 0;

    lora_read_reg(REG_PKT_RSSI_VALUE, &rssi_raw);
    lora_read_reg(REG_PKT_SNR_VALUE, &snr_raw);

    int8_t snr_signed = (int8_t)snr_raw;
    float snr_db = snr_signed / 4.0f;

    int rssi_dbm = -157 + (int)rssi_raw;

    ESP_LOGI(TAG, "RSSI=%d dBm, SNR=%.2f dB", rssi_dbm, snr_db);
}

/* ===============================
   app_main
   =============================== */
void app_main(void)
{
    ESP_LOGI(TAG, "SPI init...");
    ESP_ERROR_CHECK(lora_spi_init());

    ESP_LOGI(TAG, "Reset...");
    lora_reset();

    uint8_t version = 0;
    lora_read_reg(REG_VERSION, &version);
    ESP_LOGI(TAG, "RegVersion = 0x%02X (erwartet 0x12)", version);

    if (version != 0x12) {
        ESP_LOGE(TAG, "Kein SX1276 gefunden -> Abbruch");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "LoRa Mode (Sleep)...");
    lora_enter_lora_mode_sleep();

    ESP_LOGI(TAG, "Set frequency 433 MHz...");
    lora_set_frequency_433();

    ESP_LOGI(TAG, "Set modem defaults...");
    lora_set_modem_defaults();

    ESP_LOGI(TAG, "Standby...");
    lora_set_standby();

    ESP_LOGI(TAG, "Starte RX Continuous...");
    lora_clear_irqs();
    lora_rx_continuous();

    while (1) {
        uint8_t irq = 0;
        lora_read_reg(REG_IRQ_FLAGS, &irq);

        // RX done?
        if (irq & IRQ_RX_DONE) {

            // CRC Fehler?
            if (irq & IRQ_PAYLOAD_CRC_ERROR) {
                ESP_LOGW(TAG, "RX done, aber CRC error (IRQ=0x%02X)", irq);
                lora_clear_irqs();
                continue;
            }

            // Anzahl Bytes im FIFO
            uint8_t bytes = 0;
            lora_read_reg(REG_RX_NB_BYTES, &bytes);

            // Aktuelle FIFO RX Adresse
            uint8_t fifo_current = 0;
            lora_read_reg(REG_FIFO_RX_CURRENT_ADDR, &fifo_current);

            // FIFO Pointer auf aktuelle RX Adresse setzen
            lora_write_reg(REG_FIFO_ADDR_PTR, fifo_current);

            // Payload lesen
            uint8_t buf[256];
            if (bytes > sizeof(buf) - 1) bytes = sizeof(buf) - 1;

            lora_fifo_read(buf, bytes);
            buf[bytes] = 0; // als String terminieren

            ESP_LOGI(TAG, "RX Paket (%u Bytes): \"%s\"", bytes, (char*)buf);
            lora_print_rssi_snr();

            // IRQ Flags löschen und weiter empfangen
            lora_clear_irqs();
            lora_rx_continuous();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
