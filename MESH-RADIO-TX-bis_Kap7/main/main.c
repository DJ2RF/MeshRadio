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
    SX1276 LoRa TX Test (Polling)
    Board: TTGO T-Beam V1.1 (433 MHz)
    Chip:  SX1276
    =========================================================

    Ziel:
      - LoRa initialisieren
      - Ein kurzes Textpaket senden
      - TX Done Flag pollen und ausgeben

    Hinweis:
      - Kein Interrupt nötig
      - Kein Empfänger nötig, um TX Done zu sehen
      - Für späteren RX-Test braucht man ein zweites Board
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

static const char *TAG = "LORA_TX";
static spi_device_handle_t lora_spi = NULL;

/* ===============================
   Register (SX1276 LoRa Mode)
   =============================== */
#define REG_OP_MODE            0x01
#define REG_FRF_MSB            0x06
#define REG_FRF_MID            0x07
#define REG_FRF_LSB            0x08
#define REG_PA_CONFIG          0x09
#define REG_FIFO               0x00
#define REG_FIFO_ADDR_PTR      0x0D
#define REG_FIFO_TX_BASE_ADDR  0x0E
#define REG_PAYLOAD_LENGTH     0x22
#define REG_IRQ_FLAGS          0x12
#define REG_IRQ_FLAGS_MASK     0x11
#define REG_MODEM_CONFIG_1     0x1D
#define REG_MODEM_CONFIG_2     0x1E
#define REG_MODEM_CONFIG_3     0x26
#define REG_VERSION            0x42

/* IRQ Bits */
#define IRQ_TX_DONE            0x08

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
static void lora_set_frequency_433(void)
{
    uint32_t frf = 0x6C8000; // 433 MHz
    lora_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    lora_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
    lora_write_reg(REG_FRF_LSB, (uint8_t)(frf));
}

static void lora_enter_lora_mode_sleep(void)
{
    // Sleep + LoRa
    lora_write_reg(REG_OP_MODE, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void lora_set_standby(void)
{
    // LoRa + Standby
    lora_write_reg(REG_OP_MODE, 0x81);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void lora_set_tx_power_dbm_14(void)
{
    /*
      RegPaConfig:
        Bit7    PaSelect (1 = PA_BOOST)
        Bit6-4  MaxPower
        Bit3-0  OutputPower

      Für viele Boards ist PA_BOOST korrekt.
      0x8E ist eine gängige Einstellung ~14 dBm.
    */
    lora_write_reg(REG_PA_CONFIG, 0x8E);
}

static void lora_set_modem_defaults(void)
{
    /*
      Damit TX zuverlässig funktioniert, setzen wir konservative Defaults:
      - BW 125 kHz
      - CR 4/5
      - SF7
      - CRC an
      - LowDataRateOptimize aus (bei SF7/BW125 nicht nötig)

      Hinweis: Diese Werte sind ein Startpunkt fürs Buch.
    */

    // RegModemConfig1: BW=125kHz(0x70), CR=4/5(0x02), ExplicitHeader(0)
    lora_write_reg(REG_MODEM_CONFIG_1, 0x72);

    // RegModemConfig2: SF7(0x70), CRC On(0x04)
    lora_write_reg(REG_MODEM_CONFIG_2, 0x74);

    // RegModemConfig3: AGC Auto On (0x04), LowDataRateOptimize (0)
    lora_write_reg(REG_MODEM_CONFIG_3, 0x04);
}

static void lora_clear_irqs(void)
{
    // IRQ Flags werden gelöscht, indem man 1en schreibt
    lora_write_reg(REG_IRQ_FLAGS, 0xFF);
}

static void lora_fifo_write(const uint8_t *data, size_t len)
{
    // FIFO ist Register 0x00. Write läuft wie normale Registerwrites.
    for (size_t i = 0; i < len; i++) {
        lora_write_reg(REG_FIFO, data[i]);
    }
}

static void lora_tx_packet(const uint8_t *payload, size_t len)
{
    // 1) IRQs löschen
    lora_clear_irqs();

    // 2) TX Base Address setzen (typisch 0x00)
    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0x00);

    // 3) FIFO Pointer auf TX Base setzen
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x00);

    // 4) Payload ins FIFO schreiben
    lora_fifo_write(payload, len);

    // 5) Payload length setzen
    lora_write_reg(REG_PAYLOAD_LENGTH, (uint8_t)len);

    // 6) TX starten: LoRa + TX mode
    lora_write_reg(REG_OP_MODE, 0x83);
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

    ESP_LOGI(TAG, "Set TX power...");
    lora_set_tx_power_dbm_14();

    ESP_LOGI(TAG, "Standby...");
    lora_set_standby();

    const char *msg = "DJ2RF test TX";
    ESP_LOGI(TAG, "Sende: \"%s\"", msg);

    lora_tx_packet((const uint8_t*)msg, strlen(msg));

    // TX done pollen
    while (1) {
        uint8_t irq = 0;
        lora_read_reg(REG_IRQ_FLAGS, &irq);

        if (irq & IRQ_TX_DONE) {
            ESP_LOGI(TAG, "TX done ✅ IRQ_FLAGS=0x%02X", irq);
            lora_clear_irqs();
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Zur Demo: alle 3 Sekunden erneut senden
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(3000));
        ESP_LOGI(TAG, "Sende erneut...");
        lora_tx_packet((const uint8_t*)msg, strlen(msg));

        while (1) {
            uint8_t irq = 0;
            lora_read_reg(REG_IRQ_FLAGS, &irq);
            if (irq & IRQ_TX_DONE) {
                ESP_LOGI(TAG, "TX done ✅ IRQ_FLAGS=0x%02X", irq);
                lora_clear_irqs();
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}
