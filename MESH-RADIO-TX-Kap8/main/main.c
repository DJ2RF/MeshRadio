#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_err.h"

/*
    =========================================================
    MeshRadio Frame TX (Polling)
    Board: TTGO T-Beam V1.1 (433 MHz)
    Chip : SX1276
    =========================================================

    Ziel:
      - MR-Frame (Header + Payload) bauen
      - ins FIFO schreiben
      - TX starten
      - auf TxDone pollen (RegIrqFlags)
      - alle 3 Sekunden erneut senden

    Vorteil:
      - Kein DIO0 nötig
      - Funktioniert unabhängig von Board-Revisionen
*/

/* ===============================
   Pins TTGO T-Beam V1.1 (SX1276)
   =============================== */
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 27
#define PIN_NUM_CLK  5
#define PIN_NUM_CS   18
#define PIN_NUM_RST  23

#define LORA_SPI_HOST VSPI_HOST

static const char *TAG = "MR_TX_POLL";
static spi_device_handle_t lora_spi = NULL;

/* ===============================
   SX1276 Register (LoRa)
   =============================== */
#define REG_FIFO               0x00
#define REG_OP_MODE            0x01
#define REG_FRF_MSB            0x06
#define REG_FRF_MID            0x07
#define REG_FRF_LSB            0x08
#define REG_PA_CONFIG          0x09
#define REG_FIFO_ADDR_PTR      0x0D
#define REG_FIFO_TX_BASE_ADDR  0x0E
#define REG_PAYLOAD_LENGTH     0x22
#define REG_IRQ_FLAGS          0x12
#define REG_MODEM_CONFIG_1     0x1D
#define REG_MODEM_CONFIG_2     0x1E
#define REG_MODEM_CONFIG_3     0x26
#define REG_VERSION            0x42

/* IRQ Bits */
#define IRQ_TX_DONE            0x08

/* ===============================
   MeshRadio Frame v1
   =============================== */
#pragma pack(push, 1)
typedef struct {
    uint8_t magic[2];     // 'M','R'
    uint8_t version;      // 1
    uint8_t flags;        // reserved (später ACK/BEACON/etc.)
    uint8_t ttl;          // hop limit (für Mesh später)
    uint16_t msg_id;      // message id (Dedup später)
    char src[7];          // callsign padded with spaces
    char dst[7];          // callsign padded with spaces or "*" target
    uint8_t payload_len;  // n
    // payload follows
} mr_hdr_t;
#pragma pack(pop)

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

static void lora_fifo_write(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        lora_write_reg(REG_FIFO, data[i]);
    }
}

/* ===============================
   LoRa helpers
   =============================== */
static void lora_clear_irqs(void)
{
    lora_write_reg(REG_IRQ_FLAGS, 0xFF);
}

static void lora_reset(void)
{
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
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
    // BW 125kHz, CR 4/5, Explicit header
    lora_write_reg(REG_MODEM_CONFIG_1, 0x72);

    // SF7, CRC on
    lora_write_reg(REG_MODEM_CONFIG_2, 0x74);

    // AGC auto on
    lora_write_reg(REG_MODEM_CONFIG_3, 0x04);
}

static void lora_set_tx_power_dbm_14(void)
{
    // PA_BOOST + ~14dBm (Startwert)
    lora_write_reg(REG_PA_CONFIG, 0x8E);
}

static void lora_set_standby(void)
{
    // LoRa + Standby
    lora_write_reg(REG_OP_MODE, 0x81);
    vTaskDelay(pdMS_TO_TICKS(10));
}

/* TX Start */
static void lora_start_tx(const uint8_t *packet, size_t len)
{
    lora_clear_irqs();

    // TX FIFO base + pointer
    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0x00);
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x00);

    // Packet -> FIFO
    lora_fifo_write(packet, len);

    // Length
    lora_write_reg(REG_PAYLOAD_LENGTH, (uint8_t)len);

    // TX Mode: LoRa + TX
    lora_write_reg(REG_OP_MODE, 0x83);
}

/* Warten auf TxDone (Polling) */
static bool wait_tx_done_polling(int timeout_ms)
{
    while (timeout_ms > 0) {
        uint8_t irq = 0;
        lora_read_reg(REG_IRQ_FLAGS, &irq);

        if (irq & IRQ_TX_DONE) {
            lora_clear_irqs();
            return true;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
        timeout_ms -= 10;
    }
    return false;
}

/* ===============================
   MR Frame builder
   =============================== */
static uint16_t g_msg_id = 1;

static size_t mr_build_frame(uint8_t *out, size_t out_max,
                             const char *src_call,
                             const char *dst_call,
                             uint8_t ttl,
                             const uint8_t *payload,
                             size_t payload_len)
{
    if (out_max < sizeof(mr_hdr_t)) return 0;
    if (payload_len > 200) payload_len = 200;
    if (sizeof(mr_hdr_t) + payload_len > out_max) return 0;

    mr_hdr_t hdr;
    memset(&hdr, 0, sizeof(hdr));

    hdr.magic[0] = 'M';
    hdr.magic[1] = 'R';
    hdr.version  = 1;
    hdr.flags    = 0;
    hdr.ttl      = ttl;
    hdr.msg_id   = g_msg_id++;

    memset(hdr.src, ' ', sizeof(hdr.src));
    memset(hdr.dst, ' ', sizeof(hdr.dst));

    size_t src_len = strlen(src_call);
    if (src_len > 7) src_len = 7;
    memcpy(hdr.src, src_call, src_len);

    size_t dst_len = strlen(dst_call);
    if (dst_len > 7) dst_len = 7;
    memcpy(hdr.dst, dst_call, dst_len);

    hdr.payload_len = (uint8_t)payload_len;

    memcpy(out, &hdr, sizeof(hdr));
    memcpy(out + sizeof(hdr), payload, payload_len);

    return sizeof(hdr) + payload_len;
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
    ESP_LOGI(TAG, "RegVersion=0x%02X (expected 0x12)", version);
    if (version != 0x12) {
        ESP_LOGE(TAG, "No SX1276 -> abort");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    lora_enter_lora_mode_sleep();
    lora_set_frequency_433();
    lora_set_modem_defaults();
    lora_set_tx_power_dbm_14();
    lora_set_standby();

    ESP_LOGI(TAG, "MR TX (polling) started.");

    while (1) {
        const char *payload_txt = "Hallo MeshRadio 4.0 von DJ2RF";
        uint8_t packet[256];

        size_t pkt_len = mr_build_frame(
            packet, sizeof(packet),
            "DJ2RF",   // src
            "*",       // dst (broadcast)
            3,         // ttl
            (const uint8_t*)payload_txt,
            strlen(payload_txt)
        );

        if (pkt_len == 0) {
            ESP_LOGE(TAG, "Frame build failed");
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        uint16_t this_id = (uint16_t)(g_msg_id - 1);
        ESP_LOGI(TAG, "Sende MR Frame: len=%u id=%u", (unsigned)pkt_len, (unsigned)this_id);

        lora_start_tx(packet, pkt_len);

        if (wait_tx_done_polling(2000)) {
            ESP_LOGI(TAG, "TX done ✅ (poll)");
        } else {
            ESP_LOGW(TAG, "TX timeout (kein TxDone)");
            lora_clear_irqs();
            lora_set_standby();
        }

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
