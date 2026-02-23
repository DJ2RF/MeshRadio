#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_err.h"

/* ===============================
   Pins TTGO T-Beam V1.1
   =============================== */
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 27
#define PIN_NUM_CLK  5
#define PIN_NUM_CS   18
#define PIN_NUM_RST  23
#define PIN_NUM_DIO0 26

#define LORA_SPI_HOST VSPI_HOST

static const char *TAG = "MR_RX_INT";
static spi_device_handle_t lora_spi = NULL;

/* ===============================
   SX1276 Register
   =============================== */
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_FIFO_ADDR_PTR        0x0D
#define REG_FIFO_RX_BASE_ADDR    0x0F
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_MODEM_CONFIG_3       0x26
#define REG_VERSION              0x42
#define REG_DIO_MAPPING_1        0x40

#define IRQ_RX_DONE              0x40
#define IRQ_PAYLOAD_CRC_ERROR    0x20

/* ===============================
   MeshRadio Frame v1
   =============================== */
#pragma pack(push, 1)
typedef struct {
    uint8_t magic[2];     // 'M','R'
    uint8_t version;      // 1
    uint8_t flags;        // reserved
    uint8_t ttl;          // hop limit
    uint16_t msg_id;      // message id
    char src[7];          // callsign padded
    char dst[7];          // callsign padded
    uint8_t payload_len;  // n
    // payload follows
} mr_hdr_t;
#pragma pack(pop)

/* ===============================
   IRQ handling (ISR -> Queue)
   =============================== */
static QueueHandle_t dio0_evt_queue;

static void IRAM_ATTR dio0_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(dio0_evt_queue, &gpio_num, NULL);
}

/* ===============================
   SPI + Register access
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

/* FIFO lesen */
static void lora_fifo_read(uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        lora_read_reg(REG_FIFO, &buf[i]);
    }
}

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
    lora_write_reg(REG_OP_MODE, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void lora_set_frequency_433(void)
{
    uint32_t frf = 0x6C8000;
    lora_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    lora_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
    lora_write_reg(REG_FRF_LSB, (uint8_t)(frf));
}

static void lora_set_modem_defaults(void)
{
    lora_write_reg(REG_MODEM_CONFIG_1, 0x72);
    lora_write_reg(REG_MODEM_CONFIG_2, 0x74);
    lora_write_reg(REG_MODEM_CONFIG_3, 0x04);
}

static void lora_set_standby(void)
{
    lora_write_reg(REG_OP_MODE, 0x81);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void lora_rx_continuous(void)
{
    lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0x00);
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x00);

    // DIO0 mapping: 00 = RxDone (in RX mode)
    lora_write_reg(REG_DIO_MAPPING_1, 0x00);

    // RX Continuous: LoRa + RXCONT
    lora_write_reg(REG_OP_MODE, 0x85);
}

/* ===============================
   Paket verarbeiten
   =============================== */
static void handle_rx_packet(void)
{
    uint8_t irq = 0;
    lora_read_reg(REG_IRQ_FLAGS, &irq);

    if (!(irq & IRQ_RX_DONE)) {
        // spurious interrupt
        return;
    }

    if (irq & IRQ_PAYLOAD_CRC_ERROR) {
        ESP_LOGW(TAG, "RX CRC error (IRQ=0x%02X)", irq);
        lora_clear_irqs();
        return;
    }

    uint8_t bytes = 0;
    lora_read_reg(REG_RX_NB_BYTES, &bytes);

    uint8_t fifo_current = 0;
    lora_read_reg(REG_FIFO_RX_CURRENT_ADDR, &fifo_current);

    lora_write_reg(REG_FIFO_ADDR_PTR, fifo_current);

    uint8_t buf[256];
    if (bytes > sizeof(buf)) bytes = sizeof(buf);

    lora_fifo_read(buf, bytes);

    lora_clear_irqs();

    if (bytes < sizeof(mr_hdr_t)) {
        ESP_LOGW(TAG, "RX packet too short (%u bytes)", bytes);
        return;
    }

    mr_hdr_t *hdr = (mr_hdr_t*)buf;

    if (hdr->magic[0] != 'M' || hdr->magic[1] != 'R') {
        ESP_LOGW(TAG, "RX unknown packet (no MR magic)");
        return;
    }

    if (hdr->version != 1) {
        ESP_LOGW(TAG, "RX version mismatch: %u", hdr->version);
        return;
    }

    uint8_t plen = hdr->payload_len;
    size_t header_len = sizeof(mr_hdr_t);
    if (header_len + plen > bytes) {
        ESP_LOGW(TAG, "RX length invalid (hdr=%u, pkt=%u)", (unsigned)header_len, bytes);
        return;
    }

    char src[8] = {0};
    char dst[8] = {0};
    memcpy(src, hdr->src, 7);
    memcpy(dst, hdr->dst, 7);

    // Payload als Text (für Test)
    char payload[200];
    if (plen >= sizeof(payload)) plen = sizeof(payload) - 1;
    memcpy(payload, buf + header_len, plen);
    payload[plen] = 0;

    ESP_LOGI(TAG, "MR frame: src='%s' dst='%s' ttl=%u id=%u len=%u",
             src, dst, hdr->ttl, hdr->msg_id, hdr->payload_len);

    ESP_LOGI(TAG, "Payload: \"%s\"", payload);
}

/* ===============================
   DIO0 Task
   =============================== */
static void dio0_task(void *arg)
{
    uint32_t io_num;
    while (1) {
        if (xQueueReceive(dio0_evt_queue, &io_num, portMAX_DELAY)) {
            // DIO0 fired -> check IRQ flags and handle RX
            handle_rx_packet();

            // Ensure we remain in RX
            lora_rx_continuous();
        }
    }
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
    lora_set_standby();

    // Queue + ISR
    dio0_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIN_NUM_DIO0),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_NUM_DIO0, dio0_isr_handler, (void*)PIN_NUM_DIO0);

    // Start RX
    lora_clear_irqs();
    lora_rx_continuous();

    // Task that processes packets
    xTaskCreate(dio0_task, "dio0_task", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "RX via DIO0 interrupt started.");
}
