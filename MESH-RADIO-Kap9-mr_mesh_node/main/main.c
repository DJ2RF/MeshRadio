#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_err.h"

/*
    =========================================================
    MeshRadio Node (Flooding Light)
    - RX via DIO0 interrupt
    - TX via polling
    - TTL decrement
    - Dedupe cache
    =========================================================
*/

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

static const char *TAG = "MR_MESH";
static spi_device_handle_t lora_spi = NULL;
static QueueHandle_t dio0_evt_queue;

/* ===============================
   SX1276 Register
   =============================== */
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_FIFO_ADDR_PTR        0x0D
#define REG_FIFO_TX_BASE_ADDR    0x0E
#define REG_FIFO_RX_BASE_ADDR    0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_MODEM_CONFIG_3       0x26
#define REG_VERSION              0x42

#define IRQ_RX_DONE              0x40
#define IRQ_PAYLOAD_CRC_ERROR    0x20
#define IRQ_TX_DONE              0x08

/* ===============================
   MeshRadio Frame
   =============================== */
#pragma pack(push, 1)
typedef struct {
    uint8_t magic[2];
    uint8_t version;
    uint8_t flags;
    uint8_t ttl;
    uint16_t msg_id;
    char src[7];
    char dst[7];
    uint8_t payload_len;
} mr_hdr_t;
#pragma pack(pop)

/* ===============================
   Dedupe Cache
   =============================== */
#define DEDUPE_SIZE 20

typedef struct {
    uint16_t msg_id;
    char src[7];
} dedupe_entry_t;

static dedupe_entry_t dedupe_cache[DEDUPE_SIZE];
static int dedupe_index = 0;

static bool is_duplicate(uint16_t msg_id, char *src)
{
    for (int i = 0; i < DEDUPE_SIZE; i++) {
        if (dedupe_cache[i].msg_id == msg_id &&
            memcmp(dedupe_cache[i].src, src, 7) == 0) {
            return true;
        }
    }
    return false;
}

static void add_to_dedupe(uint16_t msg_id, char *src)
{
    dedupe_cache[dedupe_index].msg_id = msg_id;
    memcpy(dedupe_cache[dedupe_index].src, src, 7);

    dedupe_index++;
    if (dedupe_index >= DEDUPE_SIZE)
        dedupe_index = 0;
}

/* ===============================
   SPI + Register
   =============================== */
static esp_err_t lora_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { reg | 0x80, value };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx };
    return spi_device_transmit(lora_spi, &t);
}

static esp_err_t lora_read_reg(uint8_t reg, uint8_t *value)
{
    uint8_t tx[2] = { reg & 0x7F, 0 };
    uint8_t rx[2] = { 0 };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx, .rx_buffer = rx };
    esp_err_t err = spi_device_transmit(lora_spi, &t);
    if (err == ESP_OK) *value = rx[1];
    return err;
}

/* ===============================
   TX (Polling)
   =============================== */
static bool wait_tx_done(void)
{
    for (int i = 0; i < 200; i++) {
        uint8_t irq;
        lora_read_reg(REG_IRQ_FLAGS, &irq);
        if (irq & IRQ_TX_DONE) {
            lora_write_reg(REG_IRQ_FLAGS, 0xFF);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return false;
}

static void forward_packet(uint8_t *packet, size_t len)
{
    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0x00);
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x00);

    for (size_t i = 0; i < len; i++)
        lora_write_reg(REG_FIFO, packet[i]);

    lora_write_reg(REG_PAYLOAD_LENGTH, len);
    lora_write_reg(REG_OP_MODE, 0x83); // TX

    if (wait_tx_done())
        ESP_LOGI(TAG, "Forwarded.");
    else
        ESP_LOGW(TAG, "Forward timeout.");

    lora_write_reg(REG_OP_MODE, 0x85); // back to RX
}

/* ===============================
   RX Handler
   =============================== */
static void handle_rx(void)
{
    uint8_t irq;
    lora_read_reg(REG_IRQ_FLAGS, &irq);

    if (!(irq & IRQ_RX_DONE))
        return;

    if (irq & IRQ_PAYLOAD_CRC_ERROR) {
        lora_write_reg(REG_IRQ_FLAGS, 0xFF);
        return;
    }

    uint8_t bytes;
    lora_read_reg(REG_RX_NB_BYTES, &bytes);

    uint8_t fifo_addr;
    lora_read_reg(REG_FIFO_RX_CURRENT_ADDR, &fifo_addr);
    lora_write_reg(REG_FIFO_ADDR_PTR, fifo_addr);

    uint8_t buffer[256];
    for (int i = 0; i < bytes; i++)
        lora_read_reg(REG_FIFO, &buffer[i]);

    lora_write_reg(REG_IRQ_FLAGS, 0xFF);

    if (bytes < sizeof(mr_hdr_t))
        return;

    mr_hdr_t *hdr = (mr_hdr_t*)buffer;

    if (hdr->magic[0] != 'M' || hdr->magic[1] != 'R')
        return;

    if (is_duplicate(hdr->msg_id, hdr->src)) {
        ESP_LOGI(TAG, "Duplicate ignored.");
        return;
    }

    add_to_dedupe(hdr->msg_id, hdr->src);

    ESP_LOGI(TAG, "RX from %.7s TTL=%u ID=%u",
             hdr->src, hdr->ttl, hdr->msg_id);

    if (hdr->ttl > 0) {
        hdr->ttl--;
        forward_packet(buffer, bytes);
    }
}

/* ===============================
   ISR
   =============================== */
static void IRAM_ATTR dio0_isr(void *arg)
{
    uint32_t gpio = (uint32_t)arg;
    xQueueSendFromISR(dio0_evt_queue, &gpio, NULL);
}

static void dio0_task(void *arg)
{
    uint32_t io;
    while (1) {
        if (xQueueReceive(dio0_evt_queue, &io, portMAX_DELAY)) {
            handle_rx();
        }
    }
}

/* ===============================
   app_main
   =============================== */
void app_main(void)
{
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
    };

    spi_bus_initialize(LORA_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
    };

    spi_bus_add_device(LORA_SPI_HOST, &devcfg, &lora_spi);

    // Basic LoRa Init (wie in vorherigen Kapiteln)
    lora_write_reg(REG_OP_MODE, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));

    uint32_t frf = 0x6C8000;
    lora_write_reg(REG_FRF_MSB, frf >> 16);
    lora_write_reg(REG_FRF_MID, frf >> 8);
    lora_write_reg(REG_FRF_LSB, frf);

    lora_write_reg(REG_MODEM_CONFIG_1, 0x72);
    lora_write_reg(REG_MODEM_CONFIG_2, 0x74);
    lora_write_reg(REG_MODEM_CONFIG_3, 0x04);

    lora_write_reg(REG_OP_MODE, 0x85); // RX Continuous

    dio0_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIN_NUM_DIO0),
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_NUM_DIO0, dio0_isr, (void*)PIN_NUM_DIO0);

    xTaskCreate(dio0_task, "dio0_task", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "Mesh node running.");
}
