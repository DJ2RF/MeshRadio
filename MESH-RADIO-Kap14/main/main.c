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
S
// ============================================================================
// MeshRadio 4.0 – Kapitel 14
// ----------------------------------------------------------------------------
// SMART FORWARDING (RSSI-basiertes Flooding)
//
// Erweiterung zu Kapitel 13:
//
//   ✔ Flooding bleibt erhalten
//   ✔ Duplicate Detection bleibt erhalten
//   ✔ NEU: Weiterleitung nur bei ausreichendem RSSI
//   ✔ NEU: Zufalls-Delay gegen Kollisionen
//
// Ziel:
//   Weniger Airtime, stabileres Mesh, weniger unnötige Retransmits.
//
// ESP-IDF: v5.5.x
// Hardware: ESP32 + SX1276 (TTGO T-Beam etc.)
// ============================================================================

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_random.h"

// ============================================================================
// KONFIGURATION
// ============================================================================

#define MY_CALL "DJ2RF"

#define BEACON_INTERVAL_S 10
#define SEEN_CACHE_SIZE   32

// Smart Forwarding Schwelle (433 MHz typisch)
#define FORWARD_MIN_RSSI  (-105)

// optionaler Random Delay vor Forward (ms)
#define FORWARD_RANDOM_DELAY_MAX 50

// Frame Flags
#define MR_FLAG_BEACON 0x04
#define MR_FLAG_CHAT   0x08

// ============================================================================
// LoRa Pins (TTGO T-Beam V1.1)
// ============================================================================

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 27
#define PIN_NUM_CLK  5
#define PIN_NUM_CS   18
#define PIN_NUM_RST  23
#define PIN_NUM_DIO0 26

#define LORA_SPI_HOST VSPI_HOST

// ============================================================================
// SX1276 Register
// ============================================================================

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
#define REG_PKT_RSSI_VALUE       0x1A
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_MODEM_CONFIG_3       0x26
#define REG_VERSION              0x42

#define IRQ_RX_DONE 0x40
#define IRQ_TX_DONE 0x08

// ============================================================================
// MeshRadio Frame Header
// ============================================================================

#pragma pack(push,1)
typedef struct {
    uint8_t  magic[2];      // "MR"
    uint8_t  version;
    uint8_t  flags;
    uint8_t  ttl;
    uint16_t msg_id;
    char     src[7];
    char     dst[7];
    uint8_t  payload_len;
} mr_hdr_t;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct {
    mr_hdr_t h;
    char payload[64];
} mr_chat_frame_t;
#pragma pack(pop)

// ============================================================================
// Duplicate Cache
// ============================================================================

typedef struct {
    bool used;
    char src[7];
    uint16_t msg_id;
} seen_msg_t;

static seen_msg_t seen_cache[SEEN_CACHE_SIZE];

// ============================================================================
// Globals
// ============================================================================

static const char *TAG = "MR14";

static spi_device_handle_t lora_spi;
static QueueHandle_t dio0_evt_queue;

static uint16_t g_msg_id = 1;

// UART Chat Input
static char uart_line[80];
static int uart_pos = 0;

// ============================================================================
// Helper
// ============================================================================

static void call7_set(char out7[7], const char *call)
{
    memset(out7, ' ', 7);
    size_t n = strlen(call);
    if (n > 7) n = 7;
    memcpy(out7, call, n);
}

static void call7_to_cstr(char out8[8], const char in7[7])
{
    memcpy(out8, in7, 7);
    out8[7] = 0;
}

// ============================================================================
// Duplicate Detection
// ============================================================================

static bool seen_before(const char src[7], uint16_t id)
{
    for (int i = 0; i < SEEN_CACHE_SIZE; i++) {
        if (!seen_cache[i].used) continue;

        if (memcmp(seen_cache[i].src, src, 7) == 0 &&
            seen_cache[i].msg_id == id)
            return true;
    }
    return false;
}

static void remember_msg(const char src[7], uint16_t id)
{
    static int idx = 0;

    memcpy(seen_cache[idx].src, src, 7);
    seen_cache[idx].msg_id = id;
    seen_cache[idx].used = true;

    idx = (idx + 1) % SEEN_CACHE_SIZE;
}

// ============================================================================
// SPI Register Zugriff
// ============================================================================

static void lora_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), val };
    spi_transaction_t t = {.length = 16, .tx_buffer = tx};
    spi_device_transmit(lora_spi, &t);
}

static uint8_t lora_read_reg(uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), 0 };
    uint8_t rx[2] = {0};

    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx
    };

    spi_device_transmit(lora_spi, &t);
    return rx[1];
}

static void lora_clear_irqs(void)
{
    lora_write_reg(REG_IRQ_FLAGS, 0xFF);
}

// ============================================================================
// LoRa Init
// ============================================================================

static void lora_reset(void)
{
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void lora_init(void)
{
    lora_write_reg(REG_OP_MODE, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 433 MHz
    uint32_t frf = 0x6C8000;
    lora_write_reg(REG_FRF_MSB, frf >> 16);
    lora_write_reg(REG_FRF_MID, frf >> 8);
    lora_write_reg(REG_FRF_LSB, frf);

    // BW125 / SF7 / CRC
    lora_write_reg(REG_MODEM_CONFIG_1, 0x72);
    lora_write_reg(REG_MODEM_CONFIG_2, 0x74);
    lora_write_reg(REG_MODEM_CONFIG_3, 0x04);

    // TX Power
    lora_write_reg(REG_PA_CONFIG, 0x8E);

    // RX continuous
    lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0);
    lora_write_reg(REG_FIFO_ADDR_PTR, 0);
    lora_write_reg(REG_OP_MODE, 0x85);
}

// ============================================================================
// TX
// ============================================================================

static bool lora_wait_tx_done_polling(int timeout_ms)
{
    while (timeout_ms > 0) {
        uint8_t irq = lora_read_reg(REG_IRQ_FLAGS);

        if (irq & IRQ_TX_DONE) {
            lora_clear_irqs();
            return true;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
        timeout_ms -= 10;
    }
    return false;
}

static void lora_send_packet(uint8_t *data, size_t len)
{
    lora_clear_irqs();

    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0);
    lora_write_reg(REG_FIFO_ADDR_PTR, 0);

    for (size_t i = 0; i < len; i++)
        lora_write_reg(REG_FIFO, data[i]);

    lora_write_reg(REG_PAYLOAD_LENGTH, (uint8_t)len);
    lora_write_reg(REG_OP_MODE, 0x83);

    lora_wait_tx_done_polling(2000);

    // zurück in RX
    lora_write_reg(REG_OP_MODE, 0x85);
}

// ============================================================================
// SMART FORWARDING (Kapitel 14 Kern)
// ============================================================================

static void forward_packet(uint8_t *buf, size_t len, int rssi_dbm)
{
    mr_hdr_t *h = (mr_hdr_t*)buf;

    // TTL abgelaufen?
    if (h->ttl <= 1)
        return;

    // RSSI zu schwach → NICHT weiterleiten
    if (rssi_dbm < FORWARD_MIN_RSSI) {
        ESP_LOGI(TAG, "DROP weak link RSSI=%d", rssi_dbm);
        return;
    }

    // optionaler Random Delay gegen gleichzeitiges Senden
    int delay = esp_random() % FORWARD_RANDOM_DELAY_MAX;
    vTaskDelay(pdMS_TO_TICKS(delay));

    h->ttl--;

    ESP_LOGI(TAG,
             "SMART FWD msg=%d ttl=%d RSSI=%d",
             h->msg_id, h->ttl, rssi_dbm);

    lora_send_packet(buf, len);
}

// ============================================================================
// Beacon
// ============================================================================

static void send_beacon(void)
{
    mr_hdr_t h = {0};

    h.magic[0] = 'M';
    h.magic[1] = 'R';
    h.version  = 1;
    h.flags    = MR_FLAG_BEACON;
    h.ttl      = 1;
    h.msg_id   = g_msg_id++;

    call7_set(h.src, MY_CALL);
    call7_set(h.dst, "*");

    lora_send_packet((uint8_t*)&h, sizeof(h));
}

// ============================================================================
// Chat TX
// ============================================================================

static void send_chat_message(const char *text)
{
    mr_chat_frame_t f = {0};

    f.h.magic[0] = 'M';
    f.h.magic[1] = 'R';
    f.h.version  = 1;
    f.h.flags    = MR_FLAG_CHAT;
    f.h.ttl      = 3;      // max 3 Hops
    f.h.msg_id   = g_msg_id++;

    call7_set(f.h.src, MY_CALL);
    call7_set(f.h.dst, "*");

    size_t len = strlen(text);
    if (len > 64) len = 64;

    memcpy(f.payload, text, len);
    f.h.payload_len = (uint8_t)len;

    ESP_LOGI(TAG, "TX CHAT: %s", text);

    lora_send_packet((uint8_t*)&f, sizeof(mr_hdr_t) + len);
}

// ============================================================================
// UART Input
// ============================================================================

static void uart_poll_input(void)
{
    int c = getchar();
    if (c == EOF) return;

    if (c == '\n' || c == '\r') {
        uart_line[uart_pos] = 0;
        if (uart_pos > 0)
            send_chat_message(uart_line);
        uart_pos = 0;
        return;
    }

    if (uart_pos < (int)sizeof(uart_line)-1)
        uart_line[uart_pos++] = (char)c;
}

// ============================================================================
// RX Handler
// ============================================================================

static void handle_rx_packet(void)
{
    uint8_t irq = lora_read_reg(REG_IRQ_FLAGS);
    if (!(irq & IRQ_RX_DONE)) return;

    uint8_t len = lora_read_reg(REG_RX_NB_BYTES);

    uint8_t addr = lora_read_reg(REG_FIFO_RX_CURRENT_ADDR);
    lora_write_reg(REG_FIFO_ADDR_PTR, addr);

    uint8_t buf[256];
    for (int i = 0; i < len; i++)
        buf[i] = lora_read_reg(REG_FIFO);

    lora_clear_irqs();

    if (len < sizeof(mr_hdr_t)) return;

    mr_hdr_t *h = (mr_hdr_t*)buf;

    // Duplicate Filter
    if (seen_before(h->src, h->msg_id)) {
        ESP_LOGI(TAG, "Duplicate -> ignore");
        return;
    }

    remember_msg(h->src, h->msg_id);

    // RSSI berechnen (433 MHz)
    int rssi_dbm = (int)lora_read_reg(REG_PKT_RSSI_VALUE) - 157;

    // Chat anzeigen
    if (h->flags & MR_FLAG_CHAT) {

        mr_chat_frame_t *c = (mr_chat_frame_t*)buf;

        char src[8];
        call7_to_cstr(src, c->h.src);

        char msg[65];
        memcpy(msg, c->payload, c->h.payload_len);
        msg[c->h.payload_len] = 0;

        ESP_LOGI(TAG,
                 "CHAT %.7s > %s (RSSI=%d)",
                 src, msg, rssi_dbm);
    }

    // Smart Forward
    forward_packet(buf, len, rssi_dbm);
}

// ============================================================================
// ISR + Task
// ============================================================================

static void IRAM_ATTR dio0_isr(void *arg)
{
    uint32_t n = (uint32_t)arg;
    xQueueSendFromISR(dio0_evt_queue, &n, NULL);
}

static void dio0_task(void *arg)
{
    uint32_t io;
    while (1) {
        if (xQueueReceive(dio0_evt_queue, &io, portMAX_DELAY))
            handle_rx_packet();
    }
}

// ============================================================================
// SPI Init
// ============================================================================

static void init_spi(void)
{
    spi_bus_config_t bus = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK
    };

    spi_bus_initialize(LORA_SPI_HOST, &bus, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t dev = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1
    };

    spi_bus_add_device(LORA_SPI_HOST, &dev, &lora_spi);
}

// ============================================================================
// MAIN
// ============================================================================

void app_main(void)
{
    ESP_LOGI(TAG, "Kapitel 14 start (%s)", MY_CALL);

    init_spi();

    lora_reset();

    uint8_t v = lora_read_reg(REG_VERSION);
    ESP_LOGI(TAG, "SX1276 Version=0x%02X", v);

    lora_init();

    dio0_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    gpio_config_t io = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIN_NUM_DIO0),
        .pull_up_en = 1
    };
    gpio_config(&io);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_NUM_DIO0, dio0_isr, (void*)PIN_NUM_DIO0);

    xTaskCreate(dio0_task, "dio0", 4096, NULL, 10, NULL);

    int beacon_t = 0;

    while (1) {

        vTaskDelay(pdMS_TO_TICKS(50));

        uart_poll_input();

        beacon_t++;
        if (beacon_t >= BEACON_INTERVAL_S * 20) {
            send_beacon();
            beacon_t = 0;
        }
    }
}
