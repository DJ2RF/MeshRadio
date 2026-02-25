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
// https://nerdverlag.com// ============================================================================
// MeshRadio – Kapitel 15 + 15A (KORRIGIERT)
// ----------------------------------------------------------------------------
// Fix für deinen Compiler-Fehler:
//   forward_packet() ruft lora_send_packet() auf, bevor die Funktion bekannt ist.
//   -> Lösung: Forward Declarations (Prototypen) vor der ersten Nutzung.
//
// Zusätzlich (wichtig):
//   In der vorherigen "integrierten" Fassung fehlten UART-Chat (poll + send)
//   und Beacon-Senden im Main-Loop (hatte ich versehentlich rausgekürzt).
//   -> Beides ist hier wieder sauber drin.
//
// Inhalt:
//   ✔ Duplicate Detection (Seen Cache)
//   ✔ Neighbor Tabelle (RSSI + last_seen) + Cleanup
//   ✔ Smart Forwarding (RSSI-Gate)
//   ✔ Preferred Forwarding (Top-N Ranking)
//   ✔ UART Chat (Eingabe über PuTTY/Monitor)
//   ✔ 15A: UART Statusausgabe (Neighbors + Preferred Markierung)
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

// Eigenes Rufzeichen (pro Node anpassen)
#define MY_CALL "DJ1ABC"

// Beacon Timing
#define BEACON_INTERVAL_S 10

// Duplicate Cache
#define SEEN_CACHE_SIZE 32

// Neighbor Tabelle
#define MAX_NEIGHBORS          20
#define NEIGHBOR_TIMEOUT_MS    60000   // Nachbar nach 60s ohne RX "vergessen"

// Preferred Forwarding
#define PREFERRED_TOP_N        3       // nur Top-N RSSI Nachbarn forwarden

// RSSI Gate (433 MHz)
#define FORWARD_MIN_RSSI       (-105)

// Random Delay vor Forward (Kollisionsreduktion)
#define FORWARD_RANDOM_DELAY_MAX 50

// 15A – UART Statusintervall (Sekunden, 0 = aus)
#define UART_STATUS_INTERVAL_S 5

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
// Frame Definition
// ============================================================================

#pragma pack(push,1)
typedef struct {
    uint8_t  magic[2];      // "MR"
    uint8_t  version;
    uint8_t  flags;
    uint8_t  ttl;           // Hop-Limit
    uint16_t msg_id;        // Message-ID (pro SRC eindeutig)
    char     src[7];        // Source Callsign (7 chars padded)
    char     dst[7];        // Destination (für später)
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
// Datenstrukturen: Seen Cache + Neighbor Tabelle
// ============================================================================

typedef struct {
    bool used;
    char src[7];
    uint16_t msg_id;
} seen_msg_t;

typedef struct {
    bool used;
    char call[7];
    int  rssi_dbm;
    uint32_t last_seen_ms;
} neighbor_t;

// ============================================================================
// Globals
// ============================================================================

static const char *TAG = "MR15A_FIX";

static spi_device_handle_t lora_spi;
static QueueHandle_t dio0_evt_queue;

static uint16_t g_msg_id = 1;

static seen_msg_t seen_cache[SEEN_CACHE_SIZE];
static neighbor_t neighbors[MAX_NEIGHBORS];

// UART line input buffer
static char uart_line[80];
static int  uart_pos = 0;

// ============================================================================
// WICHTIG: Forward Declarations (Prototypen)
// ----------------------------------------------------------------------------
// C braucht Funktionssignaturen bevor eine Funktion benutzt wird.
// Das verhindert "implicit declaration" und "static follows non-static" Fehler.
// ============================================================================
static void lora_send_packet(uint8_t *data, size_t len);
static bool lora_wait_tx_done_polling(int timeout_ms);
static void forward_packet(uint8_t *buf, size_t len, int rssi_dbm);

// ============================================================================
// Helper
// ============================================================================

static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

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
// Duplicate Detection (Seen Cache)
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
// Neighbor Tabelle
// ============================================================================

static void neighbor_update(const char call7[7], int rssi_dbm)
{
    uint32_t t = now_ms();

    // Update wenn vorhanden
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!neighbors[i].used) continue;

        if (memcmp(neighbors[i].call, call7, 7) == 0) {
            neighbors[i].rssi_dbm = rssi_dbm;
            neighbors[i].last_seen_ms = t;
            return;
        }
    }

    // Neuen Slot belegen
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!neighbors[i].used) {
            neighbors[i].used = true;
            memcpy(neighbors[i].call, call7, 7);
            neighbors[i].rssi_dbm = rssi_dbm;
            neighbors[i].last_seen_ms = t;
            return;
        }
    }
}

static void neighbor_cleanup(void)
{
    uint32_t t = now_ms();

    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!neighbors[i].used) continue;

        if ((t - neighbors[i].last_seen_ms) > NEIGHBOR_TIMEOUT_MS) {
            neighbors[i].used = false;
        }
    }
}

// Preferred = gehört zu den Top-N stärksten Nachbarn
static bool neighbor_is_preferred(const char call7[7])
{
    int my_rssi = -9999;
    int better = 0;

    // meinen RSSI finden
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!neighbors[i].used) continue;

        if (memcmp(neighbors[i].call, call7, 7) == 0) {
            my_rssi = neighbors[i].rssi_dbm;
            break;
        }
    }

    if (my_rssi == -9999)
        return false;

    // zählen, wie viele Nachbarn stärker sind
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!neighbors[i].used) continue;

        if (neighbors[i].rssi_dbm > my_rssi)
            better++;
    }

    return (better < PREFERRED_TOP_N);
}

// ============================================================================
// 15A – UART Statusanzeige (Neighbors + Preferred Markierung)
// ----------------------------------------------------------------------------
// Ausgabe wie:
//   [P] DL1XYZ RSSI=-78
//   [ ] OE3AAA RSSI=-110
// ============================================================================
static void uart_show_neighbors_status(void)
{
    ESP_LOGI(TAG, "----------------------------");
    ESP_LOGI(TAG, "Mesh Status CALL=%s (TopN=%d, MinRSSI=%d)",
             MY_CALL, PREFERRED_TOP_N, FORWARD_MIN_RSSI);

    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!neighbors[i].used) continue;

        char call8[8];
        call7_to_cstr(call8, neighbors[i].call);

        bool pref = neighbor_is_preferred(neighbors[i].call);

        ESP_LOGI(TAG, "[%c] %.7s RSSI=%4d",
                 pref ? 'P' : ' ',
                 call8,
                 neighbors[i].rssi_dbm);
    }

    ESP_LOGI(TAG, "----------------------------");
}

// ============================================================================
// SPI Register Zugriff
// ============================================================================

static void lora_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), val };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx };
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
    // Sleep + LoRa
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
// TX – Polling (ohne DIO0 TX Done)
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

    // TX mode
    lora_write_reg(REG_OP_MODE, 0x83);

    // warten bis TX done (poll)
    lora_wait_tx_done_polling(2000);

    // zurück in RX continuous
    lora_write_reg(REG_OP_MODE, 0x85);
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
// UART Chat: Eingabe lesen und Nachricht senden
// ============================================================================

static void send_chat_message(const char *text)
{
    mr_chat_frame_t f = {0};

    f.h.magic[0] = 'M';
    f.h.magic[1] = 'R';
    f.h.version  = 1;
    f.h.flags    = MR_FLAG_CHAT;
    f.h.ttl      = 3;               // max 3 Hops
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

    if (uart_pos < (int)sizeof(uart_line) - 1) {
        uart_line[uart_pos++] = (char)c;
    }
}

// ============================================================================
// Preferred Forwarding (Kapitel 15 Kern)
// ============================================================================

static void forward_packet(uint8_t *buf, size_t len, int rssi_dbm)
{
    mr_hdr_t *h = (mr_hdr_t*)buf;

    // TTL abgelaufen?
    if (h->ttl <= 1)
        return;

    // RSSI Gate
    if (rssi_dbm < FORWARD_MIN_RSSI)
        return;

    // Preferred Forwarding: nur Top-N
    if (!neighbor_is_preferred(h->src))
        return;

    // Random Delay gegen Kollisionen
    int delay = esp_random() % FORWARD_RANDOM_DELAY_MAX;
    vTaskDelay(pdMS_TO_TICKS(delay));

    h->ttl--;

    ESP_LOGI(TAG, "PREF FWD msg=%d ttl=%d RSSI=%d",
             h->msg_id, h->ttl, rssi_dbm);

    lora_send_packet(buf, len);
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
    if (seen_before(h->src, h->msg_id))
        return;

    remember_msg(h->src, h->msg_id);

    // RSSI (433 MHz Offset)
    int rssi_dbm = (int)lora_read_reg(REG_PKT_RSSI_VALUE) - 157;

    // Neighbor Tabelle aktualisieren
    neighbor_update(h->src, rssi_dbm);

    // Chat anzeigen
    if (h->flags & MR_FLAG_CHAT) {
        mr_chat_frame_t *c = (mr_chat_frame_t*)buf;

        char src8[8];
        call7_to_cstr(src8, c->h.src);

        char msg[65];
        memcpy(msg, c->payload, c->h.payload_len);
        msg[c->h.payload_len] = 0;

        ESP_LOGI(TAG, "CHAT %.7s > %s (RSSI=%d)",
                 src8, msg, rssi_dbm);
    }

    // Preferred Forwarding
    forward_packet(buf, len, rssi_dbm);
}

// ============================================================================
// DIO0 ISR + Task
// ============================================================================

static void IRAM_ATTR dio0_isr(void *arg)
{
    uint32_t n = (uint32_t)arg;
    xQueueSendFromISR(dio0_evt_queue, &n, NULL);
}

static void dio0_task(void *arg)
{
    uint32_t io;
    (void)arg;

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
    ESP_LOGI(TAG, "Kapitel 15+15A start (%s)", MY_CALL);

    // SPI + LoRa
    init_spi();
    lora_reset();

    uint8_t v = lora_read_reg(REG_VERSION);
    ESP_LOGI(TAG, "SX1276 Version=0x%02X", v);

    lora_init();

    // DIO0 IRQ Queue + GPIO
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

    // Timing
    uint32_t last_beacon_ms = now_ms();
    uint32_t last_uart_ms   = now_ms();

    while (1) {

        vTaskDelay(pdMS_TO_TICKS(50));

        // UART Chat Eingabe (PuTTY / Monitor)
        uart_poll_input();

        // Beacon
        if ((now_ms() - last_beacon_ms) >= (BEACON_INTERVAL_S * 1000)) {
            send_beacon();
            last_beacon_ms = now_ms();
        }

        // Neighbor Cleanup
        neighbor_cleanup();

        // 15A Statusausgabe
        if (UART_STATUS_INTERVAL_S > 0 &&
            (now_ms() - last_uart_ms) >= (UART_STATUS_INTERVAL_S * 1000)) {

            uart_show_neighbors_status();
            last_uart_ms = now_ms();
        }
    }
}
