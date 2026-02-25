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
// ============================================================================
// MeshRadio – Kapitel 16
// ----------------------------------------------------------------------------
// STORE-AND-FORWARD (Outbox + TX-Worker)
//
// Basierend auf Kapitel 15 (+15A):
//   ✔ Duplicate Detection (Seen Cache)
//   ✔ Neighbor Ranking + Preferred Forwarding (Top-N)
//   ✔ RSSI Gate
//   ✔ UART Chat
//   ✔ 15A UART Statusausgabe (optional)
//
// NEU in Kapitel 16:
//   ✔ Outbox (RAM-Puffer) für zu sendende Frames
//   ✔ TX-Worker Task, der Outbox abarbeitet
//   ✔ Retry + Backoff + Expire (Store-and-Forward)
//   ✔ Forwarding erfolgt NICHT mehr sofort im RX-Handler,
//     sondern wird als Job in die Outbox gelegt.
//
// Wichtige Idee:
//   RX-Handler entscheidet "darf/soll forwarded werden?"
//   -> legt Frame als Job ab
//   TX-Worker sendet zeitlich entkoppelt (weniger Kollisionen).
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
#include "freertos/semphr.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_random.h"

// ============================================================================
// KONFIGURATION
// ============================================================================

#define MY_CALL "DJ2RF"

// Beacon
#define BEACON_INTERVAL_S 10

// Duplicate Cache
#define SEEN_CACHE_SIZE 32

// Neighbor Tabelle
#define MAX_NEIGHBORS          20
#define NEIGHBOR_TIMEOUT_MS    60000

// Preferred Forwarding
#define PREFERRED_TOP_N        3

// RSSI Gate
#define FORWARD_MIN_RSSI       (-105)

// 15A – UART Statusausgabe (Sekunden, 0=aus)
#define UART_STATUS_INTERVAL_S 5

// ----------------------------------------------------------------------------
// Store-and-Forward / Outbox Settings (Kapitel 16)
// ----------------------------------------------------------------------------

// Anzahl Jobs im RAM
#define OUTBOX_SIZE            16

// Maximale Payload im Chat Frame
#define MR_MAX_PAYLOAD         64

// Maximale Framegröße, die wir puffern (Header + Payload)
#define MR_MAX_FRAME_BYTES     (sizeof(mr_hdr_t) + MR_MAX_PAYLOAD)

// Retry-Policy
#define OUTBOX_MAX_RETRIES     5

// Backoff Timing
#define OUTBOX_BACKOFF_BASE_MS 800      // 0.8s
#define OUTBOX_BACKOFF_MAX_MS  10000    // 10s
#define OUTBOX_JITTER_MS       200      // +/- jitter

// Ablaufzeit (danach Job verwerfen)
#define OUTBOX_EXPIRE_MS       60000    // 60s

// TX-Worker Poll Intervall
#define TX_WORKER_TICK_MS      50

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
    char payload[MR_MAX_PAYLOAD];
} mr_chat_frame_t;
#pragma pack(pop)

// ============================================================================
// Duplicate Cache + Neighbor Table
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
// Kapitel 16: Outbox (Store-and-Forward)
// ----------------------------------------------------------------------------
// Wir puffern komplette Frames (bytes + len) und Meta-Daten für Retries.
// ============================================================================
typedef enum {
    JOB_KIND_LOCAL_CHAT = 0,
    JOB_KIND_FORWARD    = 1,
    JOB_KIND_BEACON     = 2
} outbox_kind_t;

typedef struct {
    bool used;

    outbox_kind_t kind;

    // Frame Bytes
    uint8_t  frame[MR_MAX_FRAME_BYTES];
    uint16_t frame_len;

    // Retry / Timing
    uint8_t  retries_left;
    uint32_t next_try_ms;
    uint32_t expires_ms;

    // Debug
    uint16_t msg_id;
    char     src[7]; // für Logs
} outbox_job_t;

// ============================================================================
// Globals
// ============================================================================

static const char *TAG = "MR16";

static spi_device_handle_t lora_spi;
static QueueHandle_t dio0_evt_queue;

static uint16_t g_msg_id = 1;

static seen_msg_t  seen_cache[SEEN_CACHE_SIZE];
static neighbor_t  neighbors[MAX_NEIGHBORS];

// UART input buffer
static char uart_line[80];
static int  uart_pos = 0;

// Outbox
static outbox_job_t outbox[OUTBOX_SIZE];
static SemaphoreHandle_t outbox_mutex;

// ============================================================================
// Forward Declarations (Prototypen)
// ============================================================================
static void lora_send_packet(uint8_t *data, size_t len);
static bool lora_wait_tx_done_polling(int timeout_ms);

static void tx_worker_task(void *arg);

static void outbox_init(void);
static bool outbox_enqueue(outbox_kind_t kind, const uint8_t *frame, uint16_t len,
                           uint16_t msg_id, const char src7[7]);
static void outbox_process_due_jobs(void);

// ============================================================================
// Helper
// ============================================================================

static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static int clamp_i(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
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
// Neighbor Table
// ============================================================================

static void neighbor_update(const char call7[7], int rssi_dbm)
{
    uint32_t t = now_ms();

    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!neighbors[i].used) continue;

        if (memcmp(neighbors[i].call, call7, 7) == 0) {
            neighbors[i].rssi_dbm = rssi_dbm;
            neighbors[i].last_seen_ms = t;
            return;
        }
    }

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

        if ((t - neighbors[i].last_seen_ms) > NEIGHBOR_TIMEOUT_MS)
            neighbors[i].used = false;
    }
}

// Preferred = gehört zu den Top-N stärksten Nachbarn
static bool neighbor_is_preferred(const char call7[7])
{
    int my_rssi = -9999;
    int better = 0;

    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!neighbors[i].used) continue;

        if (memcmp(neighbors[i].call, call7, 7) == 0) {
            my_rssi = neighbors[i].rssi_dbm;
            break;
        }
    }

    if (my_rssi == -9999)
        return false;

    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!neighbors[i].used) continue;

        if (neighbors[i].rssi_dbm > my_rssi)
            better++;
    }

    return (better < PREFERRED_TOP_N);
}

// ============================================================================
// 15A – UART Statusausgabe
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
// TX – Polling (ohne TX-DIO0)
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

    // warten bis TX done
    lora_wait_tx_done_polling(2000);

    // zurück in RX
    lora_write_reg(REG_OP_MODE, 0x85);
}

// ============================================================================
// Kapitel 16: Outbox Implementation
// ============================================================================

static void outbox_init(void)
{
    outbox_mutex = xSemaphoreCreateMutex();
    memset(outbox, 0, sizeof(outbox));
}

// Einfacher Enqueue:
// - sucht freien Slot
// - kopiert Frame bytes
// - setzt Retries + Timing
static bool outbox_enqueue(outbox_kind_t kind, const uint8_t *frame, uint16_t len,
                           uint16_t msg_id, const char src7[7])
{
    if (len == 0 || len > MR_MAX_FRAME_BYTES)
        return false;

    if (xSemaphoreTake(outbox_mutex, pdMS_TO_TICKS(200)) != pdTRUE)
        return false;

    int slot = -1;
    for (int i = 0; i < OUTBOX_SIZE; i++) {
        if (!outbox[i].used) { slot = i; break; }
    }

    if (slot < 0) {
        xSemaphoreGive(outbox_mutex);
        ESP_LOGW(TAG, "OUTBOX full -> drop msg_id=%u", (unsigned)msg_id);
        return false;
    }

    outbox[slot].used = true;
    outbox[slot].kind = kind;
    memcpy(outbox[slot].frame, frame, len);
    outbox[slot].frame_len = len;

    outbox[slot].retries_left = OUTBOX_MAX_RETRIES;

    uint32_t t = now_ms();
    outbox[slot].next_try_ms = t;                 // sofort (TX-Worker entkoppelt trotzdem)
    outbox[slot].expires_ms  = t + OUTBOX_EXPIRE_MS;

    outbox[slot].msg_id = msg_id;
    memcpy(outbox[slot].src, src7, 7);

    xSemaphoreGive(outbox_mutex);

    char src8[8]; call7_to_cstr(src8, src7);
    ESP_LOGI(TAG, "OUTBOX add kind=%d id=%u src=%.7s len=%u",
             (int)kind, (unsigned)msg_id, src8, (unsigned)len);

    return true;
}

// Backoff berechnen (exponentiell + jitter)
static uint32_t outbox_calc_backoff_ms(uint8_t retries_left)
{
    // retries_left zählt runter; für Backoff brauchen wir "Versuch Nummer"
    // Versuch 1 => retries_left = OUTBOX_MAX_RETRIES (noch volle retries)
    // Versuch 2 => retries_left = OUTBOX_MAX_RETRIES-1
    uint8_t attempt = (uint8_t)(OUTBOX_MAX_RETRIES - retries_left + 1);

    // base * 2^(attempt-1)
    uint32_t backoff = OUTBOX_BACKOFF_BASE_MS << (attempt > 1 ? (attempt - 1) : 0);
    backoff = (uint32_t)clamp_i((int)backoff, OUTBOX_BACKOFF_BASE_MS, OUTBOX_BACKOFF_MAX_MS);

    int jitter = (int)(esp_random() % (2 * OUTBOX_JITTER_MS + 1)) - OUTBOX_JITTER_MS; // [-J..+J]
    int with_jitter = (int)backoff + jitter;
    if (with_jitter < 0) with_jitter = 0;

    return (uint32_t)with_jitter;
}

// TX-Worker: alle fälligen Jobs senden / neu terminieren / löschen
static void outbox_process_due_jobs(void)
{
    uint32_t t = now_ms();

    if (xSemaphoreTake(outbox_mutex, pdMS_TO_TICKS(50)) != pdTRUE)
        return;

    for (int i = 0; i < OUTBOX_SIZE; i++) {
        if (!outbox[i].used) continue;

        // abgelaufen?
        if (t >= outbox[i].expires_ms) {
            ESP_LOGW(TAG, "OUTBOX drop expired id=%u", (unsigned)outbox[i].msg_id);
            outbox[i].used = false;
            continue;
        }

        // noch nicht dran?
        if (t < outbox[i].next_try_ms)
            continue;

        // senden
        ESP_LOGI(TAG, "TX_WORKER send id=%u try_left=%u len=%u",
                 (unsigned)outbox[i].msg_id,
                 (unsigned)outbox[i].retries_left,
                 (unsigned)outbox[i].frame_len);

        lora_send_packet(outbox[i].frame, outbox[i].frame_len);

        // Bei Broadcast ohne ACK gilt: "gesendet" => Job erledigt.
        // Store-and-Forward wirkt hier vor allem als Entkopplung + Backoff-Streuer.
        //
        // Wenn du später ACK baust, wird hier erst bei ACK gelöscht.
        outbox[i].used = false;
    }

    xSemaphoreGive(outbox_mutex);
}

// TX-Worker Task (läuft immer)
static void tx_worker_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "TX_WORKER started");

    while (1) {
        outbox_process_due_jobs();
        vTaskDelay(pdMS_TO_TICKS(TX_WORKER_TICK_MS));
    }
}

// ============================================================================
// Beacon / Chat: jetzt ENQUEUE statt direkt senden
// ============================================================================

static void build_beacon_frame(mr_hdr_t *h)
{
    memset(h, 0, sizeof(*h));
    h->magic[0] = 'M';
    h->magic[1] = 'R';
    h->version  = 1;
    h->flags    = MR_FLAG_BEACON;
    h->ttl      = 1;
    h->msg_id   = g_msg_id++;
    call7_set(h->src, MY_CALL);
    call7_set(h->dst, "*");
    h->payload_len = 0;
}

static void enqueue_beacon(void)
{
    mr_hdr_t h;
    build_beacon_frame(&h);

    outbox_enqueue(JOB_KIND_BEACON,
                   (const uint8_t*)&h,
                   (uint16_t)sizeof(h),
                   h.msg_id,
                   h.src);
}

static void enqueue_local_chat(const char *text)
{
    mr_chat_frame_t f;
    memset(&f, 0, sizeof(f));

    f.h.magic[0] = 'M';
    f.h.magic[1] = 'R';
    f.h.version  = 1;
    f.h.flags    = MR_FLAG_CHAT;
    f.h.ttl      = 3;
    f.h.msg_id   = g_msg_id++;

    call7_set(f.h.src, MY_CALL);
    call7_set(f.h.dst, "*");

    size_t len = strlen(text);
    if (len > MR_MAX_PAYLOAD) len = MR_MAX_PAYLOAD;

    memcpy(f.payload, text, len);
    f.h.payload_len = (uint8_t)len;

    outbox_enqueue(JOB_KIND_LOCAL_CHAT,
                   (const uint8_t*)&f,
                   (uint16_t)(sizeof(mr_hdr_t) + len),
                   f.h.msg_id,
                   f.h.src);

    ESP_LOGI(TAG, "CHAT queued: %s", text);
}

// UART Input (PuTTY/Monitor)
static void uart_poll_input(void)
{
    int c = getchar();
    if (c == EOF) return;

    if (c == '\n' || c == '\r') {
        uart_line[uart_pos] = 0;
        if (uart_pos > 0)
            enqueue_local_chat(uart_line);
        uart_pos = 0;
        return;
    }

    if (uart_pos < (int)sizeof(uart_line) - 1)
        uart_line[uart_pos++] = (char)c;
}

// ============================================================================
// RX Handler: Anzeige + Entscheidung + Enqueue(Forward)
// ============================================================================

static void maybe_enqueue_forward(uint8_t *buf, uint16_t len, int rssi_dbm)
{
    mr_hdr_t *h = (mr_hdr_t*)buf;

    // TTL abgelaufen?
    if (h->ttl <= 1)
        return;

    // RSSI Gate
    if (rssi_dbm < FORWARD_MIN_RSSI)
        return;

    // Preferred Forwarding: nur Top-N Nachbarn
    if (!neighbor_is_preferred(h->src))
        return;

    // TTL reduzieren, weil wir weiterleiten
    h->ttl--;

    // Random Delay wird als "next_try" im Outbox-Konzept abgebildet:
    // Wir enqueuen jetzt, TX-Worker entkoppelt ohnehin.
    // Optional könnte man hier next_try_ms setzen; für Einfachheit:
    // Enqueue sofort, Kollisionen reduziert sich schon stark durch Worker.
    outbox_enqueue(JOB_KIND_FORWARD, buf, len, h->msg_id, h->src);
}

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

    // Magic prüfen
    if (h->magic[0] != 'M' || h->magic[1] != 'R')
        return;

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
        if (len >= sizeof(mr_hdr_t)) {
            uint8_t pl = h->payload_len;
            if (pl > MR_MAX_PAYLOAD) pl = MR_MAX_PAYLOAD;

            char src8[8];
            call7_to_cstr(src8, h->src);

            // Payload sicher ausgeben
            char msg[MR_MAX_PAYLOAD + 1];
            memset(msg, 0, sizeof(msg));

            uint16_t max_copy = (uint16_t)pl;
            uint16_t available = (uint16_t)(len - sizeof(mr_hdr_t));
            if (max_copy > available) max_copy = available;

            memcpy(msg, buf + sizeof(mr_hdr_t), max_copy);
            msg[max_copy] = 0;

            ESP_LOGI(TAG, "CHAT %.7s > %s (RSSI=%d)", src8, msg, rssi_dbm);
        }
    }

    // Kapitel 16: Forward NICHT sofort senden, sondern in Outbox legen
    maybe_enqueue_forward(buf, len, rssi_dbm);
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
    (void)arg;
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
    ESP_LOGI(TAG, "Kapitel 16 start (%s)", MY_CALL);

    // Init SPI + LoRa
    init_spi();
    lora_reset();

    uint8_t v = lora_read_reg(REG_VERSION);
    ESP_LOGI(TAG, "SX1276 Version=0x%02X", v);

    lora_init();

    // Outbox + TX Worker
    outbox_init();
    xTaskCreate(tx_worker_task, "tx_worker", 4096, NULL, 9, NULL);

    // RX IRQ
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

        // UART Chat
        uart_poll_input();

        // Beacon: wird in Outbox gelegt (TX-Worker sendet)
        if ((now_ms() - last_beacon_ms) >= (BEACON_INTERVAL_S * 1000)) {
            enqueue_beacon();
            last_beacon_ms = now_ms();
        }

        // Neighbor cleanup
        neighbor_cleanup();

        // 15A Statusausgabe
        if (UART_STATUS_INTERVAL_S > 0 &&
            (now_ms() - last_uart_ms) >= (UART_STATUS_INTERVAL_S * 1000)) {
            uart_show_neighbors_status();
            last_uart_ms = now_ms();
        }
    }
}
