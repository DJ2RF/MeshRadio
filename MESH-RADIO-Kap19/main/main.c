// ============================================================================
// MeshRadio 4.0 – Kapitel 19 (Sourcen komplett, sauber dokumentiert)
// ----------------------------------------------------------------------------
// PROTOKOLL V2: final_dst + next_hop getrennt  (echtes Next-Hop Routing)
//
// Basierend auf Kap. 17/18:
//   ✔ Outbox + TX Worker (Store-and-Forward)
//   ✔ Directed Messages (@CALL text)
//   ✔ ACK + Retry für Directed Messages
//   ✔ Duplicate Detection
//   ✔ Neighbor Ranking / Preferred Forwarding
//
// NEU in Kapitel 19:
//   ✔ Header V2: final_dst (Endziel) + next_hop (Hop-by-Hop)
//   ✔ Route Table: destination -> next_hop
//   ✔ Forward-Regeln:
//        - next_hop == MY_CALL  -> wir sind dran -> next_hop neu setzen und senden
//        - next_hop == "*"      -> Discovery/Fallback -> forwarden unter Regeln
//        - sonst                -> nicht zuständig, ignorieren
//   ✔ ACK wird ebenfalls geroutet (final_dst = ursprünglicher Sender)
//   ✔ Route Aging + optional Route Drop bei ACK Fail
//
// Amateurfunk-konform:
//   - keine Verschlüsselung
//   - Callsigns sichtbar
//
// ESP-IDF v5.5.x
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

#define MY_CALL "DJ1ABC"

// Beacon (optional, hier aus – wir nutzen es später wieder)
#define BEACON_INTERVAL_S 10
#define ENABLE_BEACON     0

// Duplicate Cache
#define SEEN_CACHE_SIZE 32

// Neighbor Tabelle
#define MAX_NEIGHBORS          20
#define NEIGHBOR_TIMEOUT_MS    60000

// Preferred Forwarding
#define PREFERRED_TOP_N        3
#define FORWARD_MIN_RSSI       (-105)

// UART Statusausgabe
#define UART_STATUS_INTERVAL_S 5
#define ENABLE_STATUS_PRINT    0   // 1 = Logs, 0 = ruhig

// Routing Tabelle
#define MAX_ROUTES          20
#define ROUTE_TIMEOUT_MS    120000
#define ROUTE_MIN_RSSI      (-115)

// Bei ACK Fail Route fürs Ziel löschen (damit nächster Versuch discovery/flood nutzt)
#define ACK_FAIL_DROP_ROUTE 1

// Flags
#define MR_FLAG_BEACON 0x04
#define MR_FLAG_CHAT   0x08
#define MR_FLAG_ACK    0x10

// Outbox / Store-and-Forward
#define OUTBOX_SIZE            16
#define OUTBOX_MAX_RETRIES     5
#define OUTBOX_EXPIRE_MS       60000
#define OUTBOX_BACKOFF_BASE_MS 800
#define OUTBOX_BACKOFF_MAX_MS  10000
#define OUTBOX_JITTER_MS       200
#define TX_WORKER_TICK_MS      50

// Payload
#define MR_MAX_PAYLOAD 64

// Protokoll Version
#define MR_PROTO_VERSION 2

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
// PROTOKOLL V2: Header mit final_dst + next_hop
// ============================================================================

#pragma pack(push,1)
typedef struct {
    uint8_t  magic[2];       // "MR"
    uint8_t  version;        // 2
    uint8_t  flags;

    uint8_t  ttl;            // Hop limit
    uint16_t msg_id;         // per SRC eindeutig

    char     src[7];         // ursprünglicher Sender
    char     final_dst[7];   // Endziel ("*" für broadcast)
    char     next_hop[7];    // nächster Hop ("*" = discovery/broadcast)

    uint8_t  payload_len;
} mr_hdr_v2_t;
#pragma pack(pop)

// Chat Frame
#pragma pack(push,1)
typedef struct {
    mr_hdr_v2_t h;
    char payload[MR_MAX_PAYLOAD];
} mr_chat_v2_t;
#pragma pack(pop)

// ACK Frame: bestätigt msg_id eines Chat/Frames
#pragma pack(push,1)
typedef struct {
    mr_hdr_v2_t h;
    uint16_t ack_id;
} mr_ack_v2_t;
#pragma pack(pop)

// ============================================================================
// Seen + Neighbor + Routing
// ============================================================================

typedef struct {
    bool used;
    char src[7];
    uint16_t msg_id;
} seen_msg_t;

typedef struct {
    bool used;
    char call[7];
    int rssi_dbm;
    uint32_t last_seen_ms;
} neighbor_t;

typedef struct {
    bool used;
    char destination[7];   // Endziel
    char next_hop[7];      // über wen erreichbar
    int  rssi_dbm;
    uint32_t last_seen_ms;
} route_entry_t;

// ============================================================================
// Outbox Job
// ----------------------------------------------------------------------------
// needs_ack: nur für directed Chats und deren ACK-Retry
// final_dst: Endziel (für Route-Drop bei ACK Fail)
// ============================================================================
typedef enum {
    JOB_CHAT_LOCAL = 0,
    JOB_FORWARD    = 1,
    JOB_BEACON     = 2,
    JOB_ACK        = 3
} outbox_kind_t;

typedef struct {
    bool used;
    outbox_kind_t kind;

    uint8_t  frame[256];
    uint16_t frame_len;

    uint8_t  retries_left;
    uint32_t next_try_ms;
    uint32_t expires_ms;

    uint16_t msg_id;
    char src[7];

    bool needs_ack;
    char final_dst[7];
} outbox_job_t;

// ============================================================================
// Globals
// ============================================================================

static const char *TAG = "MR19";

static spi_device_handle_t lora_spi;
static QueueHandle_t dio0_evt_queue;

static uint16_t g_msg_id = 1;

static seen_msg_t    seen_cache[SEEN_CACHE_SIZE];
static neighbor_t    neighbors[MAX_NEIGHBORS];
static route_entry_t routes[MAX_ROUTES];

static outbox_job_t outbox[OUTBOX_SIZE];
static SemaphoreHandle_t outbox_mutex;

static char uart_line[80];
static int uart_pos = 0;

// ============================================================================
// Prototypen
// ============================================================================

static void lora_send_packet(uint8_t *data, size_t len);
static bool lora_wait_tx_done_polling(int timeout_ms);

static void outbox_process_due_jobs(void);
static void outbox_ack_received(uint16_t ack_id);
static void outbox_drop_route_on_ack_fail(const char final_dst[7]);

static void route_update(const char destination[7], const char next_hop[7], int rssi_dbm);
static bool route_lookup(const char destination[7], char out_next_hop[7]);
static void route_delete(const char destination[7]);
static void route_cleanup(void);

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

static bool call7_eq(const char a[7], const char b[7])
{
    return memcmp(a, b, 7) == 0;
}

static void call7_star(char out7[7]) { call7_set(out7, "*"); }

// ============================================================================
// Duplicate Detection
// ============================================================================

static bool seen_before(const char src[7], uint16_t id)
{
    for (int i = 0; i < SEEN_CACHE_SIZE; i++) {
        if (!seen_cache[i].used) continue;
        if (call7_eq(seen_cache[i].src, src) && seen_cache[i].msg_id == id)
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
// Neighbor
// ============================================================================

static void neighbor_update(const char call[7], int rssi_dbm)
{
    uint32_t t = now_ms();

    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!neighbors[i].used) continue;
        if (call7_eq(neighbors[i].call, call)) {
            neighbors[i].rssi_dbm = rssi_dbm;
            neighbors[i].last_seen_ms = t;
            return;
        }
    }
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!neighbors[i].used) {
            neighbors[i].used = true;
            memcpy(neighbors[i].call, call, 7);
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
        if (t - neighbors[i].last_seen_ms > NEIGHBOR_TIMEOUT_MS)
            neighbors[i].used = false;
    }
}

// Top-N nach RSSI (einfacher Vergleichszähler)
static bool neighbor_is_preferred(const char call[7])
{
    int my = -9999;
    int better = 0;

    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!neighbors[i].used) continue;
        if (call7_eq(neighbors[i].call, call))
            my = neighbors[i].rssi_dbm;
    }
    if (my == -9999) return false;

    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!neighbors[i].used) continue;
        if (neighbors[i].rssi_dbm > my)
            better++;
    }
    return (better < PREFERRED_TOP_N);
}

// ============================================================================
// Kapitel 19: Route Table
// ----------------------------------------------------------------------------
// destination -> next_hop
// - update bei RX (wenn wir etwas von destination hören)
// - update auch bei ACK (implizit über RX)
// ============================================================================
static void route_update(const char destination[7], const char next_hop[7], int rssi_dbm)
{
    if (rssi_dbm < ROUTE_MIN_RSSI)
        return;

    uint32_t t = now_ms();

    for (int i = 0; i < MAX_ROUTES; i++) {
        if (!routes[i].used) continue;
        if (call7_eq(routes[i].destination, destination)) {
            memcpy(routes[i].next_hop, next_hop, 7);
            routes[i].rssi_dbm = rssi_dbm;
            routes[i].last_seen_ms = t;
            return;
        }
    }

    for (int i = 0; i < MAX_ROUTES; i++) {
        if (!routes[i].used) {
            routes[i].used = true;
            memcpy(routes[i].destination, destination, 7);
            memcpy(routes[i].next_hop, next_hop, 7);
            routes[i].rssi_dbm = rssi_dbm;
            routes[i].last_seen_ms = t;
            return;
        }
    }

    // Tabelle voll: ältesten überschreiben
    int oldest = 0;
    for (int i = 1; i < MAX_ROUTES; i++) {
        if (!routes[i].used) { oldest = i; break; }
        if (routes[i].last_seen_ms < routes[oldest].last_seen_ms)
            oldest = i;
    }
    routes[oldest].used = true;
    memcpy(routes[oldest].destination, destination, 7);
    memcpy(routes[oldest].next_hop, next_hop, 7);
    routes[oldest].rssi_dbm = rssi_dbm;
    routes[oldest].last_seen_ms = t;
}

static bool route_lookup(const char destination[7], char out_next_hop[7])
{
    for (int i = 0; i < MAX_ROUTES; i++) {
        if (!routes[i].used) continue;
        if (call7_eq(routes[i].destination, destination)) {
            memcpy(out_next_hop, routes[i].next_hop, 7);
            return true;
        }
    }
    return false;
}

static void route_delete(const char destination[7])
{
    for (int i = 0; i < MAX_ROUTES; i++) {
        if (!routes[i].used) continue;
        if (call7_eq(routes[i].destination, destination))
            routes[i].used = false;
    }
}

static void route_cleanup(void)
{
    uint32_t t = now_ms();
    for (int i = 0; i < MAX_ROUTES; i++) {
        if (!routes[i].used) continue;
        if (t - routes[i].last_seen_ms > ROUTE_TIMEOUT_MS)
            routes[i].used = false;
    }
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
    spi_transaction_t t = { .length = 16, .tx_buffer = tx, .rx_buffer = rx };
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

    // 433 MHz (wie bisher)
    uint32_t frf = 0x6C8000;
    lora_write_reg(REG_FRF_MSB, frf >> 16);
    lora_write_reg(REG_FRF_MID, frf >> 8);
    lora_write_reg(REG_FRF_LSB, frf);

    lora_write_reg(REG_MODEM_CONFIG_1, 0x72);
    lora_write_reg(REG_MODEM_CONFIG_2, 0x74);
    lora_write_reg(REG_MODEM_CONFIG_3, 0x04);

    lora_write_reg(REG_PA_CONFIG, 0x8E);

    lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0);
    lora_write_reg(REG_FIFO_ADDR_PTR, 0);

    // RX continuous
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

    // TX mode
    lora_write_reg(REG_OP_MODE, 0x83);
    lora_wait_tx_done_polling(2000);

    // back to RX
    lora_write_reg(REG_OP_MODE, 0x85);
}

// ============================================================================
// Outbox
// ============================================================================

static void outbox_init(void)
{
    outbox_mutex = xSemaphoreCreateMutex();
    memset(outbox, 0, sizeof(outbox));
}

static bool outbox_enqueue(outbox_kind_t kind,
                           const uint8_t *frame, uint16_t len,
                           uint16_t msg_id, const char src[7],
                           bool needs_ack,
                           const char final_dst[7])
{
    if (len == 0 || len > sizeof(outbox[0].frame))
        return false;

    if (xSemaphoreTake(outbox_mutex, pdMS_TO_TICKS(100)) != pdTRUE)
        return false;

    int slot = -1;
    for (int i = 0; i < OUTBOX_SIZE; i++) {
        if (!outbox[i].used) { slot = i; break; }
    }

    if (slot < 0) {
        xSemaphoreGive(outbox_mutex);
        ESP_LOGW(TAG, "OUTBOX FULL -> drop msg_id=%u", (unsigned)msg_id);
        return false;
    }

    outbox[slot].used = true;
    outbox[slot].kind = kind;

    memcpy(outbox[slot].frame, frame, len);
    outbox[slot].frame_len = len;

    outbox[slot].retries_left = OUTBOX_MAX_RETRIES;
    outbox[slot].next_try_ms  = now_ms();
    outbox[slot].expires_ms   = now_ms() + OUTBOX_EXPIRE_MS;

    outbox[slot].msg_id = msg_id;
    memcpy(outbox[slot].src, src, 7);

    outbox[slot].needs_ack = needs_ack;

    if (final_dst) memcpy(outbox[slot].final_dst, final_dst, 7);
    else call7_star(outbox[slot].final_dst);

    xSemaphoreGive(outbox_mutex);
    return true;
}

static void outbox_ack_received(uint16_t ack_id)
{
    if (xSemaphoreTake(outbox_mutex, pdMS_TO_TICKS(100)) != pdTRUE)
        return;

    for (int i = 0; i < OUTBOX_SIZE; i++) {
        if (!outbox[i].used) continue;
        if (outbox[i].needs_ack && outbox[i].msg_id == ack_id) {
            ESP_LOGI(TAG, "ACK RX id=%u -> remove job", (unsigned)ack_id);
            outbox[i].used = false;
        }
    }

    xSemaphoreGive(outbox_mutex);
}

static void outbox_drop_route_on_ack_fail(const char final_dst[7])
{
#if ACK_FAIL_DROP_ROUTE
    route_delete(final_dst);

    char d8[8];
    call7_to_cstr(d8, final_dst);
    ESP_LOGW(TAG, "ACK FAIL -> drop route for %.7s", d8);
#else
    (void)final_dst;
#endif
}

// TX worker: send + reschedule if needs_ack
static void outbox_process_due_jobs(void)
{
    uint32_t t = now_ms();

    if (xSemaphoreTake(outbox_mutex, pdMS_TO_TICKS(50)) != pdTRUE)
        return;

    for (int i = 0; i < OUTBOX_SIZE; i++) {
        if (!outbox[i].used) continue;

        if (t >= outbox[i].expires_ms) {
            ESP_LOGW(TAG, "OUTBOX expired id=%u", (unsigned)outbox[i].msg_id);
            if (outbox[i].needs_ack)
                outbox_drop_route_on_ack_fail(outbox[i].final_dst);
            outbox[i].used = false;
            continue;
        }

        if (t < outbox[i].next_try_ms)
            continue;

        // SEND
        lora_send_packet(outbox[i].frame, outbox[i].frame_len);

        // Wenn kein ACK erwartet: Job fertig
        if (!outbox[i].needs_ack) {
            outbox[i].used = false;
            continue;
        }

        // ACK erwartet: reschedule oder aufgeben
        if (outbox[i].retries_left == 0) {
            ESP_LOGW(TAG, "OUTBOX no retries id=%u", (unsigned)outbox[i].msg_id);
            outbox_drop_route_on_ack_fail(outbox[i].final_dst);
            outbox[i].used = false;
            continue;
        }

        outbox[i].retries_left--;

        // Backoff (einfach, exponentiell)
        uint8_t attempt = (uint8_t)(OUTBOX_MAX_RETRIES - outbox[i].retries_left);
        uint32_t backoff = OUTBOX_BACKOFF_BASE_MS << (attempt > 5 ? 5 : attempt);
        if (backoff > OUTBOX_BACKOFF_MAX_MS) backoff = OUTBOX_BACKOFF_MAX_MS;
        backoff += (esp_random() % OUTBOX_JITTER_MS);

        outbox[i].next_try_ms = t + backoff;
    }

    xSemaphoreGive(outbox_mutex);
}

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
// Packet Builder: NEXT_HOP setzen
// ----------------------------------------------------------------------------
// Wenn Route bekannt: next_hop = route(final_dst)
// sonst: next_hop = "*" (discovery/fallback)
// ============================================================================
static void set_next_hop_for_final_dst(char next_hop_out[7], const char final_dst[7])
{
    char nh[7];
    if (route_lookup(final_dst, nh)) {
        memcpy(next_hop_out, nh, 7);
    } else {
        call7_star(next_hop_out);
    }
}

// ============================================================================
// ACK senden (geroutet wie normale directed Nachricht)
// ----------------------------------------------------------------------------
// ACK final_dst = ursprünglicher Sender
// ACK next_hop  = route(original_sender) oder "*" fallback
// ============================================================================
static void enqueue_ack_for_sender(const char sender_src[7], uint16_t ack_id)
{
    mr_ack_v2_t a;
    memset(&a, 0, sizeof(a));

    a.h.magic[0] = 'M';
    a.h.magic[1] = 'R';
    a.h.version  = MR_PROTO_VERSION;
    a.h.flags    = MR_FLAG_ACK;

    a.h.ttl      = 5;
    a.h.msg_id   = g_msg_id++;

    call7_set(a.h.src, MY_CALL);
    memcpy(a.h.final_dst, sender_src, 7);
    set_next_hop_for_final_dst(a.h.next_hop, a.h.final_dst);

    a.h.payload_len = 2;
    a.ack_id = ack_id;

    outbox_enqueue(JOB_ACK,
                   (uint8_t*)&a,
                   (uint16_t)sizeof(a),
                   a.h.msg_id,
                   a.h.src,
                   false,
                   NULL);
}

// ============================================================================
// UART Chat Parser + Enqueue
// ----------------------------------------------------------------------------
// Eingabe:
//   Broadcast: Hallo an alle
//   Directed : @DL1XYZ Hallo Fritz
//
// V2-Header:
//   final_dst = "*" oder Ziel
//   next_hop  = route(final_dst) oder "*"
// ============================================================================
static void enqueue_chat_line(const char *line)
{
    mr_chat_v2_t f;
    memset(&f, 0, sizeof(f));

    f.h.magic[0] = 'M';
    f.h.magic[1] = 'R';
    f.h.version  = MR_PROTO_VERSION;
    f.h.flags    = MR_FLAG_CHAT;
    f.h.ttl      = 8;
    f.h.msg_id   = g_msg_id++;

    call7_set(f.h.src, MY_CALL);

    const char *text = line;
    bool directed = false;

    char final_dst7[7];
    call7_star(final_dst7);

    if (line[0] == '@') {
        directed = true;

        char call[8] = {0};
        int i = 1, j = 0;
        while (line[i] && line[i] != ' ' && j < 7)
            call[j++] = line[i++];

        call7_set(final_dst7, call);
        if (line[i] == ' ')
            text = &line[i + 1];
    }

    memcpy(f.h.final_dst, final_dst7, 7);

    // next_hop aus Routing ableiten (oder "*")
    set_next_hop_for_final_dst(f.h.next_hop, f.h.final_dst);

    // Payload
    size_t len = strlen(text);
    if (len > MR_MAX_PAYLOAD) len = MR_MAX_PAYLOAD;
    memcpy(f.payload, text, len);
    f.h.payload_len = (uint8_t)len;

    // Outbox: Directed => needs_ack=true, final_dst merken
    outbox_enqueue(JOB_CHAT_LOCAL,
                   (uint8_t*)&f,
                   (uint16_t)(sizeof(mr_hdr_v2_t) + len),
                   f.h.msg_id,
                   f.h.src,
                   directed,
                   directed ? f.h.final_dst : NULL);

    ESP_LOGI(TAG, "CHAT queued id=%u directed=%d", (unsigned)f.h.msg_id, directed);
}

static void uart_poll_input(void)
{
    int c = getchar();
    if (c == EOF) return;

    if (c == '\n' || c == '\r') {
        uart_line[uart_pos] = 0;
        if (uart_pos > 0)
            enqueue_chat_line(uart_line);
        uart_pos = 0;
        return;
    }

    if (uart_pos < (int)sizeof(uart_line) - 1)
        uart_line[uart_pos++] = (char)c;
}

// ============================================================================
// Forwarding Regeln (Kapitel 19)
// ----------------------------------------------------------------------------
// Wir forwarden nur, wenn:
//   - TTL > 1
//   - RSSI ok
//   - preferred ok
// und zusätzlich:
//   A) next_hop == "*"       (Discovery/Fallback) -> forward erlauben
//   B) next_hop == MY_CALL   -> wir sind dran -> next_hop neu setzen -> senden
//   C) sonst                 -> nicht zuständig -> NICHT forwarden
//
// Außerdem:
//   Wenn wir final_dst sind -> anzeigen + ACK (bei directed) + NICHT forwarden
// ============================================================================
static void maybe_forward_v2(mr_hdr_v2_t *h, uint8_t *buf, uint16_t len, int rssi_dbm)
{
    char my7[7]; call7_set(my7, MY_CALL);
    char star7[7]; call7_star(star7);

    // Endziel erreicht?
    if (!call7_eq(h->final_dst, star7) && call7_eq(h->final_dst, my7)) {
        // Wir sind das Endziel -> nicht forwarden
        return;
    }

    // TTL?
    if (h->ttl <= 1) return;

    // RSSI gate
    if (rssi_dbm < FORWARD_MIN_RSSI) return;

    // preferred gate: wir nutzen src (original sender) als Nachbar-ID nur als grobe Heuristik.
    // In kleinen Netzen ok. Später könnte man "last_hop" extra einführen.
    if (!neighbor_is_preferred(h->src)) return;

    // Zuständigkeit über next_hop
    bool next_is_star = call7_eq(h->next_hop, star7);
    bool next_is_me   = call7_eq(h->next_hop, my7);

    if (!next_is_star && !next_is_me) {
        // nicht zuständig
        return;
    }

    // Wir forwarden: TTL--
    h->ttl--;

    // Wenn wir dran sind (next_hop==MY_CALL) oder Discovery, versuchen wir jetzt next_hop zu setzen:
    // Ziel = final_dst, next_hop = route_lookup(final_dst) oder "*"
    if (!call7_eq(h->final_dst, star7)) {
        set_next_hop_for_final_dst(h->next_hop, h->final_dst);
    } else {
        // Broadcast bleibt Broadcast (next_hop="*")
        call7_star(h->next_hop);
    }

    outbox_enqueue(JOB_FORWARD, buf, len, h->msg_id, h->src, false, NULL);
}

// ============================================================================
// RX Handler
// ----------------------------------------------------------------------------
// Route Learning:
//   - Wenn wir ein Paket hören, lernen wir: destination=src über next_hop = ???
//
// In echter Funk-L2 wäre next_hop = die Station, die tatsächlich gesendet hat.
// Da wir das hier nicht haben, nutzen wir folgende Näherung:
//   - Wenn Frame next_hop == MY_CALL => der Sender dieses Frames ist unser direkter Nachbar
//   - Wenn Frame next_hop == "*"     => ebenfalls direkter Nachbar (wir haben ihn direkt gehört)
//
// Dann setzen wir route(src) = src (direkt).
//
// Das ist für kleine Setups brauchbar. Für „perfekt“ bräuchten wir ein Feld last_hop.
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

    if (len < sizeof(mr_hdr_v2_t)) return;

    mr_hdr_v2_t *h = (mr_hdr_v2_t*)buf;

    // Check magic/version
    if (h->magic[0] != 'M' || h->magic[1] != 'R') return;
    if (h->version != MR_PROTO_VERSION) return;

    // Duplicate filter (src,msg_id)
    if (seen_before(h->src, h->msg_id)) return;
    remember_msg(h->src, h->msg_id);

    // RSSI
    int rssi_dbm = (int)lora_read_reg(REG_PKT_RSSI_VALUE) - 157;

    // Neighbor update: wir tragen src als "heard"
    neighbor_update(h->src, rssi_dbm);

    // Route update: src ist (für uns) erreichbar über src (direkt gehört)
    route_update(h->src, h->src, rssi_dbm);

    // ACK?
    if (h->flags & MR_FLAG_ACK) {
        char my7[7]; call7_set(my7, MY_CALL);
        // ACK interessiert uns, wenn wir final_dst sind (Endziel)
        if (call7_eq(h->final_dst, my7)) {
            mr_ack_v2_t *a = (mr_ack_v2_t*)buf;
            outbox_ack_received(a->ack_id);
        }
        // ACK kann ebenfalls weitergeroutet werden (falls nicht Endziel):
        maybe_forward_v2(h, buf, len, rssi_dbm);
        return;
    }

    // CHAT?
    if (h->flags & MR_FLAG_CHAT) {
        // Payload sicher extrahieren
        uint8_t pl = h->payload_len;
        if (pl > MR_MAX_PAYLOAD) pl = MR_MAX_PAYLOAD;

        uint16_t avail = (uint16_t)(len - sizeof(mr_hdr_v2_t));
        uint16_t cp = pl;
        if (cp > avail) cp = avail;

        char msg[MR_MAX_PAYLOAD + 1];
        memset(msg, 0, sizeof(msg));
        memcpy(msg, buf + sizeof(mr_hdr_v2_t), cp);
        msg[cp] = 0;

        char src8[8]; call7_to_cstr(src8, h->src);

        // Sind wir Endziel?
        char my7[7]; call7_set(my7, MY_CALL);
        char star7[7]; call7_star(star7);

        if (!call7_eq(h->final_dst, star7) && call7_eq(h->final_dst, my7)) {
            // Endziel: anzeigen
            ESP_LOGI(TAG, "CHAT %.7s > %s (RSSI=%d) [TO ME]", src8, msg, rssi_dbm);

            // Directed => ACK senden
            enqueue_ack_for_sender(h->src, h->msg_id);

            // nicht forwarden
            return;
        }

        // Broadcast oder nicht-Endziel: optional anzeigen (für Debug)
        ESP_LOGI(TAG, "CHAT %.7s > %s (RSSI=%d)", src8, msg, rssi_dbm);

        // Weiterleitung nach Kapitel 19 Regeln
        maybe_forward_v2(h, buf, len, rssi_dbm);
    }
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
    ESP_LOGI(TAG, "Kapitel 19 start (%s)", MY_CALL);

    init_spi();
    lora_reset();

    uint8_t v = lora_read_reg(REG_VERSION);
    ESP_LOGI(TAG, "SX1276 version=0x%02X", v);

    lora_init();

    outbox_init();
    xTaskCreate(tx_worker_task, "tx_worker", 4096, NULL, 9, NULL);

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
    xTaskCreate(dio0_task, "dio0_task", 4096, NULL, 10, NULL);

    uint32_t last_beacon = now_ms();
    uint32_t last_status = now_ms();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(50));

        // UART Chat
        uart_poll_input();

        // Maintenance
        neighbor_cleanup();
        route_cleanup();

#if ENABLE_BEACON
        if (now_ms() - last_beacon > BEACON_INTERVAL_S * 1000) {
            last_beacon = now_ms();
            // Beacon könnte hier wie in Kap16 implementiert werden (v2 header)
        }
#endif

#if ENABLE_STATUS_PRINT
        if (UART_STATUS_INTERVAL_S > 0 &&
            now_ms() - last_status > UART_STATUS_INTERVAL_S * 1000) {
            last_status = now_ms();
            ESP_LOGI(TAG, "Status: routes/neighbor maintenance ok");
        }
#else
        (void)last_beacon;
        (void)last_status;
#endif
    }
}
