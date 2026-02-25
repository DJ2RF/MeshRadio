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
// MeshRadio – Kapitel 27 (ETX-Light Routing) – KORRIGIERT & SAUBER DOKUMENTIERT
// ----------------------------------------------------------------------------
// Was Kapitel 27 leistet:
//
//   1) ETX-Light pro Link (Neighbor):
//        - Jeder Link sammelt TX-Versuche und erfolgreiche ACKs.
//        - ETX = TX / ACK   (als Fixpunkt: x100, z.B. 125 = 1.25)
//
//   2) Routing-Metric = ETX (kleiner ist besser):
//        - Route-Update-Regeln aus Kapitel 26 bleiben (seq gewinnt).
//        - Bei gleicher seq gewinnt die Route mit kleinerem ETX.
//
//   3) Hop-by-Hop ACK + Retry (nur wenn ACKREQ gesetzt):
//        - DATA kann ACKREQ tragen.
//        - Receiver sendet ACK an den vorherigen Hop (last_hop).
//        - Sender wartet + retry (klein & kontrolliert) -> ETX misst Airtime-Kosten.
//
//   4) Web UI (mehr zu sehen):
//        - "/" Dashboard
//        - "/api/status" JSON (neighbors + routes + settings)
//        - "/api/send" (dst,msg,ack=0/1)
//        - "/api/bcfallback" (enable=0/1)
//        - UI zeigt Fehler, wenn /api/status nicht parsebar ist
//
// ----------------------------------------------------------------------------
// Korrekturen gegenüber der ersten 27-Version aus dem Chat:
//
//   FIX-1: Web UI zeigt bei Fetch/JSON Fehlern einen Hinweis (statt "loading…").
//   FIX-2: /api/status Buffer etwas größer + Stack des HTTPD erhöht.
//   FIX-3: JSON-Builder gegen Überlauf abgesichert (früher: stiller Abbruch -> ungültiges JSON).
//   FIX-4: ETX-Default: wenn tx==0 => ETX 1.00 (100), wenn ack==0 aber tx>0 => ETX max.
//   FIX-5: ACK wird NICHT im seen_cache gefiltert (ACK muss durch, sonst ETX kaputt).
//          -> Wir umgehen den seen_filter für ACK frames bewusst.
//
// Hinweis:
//   - Kapitel 28 bringt dann Rate-Limits, Token-Bucket, Hold-Down usw.
// ============================================================================


// ============================================================================
// USER DEFINES WICHTIG  
// ============================================================================
#define MR_CALLSIGN        "DJ1ABC"            // <<< Rufzeichen hier ändern (max 7 Zeichen)
#define MR_WIFI_AP_SSID    "MeshRadio-Setup2"  // <<< WLAN AP Name
#define MR_WIFI_AP_PASS    ""                 // leer = offen

// Beacon / Routing
#define BEACON_INTERVAL_MS 15000
#define BEACON_JITTER_MS   2000

// ACK/Retry (für ETX Messung + optionale Zuverlässigkeit)
#define ACK_TIMEOUT_MS     1200
#define ACK_RETRY_MAX      2
#define ACK_BACKOFF_MS     350

// ETX Parameter
#define ETX_MAX_X100       1000   // 10.00
#define ETX_DECAY_MS       60000  // alle 60s: Statistik halbieren (sanftes Vergessen)


// ============================================================================
// Includes
// ============================================================================
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <inttypes.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_random.h"
#include "esp_system.h"

#include "nvs_flash.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"


// ============================================================================
// PROTOKOLL
// ============================================================================
#define MR_PROTO_VERSION 5

#define MR_FLAG_DATA      0x10
#define MR_FLAG_BEACON    0x20
#define MR_FLAG_ACK       0x40

// Zusatzflag innerhalb DATA (ACKREQ)
#define MR_FLAG_ACKREQ    0x01

#define DATA_TTL   4
#define BEACON_TTL 2
#define ACK_TTL    4

#define MAX_PAYLOAD 120


// ============================================================================
// LIMITS / TABLES
// ============================================================================
#define SEEN_CACHE_SIZE 48

#define MAX_NEIGHBORS 24
#define NEIGHBOR_TIMEOUT_MS 60000

#define MAX_ROUTES 32
#define ROUTE_TIMEOUT_MS 180000

#define MAX_PENDING_ACK  10


// ============================================================================
// LORA PINS (TTGO T-Beam V1.1 default)
// ============================================================================
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 27
#define PIN_NUM_CLK  5
#define PIN_NUM_CS   18
#define PIN_NUM_RST  23
#define PIN_NUM_DIO0 26
#define LORA_SPI_HOST VSPI_HOST


// ============================================================================
// SX1276 REG
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
// HEADER (seq + ack support)
// ============================================================================
#pragma pack(push,1)
typedef struct {
    uint8_t magic[2];
    uint8_t version;
    uint8_t flags;

    uint8_t ttl;
    uint16_t msg_id;

    // seq für Routing-Stabilität aus Kapitel 26
    uint16_t seq;

    char src[7];
    char final_dst[7];
    char next_hop[7];
    char last_hop[7];

    uint8_t payload_len;
} mr_hdr_v5_t;
#pragma pack(pop)


// ============================================================================
// STRUCTS
// ============================================================================
typedef struct { bool used; char src[7]; uint16_t id; } seen_t;

typedef struct {
    bool used;
    char call[7];
    int rssi;
    uint32_t t_ms;

    // ETX-Light Statistik:
    uint32_t tx_attempts;      // alle TX-Versuche zu diesem Neighbor (inkl. Retries)
    uint32_t ack_ok;           // erfolgreiche ACKs von diesem Neighbor
    uint32_t last_decay_ms;    // wann zuletzt halbiert wurde
} neighbor_t;

typedef struct {
    bool used;
    char dst[7];
    char next[7];

    uint16_t seq;
    uint16_t etx_x100;   // Routing-Metric (kleiner ist besser)

    int last_rssi;       // nur Anzeige
    uint32_t t_ms;
} route_t;

typedef struct {
    bool used;

    uint16_t msg_id;           // DATA msg_id, auf die wir ACK erwarten
    char expect_from[7];       // ACK muss von diesem Neighbor kommen
    uint32_t deadline_ms;      // Timeout für Retry
    uint8_t retries_left;

    uint8_t frame[sizeof(mr_hdr_v5_t) + MAX_PAYLOAD];
    uint16_t frame_len;
} pending_ack_t;


// ============================================================================
// GLOBALS
// ============================================================================
static const char *TAG="MR27";

static spi_device_handle_t lora_spi;
static QueueHandle_t dio_q;
static SemaphoreHandle_t g_mutex;
static httpd_handle_t g_http=NULL;

static uint16_t g_msg_id=1;
static uint16_t g_my_seq=1;

static seen_t seen_cache[SEEN_CACHE_SIZE];
static neighbor_t neighbors[MAX_NEIGHBORS];
static route_t routes[MAX_ROUTES];
static pending_ack_t pend[MAX_PENDING_ACK];

static bool g_bc_fallback=true;

static bool g_beacon_enabled=true;
static uint32_t g_next_beacon_ms=0;


// ============================================================================
// Helpers (Calls, Zeit, ETX)
// ============================================================================
static uint32_t now_ms(void){ return xTaskGetTickCount()*portTICK_PERIOD_MS; }

static void call7_set(char o[7], const char *s)
{
    memset(o,' ',7);
    size_t n=strlen(s); if(n>7)n=7;
    memcpy(o,s,n);
}

static void call7_to_str(char o[8], const char i[7])
{
    memcpy(o,i,7); o[7]=0;
    for(int k=6;k>=0;k--){
        if(o[k]==' ') o[k]=0; else break;
    }
}

static bool call7_eq(const char a[7], const char b[7]){ return memcmp(a,b,7)==0; }

static bool call7_is_wild(const char a[7])
{
    return (a[0]=='*' && a[1]==' ' && a[2]==' ' && a[3]==' ' &&
            a[4]==' ' && a[5]==' ' && a[6]==' ');
}

// ETX = TX/ACK (x100). Regeln:
//   - Noch keine Datenbasis (tx==0): ETX=1.00
//   - tx>0 aber ack==0: ETX=ETX_MAX
static uint16_t etx_compute_x100(uint32_t tx, uint32_t ack)
{
    if(tx == 0) return 100;
    if(ack == 0) return ETX_MAX_X100;

    uint32_t v = (tx * 100U) / ack;
    if(v < 100) v = 100;
    if(v > ETX_MAX_X100) v = ETX_MAX_X100;
    return (uint16_t)v;
}


// ============================================================================
// SEEN (Duplikatfilter für DATA/BEACON)
//   - WICHTIG: ACK NICHT hier filtern, sonst kommt ACK evtl. nie an.
// ============================================================================
static bool seen_before(const char src[7], uint16_t id)
{
    for(int i=0;i<SEEN_CACHE_SIZE;i++){
        if(!seen_cache[i].used) continue;
        if(call7_eq(seen_cache[i].src,src) && seen_cache[i].id==id) return true;
    }
    return false;
}

static void remember_msg(const char src[7], uint16_t id)
{
    static int idx=0;
    seen_cache[idx].used=true;
    memcpy(seen_cache[idx].src,src,7);
    seen_cache[idx].id=id;
    idx=(idx+1)%SEEN_CACHE_SIZE;
}


// ============================================================================
// NEIGHBORS (ETX-Light)
// ============================================================================
static int neighbor_find_locked(const char call[7])
{
    for(int i=0;i<MAX_NEIGHBORS;i++){
        if(!neighbors[i].used) continue;
        if(call7_eq(neighbors[i].call, call)) return i;
    }
    return -1;
}

static int neighbor_ensure_locked(const char call[7])
{
    int idx = neighbor_find_locked(call);
    if(idx >= 0) return idx;

    for(int i=0;i<MAX_NEIGHBORS;i++){
        if(!neighbors[i].used){
            neighbors[i].used=true;
            memcpy(neighbors[i].call, call, 7);
            neighbors[i].rssi=-127;
            neighbors[i].t_ms=now_ms();
            neighbors[i].tx_attempts=0;
            neighbors[i].ack_ok=0;
            neighbors[i].last_decay_ms=now_ms();
            return i;
        }
    }
    return -1;
}

static void neighbor_decay_locked(neighbor_t *n)
{
    uint32_t t=now_ms();
    if(t - n->last_decay_ms < ETX_DECAY_MS) return;

    // Sanftes Vergessen (damit ETX sich anpasst):
    n->tx_attempts = (n->tx_attempts + 1) / 2;
    n->ack_ok      = (n->ack_ok + 1) / 2;
    n->last_decay_ms = t;
}

static void neighbor_update_rssi_locked(const char call[7], int rssi)
{
    int idx = neighbor_ensure_locked(call);
    if(idx < 0) return;
    neighbors[idx].rssi=rssi;
    neighbors[idx].t_ms=now_ms();
}

static void neighbor_tx_attempt_locked(const char call[7])
{
    int idx = neighbor_ensure_locked(call);
    if(idx < 0) return;
    neighbor_decay_locked(&neighbors[idx]);
    neighbors[idx].tx_attempts++;
}

static void neighbor_ack_ok_locked(const char call[7])
{
    int idx = neighbor_ensure_locked(call);
    if(idx < 0) return;
    neighbor_decay_locked(&neighbors[idx]);
    neighbors[idx].ack_ok++;
}

static void neighbor_cleanup_locked(void)
{
    uint32_t t=now_ms();
    for(int i=0;i<MAX_NEIGHBORS;i++){
        if(!neighbors[i].used) continue;
        if(t - neighbors[i].t_ms > NEIGHBOR_TIMEOUT_MS) neighbors[i].used=false;
    }
}


// ============================================================================
// ROUTES (Kap.26 Regeln + Kap.27 Metric=ETX)
// ============================================================================
static void route_update_locked(const char dst[7],
                                const char next[7],
                                uint16_t seq,
                                uint16_t etx_x100,
                                int last_rssi)
{
    // Feasibility Light: nie next==ME
    char me[7]; call7_set(me, MR_CALLSIGN);
    if(call7_eq(next, me)) return;

    uint32_t t=now_ms();

    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used) continue;
        if(!call7_eq(routes[i].dst, dst)) continue;

        bool replace=false;

        if(seq > routes[i].seq){
            replace=true; // neuere Information
        }else if(seq == routes[i].seq && etx_x100 < routes[i].etx_x100){
            replace=true; // gleich alt, aber bessere Qualität (kleinerer ETX)
        }

        if(replace){
            memcpy(routes[i].next, next, 7);
            routes[i].seq = seq;
            routes[i].etx_x100 = etx_x100;
            routes[i].last_rssi = last_rssi;
        }

        routes[i].t_ms = t;
        return;
    }

    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used){
            routes[i].used=true;
            memcpy(routes[i].dst, dst, 7);
            memcpy(routes[i].next, next, 7);
            routes[i].seq = seq;
            routes[i].etx_x100 = etx_x100;
            routes[i].last_rssi = last_rssi;
            routes[i].t_ms = t;
            return;
        }
    }
}

static bool route_lookup_locked(const char dst[7], char out_next[7])
{
    uint32_t t=now_ms();
    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used) continue;
        if(!call7_eq(routes[i].dst, dst)) continue;
        if(t - routes[i].t_ms > ROUTE_TIMEOUT_MS) return false;
        memcpy(out_next, routes[i].next, 7);
        return true;
    }
    return false;
}

static void route_cleanup_locked(void)
{
    uint32_t t=now_ms();
    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used) continue;
        if(t - routes[i].t_ms > ROUTE_TIMEOUT_MS) routes[i].used=false;
    }
}


// ============================================================================
// SPI / LoRa
// ============================================================================
static void lora_wr(uint8_t r,uint8_t v)
{
    uint8_t tx[2]={r|0x80,v};
    spi_transaction_t t={.length=16,.tx_buffer=tx};
    spi_device_transmit(lora_spi,&t);
}

static uint8_t lora_rd(uint8_t r)
{
    uint8_t tx[2]={r&0x7F,0};
    uint8_t rx[2]={0};
    spi_transaction_t t={.length=16,.tx_buffer=tx,.rx_buffer=rx};
    spi_device_transmit(lora_spi,&t);
    return rx[1];
}

static void lora_clear(void){ lora_wr(REG_IRQ_FLAGS,0xFF); }

static void lora_send(uint8_t *d,size_t len)
{
    lora_clear();
    lora_wr(REG_FIFO_TX_BASE_ADDR,0);
    lora_wr(REG_FIFO_ADDR_PTR,0);

    for(size_t i=0;i<len;i++) lora_wr(REG_FIFO,d[i]);
    lora_wr(REG_PAYLOAD_LENGTH,(uint8_t)len);

    lora_wr(REG_OP_MODE,0x83); // TX
    while(!(lora_rd(REG_IRQ_FLAGS)&IRQ_TX_DONE))
        vTaskDelay(pdMS_TO_TICKS(5));

    lora_clear();
    lora_wr(REG_OP_MODE,0x85); // RX cont
}

static void lora_reset(void)
{
    gpio_set_direction(PIN_NUM_RST,GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_RST,0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_NUM_RST,1);
    vTaskDelay(pdMS_TO_TICKS(10));
}


// ============================================================================
// Apply Config (Default: 433.775 MHz / SF7 / BW125 / CRC on)
// ============================================================================
static uint32_t hz_to_frf(uint32_t f)
{
    uint64_t frf=((uint64_t)f<<19)/32000000ULL;
    return (uint32_t)frf;
}

static void lora_apply_locked(void)
{
    lora_wr(REG_OP_MODE,0x80);
    vTaskDelay(pdMS_TO_TICKS(10));

    uint32_t frf=hz_to_frf(433775000);
    lora_wr(REG_FRF_MSB,frf>>16);
    lora_wr(REG_FRF_MID,frf>>8);
    lora_wr(REG_FRF_LSB,frf);

    // BW125, CR 4/5 (implizit), Explicit header, CRC on, SF7
    lora_wr(REG_MODEM_CONFIG_1,0x72);
    lora_wr(REG_MODEM_CONFIG_2,(7<<4)|(1<<2)|0x03);
    lora_wr(REG_MODEM_CONFIG_3,0x04);

    lora_wr(REG_PA_CONFIG,0x8E);

    lora_wr(REG_FIFO_RX_BASE_ADDR,0);
    lora_wr(REG_FIFO_ADDR_PTR,0);
    lora_wr(REG_OP_MODE,0x85);
}


// ============================================================================
// Pending ACK / Retry
// ============================================================================
static void pending_clear_slot_locked(int i){ pend[i].used=false; }

static int pending_alloc_slot_locked(void)
{
    for(int i=0;i<MAX_PENDING_ACK;i++){
        if(!pend[i].used) return i;
    }
    int oldest=0;
    uint32_t od=pend[0].deadline_ms;
    for(int i=1;i<MAX_PENDING_ACK;i++){
        if(!pend[i].used) return i;
        if(pend[i].deadline_ms < od){ oldest=i; od=pend[i].deadline_ms; }
    }
    return oldest;
}

static void pending_add_locked(uint16_t msg_id,
                               const char expect_from[7],
                               const uint8_t *frame,
                               uint16_t frame_len)
{
    int i = pending_alloc_slot_locked();
    pend[i].used=true;
    pend[i].msg_id=msg_id;
    memcpy(pend[i].expect_from, expect_from, 7);
    pend[i].deadline_ms = now_ms() + ACK_TIMEOUT_MS;
    pend[i].retries_left = ACK_RETRY_MAX;

    if(frame_len > sizeof(pend[i].frame)) frame_len = sizeof(pend[i].frame);
    memcpy(pend[i].frame, frame, frame_len);
    pend[i].frame_len = frame_len;
}

static bool pending_mark_acked_locked(uint16_t msg_id, const char src[7])
{
    for(int i=0;i<MAX_PENDING_ACK;i++){
        if(!pend[i].used) continue;
        if(pend[i].msg_id != msg_id) continue;
        if(!call7_eq(pend[i].expect_from, src)) continue;
        pending_clear_slot_locked(i);
        return true;
    }
    return false;
}

static void retry_task(void *arg)
{
    (void)arg;
    while(1){
        vTaskDelay(pdMS_TO_TICKS(100));

        xSemaphoreTake(g_mutex, portMAX_DELAY);
        uint32_t t=now_ms();

        for(int i=0;i<MAX_PENDING_ACK;i++){
            if(!pend[i].used) continue;
            if(t < pend[i].deadline_ms) continue;

            if(pend[i].retries_left == 0){
                char s[8]; call7_to_str(s, pend[i].expect_from);
                ESP_LOGW(TAG,"ACK FAIL msg=%u from=%s", pend[i].msg_id, s);
                pending_clear_slot_locked(i);
                continue;
            }

            pend[i].retries_left--;
            pend[i].deadline_ms = t + ACK_TIMEOUT_MS;

            // Backoff (entkoppelt gleichzeitige Retries)
            uint32_t bo = (esp_random() % (ACK_BACKOFF_MS+1));
            xSemaphoreGive(g_mutex);
            vTaskDelay(pdMS_TO_TICKS(bo));
            xSemaphoreTake(g_mutex, portMAX_DELAY);

            // Retry senden (TTL reduzieren, damit Loops begrenzt bleiben)
            mr_hdr_v5_t *h = (mr_hdr_v5_t*)pend[i].frame;
            if(pend[i].frame_len >= sizeof(mr_hdr_v5_t) && h->ttl > 1) h->ttl--;

            // Retry zählt als neuer TX-Attempt auf diesen Link
            neighbor_tx_attempt_locked(pend[i].expect_from);

            ESP_LOGW(TAG,"RETRY msg=%u left=%u", pend[i].msg_id, pend[i].retries_left);
            lora_send(pend[i].frame, pend[i].frame_len);
        }

        xSemaphoreGive(g_mutex);
    }
}


// ============================================================================
// BEACON TX (mit seq)
// ============================================================================
static void beacon_schedule_next(void)
{
    g_next_beacon_ms = now_ms() + BEACON_INTERVAL_MS + (esp_random()%BEACON_JITTER_MS);
}

static void send_beacon(void)
{
    mr_hdr_v5_t h={0};
    h.magic[0]='M'; h.magic[1]='R';
    h.version=MR_PROTO_VERSION;
    h.flags=MR_FLAG_BEACON;
    h.ttl=BEACON_TTL;
    h.msg_id=g_msg_id++;

    // Kapitel 26: Sequence Number steigt
    h.seq = g_my_seq++;

    call7_set(h.src, MR_CALLSIGN);
    call7_set(h.last_hop, MR_CALLSIGN);
    call7_set(h.final_dst,"*");
    call7_set(h.next_hop,"*");
    h.payload_len=0;

    ESP_LOGI(TAG,"TX BEACON seq=%u", h.seq);
    lora_send((uint8_t*)&h, sizeof(h));
    beacon_schedule_next();
}


// ============================================================================
// ACK TX (Hop-by-Hop): ACK geht an den vorherigen Hop (rx->last_hop)
// ============================================================================
static void send_ack(const mr_hdr_v5_t *rx)
{
    mr_hdr_v5_t h={0};
    h.magic[0]='M'; h.magic[1]='R';
    h.version=MR_PROTO_VERSION;
    h.flags=MR_FLAG_ACK;
    h.ttl=ACK_TTL;

    // ACK bestätigt die DATA msg_id
    h.msg_id = rx->msg_id;
    h.seq = g_my_seq;

    call7_set(h.src, MR_CALLSIGN);
    call7_set(h.last_hop, MR_CALLSIGN);

    // Ziel des ACK = previous hop
    memcpy(h.final_dst, rx->last_hop, 7);

    // Route zum previous hop finden
    char nh[7];
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    bool has = route_lookup_locked(h.final_dst, nh);
    xSemaphoreGive(g_mutex);

    if(has) memcpy(h.next_hop, nh, 7);
    else call7_set(h.next_hop, "*"); // fallback broadcast

    h.payload_len=0;
    lora_send((uint8_t*)&h, sizeof(h));
}


// ============================================================================
// DATA SEND (mit optional ACKREQ) + ETX Statistik
// ============================================================================
static void send_data_to(const char *dst_str, const char *txt, bool ackreq)
{
    uint8_t buf[sizeof(mr_hdr_v5_t) + MAX_PAYLOAD]={0};
    mr_hdr_v5_t *h=(mr_hdr_v5_t*)buf;

    h->magic[0]='M'; h->magic[1]='R';
    h->version=MR_PROTO_VERSION;
    h->flags=MR_FLAG_DATA | (ackreq ? MR_FLAG_ACKREQ : 0);
    h->ttl=DATA_TTL;
    h->msg_id=g_msg_id++;
    h->seq=g_my_seq;

    call7_set(h->src, MR_CALLSIGN);
    call7_set(h->last_hop, MR_CALLSIGN);

    char dst7[7]; call7_set(dst7, dst_str);
    memcpy(h->final_dst, dst7, 7);

    // next_hop aus Routing
    char nh[7]; bool has=false;
    xSemaphoreTake(g_mutex,portMAX_DELAY);
    has = route_lookup_locked(dst7, nh);
    bool bc = g_bc_fallback;
    xSemaphoreGive(g_mutex);

    if(has) memcpy(h->next_hop, nh, 7);
    else call7_set(h->next_hop, bc ? "*" : "NONE");

    size_t n=strlen(txt);
    if(n>MAX_PAYLOAD) n=MAX_PAYLOAD;
    h->payload_len=(uint8_t)n;
    memcpy(buf+sizeof(mr_hdr_v5_t), txt, n);

    // Drop wenn keine Route und Broadcast-Fallback aus
    if(h->next_hop[0]=='N' && h->next_hop[1]=='O'){
        ESP_LOGW(TAG,"TX DATA blocked: no route (bc_fallback=0)");
        return;
    }

    // ETX Statistik:
    // Nur wenn next_hop ein konkreter Neighbor ist (kein "*")
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    if(!call7_is_wild(h->next_hop)){
        neighbor_tx_attempt_locked(h->next_hop);

        if(ackreq){
            pending_add_locked(h->msg_id, h->next_hop, buf, (uint16_t)(sizeof(mr_hdr_v5_t)+n));
        }
    }
    xSemaphoreGive(g_mutex);

    lora_send(buf, sizeof(mr_hdr_v5_t)+n);
}


// ============================================================================
// FORWARD: nur wenn wir next_hop sind (oder fallback "*"), TTL--
// ============================================================================
static void forward_data(uint8_t *buf, size_t len)
{
    if(len < sizeof(mr_hdr_v5_t)) return;
    mr_hdr_v5_t *h=(mr_hdr_v5_t*)buf;

    if(h->ttl <= 1) return;
    h->ttl--;

    call7_set(h->last_hop, MR_CALLSIGN);

    // next_hop neu bestimmen
    char nh[7]; bool has=false;
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    has = route_lookup_locked(h->final_dst, nh);
    bool bc = g_bc_fallback;
    xSemaphoreGive(g_mutex);

    if(has) memcpy(h->next_hop, nh, 7);
    else call7_set(h->next_hop, bc ? "*" : "NONE");

    if(h->next_hop[0]=='N' && h->next_hop[1]=='O') return;

    // ETX Statistik auf dem Forward-Link
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    if(!call7_is_wild(h->next_hop)){
        neighbor_tx_attempt_locked(h->next_hop);

        if((h->flags & MR_FLAG_ACKREQ) != 0){
            pending_add_locked(h->msg_id, h->next_hop, buf, (uint16_t)len);
        }
    }
    xSemaphoreGive(g_mutex);

    lora_send(buf, len);
}


// ============================================================================
// RX
// ============================================================================
static void handle_rx(void)
{
    uint8_t irq=lora_rd(REG_IRQ_FLAGS);
    if(!(irq&IRQ_RX_DONE)) return;

    uint8_t len=lora_rd(REG_RX_NB_BYTES);
    uint8_t addr=lora_rd(REG_FIFO_RX_CURRENT_ADDR);
    lora_wr(REG_FIFO_ADDR_PTR,addr);

    uint8_t buf[256];
    for(int i=0;i<len;i++) buf[i]=lora_rd(REG_FIFO);

    lora_clear();
    if(len < sizeof(mr_hdr_v5_t)) return;

    mr_hdr_v5_t *h=(mr_hdr_v5_t*)buf;
    if(h->magic[0]!='M'||h->magic[1]!='R') return;
    if(h->version!=MR_PROTO_VERSION) return;

    // RSSI lesen (für Nachbarn + Anzeige)
    int rssi=(int)lora_rd(REG_PKT_RSSI_VALUE)-157;

    // ------------------------------------------------------------------------
    // ACK: ACK darf NICHT durch seen_cache gefiltert werden (FIX-5)
    // ------------------------------------------------------------------------
    if(h->flags & MR_FLAG_ACK){
        // Neighbor RSSI Update (wir sehen last_hop)
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        neighbor_update_rssi_locked(h->last_hop, rssi);

        // ACK kommt vom Absender h->src (nicht last_hop!),
        // aber in unserem Hop-by-Hop Modell ist src==Neighbor.
        neighbor_ack_ok_locked(h->src);

        bool ok = pending_mark_acked_locked(h->msg_id, h->src);
        xSemaphoreGive(g_mutex);

        if(ok){
            char s[8]; call7_to_str(s, h->src);
            ESP_LOGI(TAG,"ACK OK msg=%u from=%s", h->msg_id, s);
        }
        return;
    }

    // ------------------------------------------------------------------------
    // DATA/BEACON: Duplikatfilter anwenden
    // ------------------------------------------------------------------------
    if(seen_before(h->src,h->msg_id)) return;
    remember_msg(h->src,h->msg_id);

    // Neighbor RSSI Update (wir sehen last_hop)
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    neighbor_update_rssi_locked(h->last_hop, rssi);
    xSemaphoreGive(g_mutex);

    // ------------------------------------------------------------------------
    // BEACON -> Route Learning (dst=src via last_hop)
    //            Metric = ETX des Links zu last_hop
    // ------------------------------------------------------------------------
    if(h->flags & MR_FLAG_BEACON){
        xSemaphoreTake(g_mutex, portMAX_DELAY);

        int ni = neighbor_ensure_locked(h->last_hop);
        uint16_t etx = 100;
        if(ni >= 0){
            neighbor_decay_locked(&neighbors[ni]);
            etx = etx_compute_x100(neighbors[ni].tx_attempts, neighbors[ni].ack_ok);
        }

        route_update_locked(h->src, h->last_hop, h->seq, etx, rssi);

        xSemaphoreGive(g_mutex);
        return;
    }

    // ------------------------------------------------------------------------
    // DATA -> deliver / forward / ACK wenn angefordert
    // ------------------------------------------------------------------------
    if(h->flags & MR_FLAG_DATA){
        char my7[7]; call7_set(my7, MR_CALLSIGN);

        bool iam_dst = call7_eq(h->final_dst, my7);
        bool iam_nexthop = call7_eq(h->next_hop, my7);
        bool nh_wild = call7_is_wild(h->next_hop);
        bool nh_none = (h->next_hop[0]=='N' && h->next_hop[1]=='O');

        if(iam_dst){
            char txt[MAX_PAYLOAD+1]={0};
            if(h->payload_len>0 && h->payload_len<=MAX_PAYLOAD){
                memcpy(txt, buf+sizeof(mr_hdr_v5_t), h->payload_len);
                txt[h->payload_len]=0;
            }
            ESP_LOGI(TAG,"DATA delivered ✅ \"%s\"", txt);

            // Hop-by-Hop ACK: wir bestätigen dem vorherigen Hop
            if((h->flags & MR_FLAG_ACKREQ) != 0){
                send_ack(h);
            }
            return;
        }

        if(nh_none) return;

        if((iam_nexthop || nh_wild) && h->ttl > 1){
            forward_data(buf, len);

            // Forwarder bestätigt ebenfalls dem vorherigen Hop (simple & effektiv)
            if((h->flags & MR_FLAG_ACKREQ) != 0){
                send_ack(h);
            }
        }
        return;
    }
}


// ============================================================================
// DIO0 ISR + Task
// ============================================================================
static void IRAM_ATTR dio_isr(void*arg)
{
    uint32_t n=(uint32_t)arg;
    xQueueSendFromISR(dio_q,&n,NULL);
}

static void dio_task(void*arg)
{
    (void)arg;
    uint32_t io;
    while(1){
        if(xQueueReceive(dio_q,&io,portMAX_DELAY))
            handle_rx();
    }
}


// ============================================================================
// SPI INIT
// ============================================================================
static void init_spi(void)
{
    spi_bus_config_t bus={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LORA_SPI_HOST,&bus,SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev={
        .clock_speed_hz=1000000,
        .mode=0,
        .spics_io_num=PIN_NUM_CS,
        .queue_size=1
    };
    ESP_ERROR_CHECK(spi_bus_add_device(LORA_SPI_HOST,&dev,&lora_spi));
}


// ============================================================================
// WIFI AP
// ============================================================================
static void wifi_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg=WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap={0};
    strcpy((char*)ap.ap.ssid, MR_WIFI_AP_SSID);
    ap.ap.authmode=WIFI_AUTH_OPEN;
    ap.ap.max_connection=4;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP,&ap));
    ESP_ERROR_CHECK(esp_wifi_start());
}


// ============================================================================
// HTTP helpers (Form) – wie Kapitel 25
// ============================================================================
static esp_err_t http_send_text(httpd_req_t *req, const char *txt)
{
    httpd_resp_set_type(req,"text/plain");
    return httpd_resp_send(req,txt,HTTPD_RESP_USE_STRLEN);
}

static bool http_read_body(httpd_req_t *req, char *buf, size_t buf_sz)
{
    if(req->content_len<=0 || req->content_len >= (int)buf_sz) return false;
    int r=httpd_req_recv(req,buf,req->content_len);
    if(r<=0) return false;
    buf[r]=0;
    return true;
}

static void url_decode_inplace(char *s)
{
    char *p=s,*o=s;
    while(*p){
        if(*p=='+'){*o++=' ';p++;continue;}
        if(*p=='%' && p[1] && p[2]){
            char hex[3]={p[1],p[2],0};
            *o++=(char)strtoul(hex,NULL,16);
            p+=3; continue;
        }
        *o++=*p++;
    }
    *o=0;
}

// FIX: restores '=' also on match
static bool form_get(char *body,const char *key,char *out,size_t out_sz)
{
    size_t klen=strlen(key);
    char *p=body;

    while(p && *p){
        char *amp=strchr(p,'&');
        if(amp) *amp=0;

        char *eq=strchr(p,'=');
        if(eq){
            *eq=0;
            char *k=p;
            char *v=eq+1;

            url_decode_inplace(k);
            url_decode_inplace(v);

            if(strlen(k)==klen && strcmp(k,key)==0){
                strncpy(out,v,out_sz-1);
                out[out_sz-1]=0;

                *eq='=';
                if(amp) *amp='&';
                return true;
            }
            *eq='=';
        }

        if(amp){
            *amp='&';
            p=amp+1;
        }else{
            break;
        }
    }
    return false;
}


// ============================================================================
// Web UI – Dashboard (mit Fehleranzeige, wenn /api/status kaputt ist)
// ============================================================================
static const char *INDEX_HTML =
"<!doctype html><html><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>MeshRadio Kapitel 27</title>"
"<style>"
"body{font-family:system-ui;margin:16px;max-width:1100px}"
".row{display:grid;grid-template-columns:1fr;gap:14px}"
"@media(min-width:900px){.row{grid-template-columns:1fr 1fr}}"
"button{padding:10px 14px;margin:6px 4px;font-size:16px}"
"input{padding:10px;margin:6px 0;font-size:16px;width:100%}"
"table{border-collapse:collapse;width:100%;font-size:14px}"
"th,td{border-bottom:1px solid #ddd;padding:6px 6px;text-align:left;white-space:nowrap}"
"th{position:sticky;top:0;background:#f6f6f6}"
"code{background:#eee;padding:2px 4px}"
".card{border:1px solid #ddd;border-radius:12px;padding:12px;box-shadow:0 1px 3px rgba(0,0,0,.06)}"
".muted{color:#666}"
".err{color:#b00020;font-weight:600}"
"</style></head><body>"
"<h2>MeshRadio Kapitel 27 – ETX-Light Routing</h2>"
"<p class='muted'>AP: <code>" MR_WIFI_AP_SSID "</code> • URL: <code>http://192.168.4.1</code></p>"
"<div class='card'>"
"<div><b>Status:</b> <span id='st'>loading…</span></div>"
"<div id='err' class='err'></div>"
"<button onclick='refreshAll()'>Refresh</button>"
"<button onclick='setBC(1)'>BC fallback ON</button>"
"<button onclick='setBC(0)'>BC fallback OFF</button>"
"</div>"
"<div class='row'>"
" <div class='card'>"
"  <h3>DATA senden</h3>"
"  <label>Ziel (dst)</label><input id='dst' value='DJ1ABC'>"
"  <label>Text</label><input id='msg' value='Hello 27'>"
"  <label>ACK (0/1)</label><input id='ack' value='1'>"
"  <button onclick='send()'>SEND</button>"
"  <div id='m' class='muted'></div>"
" </div>"
" <div class='card'>"
"  <h3>Routing (dst → next)</h3>"
"  <div class='muted'>Metric = ETX (kleiner ist besser)</div>"
"  <div style='overflow:auto;max-height:360px'>"
"   <table id='rt'><thead><tr>"
"    <th>dst</th><th>next</th><th>seq</th><th>ETX</th><th>age_ms</th>"
"   </tr></thead><tbody></tbody></table>"
"  </div>"
" </div>"
"</div>"
"<div class='card'>"
" <h3>Nachbarn (Link-Stats)</h3>"
" <div class='muted'>ETX basiert auf TX/ACK zu diesem Nachbarn (mit Decay).</div>"
" <div style='overflow:auto;max-height:420px'>"
"  <table id='nb'><thead><tr>"
"   <th>call</th><th>rssi</th><th>tx</th><th>ack</th><th>ETX</th><th>age_ms</th>"
"  </tr></thead><tbody></tbody></table>"
" </div>"
"</div>"
"<script>"
"function esc(s){return (s||'').toString().replace(/[&<>]/g,c=>({"
" '&':'&amp;','<':'&lt;','>':'&gt;'"
"}[c]));}"
"async function post(url,body){"
" let r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body});"
" return await r.text();"
"}"
"function etxFmt(x100){ if(x100===undefined||x100===null) return '-'; return (x100/100).toFixed(2); }"
"async function send(){"
" let dst=document.getElementById('dst').value;"
" let msg=document.getElementById('msg').value;"
" let ack=document.getElementById('ack').value;"
" let t=await post('/api/send','dst='+encodeURIComponent(dst)+'&msg='+encodeURIComponent(msg)+'&ack='+encodeURIComponent(ack));"
" document.getElementById('m').textContent=t;"
" refreshAll();"
"}"
"async function setBC(v){"
" let t=await post('/api/bcfallback','enable='+encodeURIComponent(v));"
" document.getElementById('m').textContent=t;"
" refreshAll();"
"}"
"async function refreshAll(){"
" document.getElementById('err').textContent='';"
" try{"
"  let r=await fetch('/api/status');"
"  let txt=await r.text();"
"  let j=JSON.parse(txt);"
"  document.getElementById('st').textContent="
"    'call='+j.call+' • bc_fallback='+j.bc_fallback+' • routes='+j.routes.length+' • neighbors='+j.neighbors.length;"
"  let rb=document.querySelector('#rt tbody'); rb.innerHTML='';"
"  for(let x of j.routes){"
"    let tr=document.createElement('tr');"
"    tr.innerHTML='<td>'+esc(x.dst)+'</td><td>'+esc(x.next)+'</td><td>'+x.seq+'</td><td>'+etxFmt(x.etx_x100)+'</td><td>'+x.age_ms+'</td>';"
"    rb.appendChild(tr);"
"  }"
"  let nb=document.querySelector('#nb tbody'); nb.innerHTML='';"
"  for(let n of j.neighbors){"
"    let tr=document.createElement('tr');"
"    tr.innerHTML='<td>'+esc(n.call)+'</td><td>'+n.rssi+'</td><td>'+n.tx+'</td><td>'+n.ack+'</td><td>'+etxFmt(n.etx_x100)+'</td><td>'+n.age_ms+'</td>';"
"    nb.appendChild(tr);"
"  }"
" }catch(e){"
"  document.getElementById('st').textContent='ERROR';"
"  document.getElementById('err').textContent='Dashboard error: '+e;"
" }"
"}"
"refreshAll();"
"setInterval(refreshAll, 3000);"
"</script></body></html>";

static esp_err_t index_get(httpd_req_t *req)
{
    httpd_resp_set_type(req,"text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}


// ============================================================================
// API: /api/status (JSON)
//   - FIX: sichere Buffer-Begrenzung -> immer gültiges JSON liefern
// ============================================================================
static esp_err_t api_status_get(httpd_req_t *req)
{
    static char out[6144]; // etwas größer für JSON (kleines Mesh ok)
    size_t off=0;

    xSemaphoreTake(g_mutex, portMAX_DELAY);

    uint32_t t = now_ms();

    // Header
    off += snprintf(out+off, sizeof(out)-off,
        "{"
        "\"call\":\"%s\","
        "\"bc_fallback\":%u,"
        "\"neighbors\":[",
        MR_CALLSIGN,
        g_bc_fallback?1:0
    );

    // Neighbors
    bool first=true;
    for(int i=0;i<MAX_NEIGHBORS;i++){
        if(!neighbors[i].used) continue;

        neighbor_decay_locked(&neighbors[i]);

        uint16_t etx = etx_compute_x100(neighbors[i].tx_attempts, neighbors[i].ack_ok);
        char c[8]; call7_to_str(c, neighbors[i].call);
        uint32_t age = t - neighbors[i].t_ms;

        // Wenn Buffer knapp wird, abbrechen – aber JSON sauber schließen!
        if(off > sizeof(out)-512) break;

        off += snprintf(out+off, sizeof(out)-off,
            "%s{\"call\":\"%s\",\"rssi\":%d,\"tx\":%" PRIu32 ",\"ack\":%" PRIu32 ",\"etx_x100\":%u,\"age_ms\":%" PRIu32 "}",
            first?"":",", c, neighbors[i].rssi,
            neighbors[i].tx_attempts, neighbors[i].ack_ok, etx, age
        );

        first=false;
    }

    // Routes
    off += snprintf(out+off, sizeof(out)-off, "],\"routes\":[");
    first=true;

    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used) continue;

        char d[8], n[8];
        call7_to_str(d, routes[i].dst);
        call7_to_str(n, routes[i].next);
        uint32_t age = t - routes[i].t_ms;

        if(off > sizeof(out)-512) break;

        off += snprintf(out+off, sizeof(out)-off,
            "%s{\"dst\":\"%s\",\"next\":\"%s\",\"seq\":%u,\"etx_x100\":%u,\"age_ms\":%" PRIu32 "}",
            first?"":",", d, n, (unsigned)routes[i].seq, (unsigned)routes[i].etx_x100, age
        );

        first=false;
    }

    // Footer
    off += snprintf(out+off, sizeof(out)-off, "]}");

    xSemaphoreGive(g_mutex);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, out, HTTPD_RESP_USE_STRLEN);
}


// ============================================================================
// API: /api/send
// ============================================================================
static esp_err_t api_send_post(httpd_req_t *req)
{
    char body[256];
    if(!http_read_body(req, body, sizeof(body))) return http_send_text(req, "ERR body");

    char dst[16]={0}, msg[160]={0}, ackv[8]={0};
    if(!form_get(body,"dst",dst,sizeof(dst))) return http_send_text(req,"ERR missing dst");
    if(!form_get(body,"msg",msg,sizeof(msg))) return http_send_text(req,"ERR missing msg");
    form_get(body,"ack",ackv,sizeof(ackv));

    bool ack = (ackv[0] != '0'); // default ON
    send_data_to(dst, msg, ack);

    return http_send_text(req, ack ? "OK send (ack=1)" : "OK send (ack=0)");
}


// ============================================================================
// API: /api/bcfallback
// ============================================================================
static esp_err_t api_bcfallback_post(httpd_req_t *req)
{
    char body[64];
    if(!http_read_body(req, body, sizeof(body))) return http_send_text(req, "ERR body");

    char v[8]={0};
    if(!form_get(body, "enable", v, sizeof(v))) return http_send_text(req, "ERR missing enable");

    int en = (v[0]=='1');

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_bc_fallback = en;
    xSemaphoreGive(g_mutex);

    return http_send_text(req, en ? "OK bc_fallback=1" : "OK bc_fallback=0");
}


// ============================================================================
// HTTP start
// ============================================================================
static void http_start(void)
{
    httpd_config_t cfg=HTTPD_DEFAULT_CONFIG();

    // FIX: etwas mehr Stack, weil Dashboard JSON + Tables
    cfg.stack_size=16384;
    cfg.max_uri_handlers=16;

    ESP_ERROR_CHECK(httpd_start(&g_http,&cfg));

    httpd_uri_t u0={.uri="/",.method=HTTP_GET,.handler=index_get};
    httpd_register_uri_handler(g_http,&u0);

    httpd_uri_t st0={.uri="/api/status",.method=HTTP_GET,.handler=api_status_get};
    httpd_register_uri_handler(g_http,&st0);

    httpd_uri_t s0={.uri="/api/send",.method=HTTP_POST,.handler=api_send_post};
    httpd_register_uri_handler(g_http,&s0);

    httpd_uri_t b0={.uri="/api/bcfallback",.method=HTTP_POST,.handler=api_bcfallback_post};
    httpd_register_uri_handler(g_http,&b0);

    ESP_LOGI(TAG,"HTTP server started");
}


// ============================================================================
// MAIN
// ============================================================================
void app_main(void)
{
    ESP_LOGI(TAG,"MeshRadio Kapitel 27 start (ETX-Light + Dashboard)");

    g_mutex=xSemaphoreCreateMutex();
    if(!g_mutex){ ESP_LOGE(TAG,"mutex failed"); abort(); }

    ESP_ERROR_CHECK(nvs_flash_init());

    init_spi();
    lora_reset();
    ESP_LOGI(TAG,"SX1276 RegVersion=0x%02X", lora_rd(REG_VERSION));

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    lora_apply_locked();
    xSemaphoreGive(g_mutex);

    // DIO0 + RX task
    dio_q=xQueueCreate(10,sizeof(uint32_t));
    gpio_config_t io={
        .intr_type=GPIO_INTR_POSEDGE,
        .mode=GPIO_MODE_INPUT,
        .pin_bit_mask=(1ULL<<PIN_NUM_DIO0),
        .pull_up_en=1
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_NUM_DIO0, dio_isr, (void*)PIN_NUM_DIO0));
    xTaskCreate(dio_task, "dio", 4096, NULL, 10, NULL);

    // Retry task (ACK/Retry -> ETX)
    xTaskCreate(retry_task, "retry", 4096, NULL, 6, NULL);

    // WiFi + HTTP
    wifi_ap();
    http_start();

    beacon_schedule_next();

    ESP_LOGI(TAG,"CALL=%s", MR_CALLSIGN);
    ESP_LOGI(TAG,"Open http://192.168.4.1");
    ESP_LOGI(TAG,"BC fallback = %u", g_bc_fallback?1:0);

    while(1){
        vTaskDelay(pdMS_TO_TICKS(50));

        if(g_beacon_enabled && now_ms() > g_next_beacon_ms)
            send_beacon();

        xSemaphoreTake(g_mutex, portMAX_DELAY);
        neighbor_cleanup_locked();
        route_cleanup_locked();
        xSemaphoreGive(g_mutex);
    }
}