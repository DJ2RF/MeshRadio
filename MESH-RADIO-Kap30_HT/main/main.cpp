// ============================================================================
// MeshRadio – Kapitel 30_HT (AES-CCM + Replay) PORT für Kapitel 30 HELTEC V3.x
// ----------------------------------------------------------------------------
// Ziel: Original Kap30_HT (SX1276 Registerdriver) auf HELTEC WiFi LoRa 32 V3.x
//       (ESP32-S3 + SX1262) portieren.
//
// Wichtige Änderungen gegenüber Kap30_HT Original:
//   1) SX1276 Register-SPI entfernt -> RadioLib SX1262
//   2) KEIN parallel TX/RX aus verschiedenen Tasks (RadioLib nicht threadsafe)
//      - HTTP Handler / Retry Task dürfen NICHT direkt senden
//      - Stattdessen: TX Queue -> wird im main loop seriell abgearbeitet
//   3) RX wird im main loop gepollt (wie bei deinem Kap30 funktionierenden Setup)
//
// Voraussetzung:
//   - main/EspHal.h vorhanden (HAL für RadioLib unter ESP-IDF)
//   - managed_components/jgromes__radiolib Version 7.6.0
// ============================================================================


// ============================================================================
// USER DEFINES (WICHTIG – ganz am Anfang)
// ============================================================================
#define MR_CALLSIGN        "DJ2RFF"
#define MR_WIFI_AP_SSID    "MeshRadio-Setup3"
#define MR_WIFI_AP_PASS    ""

// Default Control-Plane (Kap.28)
#define DEFAULT_BEACON_INTERVAL_MS 30000
#define BEACON_JITTER_MS           2000

#define ACK_TIMEOUT_MS     1200
#define ACK_RETRY_MAX      2
#define ACK_BACKOFF_MS     350

#define ETX_MAX_X100       1000
#define ETX_DECAY_MS       60000

#define DEFAULT_ROUTEADV_ENABLE    1
#define DEFAULT_ROUTEADV_TOPN      8
#define DEFAULT_ROUTEADV_DELTA_ETX 30
#define DEFAULT_HOLDDOWN_MS        20000

#define RL_BEACON_TPS      0.05f
#define RL_BEACON_BURST    1.0f
#define RL_ROUTEADV_TPS    0.20f
#define RL_ROUTEADV_BURST  2.0f
#define RL_ACK_TPS         1.50f
#define RL_ACK_BURST       5.0f
#define RL_DATA_TPS        0.50f
#define RL_DATA_BURST      3.0f

#define DEFAULT_CAD_ENABLE 0
#define CAD_WAIT_MS        80
#define CAD_JITTER_MS      120

// Kapitel 30_HT Security Defaults
#define DEFAULT_CRYPTO_ENABLE  1

// 16-Byte Netzwerkschlüssel als HEX (32 Zeichen). Beispiel (BITTE ändern!):
#define MR_NET_KEY_HEX "00112233445566778899AABBCCDDEEFF"

// Network-ID (1 Byte) – fließt in Nonce ein
#define MR_NET_ID 0x42

// CCM Parameter
#define SEC_KEY_LEN        16
#define SEC_NONCE_LEN      12
#define SEC_TAG_LEN        8   // 8 Byte Tag spart Airtime


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

#include "esp_log.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_err.h"

#include "nvs_flash.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"

// ESP-IDF: mbedTLS (AES-CCM)
#include "mbedtls/ccm.h"

// RadioLib + ESP-IDF HAL
#include <RadioLib.h>
#include "EspHal.h"


// ============================================================================
// HELTEC V3.x (ESP32-S3 + SX1262) Pinmap
// ----------------------------------------------------------------------------
// Control Pins:
#define PIN_LORA_CS    8
#define PIN_LORA_DIO1  14
#define PIN_LORA_RST   12
#define PIN_LORA_BUSY  13
// SPI pins (typisch Heltec V3.x):
#define PIN_SPI_SCK    9
#define PIN_SPI_MISO   11
#define PIN_SPI_MOSI   10
// ============================================================================


// ============================================================================
// PROTOKOLL
// ============================================================================
#define MR_PROTO_VERSION 7

#define MR_FLAG_DATA      0x10
#define MR_FLAG_BEACON    0x20
#define MR_FLAG_ACK       0x40
#define MR_FLAG_ROUTEADV  0x80

// Zusatzflags innerhalb DATA
#define MR_FLAG_ACKREQ    0x01
#define MR_FLAG_SEC       0x08   // DATA-Payload ist AES-CCM verschlüsselt

#define DATA_TTL   4
#define BEACON_TTL 2
#define ACK_TTL    4
#define ADV_TTL    3

#define MAX_PAYLOAD      120
#define MAX_PLAINTEXT    (MAX_PAYLOAD - SEC_TAG_LEN)  // Platz für Cipher+Tag in payload


// ============================================================================
// LIMITS / TABLES
// ============================================================================
#define SEEN_CACHE_SIZE 48

#define MAX_NEIGHBORS 24
#define NEIGHBOR_TIMEOUT_MS 60000

#define MAX_ROUTES 32
#define ROUTE_TIMEOUT_MS 180000

#define MAX_PENDING_ACK  10

// RouteAdv payload: dst(7) + next(7) + etx_x100(2) + seq(2)
#define ROUTEADV_PL_LEN   (7+7+2+2)

// Replay Cache (pro src last seen seq)
#define MAX_REPLAY 24


// ============================================================================
// HEADER
// ============================================================================
#pragma pack(push,1)
typedef struct {
    uint8_t magic[2];
    uint8_t version;
    uint8_t flags;

    uint8_t ttl;
    uint16_t msg_id;

    uint16_t seq;

    char src[7];
    char final_dst[7];
    char next_hop[7];
    char last_hop[7];

    uint8_t payload_len; // bei SEC: cipher_len + TAG_LEN
} mr_hdr_v7_t;
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

    uint32_t tx_attempts;
    uint32_t ack_ok;
    uint32_t last_decay_ms;
} neighbor_t;

typedef struct {
    bool used;
    char dst[7];
    char next[7];

    uint16_t seq;
    uint16_t etx_x100;

    int last_rssi;
    uint32_t t_ms;

    uint32_t hold_until_ms;
} route_t;

typedef struct {
    bool used;
    uint16_t msg_id;
    char expect_from[7];
    uint32_t deadline_ms;
    uint8_t retries_left;

    uint8_t frame[sizeof(mr_hdr_v7_t) + MAX_PAYLOAD];
    uint16_t frame_len;
} pending_ack_t;

typedef struct {
    float tokens;
    float tps;
    float burst;
    uint32_t last_ms;
} bucket_t;

typedef struct {
    bool used;
    char src[7];
    uint16_t last_seq;
    uint32_t t_ms;
} replay_t;


// ============================================================================
// GLOBALS
// ============================================================================
static const char *TAG="MR30-HLTC";

static SemaphoreHandle_t g_mutex;
static httpd_handle_t g_http=NULL;

static uint16_t g_msg_id=1;
static uint16_t g_my_seq=1;

static seen_t seen_cache[SEEN_CACHE_SIZE];
static neighbor_t neighbors[MAX_NEIGHBORS];
static route_t routes[MAX_ROUTES];
static pending_ack_t pend[MAX_PENDING_ACK];

// Kap.30_HT Replay cache
static replay_t replay_tab[MAX_REPLAY];

// Control switches
static bool g_bc_fallback=true;
static bool g_beacon_enabled=true;
static bool g_routeadv_enable=DEFAULT_ROUTEADV_ENABLE;
static bool g_cad_enable=DEFAULT_CAD_ENABLE;

// Kap.30_HT Crypto
static bool g_crypto_enable=DEFAULT_CRYPTO_ENABLE;
static uint8_t g_net_key[SEC_KEY_LEN];

// Security counters
static uint32_t sec_decrypt_ok=0;
static uint32_t sec_decrypt_fail=0;
static uint32_t sec_mac_fail=0;
static uint32_t sec_replay_drop=0;

// Tunables
static uint32_t g_beacon_interval_ms=DEFAULT_BEACON_INTERVAL_MS;
static uint8_t  g_routeadv_topn=DEFAULT_ROUTEADV_TOPN;
static uint16_t g_routeadv_delta_etx=DEFAULT_ROUTEADV_DELTA_ETX;
static uint32_t g_holddown_ms=DEFAULT_HOLDDOWN_MS;

// Next schedules
static uint32_t g_next_beacon_ms=0;
static uint32_t g_next_routeadv_ms=0;

// Counters for UI
static uint32_t c_tx_beacon=0, c_tx_routeadv=0, c_tx_ack=0, c_tx_data=0;
static uint32_t c_defer_beacon=0, c_defer_routeadv=0, c_defer_ack=0, c_defer_data=0;
static uint32_t c_drop_routeadv=0, c_drop_data=0;

// Rate limit buckets
static bucket_t b_beacon, b_routeadv, b_ack, b_data;


// ============================================================================
// RadioLib objects (SX1262 on Heltec)
// ============================================================================
static EspHal hal;
static Module mod(&hal, PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RST, PIN_LORA_BUSY);
static SX1262 radio(&mod);

// TX done flag (interrupt-based)
static volatile bool g_tx_done = false;
static void IRAM_ATTR onPacketSent(void) { g_tx_done = true; }


// ============================================================================
// TX queue (damit HTTP/Retry nicht direkt am Radio funkt)
// ============================================================================
typedef struct {
  uint8_t frame[sizeof(mr_hdr_v7_t) + MAX_PAYLOAD];
  uint16_t len;
} tx_item_t;

static QueueHandle_t tx_q = NULL;


// ============================================================================
// Helpers
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
// Token Bucket (Rate Limit)
// ============================================================================
static void bucket_init(bucket_t *b, float tps, float burst)
{
    b->tps=tps;
    b->burst=burst;
    b->tokens=burst;
    b->last_ms=now_ms();
}

static void bucket_refill(bucket_t *b)
{
    uint32_t t=now_ms();
    uint32_t dt = t - b->last_ms;
    b->last_ms=t;

    float add = (dt/1000.0f) * b->tps;
    b->tokens += add;
    if(b->tokens > b->burst) b->tokens = b->burst;
}

static bool bucket_take(bucket_t *b)
{
    bucket_refill(b);
    if(b->tokens >= 1.0f){
        b->tokens -= 1.0f;
        return true;
    }
    return false;
}


// ============================================================================
// SEEN (Duplikatfilter für DATA/BEACON/ROUTEADV; ACK nicht filtern)
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
// ROUTES (seq + ETX + Hold-Down)
// ============================================================================
static bool holddown_active_locked(route_t *r)
{
    uint32_t t=now_ms();
    return (t < r->hold_until_ms);
}

static void route_update_locked(const char dst[7],
                                const char next[7],
                                uint16_t seq,
                                uint16_t etx_x100,
                                int last_rssi)
{
    char me[7]; call7_set(me, MR_CALLSIGN);
    if(call7_eq(next, me)) return;

    uint32_t t=now_ms();

    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used) continue;
        if(!call7_eq(routes[i].dst, dst)) continue;

        bool replace=false;

        if(seq > routes[i].seq){
            replace=true;
        }else if(seq == routes[i].seq){
            if(holddown_active_locked(&routes[i])){
                if(etx_x100 + (2*g_routeadv_delta_etx) < routes[i].etx_x100) replace=true;
            }else{
                if(etx_x100 < routes[i].etx_x100) replace=true;
            }
        }

        if(replace){
            bool next_changed = !call7_eq(routes[i].next, next);

            memcpy(routes[i].next, next, 7);
            routes[i].seq = seq;
            routes[i].etx_x100 = etx_x100;
            routes[i].last_rssi = last_rssi;

            if(next_changed){
                routes[i].hold_until_ms = t + g_holddown_ms;
            }
        }

        routes[i].t_ms = t;
        return;
    }

    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used){
            routes[i].used=true;
            memcpy(routes[i].dst, dst, 7);
            memcpy(routes[i].next, next, 7);
            routes[i].seq=seq;
            routes[i].etx_x100=etx_x100;
            routes[i].last_rssi=last_rssi;
            routes[i].t_ms=t;
            routes[i].hold_until_ms = t + 1000;
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
// Kapitel 30_HT: Replay-Schutz (pro src)
// ============================================================================
static replay_t* replay_get_locked(const char src[7])
{
    for(int i=0;i<MAX_REPLAY;i++){
        if(!replay_tab[i].used) continue;
        if(call7_eq(replay_tab[i].src, src)) return &replay_tab[i];
    }
    for(int i=0;i<MAX_REPLAY;i++){
        if(!replay_tab[i].used){
            replay_tab[i].used=true;
            memcpy(replay_tab[i].src, src, 7);
            replay_tab[i].last_seq=0;
            replay_tab[i].t_ms=now_ms();
            return &replay_tab[i];
        }
    }
    int oldest=0;
    uint32_t ot=replay_tab[0].t_ms;
    for(int i=1;i<MAX_REPLAY;i++){
        if(!replay_tab[i].used){ oldest=i; break; }
        if(replay_tab[i].t_ms < ot){ oldest=i; ot=replay_tab[i].t_ms; }
    }
    replay_tab[oldest].used=true;
    memcpy(replay_tab[oldest].src, src, 7);
    replay_tab[oldest].last_seq=0;
    replay_tab[oldest].t_ms=now_ms();
    return &replay_tab[oldest];
}

static bool replay_check_locked(const char src[7], uint16_t seq)
{
    replay_t *r = replay_get_locked(src);
    r->t_ms = now_ms();
    return (seq > r->last_seq);
}

static void replay_update_locked(const char src[7], uint16_t seq)
{
    replay_t *r = replay_get_locked(src);
    r->last_seq = seq;
    r->t_ms = now_ms();
}


// ============================================================================
// Kapitel 30_HT: AES-CCM helpers
// ============================================================================
static void sec_make_nonce(uint8_t nonce[SEC_NONCE_LEN], const mr_hdr_v7_t *h)
{
    memcpy(nonce, h->src, 7);
    nonce[7] = (uint8_t)(h->seq & 0xFF);
    nonce[8] = (uint8_t)(h->seq >> 8);
    nonce[9] = (uint8_t)(h->msg_id & 0xFF);
    nonce[10]= (uint8_t)(h->msg_id >> 8);
    nonce[11]= (uint8_t)MR_NET_ID;
}

static bool sec_encrypt_payload(const mr_hdr_v7_t *h,
                                const uint8_t *plain, size_t plain_len,
                                uint8_t *out_cipher, uint8_t out_tag[SEC_TAG_LEN])
{
    uint8_t nonce[SEC_NONCE_LEN];
    sec_make_nonce(nonce, h);

    mbedtls_ccm_context ctx;
    mbedtls_ccm_init(&ctx);

    if(mbedtls_ccm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, g_net_key, SEC_KEY_LEN*8) != 0){
        mbedtls_ccm_free(&ctx);
        return false;
    }

    const uint8_t *aad = (const uint8_t*)h;
    size_t aad_len = sizeof(mr_hdr_v7_t);

    int rc = mbedtls_ccm_encrypt_and_tag(
        &ctx,
        plain_len,
        nonce, SEC_NONCE_LEN,
        aad, aad_len,
        plain, out_cipher,
        out_tag, SEC_TAG_LEN
    );

    mbedtls_ccm_free(&ctx);
    return (rc == 0);
}

static bool sec_decrypt_payload(const mr_hdr_v7_t *h,
                                const uint8_t *cipher, size_t cipher_len,
                                const uint8_t tag[SEC_TAG_LEN],
                                uint8_t *out_plain)
{
    uint8_t nonce[SEC_NONCE_LEN];
    sec_make_nonce(nonce, h);

    mbedtls_ccm_context ctx;
    mbedtls_ccm_init(&ctx);

    if(mbedtls_ccm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, g_net_key, SEC_KEY_LEN*8) != 0){
        mbedtls_ccm_free(&ctx);
        return false;
    }

    const uint8_t *aad = (const uint8_t*)h;
    size_t aad_len = sizeof(mr_hdr_v7_t);

    int rc = mbedtls_ccm_auth_decrypt(
        &ctx,
        cipher_len,
        nonce, SEC_NONCE_LEN,
        aad, aad_len,
        cipher, out_plain,
        tag, SEC_TAG_LEN
    );

    mbedtls_ccm_free(&ctx);
    return (rc == 0);
}


// ============================================================================
// Radio TX (seriell, interrupt-basiert, wie Kap30)
// ============================================================================
static int radio_tx_blocking(const uint8_t* data, size_t len, uint32_t timeout_ms)
{
    // RX verlassen
    radio.standby();

    g_tx_done = false;

    int st = radio.startTransmit((uint8_t*)data, len);
    if(st != RADIOLIB_ERR_NONE) {
        radio.startReceive();
        return st;
    }

    uint32_t t0 = now_ms();
    while(!g_tx_done) {
        if(now_ms() - t0 > timeout_ms) {
            // Cleanup versuchen
            radio.finishTransmit();
            radio.startReceive();
            return RADIOLIB_ERR_TX_TIMEOUT; // -5
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    st = radio.finishTransmit();
    radio.startReceive();
    return st;
}


// ============================================================================
// CAD Backoff (optional) – bei SX1262 simpel als Random Delay umgesetzt
// ============================================================================
static void cad_backoff_if_enabled(void)
{
    if(!g_cad_enable) return;
    uint32_t w = (esp_random() % (CAD_JITTER_MS+1));
    vTaskDelay(pdMS_TO_TICKS(CAD_WAIT_MS + w));
}


// ============================================================================
// TX Queue push helper (für HTTP / Retry Task)
// ============================================================================
static void txq_push_frame(const uint8_t* frame, uint16_t frame_len)
{
    if(!tx_q) return;
    tx_item_t it = {};
    if(frame_len > sizeof(it.frame)) frame_len = sizeof(it.frame);
    memcpy(it.frame, frame, frame_len);
    it.len = frame_len;
    (void)xQueueSend(tx_q, &it, 0);
}


// ============================================================================
// Pending ACK / Retry – sendet NICHT direkt, sondern queued
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
                pending_clear_slot_locked(i);
                continue;
            }

            pend[i].retries_left--;
            pend[i].deadline_ms = t + ACK_TIMEOUT_MS;

            // backoff ausserhalb mutex
            uint32_t bo = (esp_random() % (ACK_BACKOFF_MS+1));
            xSemaphoreGive(g_mutex);
            vTaskDelay(pdMS_TO_TICKS(bo));
            xSemaphoreTake(g_mutex, portMAX_DELAY);

            mr_hdr_v7_t *h = (mr_hdr_v7_t*)pend[i].frame;
            if(pend[i].frame_len >= sizeof(mr_hdr_v7_t) && h->ttl > 1) h->ttl--;

            neighbor_tx_attempt_locked(pend[i].expect_from);

            if(bucket_take(&b_data)){
                c_tx_data++;
                // NICHT direkt senden -> queue
                txq_push_frame(pend[i].frame, pend[i].frame_len);
            }else{
                c_defer_data++;
            }
        }

        xSemaphoreGive(g_mutex);
    }
}


// ============================================================================
// Schedulers
// ============================================================================
static void beacon_schedule_next(void)
{
    g_next_beacon_ms = now_ms() + g_beacon_interval_ms + (esp_random()%BEACON_JITTER_MS);
}

static void routeadv_schedule_next(void)
{
    g_next_routeadv_ms = now_ms() + 20000 + (esp_random()%2000);
}


// ============================================================================
// TX: BEACON / ACK / ROUTEADV / DATA
// ----------------------------------------------------------------------------
// WICHTIG: Diese Funktionen bauen Frames; das eigentliche Senden läuft seriell
//          im main loop über tx_q (oder direkt, wenn wir im main loop sind).
//          -> Wir pushen auch hier in die Queue (einheitlich).
// ============================================================================
static void send_beacon_queue(void)
{
    mr_hdr_v7_t h={0};
    h.magic[0]='M'; h.magic[1]='R';
    h.version=MR_PROTO_VERSION;
    h.flags=MR_FLAG_BEACON;
    h.ttl=BEACON_TTL;
    h.msg_id=g_msg_id++;
    h.seq = g_my_seq++;

    call7_set(h.src, MR_CALLSIGN);
    call7_set(h.last_hop, MR_CALLSIGN);
    call7_set(h.final_dst,"*");
    call7_set(h.next_hop,"*");
    h.payload_len=0;

    if(bucket_take(&b_beacon)){
        c_tx_beacon++;
        txq_push_frame((uint8_t*)&h, sizeof(h));
    }else{
        c_defer_beacon++;
    }

    beacon_schedule_next();
}

static void send_ack_queue(const mr_hdr_v7_t *rx)
{
    mr_hdr_v7_t h={0};
    h.magic[0]='M'; h.magic[1]='R';
    h.version=MR_PROTO_VERSION;
    h.flags=MR_FLAG_ACK;
    h.ttl=ACK_TTL;
    h.msg_id = rx->msg_id;
    h.seq = g_my_seq;

    call7_set(h.src, MR_CALLSIGN);
    call7_set(h.last_hop, MR_CALLSIGN);

    memcpy(h.final_dst, rx->last_hop, 7);

    char nh[7];
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    bool has = route_lookup_locked(h.final_dst, nh);
    xSemaphoreGive(g_mutex);

    if(has) memcpy(h.next_hop, nh, 7);
    else call7_set(h.next_hop, "*");

    h.payload_len=0;

    if(bucket_take(&b_ack)){
        c_tx_ack++;
        // jitter
        vTaskDelay(pdMS_TO_TICKS(esp_random()%150));
        txq_push_frame((uint8_t*)&h, sizeof(h));
    }else{
        c_defer_ack++;
    }
}

static void send_routeadv_one_queue(const route_t *r)
{
    uint8_t buf[sizeof(mr_hdr_v7_t) + ROUTEADV_PL_LEN]={0};
    mr_hdr_v7_t *h=(mr_hdr_v7_t*)buf;

    h->magic[0]='M'; h->magic[1]='R';
    h->version=MR_PROTO_VERSION;
    h->flags=MR_FLAG_ROUTEADV;
    h->ttl=ADV_TTL;
    h->msg_id=g_msg_id++;
    h->seq=g_my_seq;

    call7_set(h->src, MR_CALLSIGN);
    call7_set(h->last_hop, MR_CALLSIGN);

    call7_set(h->final_dst,"*");
    call7_set(h->next_hop,"*");

    h->payload_len=ROUTEADV_PL_LEN;

    uint8_t *p = buf + sizeof(mr_hdr_v7_t);
    memcpy(p, r->dst, 7); p += 7;
    memcpy(p, r->next, 7); p += 7;

    uint16_t etx = r->etx_x100;
    uint16_t seq = r->seq;

    *p++ = (uint8_t)(etx & 0xFF);
    *p++ = (uint8_t)(etx >> 8);
    *p++ = (uint8_t)(seq & 0xFF);
    *p++ = (uint8_t)(seq >> 8);

    if(bucket_take(&b_routeadv)){
        c_tx_routeadv++;
        vTaskDelay(pdMS_TO_TICKS(esp_random()%200));
        txq_push_frame(buf, sizeof(mr_hdr_v7_t)+ROUTEADV_PL_LEN);
    }else{
        c_defer_routeadv++;
    }
}

static void maybe_advertise_routes_queue(void)
{
    if(!g_routeadv_enable) return;

    route_t tmp[MAX_ROUTES];
    int n=0;

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used) continue;
        tmp[n++] = routes[i];
        if(n>=MAX_ROUTES) break;
    }
    xSemaphoreGive(g_mutex);

    for(int i=0;i<n;i++){
        for(int j=i+1;j<n;j++){
            if(tmp[j].etx_x100 < tmp[i].etx_x100){
                route_t t=tmp[i]; tmp[i]=tmp[j]; tmp[j]=t;
            }
        }
    }

    int top = (n < (int)g_routeadv_topn) ? n : (int)g_routeadv_topn;
    for(int i=0;i<top;i++){
        send_routeadv_one_queue(&tmp[i]);
    }
}

// DATA build + queue
static void send_data_to_queue(const char *dst_str, const char *txt, bool ackreq)
{
    uint8_t buf[sizeof(mr_hdr_v7_t) + MAX_PAYLOAD]={0};
    mr_hdr_v7_t *h=(mr_hdr_v7_t*)buf;

    h->magic[0]='M'; h->magic[1]='R';
    h->version=MR_PROTO_VERSION;
    h->flags=MR_FLAG_DATA | (ackreq?MR_FLAG_ACKREQ:0);
    h->ttl=DATA_TTL;
    h->msg_id=g_msg_id++;
    h->seq=g_my_seq++;

    call7_set(h->src, MR_CALLSIGN);
    call7_set(h->last_hop, MR_CALLSIGN);

    char dst7[7]; call7_set(dst7, dst_str);
    memcpy(h->final_dst, dst7, 7);

    char nh[7]; bool has=false;
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    has = route_lookup_locked(dst7, nh);
    bool bc = g_bc_fallback;
    bool crypto = g_crypto_enable;
    xSemaphoreGive(g_mutex);

    if(has) memcpy(h->next_hop, nh, 7);
    else call7_set(h->next_hop, bc ? "*" : "NONE");

    if(h->next_hop[0]=='N' && h->next_hop[1]=='O'){
        c_drop_data++;
        return;
    }

    uint8_t *pl = buf + sizeof(mr_hdr_v7_t);

    size_t n=strlen(txt);
    if(n>MAX_PLAINTEXT) n=MAX_PLAINTEXT;

    if(crypto){
        h->flags |= MR_FLAG_SEC;
        h->payload_len = (uint8_t)(n + SEC_TAG_LEN);

        uint8_t tag[SEC_TAG_LEN]={0};
        if(!sec_encrypt_payload(h, (const uint8_t*)txt, n, pl, tag)){
            c_drop_data++;
            return;
        }
        memcpy(pl + n, tag, SEC_TAG_LEN);
    }else{
        h->payload_len=(uint8_t)n;
        memcpy(pl, txt, n);
    }

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    if(!call7_is_wild(h->next_hop)){
        neighbor_tx_attempt_locked(h->next_hop);
        if(ackreq){
            pending_add_locked(h->msg_id, h->next_hop, buf, (uint16_t)(sizeof(mr_hdr_v7_t)+h->payload_len));
        }
    }
    xSemaphoreGive(g_mutex);

    if(bucket_take(&b_data)){
        c_tx_data++;
        txq_push_frame(buf, (uint16_t)(sizeof(mr_hdr_v7_t)+h->payload_len));
    }else{
        c_defer_data++;
    }
}


// ============================================================================
// FORWARD (DATA) – Forwarder entschlüsselt NICHT!
// ============================================================================
static void forward_data_queue(uint8_t *buf, size_t len)
{
    if(len < sizeof(mr_hdr_v7_t)) return;
    mr_hdr_v7_t *h=(mr_hdr_v7_t*)buf;

    if(h->ttl <= 1) return;
    h->ttl--;

    call7_set(h->last_hop, MR_CALLSIGN);

    char nh[7]; bool has=false;
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    has = route_lookup_locked(h->final_dst, nh);
    bool bc = g_bc_fallback;
    xSemaphoreGive(g_mutex);

    if(has) memcpy(h->next_hop, nh, 7);
    else call7_set(h->next_hop, bc ? "*" : "NONE");

    if(h->next_hop[0]=='N' && h->next_hop[1]=='O'){
        c_drop_data++;
        return;
    }

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    if(!call7_is_wild(h->next_hop)){
        neighbor_tx_attempt_locked(h->next_hop);
        if((h->flags & MR_FLAG_ACKREQ) != 0){
            pending_add_locked(h->msg_id, h->next_hop, buf, (uint16_t)len);
        }
    }
    xSemaphoreGive(g_mutex);

    if(bucket_take(&b_data)){
        c_tx_data++;
        txq_push_frame(buf, (uint16_t)len);
    }else{
        c_defer_data++;
    }
}


// ============================================================================
// RX handler (RadioLib)
// ============================================================================
static void handle_rx_frame(uint8_t *buf, size_t len, int rssi)
{
    if(len < sizeof(mr_hdr_v7_t)) return;

    mr_hdr_v7_t *h=(mr_hdr_v7_t*)buf;
    if(h->magic[0]!='M'||h->magic[1]!='R') return;
    if(h->version!=MR_PROTO_VERSION) return;

    // ACK: not filtered
    if(h->flags & MR_FLAG_ACK){
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        neighbor_update_rssi_locked(h->last_hop, rssi);
        neighbor_ack_ok_locked(h->src);
        pending_mark_acked_locked(h->msg_id, h->src);
        xSemaphoreGive(g_mutex);
        return;
    }

    // filter duplicates
    if(seen_before(h->src, h->msg_id)) return;
    remember_msg(h->src, h->msg_id);

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    neighbor_update_rssi_locked(h->last_hop, rssi);
    xSemaphoreGive(g_mutex);

    // BEACON -> learn
    if(h->flags & MR_FLAG_BEACON){
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        int ni = neighbor_ensure_locked(h->last_hop);
        uint16_t etx=100;
        if(ni>=0){
            neighbor_decay_locked(&neighbors[ni]);
            etx = etx_compute_x100(neighbors[ni].tx_attempts, neighbors[ni].ack_ok);
        }
        route_update_locked(h->src, h->last_hop, h->seq, etx, rssi);
        xSemaphoreGive(g_mutex);
        return;
    }

    // ROUTEADV -> integrate
    if(h->flags & MR_FLAG_ROUTEADV){
        if(h->payload_len != ROUTEADV_PL_LEN) return;
        uint8_t *p = buf + sizeof(mr_hdr_v7_t);

        char dst[7], next[7];
        memcpy(dst, p, 7); p+=7;
        memcpy(next, p, 7); p+=7;

        uint16_t etx = (uint16_t)p[0] | ((uint16_t)p[1]<<8); p+=2;
        uint16_t seq = (uint16_t)p[0] | ((uint16_t)p[1]<<8); p+=2;

        (void)next;

        xSemaphoreTake(g_mutex, portMAX_DELAY);

        int ni = neighbor_ensure_locked(h->src);
        uint16_t link_etx=100;
        if(ni>=0){
            neighbor_decay_locked(&neighbors[ni]);
            link_etx = etx_compute_x100(neighbors[ni].tx_attempts, neighbors[ni].ack_ok);
        }

        uint32_t path = (uint32_t)link_etx + (uint32_t)etx;
        if(path > ETX_MAX_X100) path = ETX_MAX_X100;

        route_update_locked(dst, h->src, seq, (uint16_t)path, rssi);

        xSemaphoreGive(g_mutex);
        return;
    }

    // DATA
    if(h->flags & MR_FLAG_DATA){
        char my7[7]; call7_set(my7, MR_CALLSIGN);

        bool iam_dst = call7_eq(h->final_dst, my7);
        bool iam_nexthop = call7_eq(h->next_hop, my7);
        bool nh_wild = call7_is_wild(h->next_hop);
        bool nh_none = (h->next_hop[0]=='N' && h->next_hop[1]=='O');

        if(iam_dst){
            if((h->flags & MR_FLAG_SEC) != 0){
                if(h->payload_len < SEC_TAG_LEN){
                    sec_decrypt_fail++;
                    return;
                }

                xSemaphoreTake(g_mutex, portMAX_DELAY);
                bool fresh = replay_check_locked(h->src, h->seq);
                xSemaphoreGive(g_mutex);

                if(!fresh){
                    sec_replay_drop++;
                    return;
                }

                size_t ciph_len = (size_t)h->payload_len - SEC_TAG_LEN;
                const uint8_t *pl = buf + sizeof(mr_hdr_v7_t);
                const uint8_t *tag = pl + ciph_len;

                uint8_t plain[MAX_PLAINTEXT+1];
                memset(plain,0,sizeof(plain));

                if(!sec_decrypt_payload(h, pl, ciph_len, tag, plain)){
                    sec_mac_fail++;
                    sec_decrypt_fail++;
                    return;
                }

                xSemaphoreTake(g_mutex, portMAX_DELAY);
                replay_update_locked(h->src, h->seq);
                xSemaphoreGive(g_mutex);

                sec_decrypt_ok++;

                plain[(ciph_len < MAX_PLAINTEXT)?ciph_len:MAX_PLAINTEXT] = 0;
                ESP_LOGI(TAG,"SEC DATA delivered ✅ \"%s\"", (char*)plain);
            }else{
                char txt[MAX_PAYLOAD+1]={0};
                if(h->payload_len>0 && h->payload_len<=MAX_PAYLOAD){
                    memcpy(txt, buf+sizeof(mr_hdr_v7_t), h->payload_len);
                    txt[h->payload_len]=0;
                }
                ESP_LOGI(TAG,"DATA delivered ✅ \"%s\"", txt);
            }

            if((h->flags & MR_FLAG_ACKREQ) != 0) send_ack_queue(h);
            return;
        }

        if(nh_none) return;

        if((iam_nexthop || nh_wild) && h->ttl > 1){
            forward_data_queue(buf, len);
            if((h->flags & MR_FLAG_ACKREQ) != 0) send_ack_queue(h);
        }
        return;
    }
}


// ============================================================================
// WiFi AP
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

    ESP_LOGI(TAG, "WiFi AP started: SSID='%s'", MR_WIFI_AP_SSID);
}


// ============================================================================
// HTTP helpers (Form)
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
// Kapitel 30_HT: Key parsing
// ============================================================================
static int hexval(char c)
{
    if(c>='0'&&c<='9') return c-'0';
    if(c>='a'&&c<='f') return 10+(c-'a');
    if(c>='A'&&c<='F') return 10+(c-'A');
    return -1;
}

static bool parse_key_hex16(const char *hex, uint8_t out[SEC_KEY_LEN])
{
    if(strlen(hex) < 32) return false;
    for(int i=0;i<SEC_KEY_LEN;i++){
        int hi=hexval(hex[2*i]);
        int lo=hexval(hex[2*i+1]);
        if(hi<0||lo<0) return false;
        out[i] = (uint8_t)((hi<<4)|lo);
    }
    return true;
}


// ============================================================================
// Web UI (unverändert übernommen – gekürzt? nein, komplett wie bei dir)
// ============================================================================
static const char *INDEX_HTML =
"<!doctype html><html><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>MeshRadio Kapitel 30_HT</title>"
"<style>"
"body{font-family:system-ui;margin:16px;max-width:1200px}"
".row{display:grid;grid-template-columns:1fr;gap:14px}"
"@media(min-width:980px){.row{grid-template-columns:1fr 1fr}}"
"button{padding:10px 14px;margin:6px 4px;font-size:16px}"
"input{padding:10px;margin:6px 0;font-size:16px;width:100%}"
"table{border-collapse:collapse;width:100%;font-size:13px}"
"th,td{border-bottom:1px solid #ddd;padding:6px 6px;text-align:left;white-space:nowrap}"
"th{position:sticky;top:0;background:#f6f6f6}"
"code{background:#eee;padding:2px 4px}"
".card{border:1px solid #ddd;border-radius:12px;padding:12px;box-shadow:0 1px 3px rgba(0,0,0,.06)}"
".muted{color:#666}"
".err{color:#b00020;font-weight:600}"
".ok{color:#0b7a26;font-weight:600}"
".grid3{display:grid;grid-template-columns:1fr;gap:12px}"
"@media(min-width:980px){.grid3{grid-template-columns:1fr 1fr 1fr}}"
"</style></head><body>"
"<h2>MeshRadio Kapitel 30_HT – AES-CCM + Replay-Schutz</h2>"
"<p class='muted'>AP: <code>" MR_WIFI_AP_SSID "</code> • URL: <code>http://192.168.4.1</code></p>"
"<div class='card'>"
"<div><b>Status:</b> <span id='st'>loading…</span></div>"
"<div id='err' class='err'></div>"
"<button onclick='refreshAll()'>Refresh</button>"
"</div>"
"<div class='grid3'>"
" <div class='card'>"
"  <h3>DATA senden</h3>"
"  <label>Ziel (dst)</label><input id='dst' value='DJ1ABC'>"
"  <label>Text</label><input id='msg' value='Hello 30_HT (secure)'>"
"  <label>ACK (0/1)</label><input id='ack' value='1'>"
"  <button onclick='send()'>SEND</button>"
"  <div id='m' class='muted'></div>"
"  <div class='muted'>Wenn Crypto ON ist, wird DATA automatisch verschlüsselt.</div>"
" </div>"
" <div class='card'>"
"  <h3>Security</h3>"
"  <div><b>Crypto:</b> <span id='cst' class='ok'>?</span></div>"
"  <button onclick='setCrypto(1)'>Crypto ON</button>"
"  <button onclick='setCrypto(0)'>Crypto OFF</button>"
"  <hr>"
"  <label>Net Key (32 hex chars)</label>"
"  <input id='keyhex' value='' placeholder='001122... (wird nicht angezeigt)'>"
"  <button onclick='setKey()'>Set Key</button>"
"  <div class='muted'>Key wird nur gesetzt, nicht ausgelesen.</div>"
" </div>"
" <div class='card'>"
"  <h3>Security Stats</h3>"
"  <pre id='secstats'>loading…</pre>"
"  <div class='muted'>MAC fail = Tag/AAD/Key falsch. Replay drop = seq zu alt.</div>"
" </div>"
"</div>"
"<div class='row'>"
" <div class='card'>"
"  <h3>Routing</h3>"
"  <div style='overflow:auto;max-height:380px'>"
"   <table id='rt'><thead><tr>"
"    <th>dst</th><th>next</th><th>seq</th><th>ETX</th><th>hold_ms</th><th>age_ms</th>"
"   </tr></thead><tbody></tbody></table>"
"  </div>"
" </div>"
" <div class='card'>"
"  <h3>Neighbors</h3>"
"  <div style='overflow:auto;max-height:380px'>"
"   <table id='nb'><thead><tr>"
"    <th>call</th><th>rssi</th><th>tx</th><th>ack</th><th>ETX</th><th>age_ms</th>"
"   </tr></thead><tbody></tbody></table>"
"  </div>"
" </div>"
"</div>"
"<div class='card'>"
"<h3>Control / Rate-Limits</h3>"
"<pre id='counters'>loading…</pre>"
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
"async function setCrypto(v){"
" let t=await post('/api/crypto','enable='+encodeURIComponent(v));"
" document.getElementById('m').textContent=t;"
" refreshAll();"
"}"
"async function setKey(){"
" let k=document.getElementById('keyhex').value.trim();"
" if(!k){ document.getElementById('m').textContent='ERR key empty'; return; }"
" let t=await post('/api/key','keyhex='+encodeURIComponent(k));"
" document.getElementById('m').textContent=t;"
" document.getElementById('keyhex').value='';"
" refreshAll();"
"}"
"async function refreshAll(){"
" document.getElementById('err').textContent='';"
" try{"
"  let r=await fetch('/api/status');"
"  let txt=await r.text();"
"  let j=JSON.parse(txt);"
"  document.getElementById('st').textContent="
"    'call='+j.call+' • crypto='+j.crypto_enable+' • routes='+j.routes.length+' • neighbors='+j.neighbors.length;"
"  document.getElementById('cst').textContent = (j.crypto_enable==1)?'ON':'OFF';"
"  document.getElementById('cst').className = (j.crypto_enable==1)?'ok':'err';"
"  document.getElementById('secstats').textContent=j.secstats;"
"  document.getElementById('counters').textContent=j.counters;"
"  let rb=document.querySelector('#rt tbody'); rb.innerHTML='';"
"  for(let x of j.routes){"
"    let tr=document.createElement('tr');"
"    tr.innerHTML='<td>'+esc(x.dst)+'</td><td>'+esc(x.next)+'</td><td>'+x.seq+'</td><td>'+etxFmt(x.etx_x100)+'</td><td>'+x.hold_ms+'</td><td>'+x.age_ms+'</td>';"
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
// ============================================================================
static esp_err_t api_status_get(httpd_req_t *req)
{
    static char out[12288];
    size_t off=0;

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    uint32_t t = now_ms();

    char counters[1024];
    snprintf(counters, sizeof(counters),
        "TX: beacon=%" PRIu32 " adv=%" PRIu32 " ack=%" PRIu32 " data=%" PRIu32 "\n"
        "DEFER: beacon=%" PRIu32 " adv=%" PRIu32 " ack=%" PRIu32 " data=%" PRIu32 "\n"
        "DROP: adv=%" PRIu32 " data=%" PRIu32 "\n"
        "TOKENS: beacon=%.2f adv=%.2f ack=%.2f data=%.2f\n"
        "CAD: %s\n",
        c_tx_beacon, c_tx_routeadv, c_tx_ack, c_tx_data,
        c_defer_beacon, c_defer_routeadv, c_defer_ack, c_defer_data,
        c_drop_routeadv, c_drop_data,
        b_beacon.tokens, b_routeadv.tokens, b_ack.tokens, b_data.tokens,
        g_cad_enable ? "ON" : "OFF"
    );

    char secstats[512];
    snprintf(secstats, sizeof(secstats),
        "crypto_enable=%u\n"
        "decrypt_ok=%" PRIu32 "\n"
        "decrypt_fail=%" PRIu32 "\n"
        "mac_fail=%" PRIu32 "\n"
        "replay_drop=%" PRIu32 "\n",
        g_crypto_enable?1:0,
        sec_decrypt_ok,
        sec_decrypt_fail,
        sec_mac_fail,
        sec_replay_drop
    );

    off += snprintf(out+off, sizeof(out)-off,
        "{"
        "\"call\":\"%s\","
        "\"crypto_enable\":%u,"
        "\"bc_fallback\":%u,"
        "\"counters\":\"",
        MR_CALLSIGN,
        g_crypto_enable?1:0,
        g_bc_fallback?1:0
    );

    for(const char *p=counters; *p && off < sizeof(out)-32; p++){
        if(*p=='\n'){ out[off++]='\\'; out[off++]='n'; }
        else if(*p=='"'){ out[off++]='\\'; out[off++]='"'; }
        else out[off++]=*p;
    }
    out[off++]='\"';
    out[off++]=',';

    off += snprintf(out+off, sizeof(out)-off, "\"secstats\":\"");
    for(const char *p=secstats; *p && off < sizeof(out)-32; p++){
        if(*p=='\n'){ out[off++]='\\'; out[off++]='n'; }
        else if(*p=='"'){ out[off++]='\\'; out[off++]='"'; }
        else out[off++]=*p;
    }
    out[off++]='\"';
    out[off++]=',';

    off += snprintf(out+off, sizeof(out)-off, "\"neighbors\":[");
    bool first=true;
    for(int i=0;i<MAX_NEIGHBORS;i++){
        if(!neighbors[i].used) continue;

        neighbor_decay_locked(&neighbors[i]);
        uint16_t etx = etx_compute_x100(neighbors[i].tx_attempts, neighbors[i].ack_ok);

        char c[8]; call7_to_str(c, neighbors[i].call);
        uint32_t age = t - neighbors[i].t_ms;

        if(off > sizeof(out)-512) break;

        off += snprintf(out+off, sizeof(out)-off,
            "%s{\"call\":\"%s\",\"rssi\":%d,\"tx\":%" PRIu32 ",\"ack\":%" PRIu32 ",\"etx_x100\":%u,\"age_ms\":%" PRIu32 "}",
            first?"":",", c, neighbors[i].rssi,
            neighbors[i].tx_attempts, neighbors[i].ack_ok, etx, age
        );
        first=false;
    }

    off += snprintf(out+off, sizeof(out)-off, "],\"routes\":[");
    first=true;
    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used) continue;

        char d[8], n[8];
        call7_to_str(d, routes[i].dst);
        call7_to_str(n, routes[i].next);

        uint32_t age = t - routes[i].t_ms;
        uint32_t hold = (t < routes[i].hold_until_ms) ? (routes[i].hold_until_ms - t) : 0;

        if(off > sizeof(out)-512) break;

        off += snprintf(out+off, sizeof(out)-off,
            "%s{\"dst\":\"%s\",\"next\":\"%s\",\"seq\":%u,\"etx_x100\":%u,\"hold_ms\":%" PRIu32 ",\"age_ms\":%" PRIu32 "}",
            first?"":",", d, n, (unsigned)routes[i].seq, (unsigned)routes[i].etx_x100, hold, age
        );
        first=false;
    }

    off += snprintf(out+off, sizeof(out)-off, "]}");
    xSemaphoreGive(g_mutex);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, out, HTTPD_RESP_USE_STRLEN);
}


// ============================================================================
// API: /api/send  -> queued
// ============================================================================
static esp_err_t api_send_post(httpd_req_t *req)
{
    char body[256];
    if(!http_read_body(req, body, sizeof(body))) return http_send_text(req, "ERR body");

    char dst[16]={0}, msg[160]={0}, ackv[8]={0};
    if(!form_get(body,"dst",dst,sizeof(dst))) return http_send_text(req,"ERR missing dst");
    if(!form_get(body,"msg",msg,sizeof(msg))) return http_send_text(req,"ERR missing msg");
    form_get(body,"ack",ackv,sizeof(ackv));

    bool ack = (ackv[0] != '0');

    // NICHT senden -> nur queue
    send_data_to_queue(dst, msg, ack);
    return http_send_text(req, "OK queued");
}


// ============================================================================
// API: /api/crypto
// ============================================================================
static esp_err_t api_crypto_post(httpd_req_t *req)
{
    char body[64];
    if(!http_read_body(req, body, sizeof(body))) return http_send_text(req, "ERR body");

    char v[8]={0};
    if(!form_get(body,"enable",v,sizeof(v))) return http_send_text(req,"ERR missing enable");

    int en = (v[0]=='1');

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_crypto_enable = en;
    xSemaphoreGive(g_mutex);

    return http_send_text(req, en ? "OK crypto=1" : "OK crypto=0");
}


// ============================================================================
// API: /api/key
// ============================================================================
static esp_err_t api_key_post(httpd_req_t *req)
{
    char body[128];
    if(!http_read_body(req, body, sizeof(body))) return http_send_text(req, "ERR body");

    char k[80]={0};
    if(!form_get(body,"keyhex",k,sizeof(k))) return http_send_text(req,"ERR missing keyhex");

    uint8_t tmp[SEC_KEY_LEN];
    if(!parse_key_hex16(k, tmp)) return http_send_text(req, "ERR keyhex format (need 32 hex)");

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    memcpy(g_net_key, tmp, SEC_KEY_LEN);
    xSemaphoreGive(g_mutex);

    return http_send_text(req, "OK key set");
}


// ============================================================================
// HTTP start
// ============================================================================
static void http_start(void)
{
    httpd_config_t cfg=HTTPD_DEFAULT_CONFIG();
    cfg.stack_size=20000;
    cfg.max_uri_handlers=24;

    ESP_ERROR_CHECK(httpd_start(&g_http,&cfg));

    httpd_uri_t u0={.uri="/",.method=HTTP_GET,.handler=index_get};
    httpd_register_uri_handler(g_http,&u0);

    httpd_uri_t st0={.uri="/api/status",.method=HTTP_GET,.handler=api_status_get};
    httpd_register_uri_handler(g_http,&st0);

    httpd_uri_t s0={.uri="/api/send",.method=HTTP_POST,.handler=api_send_post};
    httpd_register_uri_handler(g_http,&s0);

    httpd_uri_t cr0={.uri="/api/crypto",.method=HTTP_POST,.handler=api_crypto_post};
    httpd_register_uri_handler(g_http,&cr0);

    httpd_uri_t k0={.uri="/api/key",.method=HTTP_POST,.handler=api_key_post};
    httpd_register_uri_handler(g_http,&k0);

    ESP_LOGI(TAG,"HTTP server started");
}


// ============================================================================
// Radio init (Kap30 Heltec)
// ============================================================================
static void radio_init_heltec(void)
{
    ESP_LOGI(TAG, "Init SX1262 (Heltec V3.x) ...");

    // SPI
    hal.setSpiPins(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
    hal.setSpiHost(SPI2_HOST);
    hal.setSpiHz(1 * 1000 * 1000);   // konservativ; später 4/8 MHz testen
    hal.spiBegin();

    int state = radio.begin(
        433.775,  // MHz (433 board)
        125.0,    // BW
        7,        // SF
        5,        // CR
        0x12,     // sync word
        14,       // power dBm
        8,        // preamble
        1.6       // tcxo
    );

    if(state != RADIOLIB_ERR_NONE){
        ESP_LOGE(TAG, "Radio init failed %d", state);
        abort();
    }

    // TX done IRQ
    radio.setPacketSentAction(onPacketSent);

    // Start RX
    radio.startReceive();

    ESP_LOGI(TAG, "SX1262 READY");
}


// ============================================================================
// MAIN
// ============================================================================
extern "C" void app_main(void)
{
    ESP_LOGI(TAG,"MeshRadio Kapitel 30_HT (PORT Heltec SX1262) start");

    g_mutex=xSemaphoreCreateMutex();
    if(!g_mutex){ ESP_LOGE(TAG,"mutex failed"); abort(); }

    tx_q = xQueueCreate(16, sizeof(tx_item_t));
    if(!tx_q){ ESP_LOGE(TAG,"tx_q failed"); abort(); }

    ESP_ERROR_CHECK(nvs_flash_init());

    // init buckets
    bucket_init(&b_beacon,   RL_BEACON_TPS,   RL_BEACON_BURST);
    bucket_init(&b_routeadv, RL_ROUTEADV_TPS, RL_ROUTEADV_BURST);
    bucket_init(&b_ack,      RL_ACK_TPS,      RL_ACK_BURST);
    bucket_init(&b_data,     RL_DATA_TPS,     RL_DATA_BURST);

    // load default key
    if(!parse_key_hex16(MR_NET_KEY_HEX, g_net_key)){
        ESP_LOGE(TAG,"Invalid MR_NET_KEY_HEX – need 32 hex chars");
        abort();
    }

    // Radio init (Heltec)
    radio_init_heltec();

    // Retry task (queued resend)
    xTaskCreate(retry_task, "retry", 4096, NULL, 6, NULL);

    // WiFi + HTTP
    wifi_ap();
    http_start();

    beacon_schedule_next();
    routeadv_schedule_next();

    ESP_LOGI(TAG,"CALL=%s  crypto=%u  net_id=0x%02X", MR_CALLSIGN, g_crypto_enable?1:0, (unsigned)MR_NET_ID);
    ESP_LOGI(TAG,"Open http://192.168.4.1");

    // RX buffer
    uint8_t rxbuf[256];

    while(1){
        // 1) RX poll (kurz)
        int st = radio.receive(rxbuf, sizeof(rxbuf));
        if(st == RADIOLIB_ERR_NONE) {
            int rssi = radio.getRSSI();
            // Paketlänge ist in rxbuf enthalten; receive() füllt "bis max", aber wir
            // kennen die echte Länge nicht perfekt – in der Praxis reicht:
            // -> wir nutzen payload_len im Header und rechnen daraus len.
            // Minimal: len = sizeof(hdr)+payload_len (mit Plausibilitätscheck).
            if(sizeof(mr_hdr_v7_t) <= sizeof(rxbuf)) {
                mr_hdr_v7_t* h = (mr_hdr_v7_t*)rxbuf;
                size_t expect = sizeof(mr_hdr_v7_t) + (size_t)h->payload_len;
                if(expect <= sizeof(rxbuf)) {
                    handle_rx_frame(rxbuf, expect, rssi);
                }
            }
            radio.startReceive();
        }

        // 2) Scheduler: BEACON / ROUTEADV / Cleanup
        if(g_beacon_enabled && now_ms() > g_next_beacon_ms)
            send_beacon_queue();

        if(now_ms() > g_next_routeadv_ms){
            maybe_advertise_routes_queue();
            routeadv_schedule_next();
        }

        xSemaphoreTake(g_mutex, portMAX_DELAY);
        neighbor_cleanup_locked();
        route_cleanup_locked();
        xSemaphoreGive(g_mutex);

        // 3) TX queue abarbeiten (seriell!)
        tx_item_t it;
        while(xQueueReceive(tx_q, &it, 0) == pdTRUE) {
            cad_backoff_if_enabled();
            int txst = radio_tx_blocking(it.frame, it.len, 2000);
            if(txst != RADIOLIB_ERR_NONE) {
                ESP_LOGW(TAG, "TX fail %d len=%u", txst, (unsigned)it.len);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}