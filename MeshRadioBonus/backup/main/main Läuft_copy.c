// ============================================================================
// MeshRadio – Kapitel 34 (Roles) + Kapitel 29 (AES-CCM + Replay) + Station Addons
// ----------------------------------------------------------------------------
// FILE: main.c  (Single-file build, ESP-IDF v5.5.2)
//
// ZIEL:
//   - "profi-level stabil"
//   - gleicher Mesh/Proto Code, aber LoRa-Layer umschaltbar:
//       * SX1276/SX1278 (Register-Driver)  -> LILYGO/TTGO etc.
//       * SX1262         (Command-Driver)  -> Heltec LoRa 32 V3.x / V3.2
//
// WICHTIG (Heltec V3.x FIXES – die hier drin sind):
//   1) VEXT muss an (GPIO36 LOW), sonst ist SX1262 / Peripherie oft nicht stabil.
//   2) SX1262 WriteBuffer (0x0E) MUSS in EINER SPI-Transaktion passieren
//      (CS darf nicht zwischen Header und Payload toggeln).
//   3) SX1262 GetRxBufferStatus (0x13) korrekt takten (Opcode+Dummy+2 Bytes).
//   4) Heltec-typisches Radio-Init:
//        - Standby XOSC
//        - DIO3 TCXO Control (0x97)
//        - Calibrate (0x89)
//        - DIO2 RF-Switch Auto (0x9D)
//        - PA Config (0x95) und TxParams (0x8E) sinnvoll setzen
//
// FUNKPARAMETER (für beide Boards gleich gesetzt):
//   - Band: 863 MHz (EU 863–870)  -> DEFAULT_RF_FREQ_HZ = 863000000
//   - SF7 / BW125k / CR4/5 / CRC ON (wie zuvor, nur Frequenz angepasst)
//
// HINWEIS:
//   - LILYGO-Teil bleibt funktional unverändert (nur Frequenz auf 863 MHz angepasst).
//   - Mesh/Proto/HTTP/CLI bleibt 1:1 wie in deinem funktionierenden Stand –
//     wir ändern NUR Heltec-relevantes und den Frequenzwert.
// ============================================================================


// ============================================================================
// ============================ USER DEFINES (TOP) ============================
// ============================================================================

// ---- Board Presets (einfach hier umschalten) ----
#define MR_BOARD_LILYGO_SX1276   1
#define MR_BOARD_HELTEC_V3       2

#ifndef MR_BOARD_PRESET
#define MR_BOARD_PRESET MR_BOARD_HELTEC_V3 // MR_BOARD_HELTEC_V3  // <-- HIER UMSCHALTEN
#endif

// ---- Callsign / WiFi ----
#define MR_CALLSIGN        "DL1ABCF"
#define MR_WIFI_AP_SSID    "MeshRadio-Setup2"
#define MR_WIFI_AP_PASS    ""          // "" => OPEN; >=8 Zeichen => WPA2-PSK

// ---- RF Frequency (EU 863–870) ----
#define DEFAULT_RF_FREQ_HZ 863000000UL

// ---- Control Plane ----
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

// ---- Token bucket rate limits ----
#define RL_BEACON_TPS      0.05f
#define RL_BEACON_BURST    1.0f
#define RL_ROUTEADV_TPS    0.20f
#define RL_ROUTEADV_BURST  2.0f
#define RL_ACK_TPS         1.50f
#define RL_ACK_BURST       5.0f
#define RL_DATA_TPS        0.50f
#define RL_DATA_BURST      3.0f

// ---- CAD optional ----
#define DEFAULT_CAD_ENABLE 0
#define CAD_WAIT_MS        80
#define CAD_JITTER_MS      120

// ---- Kapitel 29 Security Defaults ----
#define DEFAULT_CRYPTO_ENABLE  1
#define MR_NET_KEY_HEX "00112233445566778899AABBCCDDEEFF"
#define MR_NET_ID 0x42

#define SEC_KEY_LEN        16
#define SEC_NONCE_LEN      12
#define SEC_TAG_LEN        8

// ---- Kapitel 34 Node Roles ----
#define DEFAULT_NODE_MODE  0   // 0=RELAY, 1=EDGE, 2=SENSOR

// ---- Power save (optional; hier erstmal aus) ----
#define MR_POWERSAVE_ENABLE       0
#define SENSOR_WAKE_PERIOD_MS     60000
#define SENSOR_RX_WINDOW_MS       800
#define EDGE_DUTY_RX_ENABLE       0
#define EDGE_RX_ON_MS             600
#define EDGE_RX_OFF_MS            1400

// ---- WiFi runtime default ----
#define MR_WIFI_RUNTIME_DEFAULT   0  // 1=WiFi+HTTP direkt beim Boot, 0=aus

// ---- Station: Relay ----
#define MR_RELAY_ENABLE           1
#define RELAY_ACTIVE_LEVEL        1   // 1: HIGH=ON, 0: LOW=ON

// ---- Station: Battery ADC ----
#define MR_BATT_ENABLE            1
#define BATT_MEASURE_INTERVAL_MS  5000
#define BATT_EMPTY_MV             3300
#define BATT_FULL_MV              4200
#define MR_ADC_CALI_MODE          0

// ---- Serial CLI ----
#define MR_CLI_ENABLE             1
#define MR_CLI_LINE_MAX           200


// ============================================================================
// ============================ BOARD PIN PRESETS =============================
// ============================================================================
//
// Einheitliche LoRa-Signale:
//   - SPI: SCK/MOSI/MISO + NSS (CS)
//   - RESET
//   - IRQ:
//        SX1276: DIO0  (RxDone/TxDone)
//        SX1262: DIO1  (IRQ line)
//   - SX1262 zusätzlich: BUSY
//
// Heltec V3.x Besonderheit:
//   - VEXT_CTRL (GPIO36) muss LOW (ON) gesetzt werden.
// ============================================================================
#if (MR_BOARD_PRESET == MR_BOARD_HELTEC_V3)
// Heltec LoRa 32 V3.x (SX1262) – typische Pins
#define MR_LORA_CHIP_SX1262 1

#define PIN_LORA_NSS     8
#define PIN_LORA_SCK     9
#define PIN_LORA_MOSI   10
#define PIN_LORA_MISO   11
#define PIN_LORA_RST    12
#define PIN_LORA_BUSY   13
#define PIN_LORA_DIO1   14     // IRQ line
#define BATT_EN_GPIO    37
#define BATT_EN_ACTIVE_LOW 1
#define VEXT_CTRL_PIN   36 
#define PIN_VEXT_CTRL   36     // Heltec V3.x: VEXT ON = LOW

#define BATT_ADC_GPIO    1      // VBAT Sense Pin am Heltec V3
// Heltec nutzt meist 100k/390k (Faktor 4.9) oder 100k/100k (Faktor 2.0)
// Für V3.2 ist 100k (Top) / 390k (Bot) üblich:
#define BATT_DIV_RTOP_OHMS  390000.0f
#define BATT_DIV_RBOT_OHMS  100000.0f

// RELAY: WICHTIG: darf NICHT 14 sein (IRQ)!
#ifndef RELAY_GPIO
#define RELAY_GPIO       4     // default: bitte prüfen, ob bei dir frei/verdrahtet
#endif

#elif (MR_BOARD_PRESET == MR_BOARD_LILYGO_SX1276)
// LILYGO / TTGO SX1276/78 – typische T-Beam V1.1 Pins
#define MR_LORA_CHIP_SX1276 1

#define PIN_LORA_NSS     18
#define PIN_LORA_SCK      5
#define PIN_LORA_MOSI    27
#define PIN_LORA_MISO    19
#define PIN_LORA_RST     23
#define PIN_LORA_DIO0    26

// ADC default
#define BATT_ADC_GPIO    34
#define BATT_DIV_RTOP_OHMS 220000.0f
#define BATT_DIV_RBOT_OHMS 100000.0f

#ifndef RELAY_GPIO
#define RELAY_GPIO       14
#endif

#else
#error "Unknown MR_BOARD_PRESET"
#endif

// GPIO conflict guard (Heltec V3: DIO1=14!)
#if defined(PIN_LORA_DIO1)
#if (MR_RELAY_ENABLE) && (RELAY_GPIO == PIN_LORA_DIO1)
#error "RELAY_GPIO conflicts with LoRa DIO1 (IRQ). Choose another pin!"
#endif
#endif


// ============================================================================
// ================================ INCLUDES ==================================
// ============================================================================
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <inttypes.h>
#include <stdlib.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "esp_log.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_err.h"

#include "nvs_flash.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"

#include "esp_sleep.h"
#include "mbedtls/ccm.h"


// ============================================================================
// ================================ PROTOKOLL =================================
// ============================================================================
#define MR_PROTO_VERSION 7

#define MR_FLAG_DATA      0x10
#define MR_FLAG_BEACON    0x20
#define MR_FLAG_ACK       0x40
#define MR_FLAG_ROUTEADV  0x80

#define MR_FLAG_ACKREQ    0x01
#define MR_FLAG_SEC       0x08

#define DATA_TTL   4
#define BEACON_TTL 2
#define ACK_TTL    4
#define ADV_TTL    3

#define MAX_PAYLOAD      120
#define MAX_PLAINTEXT    (MAX_PAYLOAD - SEC_TAG_LEN)

#define SEEN_CACHE_SIZE 48
#define MAX_NEIGHBORS 24
#define NEIGHBOR_TIMEOUT_MS 60000
#define MAX_ROUTES 32
#define ROUTE_TIMEOUT_MS 180000
#define MAX_PENDING_ACK  10
#define ROUTEADV_PL_LEN   (7+7+2+2)
#define MAX_REPLAY 24

// LoRa TX timeout (für beide Chips)
#define LORA_TX_TIMEOUT_MS 2000


// ============================================================================
// ============================== NODE ROLES ==================================
// ============================================================================
typedef enum {
    NODE_RELAY  = 0,
    NODE_EDGE   = 1,
    NODE_SENSOR = 2
} node_mode_t;

static const char* node_mode_str(node_mode_t m)
{
    switch(m){
        case NODE_RELAY:  return "RELAY";
        case NODE_EDGE:   return "EDGE";
        case NODE_SENSOR: return "SENSOR";
        default:          return "UNKNOWN";
    }
}


// ============================================================================
// ================================ HEADER ====================================
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

    uint8_t payload_len;
} mr_hdr_v7_t;
#pragma pack(pop)

// FIXED AAD für CCM (stabil, unabhängig von forwarding Feldern)
#pragma pack(push,1)
typedef struct {
    uint8_t magic[2];
    uint8_t version;
    uint8_t flags;
    uint16_t msg_id;
    uint16_t seq;
    char src[7];
    char final_dst[7];
    uint8_t payload_len;
} mr_aad_v7_t;
#pragma pack(pop)

static void sec_make_aad(mr_aad_v7_t *a, const mr_hdr_v7_t *h)
{
    memcpy(a->magic, h->magic, 2);
    a->version = h->version;
    a->flags   = h->flags;
    a->msg_id  = h->msg_id;
    a->seq     = h->seq;
    memcpy(a->src, h->src, 7);
    memcpy(a->final_dst, h->final_dst, 7);
    a->payload_len = h->payload_len;
}


// ============================================================================
// ================================ STRUCTS ===================================
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
// ================================ GLOBALS ===================================
// ============================================================================
static const char *TAG="MR34";

static spi_device_handle_t lora_spi;
static QueueHandle_t dio_q;

static SemaphoreHandle_t g_mutex;          // schützt RAM-Tabellen/Globals
static SemaphoreHandle_t g_lora_spi_mutex; // schützt LoRa SPI-Zugriffe (beide Chips)

static httpd_handle_t g_http=NULL;
static bool g_http_running=false;

static uint16_t g_msg_id=1;
static uint16_t g_my_seq=1;

static seen_t seen_cache[SEEN_CACHE_SIZE];
static neighbor_t neighbors[MAX_NEIGHBORS];
static route_t routes[MAX_ROUTES];
static pending_ack_t pend[MAX_PENDING_ACK];
static replay_t replay_tab[MAX_REPLAY];

static bool g_bc_fallback=true;
static bool g_beacon_enabled=true;
static bool g_routeadv_enable=DEFAULT_ROUTEADV_ENABLE;
static bool g_cad_enable=DEFAULT_CAD_ENABLE;

static node_mode_t g_node_mode = (node_mode_t)DEFAULT_NODE_MODE;

static bool g_crypto_enable=DEFAULT_CRYPTO_ENABLE;
static uint8_t g_net_key[SEC_KEY_LEN];

static uint32_t sec_decrypt_ok=0;
static uint32_t sec_decrypt_fail=0;
static uint32_t sec_mac_fail=0;
static uint32_t sec_replay_drop=0;

static uint32_t g_beacon_interval_ms=DEFAULT_BEACON_INTERVAL_MS;
static uint8_t  g_routeadv_topn=DEFAULT_ROUTEADV_TOPN;
static uint16_t g_routeadv_delta_etx=DEFAULT_ROUTEADV_DELTA_ETX;
static uint32_t g_holddown_ms=DEFAULT_HOLDDOWN_MS;

static uint32_t g_next_beacon_ms=0;
static uint32_t g_next_routeadv_ms=0;

static uint32_t c_tx_beacon=0, c_tx_routeadv=0, c_tx_ack=0, c_tx_data=0;
static uint32_t c_defer_beacon=0, c_defer_routeadv=0, c_defer_ack=0, c_defer_data=0;
static uint32_t c_drop_routeadv=0, c_drop_data=0;

static bucket_t b_beacon, b_routeadv, b_ack, b_data;

static bool g_wifi_enabled = (MR_WIFI_RUNTIME_DEFAULT ? true : false);
static bool g_wifi_inited  = false;

static char g_last_rx_from[8]   = "";
static char g_last_rx_text[180] = "";
static uint32_t g_last_rx_ms    = 0;

#if MR_RELAY_ENABLE
static bool g_relay_on=false;
#endif

// ADC
#if MR_BATT_ENABLE
static adc_oneshot_unit_handle_t g_adc_unit=NULL;
static adc_cali_handle_t g_adc_cali=NULL;
static bool g_adc_cali_ok=false;
static uint32_t g_batt_mv=0;
static uint32_t g_batt_pct=0;
static uint32_t g_next_batt_ms=0;
#endif

// ============================================================================
// ================================ HELPERS ===================================
// ============================================================================
static uint32_t now_ms(void){ return xTaskGetTickCount()*portTICK_PERIOD_MS; }

static inline void lora_spi_lock(void){ xSemaphoreTake(g_lora_spi_mutex, portMAX_DELAY); }
static inline void lora_spi_unlock(void){ xSemaphoreGive(g_lora_spi_mutex); }

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

static void json_escape_into(char *dst, size_t dst_sz, const char *src)
{
    size_t o=0;
    for(size_t i=0; src && src[i] && o+2 < dst_sz; i++){
        unsigned char c=(unsigned char)src[i];
        if(c=='\\' || c=='"'){ dst[o++]='\\'; dst[o++]=(char)c; }
        else if(c=='\n'){ dst[o++]='\\'; dst[o++]='n'; }
        else if(c=='\r'){ dst[o++]='\\'; dst[o++]='r'; }
        else if(c=='\t'){ dst[o++]='\\'; dst[o++]='t'; }
        else if(c>=32 && c<=126){ dst[o++]=(char)c; }
        else { /* drop */ }
        if(o >= dst_sz-1) break;
    }
    dst[o]=0;
}

#if (MR_BOARD_PRESET == MR_BOARD_HELTEC_V3)
static void heltec_vext_on(void)
{
    gpio_config_t io = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << PIN_VEXT_CTRL),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    // ✅ Heltec: LOW = VEXT an
    gpio_set_level(PIN_VEXT_CTRL, 0);

    // ✅ Stabilitäts-Delay: Peripherie + Pullups + RF-Switch hochfahren lassen
    vTaskDelay(pdMS_TO_TICKS(100));
}

static void board_power_boot_init(void)
{
#if (MR_BOARD_PRESET == MR_BOARD_HELTEC_V3)
    // VEXT ON (LOW)
    gpio_set_direction((gpio_num_t)VEXT_CTRL_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)VEXT_CTRL_PIN, 0);

    // Battery divider enable pin init (default OFF!)
    gpio_set_direction((gpio_num_t)BATT_EN_GPIO, GPIO_MODE_OUTPUT);
#if BATT_EN_ACTIVE_LOW
    gpio_set_level((gpio_num_t)BATT_EN_GPIO, 1); // OFF
#else
    gpio_set_level((gpio_num_t)BATT_EN_GPIO, 0); // OFF
#endif
    vTaskDelay(pdMS_TO_TICKS(50));
#endif
}

static inline void batt_path_enable(bool en)
{
#if (MR_BOARD_PRESET == MR_BOARD_HELTEC_V3)
#if BATT_EN_ACTIVE_LOW
    gpio_set_level((gpio_num_t)BATT_EN_GPIO, en ? 0 : 1);
#else
    gpio_set_level((gpio_num_t)BATT_EN_GPIO, en ? 1 : 0);
#endif
#else
    (void)en;
#endif
}

#endif


// ============================================================================
// ============================== TOKEN BUCKET ================================
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
    if(b->tokens >= 1.0f){ b->tokens -= 1.0f; return true; }
    return false;
}


// ============================================================================
// ================================ SEEN ======================================
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
// =============================== NEIGHBORS ==================================
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
// ================================= ROUTES ==================================
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
// ============================ REPLAY + SEQ SAFE =============================
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
static bool seq_is_newer_u16(uint16_t seq, uint16_t last_seq)
{
    return ((int16_t)(seq - last_seq) > 0);
}
static bool replay_check_locked(const char src[7], uint16_t seq)
{
    replay_t *r = replay_get_locked(src);
    r->t_ms = now_ms();
    return seq_is_newer_u16(seq, r->last_seq);
}
static void replay_update_locked(const char src[7], uint16_t seq)
{
    replay_t *r = replay_get_locked(src);
    r->last_seq = seq;
    r->t_ms = now_ms();
}


// ============================================================================
// ============================== CCM (AES-CCM) ===============================
// ============================================================================
static void sec_make_nonce(uint8_t nonce[SEC_NONCE_LEN], const mr_hdr_v7_t *h)
{
    memcpy(nonce, h->src, 7);
    nonce[7]  = (uint8_t)(h->seq & 0xFF);
    nonce[8]  = (uint8_t)(h->seq >> 8);
    nonce[9]  = (uint8_t)(h->msg_id & 0xFF);
    nonce[10] = (uint8_t)(h->msg_id >> 8);
    nonce[11] = (uint8_t)MR_NET_ID;
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

    mr_aad_v7_t aad;
    sec_make_aad(&aad, h);

    int rc = mbedtls_ccm_encrypt_and_tag(
        &ctx, plain_len,
        nonce, SEC_NONCE_LEN,
        (const uint8_t*)&aad, sizeof(aad),
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

    mr_aad_v7_t aad;
    sec_make_aad(&aad, h);

    int rc = mbedtls_ccm_auth_decrypt(
        &ctx, cipher_len,
        nonce, SEC_NONCE_LEN,
        (const uint8_t*)&aad, sizeof(aad),
        cipher, out_plain,
        tag, SEC_TAG_LEN
    );

    mbedtls_ccm_free(&ctx);
    return (rc == 0);
}


// ============================================================================
// ============================ KEY PARSE (HEX16) =============================
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
// ================================ SPI INIT ==================================
// ============================================================================
#if defined(VSPI_HOST)
  #define LORA_SPI_HOST VSPI_HOST
#elif defined(SPI3_HOST)
  #define LORA_SPI_HOST SPI3_HOST
#else
  #define LORA_SPI_HOST SPI2_HOST
#endif

static void init_spi(void)
{
    spi_bus_config_t bus={
        .miso_io_num=PIN_LORA_MISO,
        .mosi_io_num=PIN_LORA_MOSI,
        .sclk_io_num=PIN_LORA_SCK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LORA_SPI_HOST, &bus, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev={
        .clock_speed_hz=2000000, // 2MHz robust
        .mode=0,
        .spics_io_num=PIN_LORA_NSS, // Hardware CS (ESP-IDF toggles per transaction)
        .queue_size=1
    };
    ESP_ERROR_CHECK(spi_bus_add_device(LORA_SPI_HOST, &dev, &lora_spi));
}


// ============================================================================
// ============================= LoRa Abstraction =============================
// ============================================================================
static void lora_hw_reset(void)
{
    gpio_set_direction(PIN_LORA_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_LORA_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_LORA_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}


// ----------------------------- SX1276 DRIVER ------------------------------
#if defined(MR_LORA_CHIP_SX1276)

#define SX1276_REG_FIFO                 0x00
#define SX1276_REG_OP_MODE              0x01
#define SX1276_REG_FRF_MSB              0x06
#define SX1276_REG_FRF_MID              0x07
#define SX1276_REG_FRF_LSB              0x08
#define SX1276_REG_PA_CONFIG            0x09
#define SX1276_REG_FIFO_ADDR_PTR        0x0D
#define SX1276_REG_FIFO_TX_BASE_ADDR    0x0E
#define SX1276_REG_FIFO_RX_BASE_ADDR    0x0F
#define SX1276_REG_FIFO_RX_CURRENT_ADDR 0x10
#define SX1276_REG_IRQ_FLAGS            0x12
#define SX1276_REG_RX_NB_BYTES          0x13
#define SX1276_REG_PKT_RSSI_VALUE       0x1A
#define SX1276_REG_PAYLOAD_LENGTH       0x22
#define SX1276_REG_MODEM_CONFIG_1       0x1D
#define SX1276_REG_MODEM_CONFIG_2       0x1E
#define SX1276_REG_MODEM_CONFIG_3       0x26
#define SX1276_REG_VERSION              0x42

#define SX1276_IRQ_RX_DONE 0x40
#define SX1276_IRQ_TX_DONE 0x08

static void sx1276_wr(uint8_t r, uint8_t v)
{
    uint8_t tx[2]={ (uint8_t)(r|0x80), v };
    spi_transaction_t t={.length=16,.tx_buffer=tx};
    spi_device_transmit(lora_spi,&t);
}
static uint8_t sx1276_rd(uint8_t r)
{
    uint8_t tx[2]={ (uint8_t)(r&0x7F), 0 };
    uint8_t rx[2]={0};
    spi_transaction_t t={.length=16,.tx_buffer=tx,.rx_buffer=rx};
    spi_device_transmit(lora_spi,&t);
    return rx[1];
}
static void sx1276_clear_irq(void){ sx1276_wr(SX1276_REG_IRQ_FLAGS, 0xFF); }

static uint32_t hz_to_frf(uint32_t f)
{
    uint64_t frf=((uint64_t)f<<19)/32000000ULL;
    return (uint32_t)frf;
}

static void lora_init_radio(void)
{
    lora_spi_lock();

    // sleep
    sx1276_wr(SX1276_REG_OP_MODE, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 863.000 MHz / SF7 / BW125 / CR4/5 / CRC on
    uint32_t frf=hz_to_frf(DEFAULT_RF_FREQ_HZ);
    sx1276_wr(SX1276_REG_FRF_MSB, frf>>16);
    sx1276_wr(SX1276_REG_FRF_MID, frf>>8);
    sx1276_wr(SX1276_REG_FRF_LSB, frf);

    sx1276_wr(SX1276_REG_MODEM_CONFIG_1, 0x72);
    sx1276_wr(SX1276_REG_MODEM_CONFIG_2, (7<<4)|(1<<2)|0x03);
    sx1276_wr(SX1276_REG_MODEM_CONFIG_3, 0x04);

    sx1276_wr(SX1276_REG_PA_CONFIG, 0x8E);

    sx1276_wr(SX1276_REG_FIFO_RX_BASE_ADDR, 0);
    sx1276_wr(SX1276_REG_FIFO_ADDR_PTR, 0);

    // RX continuous
    sx1276_wr(SX1276_REG_OP_MODE, 0x85);

    lora_spi_unlock();
}

static void lora_set_rx_continuous(void)
{
    lora_spi_lock();
    sx1276_wr(SX1276_REG_OP_MODE, 0x85);
    lora_spi_unlock();
}

static void lora_recover_radio(void)
{
    ESP_LOGE(TAG, "LoRa recover (SX1276)");
    lora_init_radio();
}

static void lora_send_frame(const uint8_t *d, size_t len)
{
    // optional CAD backoff
    if(g_cad_enable){
        uint32_t w = (esp_random() % (CAD_JITTER_MS+1));
        vTaskDelay(pdMS_TO_TICKS(CAD_WAIT_MS + w));
    }

    lora_spi_lock();

    sx1276_clear_irq();
    sx1276_wr(SX1276_REG_FIFO_TX_BASE_ADDR, 0);
    sx1276_wr(SX1276_REG_FIFO_ADDR_PTR, 0);
    for(size_t i=0;i<len;i++) sx1276_wr(SX1276_REG_FIFO, d[i]);
    sx1276_wr(SX1276_REG_PAYLOAD_LENGTH, (uint8_t)len);

    sx1276_wr(SX1276_REG_OP_MODE, 0x83); // TX

    uint32_t t0 = now_ms();
    while(1){
        uint8_t irq = sx1276_rd(SX1276_REG_IRQ_FLAGS);
        if(irq & SX1276_IRQ_TX_DONE) break;
        if((now_ms() - t0) > LORA_TX_TIMEOUT_MS){
            lora_spi_unlock();
            ESP_LOGE(TAG, "LoRa TX timeout! Recovering radio...");
            lora_recover_radio();
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    sx1276_clear_irq();
    sx1276_wr(SX1276_REG_OP_MODE, 0x85); // RX cont

    lora_spi_unlock();
}

static bool lora_try_read_frame(uint8_t *buf, size_t maxlen, uint8_t *out_len, int *out_rssi)
{
    uint8_t len=0;
    int rssi=-127;

    lora_spi_lock();

    uint8_t irq = sx1276_rd(SX1276_REG_IRQ_FLAGS);
    if(!(irq & SX1276_IRQ_RX_DONE)){
        lora_spi_unlock();
        return false;
    }

    len = sx1276_rd(SX1276_REG_RX_NB_BYTES);
    if(len==0 || len>maxlen){
        sx1276_clear_irq();
        lora_spi_unlock();
        return false;
    }

    uint8_t addr = sx1276_rd(SX1276_REG_FIFO_RX_CURRENT_ADDR);
    sx1276_wr(SX1276_REG_FIFO_ADDR_PTR, addr);

    for(int i=0;i<len;i++) buf[i]=sx1276_rd(SX1276_REG_FIFO);

    sx1276_clear_irq();
    rssi = (int)sx1276_rd(SX1276_REG_PKT_RSSI_VALUE) - 157;

    lora_spi_unlock();

    *out_len = len;
    *out_rssi = rssi;
    return true;
}

static void lora_log_chip_info(void)
{
    lora_spi_lock();
    uint8_t v = sx1276_rd(SX1276_REG_VERSION);
    lora_spi_unlock();
    ESP_LOGI(TAG,"LoRa chip: SX1276/78 (RegVersion=0x%02X, expect ~0x12)", v);
}

#endif // MR_LORA_CHIP_SX1276


// ----------------------------- SX1262 DRIVER ------------------------------
// ----------------------------- SX1262 DRIVER (COMPLETE) -------------------
// ----------------------------- SX1262 DRIVER (FIXED) -----------------------
#if defined(MR_LORA_CHIP_SX1262)

// SX126x opcodes (SX1261/2)
#define SX126X_SET_STANDBY           0x80
#define SX126X_SET_PACKET_TYPE       0x8A
#define SX126X_SET_RF_FREQUENCY      0x86
#define SX126X_SET_MODULATION        0x8B
#define SX126X_SET_PACKET_PARAMS     0x8C
#define SX126X_SET_BUFFER_BASE       0x8F
#define SX126X_SET_TX                0x83
#define SX126X_SET_RX                0x82
#define SX126X_GET_IRQ_STATUS        0x12           //15   // ✅ korrekt
#define SX126X_CLEAR_IRQ_STATUS      0x02
#define SX126X_SET_DIO_IRQ_PARAMS    0x08
#define SX126X_WRITE_BUFFER          0x0E
#define SX126X_READ_BUFFER           0x1E
#define SX126X_GET_PACKET_STATUS     0x14
#define SX126X_GET_RX_BUFFER_STATUS  0x13

// Heltec/SX1262 spezifisch (aus deinem funktionierenden Test)
#define SX126X_SET_DIO3_TCXO_CTRL    0x97
#define SX126X_CALIBRATE             0x89
#define SX126X_SET_PA_CONFIG         0x95
#define SX126X_SET_TX_PARAMS         0x8E
#define SX126X_SET_DIO2_RFSWITCH     0x9D

// IRQ bits (SX126x)
#define SX126X_IRQ_TX_DONE           (1<<0)
#define SX126X_IRQ_RX_DONE           (1<<1)
#define SX126X_IRQ_TIMEOUT           (1<<9)
#define SX126X_IRQ_CRC_ERR           (1<<6)

static void sx126x_busy_wait(void)
{
    const uint32_t t0 = now_ms();
    while(gpio_get_level(PIN_LORA_BUSY) == 1){
        if(now_ms() - t0 > 1000){
            // BUSY hängt -> nicht endlos blockieren
            break;
        }
        esp_rom_delay_us(10);
    }
}

/*static void sx126x_xfer(const uint8_t *tx, uint8_t *rx, size_t n)
{
    spi_transaction_t t = {0};
    t.length    = n * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    spi_device_transmit(lora_spi, &t); // CS auto (spics_io_num)
}
*/
static void sx126x_xfer(const uint8_t *tx, uint8_t *rx, size_t n)
{
    spi_transaction_t t = {0};
    t.length = n * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    // Polling ist bei LoRa-Kommandos VIEL stabiler
    spi_device_polling_transmit(lora_spi, &t); 
}


// reines Kommando (Opcode+Args) ohne Readback
static void sx126x_cmd(const uint8_t *cmd, size_t n)
{
    uint8_t rx_dummy[64] = {0};
    if(n > sizeof(rx_dummy)) return;

    sx126x_busy_wait();
    sx126x_xfer(cmd, rx_dummy, n);
    sx126x_busy_wait();
}

// Lesen: opcode + dummy + n bytes
static void sx126x_read(uint8_t opcode, uint8_t *out, size_t n)
{
    if(n > 64) return;

    uint8_t tx[2 + 64] = {0};
    uint8_t rx[2 + 64] = {0};

    tx[0] = opcode;
    tx[1] = 0x00; // dummy

    sx126x_busy_wait();
    sx126x_xfer(tx, rx, 2 + n);
    sx126x_busy_wait();

    memcpy(out, rx + 2, n);
}

static void sx126x_clear_irq(uint16_t mask)
{
    uint8_t cmd[3] = {
        SX126X_CLEAR_IRQ_STATUS,
        (uint8_t)(mask >> 8),
        (uint8_t)(mask & 0xFF)
    };
    sx126x_cmd(cmd, sizeof(cmd));
}

static uint16_t sx126x_get_irq(void)
{
    uint8_t b[2] = {0};
    sx126x_read(SX126X_GET_IRQ_STATUS, b, 2);
    return (uint16_t)((b[0] << 8) | b[1]);
}

// WriteBuffer MUSS in EINER Transaktion sein: opcode + payload(offset+data)
static void sx126x_cmd_with_payload(uint8_t opcode, const uint8_t *payload, size_t n)
{
    // n = offset(1) + data(len) => max 256
    if(n > 256) return;

    uint8_t tx[1 + 256] = {0};
    uint8_t rx_dummy[1 + 256] = {0};

    tx[0] = opcode;
    memcpy(&tx[1], payload, n);

    sx126x_busy_wait();
    sx126x_xfer(tx, rx_dummy, 1 + n);
    sx126x_busy_wait();
}

// Heltec/SX1262: PacketParams müssen PayloadLen korrekt setzen
static void sx126x_set_packet_params(uint8_t payload_len)
{
    // LoRa PacketParams:
    // preamble = 8 (0x0008)
    // header   = explicit (0x00)
    // len      = payload_len
    // CRC      = on (0x01)
    // IQ       = standard (0x00)
    uint8_t pkt[7] = {
        SX126X_SET_PACKET_PARAMS,
        0x00, 0x08,
        0x00,
        payload_len,
        0x01,
        0x00
    };
    sx126x_cmd(pkt, sizeof(pkt));
}

static void lora_init_radio(void)
{
    // Nach heltec_vext_on()
    gpio_set_direction(PIN_LORA_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_LORA_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_LORA_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50)); // Warten bis Chip bereit
    // BUSY input
    gpio_set_direction(PIN_LORA_BUSY, GPIO_MODE_INPUT);

    lora_spi_lock();

    // Standby XOSC (Heltec stabil)
    { uint8_t stby[2] = { SX126X_SET_STANDBY, 0x01 }; sx126x_cmd(stby, sizeof(stby)); }

    // TCXO on DIO3 (1.8V) + delay (0x000064)
    { uint8_t tcxo[5] = { SX126X_SET_DIO3_TCXO_CTRL, 0x01, 0x00, 0x00, 0x64 }; sx126x_cmd(tcxo, sizeof(tcxo)); }
    vTaskDelay(pdMS_TO_TICKS(5)); // ✅ echtes Delay hilft Heltec

    // Calibrate all
    { uint8_t cal[2] = { SX126X_CALIBRATE, 0x7F }; sx126x_cmd(cal, sizeof(cal)); }

    // Packet type: LoRa
    { uint8_t ptype[2] = { SX126X_SET_PACKET_TYPE, 0x01 }; sx126x_cmd(ptype, sizeof(ptype)); }

    // DIO2 RF switch auto
    { uint8_t dio2[2] = { SX126X_SET_DIO2_RFSWITCH, 0x01 }; sx126x_cmd(dio2, sizeof(dio2)); }

    // PA config (Heltec example)
    { uint8_t pa[5] = { SX126X_SET_PA_CONFIG, 0x04, 0x07, 0x00, 0x01 }; sx126x_cmd(pa, sizeof(pa)); }

    // TxParams: 14 dBm + 200us ramp (safe default)
    { uint8_t txp[3] = { SX126X_SET_TX_PARAMS, 0x0E, 0x04 }; sx126x_cmd(txp, sizeof(txp)); }

    // RF frequency: DEFAULT_RF_FREQ_HZ (bei dir 863 MHz)
    uint32_t rf = (uint32_t)(((uint64_t)DEFAULT_RF_FREQ_HZ << 25) / 32000000ULL);
    uint8_t rfcmd[5] = {
        SX126X_SET_RF_FREQUENCY,
        (uint8_t)(rf >> 24), (uint8_t)(rf >> 16),
        (uint8_t)(rf >> 8),  (uint8_t)(rf)
    };
    sx126x_cmd(rfcmd, sizeof(rfcmd));

    // Modulation: SF7, BW125(0x04), CR4/5(0x01), LDRO=0
    { uint8_t mod[5] = { SX126X_SET_MODULATION, 0x07, 0x04, 0x01, 0x00 }; sx126x_cmd(mod, sizeof(mod)); }

    // Buffer base: TX=0, RX=0
    { uint8_t base[3] = { SX126X_SET_BUFFER_BASE, 0x00, 0x00 }; sx126x_cmd(base, sizeof(base)); }

    // Default PacketParams (payload len wird pro TX gesetzt)
    sx126x_set_packet_params(0xFF);

    // IRQ mapping to DIO1
    uint16_t mask = SX126X_IRQ_RX_DONE | SX126X_IRQ_TX_DONE | SX126X_IRQ_CRC_ERR | SX126X_IRQ_TIMEOUT;
    uint8_t dio[9] = {
        SX126X_SET_DIO_IRQ_PARAMS,
        (uint8_t)(mask >> 8), (uint8_t)(mask & 0xFF), // irq mask
        (uint8_t)(mask >> 8), (uint8_t)(mask & 0xFF), // dio1 mask
        0x00, 0x00, // dio2 mask
        0x00, 0x00  // dio3 mask
    };
    sx126x_cmd(dio, sizeof(dio));

    sx126x_clear_irq(0xFFFF);

    // RX continuous
    { uint8_t rx[4] = { SX126X_SET_RX, 0xFF, 0xFF, 0xFF }; sx126x_cmd(rx, sizeof(rx)); }

    lora_spi_unlock();
}

static void lora_set_rx_continuous(void)
{
    lora_spi_lock();
    uint8_t rx[4] = { SX126X_SET_RX, 0xFF, 0xFF, 0xFF };
    sx126x_cmd(rx, sizeof(rx));
    lora_spi_unlock();
}

static void lora_recover_radio(void)
{
    ESP_LOGE(TAG, "LoRa recover (SX1262): re-init");
    lora_init_radio();
}

static void lora_send_frame(const uint8_t *d, size_t len)
{
    if(g_cad_enable){
        uint32_t w = (esp_random() % (CAD_JITTER_MS + 1));
        vTaskDelay(pdMS_TO_TICKS(CAD_WAIT_MS + w));
    }

    if(len > 255) len = 255;

    lora_spi_lock();

    // WriteBuffer: opcode + [offset=0] + payload  (EINE Transaktion)
    uint8_t wb[1 + 255];
    wb[0] = 0x00; // offset
    memcpy(&wb[1], d, len);
    sx126x_cmd_with_payload(SX126X_WRITE_BUFFER, wb, 1 + len);

    // PacketParams: echte Länge setzen (Heltec wichtig)
    sx126x_set_packet_params((uint8_t)len);

    // Clear IRQ
    sx126x_clear_irq(0xFFFF);

    // SetTx timeout ~2s (in 15.625us units)
    uint32_t to = 128000;
    uint8_t txcmd[4] = { SX126X_SET_TX, (uint8_t)(to >> 16), (uint8_t)(to >> 8), (uint8_t)to };
    sx126x_cmd(txcmd, sizeof(txcmd));

    // Wait TX_DONE
    uint32_t t0 = now_ms();
    while(1){
        uint16_t irq = sx126x_get_irq();

        if(irq & SX126X_IRQ_TX_DONE){
            sx126x_clear_irq(SX126X_IRQ_TX_DONE);
            break;
        }
        if(irq & SX126X_IRQ_TIMEOUT){
            sx126x_clear_irq(SX126X_IRQ_TIMEOUT);
            lora_spi_unlock();
            ESP_LOGE(TAG, "LoRa TX timeout! Recovering radio...");
            lora_recover_radio();
            return;
        }
        if((now_ms() - t0) > (LORA_TX_TIMEOUT_MS + 200)){
            lora_spi_unlock();
            ESP_LOGE(TAG, "LoRa TX timeout (host)! Recovering radio...");
            lora_recover_radio();
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    // Back to RX continuous
    uint8_t rx[4] = { SX126X_SET_RX, 0xFF, 0xFF, 0xFF };
    sx126x_cmd(rx, sizeof(rx));

    lora_spi_unlock();
}

static bool lora_try_read_frame(uint8_t *buf, size_t maxlen, uint8_t *out_len, int *out_rssi)
{
    lora_spi_lock();

    uint16_t irq = sx126x_get_irq();

    // CRC error -> drop
    if(irq & SX126X_IRQ_CRC_ERR){
        sx126x_clear_irq(SX126X_IRQ_CRC_ERR);
        lora_spi_unlock();
        return false;
    }
    // timeout -> drop
    if(irq & SX126X_IRQ_TIMEOUT){
        sx126x_clear_irq(SX126X_IRQ_TIMEOUT);
        lora_spi_unlock();
        return false;
    }

    if(!(irq & SX126X_IRQ_RX_DONE)){
        lora_spi_unlock();
        return false;
    }

    // Clear RX_DONE early
    sx126x_clear_irq(SX126X_IRQ_RX_DONE);

    // GetRxBufferStatus: returns [payloadLen, startPointer]
    uint8_t info[2] = {0};
    sx126x_read(SX126X_GET_RX_BUFFER_STATUS, info, 2);
    uint8_t payLen = info[0];
    uint8_t start  = info[1];

    if(payLen == 0 || payLen > maxlen){
        lora_spi_unlock();
        return false;
    }

    // ReadBuffer: opcode + offset + dummy + payload (dummy clocks required)
    uint8_t txb[3 + 255] = {0};
    uint8_t rxb[3 + 255] = {0};

    txb[0] = SX126X_READ_BUFFER;
    txb[1] = start;
    txb[2] = 0x00; // dummy
    // rest remains 0x00 to clock out data

    sx126x_busy_wait();
    sx126x_xfer(txb, rxb, 3 + payLen);
    sx126x_busy_wait();

    memcpy(buf, rxb + 3, payLen);

    // RSSI from GetPacketStatus (RSSI_sync in -0.5 dB steps)
    uint8_t st[3] = {0};
    sx126x_read(SX126X_GET_PACKET_STATUS, st, 3);
    int rssi = -(int)(st[0] / 2);

    lora_spi_unlock();

    *out_len  = payLen;
    *out_rssi = rssi;
    return true;
}

// Optional: Status lesen (ohne manuelles CS!)
static uint8_t sx1262_get_status(void)
{
    // GetStatus: opcode 0xC0, returns status in first received byte
    uint8_t tx[2] = { 0xC0, 0x00 };
    uint8_t rx[2] = { 0, 0 };

    sx126x_busy_wait();
    spi_transaction_t t = { .length = 16, .tx_buffer = tx, .rx_buffer = rx };
    spi_device_polling_transmit(lora_spi, &t);
    sx126x_busy_wait();

    return rx[0];
}

static void lora_log_chip_info(void)
{
    uint8_t st = sx1262_get_status();
    ESP_LOGI(TAG, "LoRa chip: SX1262 (Heltec V3.x) status=0x%02X", st);
}

#endif // MR_LORA_CHIP_SX1262


// ============================================================================
// =============================== Relay (GPIO) ===============================
// ============================================================================
#if MR_RELAY_ENABLE
static void relay_init(void)
{
    gpio_config_t io={
        .intr_type=GPIO_INTR_DISABLE,
        .mode=GPIO_MODE_OUTPUT,
        .pin_bit_mask=(1ULL<<RELAY_GPIO),
        .pull_down_en=0,
        .pull_up_en=0
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    // default OFF
    int level_off = (RELAY_ACTIVE_LEVEL ? 0 : 1);
    gpio_set_level(RELAY_GPIO, level_off);
    g_relay_on=false;
}
static void relay_set(bool on)
{
    g_relay_on=on;
    int level = on ? (RELAY_ACTIVE_LEVEL ? 1 : 0) : (RELAY_ACTIVE_LEVEL ? 0 : 1);
    gpio_set_level(RELAY_GPIO, level);
}
static void relay_toggle(void){ relay_set(!g_relay_on); }
#endif


// ============================================================================
// =============================== Battery ADC ================================
// ============================================================================
#if MR_BATT_ENABLE
static int batt_adc_gpio_to_chan(gpio_num_t gpio)
{
    // ESP32-S3 (Heltec V3) Mapping
    #if CONFIG_IDF_TARGET_ESP32S3
    switch(gpio){
        case 1:  return ADC_CHANNEL_0; // Heltec V3.2 VBAT
        case 2:  return ADC_CHANNEL_1;
        case 3:  return ADC_CHANNEL_2;
        case 4:  return ADC_CHANNEL_3;
        default: return -1;
    }
    #else
    // Klassisches ESP32 (LILYGO) Mapping
    switch(gpio){
        case 36: return ADC_CHANNEL_0;
        case 34: return ADC_CHANNEL_6;
        // ... deine restlichen Cases
        default: return -1;
    }
    #endif
}

#if CONFIG_IDF_TARGET_ESP32S3

#if MR_BATT_ENABLE
static void batt_enable_hw(void)
{
#ifdef BATT_EN_GPIO
    gpio_config_t io = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << BATT_EN_GPIO),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    ESP_ERROR_CHECK(gpio_config(&io));

#if BATT_EN_ACTIVE_LOW
    gpio_set_level(BATT_EN_GPIO, 0);   // ✅ enable divider
#else
    gpio_set_level(BATT_EN_GPIO, 1);
#endif
    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_LOGI(TAG, "VBAT divider enabled (GPIO%d=%d)",
             BATT_EN_GPIO,
#if BATT_EN_ACTIVE_LOW
             0
#else
             1
#endif
    );
#endif
}
#endif

static void batt_init(void)
{
    // WICHTIG: Vext muss AN sein, damit der Spannungsteiler Strom bekommt
    // Das hast du bereits im Radio-Init (GPIO 36 = 0)
    batt_enable_hw();

    int ch = batt_adc_gpio_to_chan((gpio_num_t)BATT_ADC_GPIO);
    if(ch < 0){
        ESP_LOGE(TAG, "Battery ADC: GPIO%d auf S3 nicht unterstützt!", BATT_ADC_GPIO);
        return;
    }

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &g_adc_unit));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12, // S3 nutzt 12dB für volle 3.3V Range
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc_unit, (adc_channel_t)ch, &chan_cfg));

    // Kalibrierung für S3 (Line Fitting ist hier Standard)
    g_adc_cali_ok = false;
    #if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        adc_cali_line_fitting_config_t cali_cfg = {
            .unit_id = ADC_UNIT_1,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        if (adc_cali_create_scheme_line_fitting(&cali_cfg, &g_adc_cali) == ESP_OK) {
            g_adc_cali_ok = true;
        }
    #endif

    g_next_batt_ms = now_ms() + 500;
}
#endif

#ifndef CONFIG_IDF_TARGET_ESP32S3
static void batt_init(void)
{
    int ch = batt_adc_gpio_to_chan((gpio_num_t)BATT_ADC_GPIO);
    if(ch < 0){
        ESP_LOGW(TAG,"Battery ADC: GPIO%d not supported in mapping -> disabled", BATT_ADC_GPIO);
        return;
    }

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &g_adc_unit));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_11,     // wide range
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc_unit, (adc_channel_t)ch, &chan_cfg));

    // calibration (optional)
    g_adc_cali_ok=false;
    if(MR_ADC_CALI_MODE){
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        adc_cali_curve_fitting_config_t cali_cfg = {
            .unit_id  = ADC_UNIT_1,
            .chan     = (adc_channel_t)ch,
            .atten    = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT
        };
        if(adc_cali_create_scheme_curve_fitting(&cali_cfg, &g_adc_cali) == ESP_OK){
            g_adc_cali_ok=true;
        }else{
            ESP_LOGW(TAG,"ADC cali (curve fitting) failed -> using raw fallback");
        }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        adc_cali_line_fitting_config_t cali_cfg = {
            .unit_id  = ADC_UNIT_1,
            .atten    = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT
        };
        if(adc_cali_create_scheme_line_fitting(&cali_cfg, &g_adc_cali) == ESP_OK){
            g_adc_cali_ok=true;
        }else{
            ESP_LOGW(TAG,"ADC cali (line fitting) failed -> using raw fallback");
        }
#else
        ESP_LOGW(TAG,"ADC calibration not supported in this ESP-IDF build -> using raw fallback");
#endif
    }

    g_next_batt_ms = now_ms() + 500;
}

#endif

static uint32_t batt_mv_to_pct(uint32_t mv)
{
    if(mv <= BATT_EMPTY_MV) return 0;
    if(mv >= BATT_FULL_MV)  return 100;
    uint32_t span = (uint32_t)(BATT_FULL_MV - BATT_EMPTY_MV);
    return (uint32_t)(((mv - BATT_EMPTY_MV) * 100U) / span);
}

#if MR_BATT_ENABLE
static void batt_poll(void)
{
    if(!g_adc_unit) return;

    uint32_t t = now_ms();
    if(t < g_next_batt_ms) return;
    g_next_batt_ms = t + BATT_MEASURE_INTERVAL_MS;

    // --- Heltec: VBAT divider via GPIO37 (ADC_Ctrl) ---
#ifdef BATT_EN_GPIO
#if BATT_EN_ACTIVE_LOW
    gpio_set_level(BATT_EN_GPIO, 0);          // enable divider
#else
    gpio_set_level(BATT_EN_GPIO, 1);
#endif
    vTaskDelay(pdMS_TO_TICKS(4));             // settle time (3–5ms)
#endif

    // Resolve ADC channel from GPIO (works on ESP32 + ESP32-S3)
    adc_unit_t unit;
    adc_channel_t ch;
    esp_err_t e = adc_oneshot_io_to_channel((gpio_num_t)BATT_ADC_GPIO, &unit, &ch);
    if(e != ESP_OK){
#ifdef BATT_EN_GPIO
        // disable divider again if we enabled it
#if BATT_EN_ACTIVE_LOW
        gpio_set_level(BATT_EN_GPIO, 1);
#else
        gpio_set_level(BATT_EN_GPIO, 0);
#endif
#endif
        return;
    }

    // Optional: throw away first read for stability
    int raw_dummy = 0;
    (void)adc_oneshot_read(g_adc_unit, ch, &raw_dummy);

    int raw = 0;
    if(adc_oneshot_read(g_adc_unit, ch, &raw) != ESP_OK){
#ifdef BATT_EN_GPIO
#if BATT_EN_ACTIVE_LOW
        gpio_set_level(BATT_EN_GPIO, 1);
#else
        gpio_set_level(BATT_EN_GPIO, 0);
#endif
#endif
        return;
    }

#ifdef BATT_EN_GPIO
    // disable divider to save battery
#if BATT_EN_ACTIVE_LOW
    gpio_set_level(BATT_EN_GPIO, 1);
#else
    gpio_set_level(BATT_EN_GPIO, 0);
#endif
#endif

    int mv_adc = 0;
    if(g_adc_cali_ok && g_adc_cali){
        if(adc_cali_raw_to_voltage(g_adc_cali, raw, &mv_adc) != ESP_OK){
            mv_adc = 0;
        }
    }else{
        // rough fallback (calibration recommended)
        mv_adc = (raw * 3300) / 4095;
    }

    float scale = (BATT_DIV_RTOP_OHMS + BATT_DIV_RBOT_OHMS) / BATT_DIV_RBOT_OHMS;
    uint32_t vbat = (uint32_t)((float)mv_adc * scale);

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_batt_mv  = vbat;
    g_batt_pct = batt_mv_to_pct(vbat);
    xSemaphoreGive(g_mutex);
}
#endif
#endif


// ============================================================================
// =============================== Pending ACK ================================
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


// ============================================================================
// ============================ WiFi + HTTP ===================================
// ============================================================================
static void wifi_init_once(void)
{
    if(g_wifi_inited) return;

    ESP_ERROR_CHECK(esp_netif_init());
    esp_err_t e = esp_event_loop_create_default();
    if(e != ESP_OK && e != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(e);

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg=WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap={0};
    strcpy((char*)ap.ap.ssid, MR_WIFI_AP_SSID);
    ap.ap.max_connection=4;

    if(strlen(MR_WIFI_AP_PASS) >= 8){
        strcpy((char*)ap.ap.password, MR_WIFI_AP_PASS);
        ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
    }else{
        ap.ap.authmode = WIFI_AUTH_OPEN;
        ap.ap.password[0] = 0;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP,&ap));

    g_wifi_inited=true;
}

static void wifi_start_ap(void)
{
    wifi_init_once();
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG,"WiFi AP started: %s (http://192.168.4.1)", MR_WIFI_AP_SSID);
}

static void wifi_stop_ap(void)
{
    esp_err_t e=esp_wifi_stop();
    if(e != ESP_OK && e != ESP_ERR_WIFI_NOT_INIT) ESP_LOGW(TAG,"esp_wifi_stop: %s", esp_err_to_name(e));
    ESP_LOGI(TAG,"WiFi AP stopped");
}

static void http_stop(void)
{
    if(g_http_running && g_http){
        httpd_stop(g_http);
        g_http=NULL;
        g_http_running=false;
        ESP_LOGI(TAG,"HTTP server stopped");
    }
}

static void http_start_if_needed(void); // forward

static void set_wifi_enabled(bool en)
{
    if(en == g_wifi_enabled) return;
    g_wifi_enabled=en;

    if(en){
        wifi_start_ap();
        http_start_if_needed();
    }else{
        http_stop();
        wifi_stop_ap();
    }
}


// ============================================================================
// =============================== HTTP helpers ===============================
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
// =============================== Web UI HTML ================================
// ============================================================================
static const char *INDEX_HTML =
"<!doctype html><html><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>MeshRadio 34+</title>"
"<style>"
"body{font-family:system-ui;margin:16px;max-width:1200px}"
"button{padding:10px 14px;margin:6px 4px;font-size:16px}"
"input{padding:10px;margin:6px 0;font-size:16px;width:100%}"
".card{border:1px solid #ddd;border-radius:12px;padding:12px;box-shadow:0 1px 3px rgba(0,0,0,.06);margin:10px 0}"
".muted{color:#666}.err{color:#b00020;font-weight:600}.ok{color:#0b7a26;font-weight:600}"
".grid{display:grid;grid-template-columns:1fr;gap:12px}"
"@media(min-width:980px){.grid{grid-template-columns:1fr 1fr 1fr}}"
"table{width:100%;border-collapse:collapse;font-size:14px}"
"th,td{border-bottom:1px solid #eee;padding:6px 8px;text-align:left;vertical-align:top}"
"th{background:#fafafa}"
".pill{display:inline-block;padding:2px 8px;border-radius:999px;border:1px solid #ddd;font-size:12px}"
".amp{display:inline-block;width:12px;height:12px;border-radius:50%;vertical-align:middle;margin-right:6px;border:1px solid #aaa}"
"</style></head><body>"
"<h2>MeshRadio 34+ – Roles, Security, Relay, WiFi, Dashboard</h2>"
"<p class='muted'>AP: <code>" MR_WIFI_AP_SSID "</code> • URL: <code>http://192.168.4.1</code></p>"
"<div class='card'>"
"<div><b><span id='amp' class='amp'></span>UI-Ampel:</b> <span id='ampTxt'>loading…</span> <span class='pill' id='agePill'>?</span></div>"
"<div><b>Status:</b> <span id='st'>loading…</span></div>"
"<div id='err' class='err'></div>"
"<button onclick='refreshAll()'>Refresh</button>"
"</div>"
"<div class='grid'>"
" <div class='card'>"
"  <h3>DATA senden</h3>"
"  <label>Ziel (dst)</label><input id='dst' value='DJ1ABCF'>"
"  <label>ACK (0/1)</label><input id='ack' value='1'>"
"  <label>Text</label><input id='msg' value='CMD:STATUS?'>"
"  <button onclick='send()'>SEND</button>"
"  <div id='m' class='muted'></div>"
" </div>"
" <div class='card'>"
"  <h3>Node Role</h3>"
"  <div><b>Mode:</b> <span id='nm' class='ok'>?</span></div>"
"  <button onclick='setRole(0)'>RELAY</button>"
"  <button onclick='setRole(1)'>EDGE</button>"
"  <button onclick='setRole(2)'>SENSOR</button>"
" </div>"
" <div class='card'>"
"  <h3>Security</h3>"
"  <div><b>Crypto:</b> <span id='cst' class='ok'>?</span></div>"
"  <button onclick='setCrypto(1)'>Crypto ON</button>"
"  <button onclick='setCrypto(0)'>Crypto OFF</button>"
" </div>"
" <div class='card'>"
"  <h3>WiFi / Web UI</h3>"
"  <button onclick='setWiFi(1)'>WiFi ON</button>"
"  <button onclick='setWiFi(0)'>WiFi OFF</button>"
" </div>"
"</div>"
"<div class='card'><h3>Last RX</h3><div id='lastrx'>loading…</div></div>"
"<div class='card'><h3>Security Stats</h3><pre id='secstats'>loading…</pre></div>"
"<div class='card'><h3>Counters</h3><pre id='counters'>loading…</pre></div>"
"<div class='card'><h3>Neighbors</h3><div id='neigh'>loading…</div></div>"
"<div class='card'><h3>Routes</h3><div id='routes'>loading…</div></div>"
"<script>"
"async function post(url,body){let r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body});return await r.text();}"
"function esc(s){return (s||'').replaceAll('&','&amp;').replaceAll('<','&lt;').replaceAll('>','&gt;');}"
"function setAmp(color,txt,age){let a=document.getElementById('amp');a.style.background=color;document.getElementById('ampTxt').textContent=txt;document.getElementById('agePill').textContent=age;}"
"async function send(){let dst=document.getElementById('dst').value;let msg=document.getElementById('msg').value;let ack=document.getElementById('ack').value;let t=await post('/api/send','dst='+encodeURIComponent(dst)+'&msg='+encodeURIComponent(msg)+'&ack='+encodeURIComponent(ack));document.getElementById('m').textContent=t; refreshAll();}"
"async function setCrypto(v){ let t=await post('/api/crypto','enable='+encodeURIComponent(v)); document.getElementById('m').textContent=t; refreshAll(); }"
"async function setRole(m){ let t=await post('/api/role','mode='+encodeURIComponent(m)); document.getElementById('m').textContent=t; refreshAll(); }"
"async function setWiFi(v){ let t=await post('/api/wifi','enable='+encodeURIComponent(v)); document.getElementById('m').textContent=t; }"
"function renderTable(cols, rows){ let h='<table><thead><tr>' + cols.map(c=>'<th>'+esc(c)+'</th>').join('') + '</tr></thead><tbody>'; for(let r of rows){ h+='<tr>' + r.map(v=>'<td>'+esc(String(v))+'</td>').join('') + '</tr>'; } h+='</tbody></table>'; return h; }"
"async function refreshAll(){"
" document.getElementById('err').textContent='';"
" try{"
"  let j=JSON.parse(await (await fetch('/api/status')).text());"
"  document.getElementById('st').textContent='call='+j.call+' • mode='+j.node_mode_str+' • wifi='+j.wifi+' • relay='+j.relay+' • crypto='+j.crypto_enable+' • batt='+j.batt;"
"  document.getElementById('nm').textContent=j.node_mode_str;"
"  document.getElementById('cst').textContent=(j.crypto_enable==1)?'ON':'OFF';"
"  document.getElementById('cst').className=(j.crypto_enable==1)?'ok':'err';"
"  document.getElementById('secstats').textContent=j.secstats;"
"  document.getElementById('counters').textContent=j.counters;"
"  let lj=JSON.parse(await (await fetch('/api/lastrx')).text());"
"  let age=lj.age_ms;"
"  if(age<0){document.getElementById('lastrx').innerHTML='<span class=muted>No RX yet</span>'; setAmp('#bbb','NO RX','—');}"
"  else{document.getElementById('lastrx').innerHTML='<b>from</b> <code>'+esc(lj.from)+'</code><br><b>text</b> '+esc(lj.text)+'<br><span class=muted>age_ms='+age+'</span>';"
"    if(age <= 6000) setAmp('#19a34a','OK (fresh)',age+' ms'); else if(age <= 20000) setAmp('#f59e0b','STALE',age+' ms'); else setAmp('#dc2626','OLD',age+' ms');}"
"  let nj=JSON.parse(await (await fetch('/api/neighbors')).text());"
"  let nrows=nj.map(x=>[x.call, x.rssi, x.age_ms, x.tx_attempts, x.ack_ok, x.etx_x100]);"
"  document.getElementById('neigh').innerHTML = nrows.length ? renderTable(['call','rssi','age_ms','tx','ack','etx_x100'], nrows) : '<span class=muted>none</span>';"
"  let rj=JSON.parse(await (await fetch('/api/routes')).text());"
"  let rrows=rj.map(x=>[x.dst, x.next, x.etx_x100, x.seq, x.age_ms, x.hold_ms]);"
"  document.getElementById('routes').innerHTML = rrows.length ? renderTable(['dst','next','etx_x100','seq','age_ms','hold_ms'], rrows) : '<span class=muted>none</span>';"
" }catch(e){document.getElementById('st').textContent='ERROR';document.getElementById('err').textContent='Dashboard error: '+e; setAmp('#dc2626','UI ERROR','—');}"
"}"
"refreshAll(); setInterval(refreshAll, 3000);"
"</script></body></html>";

static esp_err_t index_get(httpd_req_t *req)
{
    httpd_resp_set_type(req,"text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}


// ============================================================================
// ============================ API: /api/status ==============================
// ============================================================================
static esp_err_t api_status_get(httpd_req_t *req)
{
    static char out[3072];
    size_t off=0;

    xSemaphoreTake(g_mutex, portMAX_DELAY);

    // counters text
    char counters[512];
    snprintf(counters, sizeof(counters),
        "NODE_MODE: %s (%d)\n"
        "TX: beacon=%" PRIu32 " adv=%" PRIu32 " ack=%" PRIu32 " data=%" PRIu32 "\n"
        "DEFER: beacon=%" PRIu32 " adv=%" PRIu32 " ack=%" PRIu32 " data=%" PRIu32 "\n"
        "DROP: adv=%" PRIu32 " data=%" PRIu32 "\n"
        "WIFI: %s  HTTP: %s\n",
        node_mode_str(g_node_mode), (int)g_node_mode,
        c_tx_beacon, c_tx_routeadv, c_tx_ack, c_tx_data,
        c_defer_beacon, c_defer_routeadv, c_defer_ack, c_defer_data,
        c_drop_routeadv, c_drop_data,
        g_wifi_enabled ? "ON" : "OFF",
        g_http_running ? "ON" : "OFF"
    );

    // security text
    char secstats[256];
    snprintf(secstats, sizeof(secstats),
        "crypto_enable=%u\n"
        "decrypt_ok=%" PRIu32 "\n"
        "decrypt_fail=%" PRIu32 "\n"
        "mac_fail=%" PRIu32 "\n"
        "replay_drop=%" PRIu32 "\n",
        g_crypto_enable?1:0,
        sec_decrypt_ok, sec_decrypt_fail, sec_mac_fail, sec_replay_drop
    );

    uint32_t batt_mv=0, batt_pct=0;
#if MR_BATT_ENABLE
    batt_mv = g_batt_mv;
    batt_pct = g_batt_pct;
#endif
    char batt_str[48];
    snprintf(batt_str, sizeof(batt_str), "%" PRIu32 "mV/%" PRIu32 "%%", batt_mv, batt_pct);

    off += snprintf(out+off, sizeof(out)-off,
        "{"
        "\"call\":\"%s\","
        "\"crypto_enable\":%u,"
        "\"node_mode\":%d,"
        "\"node_mode_str\":\"%s\","
        "\"wifi\":%u,"
        "\"relay\":%u,"
        "\"batt\":\"%s\","
        "\"secstats\":\"",
        MR_CALLSIGN,
        g_crypto_enable?1:0,
        (int)g_node_mode,
        node_mode_str(g_node_mode),
        g_wifi_enabled?1:0,
#if MR_RELAY_ENABLE
        g_relay_on?1:0,
#else
        0,
#endif
        batt_str
    );

    for(const char *p=secstats; *p && off < sizeof(out)-8; p++){
        if(*p=='\n'){ out[off++]='\\'; out[off++]='n'; }
        else if(*p=='"'){ out[off++]='\\'; out[off++]='"'; }
        else out[off++]=*p;
    }
    out[off++]='\"';

    off += snprintf(out+off, sizeof(out)-off, ",\"counters\":\"");
    for(const char *p=counters; *p && off < sizeof(out)-8; p++){
        if(*p=='\n'){ out[off++]='\\'; out[off++]='n'; }
        else if(*p=='"'){ out[off++]='\\'; out[off++]='"'; }
        else out[off++]=*p;
    }
    out[off++]='\"';
    out[off++]='}';

    if(off >= sizeof(out)) out[sizeof(out)-1]=0;
    else out[off]=0;

    xSemaphoreGive(g_mutex);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, out, HTTPD_RESP_USE_STRLEN);
}


// ============================================================================
// ============================ API: /api/lastrx ==============================
// ============================================================================
static esp_err_t api_lastrx_get(httpd_req_t *req)
{
    char out[512];

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    uint32_t t=now_ms();
    uint32_t lr=g_last_rx_ms;
    char from[32]; char text[220];
    json_escape_into(from, sizeof(from), g_last_rx_from);
    json_escape_into(text, sizeof(text), g_last_rx_text);
    xSemaphoreGive(g_mutex);

    int32_t age = (lr==0) ? -1 : (int32_t)(t - lr);

    snprintf(out, sizeof(out),
        "{"
        "\"from\":\"%s\","
        "\"text\":\"%s\","
        "\"age_ms\":%d"
        "}",
        from, text, (int)age
    );

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, out, HTTPD_RESP_USE_STRLEN);
}


// ============================================================================
// ============================ API: /api/neighbors ===========================
// ============================================================================
static esp_err_t api_neighbors_get(httpd_req_t *req)
{
    static char out[1536];
    size_t off=0;

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    uint32_t t=now_ms();

    off += snprintf(out+off, sizeof(out)-off, "[");

    bool first=true;
    for(int i=0;i<MAX_NEIGHBORS;i++){
        if(!neighbors[i].used) continue;

        char call[8]; call7_to_str(call, neighbors[i].call);
        uint32_t age = t - neighbors[i].t_ms;
        uint16_t etx = etx_compute_x100(neighbors[i].tx_attempts, neighbors[i].ack_ok);

        if(!first) off += snprintf(out+off, sizeof(out)-off, ",");
        first=false;

        off += snprintf(out+off, sizeof(out)-off,
            "{"
            "\"call\":\"%s\","
            "\"rssi\":%d,"
            "\"age_ms\":%" PRIu32 ","
            "\"tx_attempts\":%" PRIu32 ","
            "\"ack_ok\":%" PRIu32 ","
            "\"etx_x100\":%u"
            "}",
            call,
            neighbors[i].rssi,
            age,
            neighbors[i].tx_attempts,
            neighbors[i].ack_ok,
            (unsigned)etx
        );

        if(off > sizeof(out)-120) break;
    }

    off += snprintf(out+off, sizeof(out)-off, "]");
    if(off >= sizeof(out)) out[sizeof(out)-1]=0;
    else out[off]=0;

    xSemaphoreGive(g_mutex);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, out, HTTPD_RESP_USE_STRLEN);
}


// ============================================================================
// ============================ API: /api/routes ==============================
// ============================================================================
static esp_err_t api_routes_get(httpd_req_t *req)
{
    static char out[2048];
    size_t off=0;

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    uint32_t t=now_ms();

    off += snprintf(out+off, sizeof(out)-off, "[");
    bool first=true;

    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used) continue;

        char d[8], n[8];
        call7_to_str(d, routes[i].dst);
        call7_to_str(n, routes[i].next);

        uint32_t age = t - routes[i].t_ms;
        uint32_t hold = (t < routes[i].hold_until_ms) ? (routes[i].hold_until_ms - t) : 0;

        if(!first) off += snprintf(out+off, sizeof(out)-off, ",");
        first=false;

        off += snprintf(out+off, sizeof(out)-off,
            "{"
            "\"dst\":\"%s\","
            "\"next\":\"%s\","
            "\"etx_x100\":%u,"
            "\"seq\":%u,"
            "\"age_ms\":%" PRIu32 ","
            "\"hold_ms\":%" PRIu32
            "}",
            d, n,
            (unsigned)routes[i].etx_x100,
            (unsigned)routes[i].seq,
            age, hold
        );

        if(off > sizeof(out)-160) break;
    }

    off += snprintf(out+off, sizeof(out)-off, "]");
    if(off >= sizeof(out)) out[sizeof(out)-1]=0;
    else out[off]=0;

    xSemaphoreGive(g_mutex);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, out, HTTPD_RESP_USE_STRLEN);
}


// ============================================================================
// ============================== API: POSTs ==================================
// ============================================================================
static void send_data_to(const char *dst_str, const char *txt, bool ackreq); // forward

static esp_err_t api_send_post(httpd_req_t *req)
{
    char body[256];
    if(!http_read_body(req, body, sizeof(body))) return http_send_text(req, "ERR body");

    char dst[16]={0}, msg[160]={0}, ackv[8]={0};
    if(!form_get(body,"dst",dst,sizeof(dst))) return http_send_text(req,"ERR missing dst");
    if(!form_get(body,"msg",msg,sizeof(msg))) return http_send_text(req,"ERR missing msg");
    form_get(body,"ack",ackv,sizeof(ackv));

    bool ack = (ackv[0] != '0');
    send_data_to(dst, msg, ack);

    return http_send_text(req, "OK send");
}

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

static esp_err_t api_role_post(httpd_req_t *req)
{
    char body[64];
    if(!http_read_body(req, body, sizeof(body))) return http_send_text(req, "ERR body");

    char m[8]={0};
    if(!form_get(body,"mode",m,sizeof(m))) return http_send_text(req,"ERR missing mode");
    int v = atoi(m);
    if(v < 0 || v > 2) return http_send_text(req, "ERR mode must be 0/1/2");

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_node_mode = (node_mode_t)v;
    xSemaphoreGive(g_mutex);

    return http_send_text(req, node_mode_str(g_node_mode));
}

static esp_err_t api_wifi_post(httpd_req_t *req)
{
    char body[64];
    if(!http_read_body(req, body, sizeof(body))) return http_send_text(req, "ERR body");

    char v[8]={0};
    if(!form_get(body,"enable",v,sizeof(v))) return http_send_text(req,"ERR missing enable");
    int en = (v[0]=='1');
    set_wifi_enabled(en ? true : false);

    return http_send_text(req, en ? "OK wifi=1" : "OK wifi=0");
}


// ============================================================================
// =============================== HTTP start =================================
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

    httpd_uri_t lr0={.uri="/api/lastrx",.method=HTTP_GET,.handler=api_lastrx_get};
    httpd_register_uri_handler(g_http,&lr0);

    httpd_uri_t n0={.uri="/api/neighbors",.method=HTTP_GET,.handler=api_neighbors_get};
    httpd_register_uri_handler(g_http,&n0);

    httpd_uri_t r0={.uri="/api/routes",.method=HTTP_GET,.handler=api_routes_get};
    httpd_register_uri_handler(g_http,&r0);

    httpd_uri_t s0={.uri="/api/send",.method=HTTP_POST,.handler=api_send_post};
    httpd_register_uri_handler(g_http,&s0);

    httpd_uri_t cr0={.uri="/api/crypto",.method=HTTP_POST,.handler=api_crypto_post};
    httpd_register_uri_handler(g_http,&cr0);

    httpd_uri_t ro0={.uri="/api/role",.method=HTTP_POST,.handler=api_role_post};
    httpd_register_uri_handler(g_http,&ro0);

    httpd_uri_t w0={.uri="/api/wifi",.method=HTTP_POST,.handler=api_wifi_post};
    httpd_register_uri_handler(g_http,&w0);

    g_http_running=true;
    ESP_LOGI(TAG,"HTTP server started");
}

static void http_start_if_needed(void)
{
    if(g_http_running) return;
    http_start();
}


// ============================================================================
// =============================== Schedulers =================================
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
// ============================= Remote Commands ==============================
// ============================================================================
static void app_send_reply_to_sender(const char from7[7], const char *msg)
{
    char from_str[8];
    call7_to_str(from_str, from7);
    send_data_to(from_str, msg, true);
}

static void app_handle_cmd_if_any(const char from7[7], const char *txt)
{
    if(!txt || !txt[0]) return;
    if(g_node_mode != NODE_RELAY) return;

    if(strcmp(txt, "CMD:STATUS?")==0 || strcmp(txt, "STATUS?")==0){
        uint32_t batt_mv=0, batt_pct=0;
#if MR_BATT_ENABLE
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        batt_mv = g_batt_mv;
        batt_pct = g_batt_pct;
        xSemaphoreGive(g_mutex);
#endif
        char ans[240];
        snprintf(ans,sizeof(ans),
                 "STATUS: mode=%s relay=%s wifi=%s http=%s crypto=%s batt=%" PRIu32 "mV/%" PRIu32 "%%",
                 node_mode_str(g_node_mode),
#if MR_RELAY_ENABLE
                 g_relay_on?"ON":"OFF",
#else
                 "N/A",
#endif
                 g_wifi_enabled?"ON":"OFF",
                 g_http_running?"ON":"OFF",
                 g_crypto_enable?"ON":"OFF",
                 batt_mv, batt_pct);
        app_send_reply_to_sender(from7, ans);
        return;
    }

#if MR_RELAY_ENABLE
    if(strcmp(txt, "CMD:RELAY ON")==0){
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        relay_set(true);
        bool st=g_relay_on;
        xSemaphoreGive(g_mutex);
        app_send_reply_to_sender(from7, st?"RELAY: ON":"RELAY: OFF");
        return;
    }
    if(strcmp(txt, "CMD:RELAY OFF")==0){
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        relay_set(false);
        bool st=g_relay_on;
        xSemaphoreGive(g_mutex);
        app_send_reply_to_sender(from7, st?"RELAY: ON":"RELAY: OFF");
        return;
    }
    if(strcmp(txt, "CMD:RELAY TOGGLE")==0){
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        relay_toggle();
        bool st=g_relay_on;
        xSemaphoreGive(g_mutex);
        app_send_reply_to_sender(from7, st?"RELAY: ON":"RELAY: OFF");
        return;
    }
#endif
}


// ============================================================================
// ============================= TX: Beacon/ACK/DATA ==========================
// ============================================================================
static void send_beacon(void)
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
        lora_send_frame((uint8_t*)&h, sizeof(h));
    }else{
        c_defer_beacon++;
    }

    beacon_schedule_next();
}

static void send_ack(const mr_hdr_v7_t *rx)
{
    mr_hdr_v7_t h={0};
    h.magic[0]='M'; h.magic[1]='R';
    h.version=MR_PROTO_VERSION;
    h.flags=MR_FLAG_ACK;
    h.ttl=ACK_TTL;
    h.msg_id = rx->msg_id;
    h.seq = g_my_seq; // ack doesn't increment seq here

    call7_set(h.src, MR_CALLSIGN);
    call7_set(h.last_hop, MR_CALLSIGN);
    memcpy(h.final_dst, rx->last_hop, 7);
    call7_set(h.next_hop, "*");
    h.payload_len=0;

    if(bucket_take(&b_ack)){
        c_tx_ack++;
        vTaskDelay(pdMS_TO_TICKS(esp_random()%150));
        lora_send_frame((uint8_t*)&h, sizeof(h));
    }else{
        c_defer_ack++;
    }
}

static void send_data_to(const char *dst_str, const char *txt, bool ackreq)
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

    // Für "profi-level stabil": routing kann hier wieder rein (route_lookup_locked).
    call7_set(h->next_hop, "*");

    uint8_t *pl = buf + sizeof(mr_hdr_v7_t);
    size_t n=strlen(txt);
    if(n>MAX_PLAINTEXT) n=MAX_PLAINTEXT;

    bool crypto=g_crypto_enable;
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

    if(bucket_take(&b_data)){
        c_tx_data++;
        lora_send_frame(buf, sizeof(mr_hdr_v7_t)+h->payload_len);
    }else{
        c_defer_data++;
    }
}


// ============================================================================
// =============================== RX (unlock-safe) ===========================
// ============================================================================
static void handle_rx(void)
{
    uint8_t buf[256];
    uint8_t len=0;
    int rssi=-127;

    if(!lora_try_read_frame(buf, sizeof(buf), &len, &rssi)) return;
    if(len < sizeof(mr_hdr_v7_t)) return;

    mr_hdr_v7_t *h=(mr_hdr_v7_t*)buf;
    if(h->magic[0]!='M'||h->magic[1]!='R') return;
    if(h->version!=MR_PROTO_VERSION) return;

    // ACK: not duplicate filtered
    if(h->flags & MR_FLAG_ACK){
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        neighbor_update_rssi_locked(h->last_hop, rssi);
        neighbor_ack_ok_locked(h->src);
        pending_mark_acked_locked(h->msg_id, h->src);
        xSemaphoreGive(g_mutex);
        return;
    }

    if(seen_before(h->src, h->msg_id)) return;
    remember_msg(h->src, h->msg_id);

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    neighbor_update_rssi_locked(h->last_hop, rssi);
    xSemaphoreGive(g_mutex);

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

    if(h->flags & MR_FLAG_DATA){
        char my7[7]; call7_set(my7, MR_CALLSIGN);
        if(call7_eq(h->final_dst, my7)){
            char txt[MAX_PAYLOAD+1]={0};

            if((h->flags & MR_FLAG_SEC) != 0){
                if(h->payload_len < SEC_TAG_LEN){
                    xSemaphoreTake(g_mutex, portMAX_DELAY);
                    sec_decrypt_fail++;
                    xSemaphoreGive(g_mutex);
                    return;
                }

                xSemaphoreTake(g_mutex, portMAX_DELAY);
                bool fresh = replay_check_locked(h->src, h->seq);
                xSemaphoreGive(g_mutex);
                if(!fresh){
                    xSemaphoreTake(g_mutex, portMAX_DELAY);
                    sec_replay_drop++;
                    xSemaphoreGive(g_mutex);
                    return;
                }

                size_t ciph_len = (size_t)h->payload_len - SEC_TAG_LEN;
                const uint8_t *pl  = buf + sizeof(mr_hdr_v7_t);
                const uint8_t *tag = pl + ciph_len;

                uint8_t plain[MAX_PLAINTEXT+1];
                memset(plain,0,sizeof(plain));

                if(!sec_decrypt_payload(h, pl, ciph_len, tag, plain)){
                    xSemaphoreTake(g_mutex, portMAX_DELAY);
                    sec_mac_fail++; sec_decrypt_fail++;
                    xSemaphoreGive(g_mutex);
                    return;
                }

                xSemaphoreTake(g_mutex, portMAX_DELAY);
                replay_update_locked(h->src, h->seq);
                sec_decrypt_ok++;
                xSemaphoreGive(g_mutex);

                size_t copy = (ciph_len < MAX_PAYLOAD) ? ciph_len : MAX_PAYLOAD;
                memcpy(txt, plain, copy);
                txt[copy]=0;
            }else{
                if(h->payload_len>0 && h->payload_len<=MAX_PAYLOAD){
                    memcpy(txt, buf+sizeof(mr_hdr_v7_t), h->payload_len);
                    txt[h->payload_len]=0;
                }
            }

            char from_str[8];
            call7_to_str(from_str, h->src);
            xSemaphoreTake(g_mutex, portMAX_DELAY);
            strncpy(g_last_rx_from, from_str, sizeof(g_last_rx_from)-1);
            g_last_rx_from[sizeof(g_last_rx_from)-1]=0;
            strncpy(g_last_rx_text, txt, sizeof(g_last_rx_text)-1);
            g_last_rx_text[sizeof(g_last_rx_text)-1]=0;
            g_last_rx_ms = now_ms();
            xSemaphoreGive(g_mutex);

            ESP_LOGI(TAG,"DATA delivered ✅ from=%s rssi=%d \"%s\"", from_str, rssi, txt);

            app_handle_cmd_if_any(h->src, txt);

            if((h->flags & MR_FLAG_ACKREQ) != 0) send_ack(h);
        }
    }
}


// ============================================================================
// ============================ DIO ISR + Task ================================
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
        if(xQueueReceive(dio_q,&io,portMAX_DELAY)){
            handle_rx();
        }
    }
}

static void init_dio_irq(void)
{
    dio_q=xQueueCreate(10,sizeof(uint32_t));

#if defined(MR_LORA_CHIP_SX1276)
    const gpio_num_t pin_irq = (gpio_num_t)PIN_LORA_DIO0;
#elif defined(MR_LORA_CHIP_SX1262)
    const gpio_num_t pin_irq = (gpio_num_t)PIN_LORA_DIO1;
#else
#error "No LoRa chip defined"
#endif

    gpio_config_t io={
        .intr_type=GPIO_INTR_POSEDGE,
        .mode=GPIO_MODE_INPUT,
        .pin_bit_mask=(1ULL<<pin_irq),
#if defined(MR_LORA_CHIP_SX1262)
        .pull_up_en=0, // Heltec usually has external pullups; keep input clean
#else
        .pull_up_en=1
#endif
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(pin_irq, dio_isr, (void*)pin_irq));

    xTaskCreate(dio_task, "dio", 4096, NULL, 10, NULL);
}


// ============================================================================
// =============================== Retry Task =================================
// ============================================================================
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

            uint32_t bo = (esp_random() % (ACK_BACKOFF_MS+1));
            xSemaphoreGive(g_mutex);
            vTaskDelay(pdMS_TO_TICKS(bo));
            xSemaphoreTake(g_mutex, portMAX_DELAY);

            mr_hdr_v7_t *h = (mr_hdr_v7_t*)pend[i].frame;
            if(pend[i].frame_len >= sizeof(mr_hdr_v7_t) && h->ttl > 1) h->ttl--;

            neighbor_tx_attempt_locked(pend[i].expect_from);

            if(bucket_take(&b_data)){
                c_tx_data++;
                lora_send_frame(pend[i].frame, pend[i].frame_len);
            }else{
                c_defer_data++;
            }
        }

        xSemaphoreGive(g_mutex);
    }
}


// ============================================================================
// =============================== CLI (UART0) ================================
// ============================================================================
#if MR_CLI_ENABLE
static void cli_print_help(void)
{
    printf("\n--- MeshRadio CLI ---\n");
    printf("help (h)\n");
    printf("status?\n");
    printf("wifi on|off\n");
    printf("role 0|1|2       (0=RELAY 1=EDGE 2=SENSOR)\n");
    printf("crypto on|off\n");
#if MR_RELAY_ENABLE
    printf("relay on|off|toggle\n");
#endif
    printf("send <DST> <ACK 0|1> <TEXT...>\n");
    printf("cmd  <DST> <ACK 0|1> STATUS|RELAYON|RELAYOFF|RELAYTOG\n");
    printf("---------------------\n\n");
}

static void trim_line(char *s)
{
    size_t L=strlen(s);
    while(L>0 && (s[L-1]=='\r'||s[L-1]=='\n'||s[L-1]==' '||s[L-1]=='\t')) s[--L]=0;
    size_t i=0;
    while(s[i]==' '||s[i]=='\t') i++;
    if(i>0) memmove(s, s+i, strlen(s+i)+1);
}

static void cli_sanitize_line(char *s)
{
    char out[MR_CLI_LINE_MAX];
    size_t oi=0;

    for(size_t i=0; s[i] && oi < sizeof(out)-1; ){
        unsigned char c = (unsigned char)s[i];

        if(c == 0x1B){
            i++;
            if(s[i] == '['){
                i++;
                while(s[i]){
                    unsigned char x = (unsigned char)s[i++];
                    if(x >= 0x40 && x <= 0x7E) break;
                }
                continue;
            }else if(s[i] == ']'){
                i++;
                while(s[i]){
                    if((unsigned char)s[i] == 0x07){ i++; break; }
                    if((unsigned char)s[i] == 0x1B && s[i+1] == '\\'){ i+=2; break; }
                    i++;
                }
                continue;
            }else{
                if(s[i]) i++;
                continue;
            }
        }

        if(c == 0x08 || c == 0x7F){
            if(oi>0) oi--;
            i++;
            continue;
        }

        if(c >= 32 && c <= 126){
            out[oi++] = (char)c;
        }
        i++;
    }

    out[oi]=0;
    strcpy(s, out);
    trim_line(s);

    while(s[0] == '>'){
        memmove(s, s+1, strlen(s+1)+1);
        trim_line(s);
    }
}

static bool streq_ci(const char *a, const char *b)
{
    while(*a && *b){
        if(tolower((unsigned char)*a) != tolower((unsigned char)*b)) return false;
        a++; b++;
    }
    return (*a==0 && *b==0);
}

static void cli_handle_line(char *line)
{
    cli_sanitize_line(line);
    if(line[0]==0) return;

    char *save=NULL;
    char *cmd=strtok_r(line, " \t", &save);
    if(!cmd) return;

    if(streq_ci(cmd,"h") || streq_ci(cmd,"help")){
        cli_print_help();
        return;
    }

    if(streq_ci(cmd,"status?")){
        uint32_t batt_mv=0, batt_pct=0;
#if MR_BATT_ENABLE
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        batt_mv = g_batt_mv;
        batt_pct = g_batt_pct;
        xSemaphoreGive(g_mutex);
#endif
        printf("local: mode=%s wifi=%s http=%s crypto=%s batt=%" PRIu32 "mV/%" PRIu32 "%%\n",
               node_mode_str(g_node_mode),
               g_wifi_enabled?"ON":"OFF",
               g_http_running?"ON":"OFF",
               g_crypto_enable?"ON":"OFF",
               batt_mv, batt_pct);
#if MR_RELAY_ENABLE
        printf("local: relay=%s\n", g_relay_on?"ON":"OFF");
#endif
        return;
    }

    if(streq_ci(cmd,"wifi")){
        char *arg=strtok_r(NULL, " \t", &save);
        if(arg && streq_ci(arg,"on")){ set_wifi_enabled(true); printf("OK wifi=ON\n"); return; }
        if(arg && streq_ci(arg,"off")){ set_wifi_enabled(false); printf("OK wifi=OFF\n"); return; }
        printf("ERR wifi on|off\n");
        return;
    }

    if(streq_ci(cmd,"role")){
        char *arg=strtok_r(NULL, " \t", &save);
        if(!arg){ printf("ERR role 0|1|2\n"); return; }
        int m=atoi(arg);
        if(m<0||m>2){ printf("ERR role 0|1|2\n"); return; }

        xSemaphoreTake(g_mutex, portMAX_DELAY);
        g_node_mode=(node_mode_t)m;
        xSemaphoreGive(g_mutex);

        printf("OK role=%s\n", node_mode_str(g_node_mode));
        return;
    }

    if(streq_ci(cmd,"crypto")){
        char *arg=strtok_r(NULL, " \t", &save);
        if(!arg){ printf("ERR crypto on|off\n"); return; }
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        if(streq_ci(arg,"on")) g_crypto_enable=true;
        else if(streq_ci(arg,"off")) g_crypto_enable=false;
        else { xSemaphoreGive(g_mutex); printf("ERR crypto on|off\n"); return; }
        bool st=g_crypto_enable;
        xSemaphoreGive(g_mutex);
        printf("OK crypto=%s\n", st?"ON":"OFF");
        return;
    }

#if MR_RELAY_ENABLE
    if(streq_ci(cmd,"relay")){
        if(g_node_mode != NODE_RELAY){ printf("ERR relay only valid in RELAY role\n"); return; }
        char *arg=strtok_r(NULL, " \t", &save);
        if(!arg){ printf("ERR relay on|off|toggle\n"); return; }

        xSemaphoreTake(g_mutex, portMAX_DELAY);
        if(streq_ci(arg,"on")) relay_set(true);
        else if(streq_ci(arg,"off")) relay_set(false);
        else if(streq_ci(arg,"toggle")) relay_toggle();
        else { xSemaphoreGive(g_mutex); printf("ERR relay on|off|toggle\n"); return; }
        bool st=g_relay_on;
        xSemaphoreGive(g_mutex);

        printf("OK relay=%s\n", st?"ON":"OFF");
        return;
    }
#endif

    if(streq_ci(cmd,"send")){
        char *dst=strtok_r(NULL, " \t", &save);
        char *ack=strtok_r(NULL, " \t", &save);
        char *msg=save;
        if(!dst || !ack || !msg){ printf("ERR send <DST> <ACK 0|1> <TEXT...>\n"); return; }
        while(*msg==' '||*msg=='\t') msg++;
        if(!*msg){ printf("ERR missing text\n"); return; }
        int a=atoi(ack);
        send_data_to(dst, msg, (a!=0));
        printf("OK sent\n");
        return;
    }

    if(streq_ci(cmd,"cmd")){
        char *dst=strtok_r(NULL, " \t", &save);
        char *ack=strtok_r(NULL, " \t", &save);
        char *what=strtok_r(NULL, " \t", &save);
        if(!dst || !ack || !what){ printf("ERR cmd <DST> <ACK> STATUS|RELAYON|RELAYOFF|RELAYTOG\n"); return; }
        int a=atoi(ack);

        if(streq_ci(what,"STATUS")) send_data_to(dst, "CMD:STATUS?", (a!=0));
        else if(streq_ci(what,"RELAYON")) send_data_to(dst, "CMD:RELAY ON", (a!=0));
        else if(streq_ci(what,"RELAYOFF")) send_data_to(dst, "CMD:RELAY OFF", (a!=0));
        else if(streq_ci(what,"RELAYTOG")) send_data_to(dst, "CMD:RELAY TOGGLE", (a!=0));
        else { printf("ERR unknown cmd\n"); return; }

        printf("OK cmd sent\n");
        return;
    }

    printf("ERR unknown. type: help\n");
}

static void cli_uart_init_once(void)
{
    static bool inited=false;
    if(inited) return;

    const int rx_buf = 1024;
    const int tx_buf = 0;

    esp_err_t e = uart_driver_install(UART_NUM_0, rx_buf, tx_buf, 0, NULL, 0);
    if(e != ESP_OK && e != ESP_ERR_INVALID_STATE){
        ESP_LOGW(TAG, "uart_driver_install: %s", esp_err_to_name(e));
    }
    uart_set_rx_timeout(UART_NUM_0, 1);
    inited=true;
}

static void cli_task(void *arg)
{
    (void)arg;
    cli_uart_init_once();
    cli_print_help();

    printf("> "); fflush(stdout);

    char line[MR_CLI_LINE_MAX];
    int n=0;

    while(1){
        uint8_t c=0;
        int r = uart_read_bytes(UART_NUM_0, &c, 1, pdMS_TO_TICKS(50));
        if(r <= 0) continue;

        if(c == '\r' || c == '\n'){
            line[n]=0;
            printf("\n"); fflush(stdout);
            if(n>0) cli_handle_line(line);
            n=0;
            printf("> "); fflush(stdout);
            continue;
        }

        if(c == 0x08 || c == 0x7F){
            if(n>0){
                n--;
                printf("\b \b");
                fflush(stdout);
            }
            continue;
        }

        if(isprint((int)c)){
            if(n < (MR_CLI_LINE_MAX-1)){
                line[n++] = (char)c;
                putchar((int)c);
                fflush(stdout);
            }
        }
    }
}
#endif // MR_CLI_ENABLE

// ============================================================================
// ================================= MAIN ====================================
// ============================================================================
void app_main(void)
{
    ESP_LOGI(TAG,"MeshRadio start (ESP-IDF 5.5.2) – SX1262/SX1276 switchable @ %lu Hz", (unsigned long)DEFAULT_RF_FREQ_HZ);

#if (MR_BOARD_PRESET == MR_BOARD_HELTEC_V3)
    // ✅ 1) VEXT zuerst einschalten (sonst ist SX1262/RF-Switch/TCXO oft tot)
    // heltec_vext_on();
    board_power_boot_init();   // ✅ Heltec: VEXT an, Batt enable pin init
#endif

    g_mutex=xSemaphoreCreateMutex();
    if(!g_mutex){ ESP_LOGE(TAG,"mutex failed"); abort(); }

    g_lora_spi_mutex = xSemaphoreCreateMutex();
    if(!g_lora_spi_mutex){ ESP_LOGE(TAG,"lora spi mutex failed"); abort(); }

    ESP_ERROR_CHECK(nvs_flash_init());

    bucket_init(&b_beacon,   RL_BEACON_TPS,   RL_BEACON_BURST);
    bucket_init(&b_routeadv, RL_ROUTEADV_TPS, RL_ROUTEADV_BURST);
    bucket_init(&b_ack,      RL_ACK_TPS,      RL_ACK_BURST);
    bucket_init(&b_data,     RL_DATA_TPS,     RL_DATA_BURST);

    if(!parse_key_hex16(MR_NET_KEY_HEX, g_net_key)){
        ESP_LOGE(TAG,"Invalid MR_NET_KEY_HEX – need 32 hex chars");
        abort();
    }

#if MR_RELAY_ENABLE
    relay_init();
#endif

#if MR_BATT_ENABLE
    batt_init();
#endif

    init_spi();
    lora_hw_reset();
    lora_log_chip_info();
    lora_init_radio();

    init_dio_irq();

    xTaskCreate(retry_task, "retry", 4096, NULL, 6, NULL);

#if MR_CLI_ENABLE
    xTaskCreate(cli_task, "cli", 4096, NULL, 5, NULL);
#endif

    if(g_wifi_enabled){
        wifi_start_ap();
        http_start_if_needed();
        ESP_LOGI(TAG,"Open http://192.168.4.1");
    }else{
        ESP_LOGI(TAG,"WiFi/HTTP disabled at boot. Use CLI: wifi on");
    }

    beacon_schedule_next();
    routeadv_schedule_next();

    ESP_LOGI(TAG,"CALL=%s  mode=%s  crypto=%u  net_id=0x%02X  wifi=%u",
             MR_CALLSIGN, node_mode_str(g_node_mode), g_crypto_enable?1:0,
             (unsigned)MR_NET_ID, g_wifi_enabled?1:0);

    while(1){
        vTaskDelay(pdMS_TO_TICKS(50));

#if MR_BATT_ENABLE
        batt_poll();
#endif

        if(g_beacon_enabled && g_node_mode != NODE_SENSOR && now_ms() > g_next_beacon_ms){
            send_beacon();
        }

        xSemaphoreTake(g_mutex, portMAX_DELAY);
        neighbor_cleanup_locked();
        route_cleanup_locked();
        xSemaphoreGive(g_mutex);
    }
}
