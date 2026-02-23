// ============================================================================
// MeshRadio – Kapitel 25D (Store-and-Forward) – KOMPLETTE SOURCEN
// ----------------------------------------------------------------------------
// Baut auf Kapitel 25 (Destination Routing) + 25C (ACK/Retry) auf und erweitert:
//
// Kapitel 25D Features:
//   - Store-and-Forward (RAM Queue):
//       * Nachrichten werden lokal gepuffert, wenn Ziel/Route temporär fehlt
//       * Nachrichten werden später automatisch erneut übertragen
//       * automatische Ablaufzeit (Expiry)
//       * Speicherbegrenzung + Cleanup
//   - ACK + Retry ist jetzt "echt":
//       * DATA wird für Retries gepuffert (Store-Entry enthält komplettes Paket)
//       * bei ACK OK -> Store-Entry wird gelöscht
//       * bei ACK Timeout -> Retry, danach ggf. im Store behalten (weiter versuchen)
//   - Web UI + API:
//       * /api/send        (dst,msg,ack=0/1,exp=sec optional)
//       * /api/routes      (dst->next rssi age)
//       * /api/store       (stored messages anzeigen)
//       * /api/bcfallback  (enable=0/1)
//
// WICHTIG (User-Wunsch):
//   - Rufzeichen und WLAN-Server als #define am Anfang
//
// Hinweis:
//   - Persistenz (Flash) ist bewusst NICHT implementiert (Kapitel 25D RAM-Version).
// ============================================================================


// ============================================================================
// USER DEFINES (ANFANG – WICHTIG)
// ============================================================================
#define MR_CALLSIGN        "DJ2RF"            // <<< Rufzeichen hier ändern (max 7)
#define MR_WIFI_AP_SSID    "MeshRadio-Setup"   // <<< WLAN AP Name
#define MR_WIFI_AP_PASS    ""                  // leer = offen

// ACK/Retry Parameter
#define ACK_TIMEOUT_MS     1200                // warten auf ACK
#define ACK_RETRY_MAX      3                   // max Retries pro Send-Versuch
#define ACK_BACKOFF_MS     350                 // random backoff 0..X ms

// Store-and-Forward Parameter
#define STORE_MAX_MSG      10                  // max gespeicherte Nachrichten (RAM)
#define STORE_DEFAULT_EXP_SEC 600              // default expiry: 10 Minuten
#define STORE_RETRY_INTERVAL_MS 5000           // wie oft gespeicherte Nachrichten "gecheckt" werden

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
#define MR_PROTO_VERSION 3

#define MR_FLAG_DATA      0x10
#define MR_FLAG_BEACON    0x20
#define MR_FLAG_ACK       0x40

// DATA Zusatzflag
#define MR_FLAG_ACKREQ    0x01

#define DATA_TTL   4
#define BEACON_TTL 2
#define ACK_TTL    4

#define MAX_PAYLOAD 120

// ============================================================================
// LIMITS
// ============================================================================
#define SEEN_CACHE_SIZE 48

#define MAX_NEIGHBORS 24
#define NEIGHBOR_TIMEOUT_MS 60000

#define MAX_ROUTES 32
#define ROUTE_TIMEOUT_MS 180000

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
// HEADER
// ============================================================================
#pragma pack(push,1)
typedef struct {
    uint8_t magic[2];
    uint8_t version;
    uint8_t flags;

    uint8_t ttl;
    uint16_t msg_id;

    char src[7];
    char final_dst[7];
    char next_hop[7];
    char last_hop[7];

    uint8_t payload_len;
} mr_hdr_v3_t;
#pragma pack(pop)

// ============================================================================
// STRUCTS
// ============================================================================
typedef struct { bool used; char src[7]; uint16_t id; } seen_t;

typedef struct { bool used; char call[7]; int rssi; uint32_t t_ms; } neighbor_t;

typedef struct {
    bool used;
    char dst[7];
    char next[7];
    int rssi;
    uint32_t t_ms;
} route_t;

// Kapitel 25D: Store Entry = komplette Nachricht (Header+Payload) + Metadaten
typedef struct {
    bool used;

    // Ziel dieser Nachricht
    char final_dst[7];

    // kompletter Frame (mr_hdr_v3_t + payload)
    uint8_t frame[sizeof(mr_hdr_v3_t) + MAX_PAYLOAD];
    uint16_t frame_len;

    // Zustandsdaten für Store-and-Forward
    uint32_t created_ms;
    uint32_t expiry_ms;
    uint32_t last_try_ms;

    // ACK/Retry Zustand
    bool ackreq;
    bool awaiting_ack;
    char expect_ack_from[7];
    uint32_t ack_deadline_ms;
    uint8_t retries;
} store_msg_t;

// ============================================================================
// GLOBALS
// ============================================================================
static const char *TAG="MR25D";

static spi_device_handle_t lora_spi;
static QueueHandle_t dio_q;
static SemaphoreHandle_t g_mutex;
static httpd_handle_t g_http=NULL;

static uint16_t g_msg_id=1;

static seen_t seen_cache[SEEN_CACHE_SIZE];
static neighbor_t neighbors[MAX_NEIGHBORS];
static route_t routes[MAX_ROUTES];

static store_msg_t store_q[STORE_MAX_MSG];

// Kapitel 25: Fallback wenn keine Route
static bool g_bc_fallback=true;

// ============================================================================
// Helpers
// ============================================================================
static uint32_t now_ms(void){ return xTaskGetTickCount()*portTICK_PERIOD_MS; }

static void call7_set(char o[7], const char *s)
{
    memset(o,' ',7);
    size_t n=strlen(s); if(n>7) n=7;
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
    return (a[0]=='*' && a[1]==' ' && a[2]==' ' && a[3]==' ' && a[4]==' ' && a[5]==' ' && a[6]==' ');
}

// ============================================================================
// SEEN
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
// NEIGHBORS
// ============================================================================
static void neighbor_update_locked(const char call[7], int rssi)
{
    uint32_t t=now_ms();
    for(int i=0;i<MAX_NEIGHBORS;i++){
        if(!neighbors[i].used) continue;
        if(call7_eq(neighbors[i].call,call)){
            neighbors[i].rssi=rssi;
            neighbors[i].t_ms=t;
            return;
        }
    }
    for(int i=0;i<MAX_NEIGHBORS;i++){
        if(!neighbors[i].used){
            neighbors[i].used=true;
            memcpy(neighbors[i].call,call,7);
            neighbors[i].rssi=rssi;
            neighbors[i].t_ms=t;
            return;
        }
    }
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
// ROUTES (Kapitel 25 Basis)
// ============================================================================
static void route_update_locked(const char dst[7], const char next[7], int rssi)
{
    uint32_t t=now_ms();

    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used) continue;
        if(!call7_eq(routes[i].dst,dst)) continue;

        // simple: replace if better RSSI or expired
        bool expired = (t - routes[i].t_ms) > ROUTE_TIMEOUT_MS;
        bool better  = (rssi > routes[i].rssi);
        if(expired || better){
            memcpy(routes[i].next,next,7);
            routes[i].rssi=rssi;
        }
        routes[i].t_ms=t;
        return;
    }

    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used){
            routes[i].used=true;
            memcpy(routes[i].dst,dst,7);
            memcpy(routes[i].next,next,7);
            routes[i].rssi=rssi;
            routes[i].t_ms=t;
            return;
        }
    }

    // overwrite oldest
    int oldest=0;
    uint32_t ot=routes[0].t_ms;
    for(int i=1;i<MAX_ROUTES;i++){
        if(!routes[i].used){ oldest=i; break; }
        if(routes[i].t_ms < ot){ oldest=i; ot=routes[i].t_ms; }
    }
    routes[oldest].used=true;
    memcpy(routes[oldest].dst,dst,7);
    memcpy(routes[oldest].next,next,7);
    routes[oldest].rssi=rssi;
    routes[oldest].t_ms=t;
}

static bool route_lookup_locked(const char dst[7], char out_next[7])
{
    uint32_t t=now_ms();
    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used) continue;
        if(!call7_eq(routes[i].dst,dst)) continue;
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
// Apply Config (wie Kapitel 25, hier: kompakt mit Default 433.775 / SF7 / BW125)
// ============================================================================
static uint32_t hz_to_frf(uint32_t f)
{
    uint64_t frf=((uint64_t)f<<19)/32000000ULL;
    return (uint32_t)frf;
}

static void lora_apply_locked(void)
{
    // Standby
    lora_wr(REG_OP_MODE,0x80);
    vTaskDelay(pdMS_TO_TICKS(10));

    uint32_t frf=hz_to_frf(433775000);
    lora_wr(REG_FRF_MSB,frf>>16);
    lora_wr(REG_FRF_MID,frf>>8);
    lora_wr(REG_FRF_LSB,frf);

    // BW125 + CR4/5
    lora_wr(REG_MODEM_CONFIG_1, 0x72);
    // SF7 + CRC on + RX payload CRC / implicit off
    lora_wr(REG_MODEM_CONFIG_2, (7<<4) | (1<<2) | 0x03);
    // AGC on
    lora_wr(REG_MODEM_CONFIG_3, 0x04);

    lora_wr(REG_PA_CONFIG,0x8E);

    lora_wr(REG_FIFO_RX_BASE_ADDR,0);
    lora_wr(REG_FIFO_ADDR_PTR,0);
    lora_wr(REG_OP_MODE,0x85); // RX cont
}

// ============================================================================
// Store-and-Forward Helpers
// ============================================================================

// Sucht freien Slot, sonst überschreibt ältesten/abgelaufenen
static int store_alloc_slot_locked(void)
{
    uint32_t t=now_ms();

    // 1) freier Slot
    for(int i=0;i<STORE_MAX_MSG;i++){
        if(!store_q[i].used) return i;
    }

    // 2) abgelaufenen Slot recyceln
    for(int i=0;i<STORE_MAX_MSG;i++){
        if(store_q[i].used && t >= store_q[i].expiry_ms){
            store_q[i].used=false;
            return i;
        }
    }

    // 3) ältesten überschreiben (FIFO-ish)
    int oldest=0;
    uint32_t ot=store_q[0].created_ms;
    for(int i=1;i<STORE_MAX_MSG;i++){
        if(store_q[i].created_ms < ot){
            oldest=i; ot=store_q[i].created_ms;
        }
    }
    store_q[oldest].used=false;
    return oldest;
}

static void store_delete_locked(int idx)
{
    if(idx < 0 || idx >= STORE_MAX_MSG) return;
    store_q[idx].used=false;
}

static int store_find_by_msgid_locked(uint16_t msg_id, const char expect_from[7])
{
    for(int i=0;i<STORE_MAX_MSG;i++){
        if(!store_q[i].used) continue;
        mr_hdr_v3_t *h=(mr_hdr_v3_t*)store_q[i].frame;
        if(h->msg_id != msg_id) continue;
        if(!store_q[i].awaiting_ack) continue;
        if(!call7_eq(store_q[i].expect_ack_from, expect_from)) continue;
        return i;
    }
    return -1;
}

// Route/Next-Hop für gespeicherte Nachricht aktualisieren.
// - setzt last_hop = ME
// - berechnet next_hop aus Route Table oder setzt "*" / "NONE"
static void store_prepare_for_send_locked(store_msg_t *m)
{
    mr_hdr_v3_t *h=(mr_hdr_v3_t*)m->frame;

    // Reset TTL für neuen Versuch (damit es nicht sofort "verhungert")
    h->ttl = DATA_TTL;

    // last_hop = ME (wir sind jetzt der Sender dieses Hops)
    call7_set(h->last_hop, MR_CALLSIGN);

    // Next hop anhand Routing
    char nh[7];
    bool has = route_lookup_locked(h->final_dst, nh);

    if(has){
        memcpy(h->next_hop, nh, 7);
    }else{
        // Bei Store-and-Forward ist Broadcast oft kontraproduktiv.
        // Aber: Wir respektieren g_bc_fallback, weil es im Buch ein Schalter ist.
        call7_set(h->next_hop, g_bc_fallback ? "*" : "NONE");
    }
}

// Sendet Store-Nachricht, startet ggf. ACK-Wait
static void store_send_locked(int idx)
{
    store_msg_t *m=&store_q[idx];
    if(!m->used) return;

    uint32_t t=now_ms();
    m->last_try_ms=t;

    // Route berechnen/aktualisieren
    store_prepare_for_send_locked(m);

    mr_hdr_v3_t *h=(mr_hdr_v3_t*)m->frame;

    // Wenn next_hop == "NONE" => nicht senden, im Store bleiben
    if(h->next_hop[0]=='N' && h->next_hop[1]=='O'){
        char d[8]; call7_to_str(d, h->final_dst);
        ESP_LOGI(TAG,"STORE hold (no route) msg=%u dst=%s", h->msg_id, d);
        return;
    }

    char dsts[8], nhs[8];
    call7_to_str(dsts, h->final_dst);
    call7_to_str(nhs, h->next_hop);

    ESP_LOGI(TAG,"STORE TX msg=%u dst=%s next=%s ack=%u try=%u",
             h->msg_id, dsts, nhs, m->ackreq?1:0, (unsigned)m->retries);

    lora_send(m->frame, m->frame_len);

    if(m->ackreq && !call7_is_wild(h->next_hop)){
        m->awaiting_ack=true;
        memcpy(m->expect_ack_from, h->next_hop, 7);
        m->ack_deadline_ms = t + ACK_TIMEOUT_MS;
    }else{
        // kein ACK angefordert -> wir betrachten "gesendet" als erledigt
        // (Store-and-Forward ohne ACK macht wenig Sinn, aber fürs Buch okay)
        store_delete_locked(idx);
    }
}

// ============================================================================
// ACK SEND (Hop-by-Hop)
// ============================================================================
static void send_ack(const mr_hdr_v3_t *rx)
{
    uint8_t buf[sizeof(mr_hdr_v3_t)]={0};
    mr_hdr_v3_t *h=(mr_hdr_v3_t*)buf;

    h->magic[0]='M'; h->magic[1]='R';
    h->version=MR_PROTO_VERSION;
    h->flags=MR_FLAG_ACK;
    h->ttl=ACK_TTL;

    // ACK bestätigt DATA msg_id
    h->msg_id = rx->msg_id;

    call7_set(h->src, MR_CALLSIGN);
    call7_set(h->last_hop, MR_CALLSIGN);

    // Hop-by-Hop: ACK geht an den vorherigen Hop (last_hop aus dem RX-DATA)
    memcpy(h->final_dst, rx->last_hop, 7);

    // Next-Hop wie DATA: route_lookup auf final_dst, sonst Broadcast
    char nh[7];
    xSemaphoreTake(g_mutex,portMAX_DELAY);
    bool has = route_lookup_locked(h->final_dst, nh);
    xSemaphoreGive(g_mutex);

    if(has) memcpy(h->next_hop, nh, 7);
    else call7_set(h->next_hop, "*");

    call7_set(h->final_dst, (char[8]){0}); // (nur um Analyzer zu beruhigen)
    memcpy(h->final_dst, rx->last_hop, 7);

    h->payload_len=0;

    char d[8]; call7_to_str(d, h->final_dst);
    ESP_LOGI(TAG,"TX ACK msg=%u to=%s", h->msg_id, d);

    lora_send((uint8_t*)h, sizeof(*h));
}

// ============================================================================
// CREATE STORE MESSAGE FROM USER SEND
// ----------------------------------------------------------------------------
// Wir legen immer einen Store-Entry an, wenn:
//   - ackreq==true  (damit Retries möglich sind)
//   - oder keine Route vorhanden (Store-and-Forward)
// ============================================================================
static void enqueue_store_message(const char *dst_str, const char *txt,
                                  bool ackreq, uint32_t exp_sec)
{
    char dst7[7]; call7_set(dst7, dst_str);

    uint32_t t=now_ms();
    uint32_t exp_ms = (exp_sec==0 ? (STORE_DEFAULT_EXP_SEC*1000UL) : (exp_sec*1000UL));
    uint32_t expiry = t + exp_ms;

    xSemaphoreTake(g_mutex,portMAX_DELAY);
    int idx = store_alloc_slot_locked();
    store_msg_t *m=&store_q[idx];

    memset(m,0,sizeof(*m));
    m->used=true;
    memcpy(m->final_dst, dst7, 7);
    m->created_ms=t;
    m->expiry_ms=expiry;
    m->last_try_ms=0;
    m->ackreq=ackreq;
    m->awaiting_ack=false;
    m->retries=0;

    // Frame bauen
    mr_hdr_v3_t *h=(mr_hdr_v3_t*)m->frame;
    memset(h,0,sizeof(*h));

    h->magic[0]='M'; h->magic[1]='R';
    h->version=MR_PROTO_VERSION;
    h->flags=MR_FLAG_DATA | (ackreq?MR_FLAG_ACKREQ:0);
    h->ttl=DATA_TTL;
    h->msg_id=g_msg_id++;

    call7_set(h->src, MR_CALLSIGN);
    call7_set(h->last_hop, MR_CALLSIGN);

    memcpy(h->final_dst, dst7, 7);

    // payload
    size_t n=strlen(txt);
    if(n>MAX_PAYLOAD) n=MAX_PAYLOAD;
    h->payload_len=(uint8_t)n;
    memcpy(m->frame + sizeof(mr_hdr_v3_t), txt, n);

    m->frame_len = (uint16_t)(sizeof(mr_hdr_v3_t) + n);

    // Direkt versuchen zu senden (wenn möglich)
    // store_send_locked() berechnet next_hop & startet ACK-Wait.
    store_send_locked(idx);

    xSemaphoreGive(g_mutex);
}

// ============================================================================
// STORE TASK (25D Kern)
// ----------------------------------------------------------------------------
// - kümmert sich um:
//     * Expiry
//     * ACK-Timeout -> Retry (mit Backoff)
//     * "später erneut übertragen", wenn zuvor keine Route vorhanden war
// ============================================================================
static void store_task(void *arg)
{
    (void)arg;

    uint32_t next_scan = 0;

    while(1){
        vTaskDelay(pdMS_TO_TICKS(100));

        uint32_t t=now_ms();
        if(t < next_scan) continue;
        next_scan = t + STORE_RETRY_INTERVAL_MS;

        xSemaphoreTake(g_mutex,portMAX_DELAY);

        for(int i=0;i<STORE_MAX_MSG;i++){
            if(!store_q[i].used) continue;

            store_msg_t *m=&store_q[i];
            mr_hdr_v3_t *h=(mr_hdr_v3_t*)m->frame;

            // 1) Expiry
            if(t >= m->expiry_ms){
                char d[8]; call7_to_str(d, h->final_dst);
                ESP_LOGW(TAG,"STORE expire msg=%u dst=%s", h->msg_id, d);
                store_delete_locked(i);
                continue;
            }

            // 2) Wenn auf ACK warten
            if(m->awaiting_ack){
                if(t < m->ack_deadline_ms) continue;

                // ACK Timeout -> Retry oder "weiter speichern"
                if(m->retries >= ACK_RETRY_MAX){
                    char d[8], e[8];
                    call7_to_str(d, h->final_dst);
                    call7_to_str(e, m->expect_ack_from);
                    ESP_LOGW(TAG,"ACK FAIL msg=%u dst=%s last_expect=%s -> keep stored",
                             h->msg_id, d, e);

                    // Wir behalten die Nachricht im Store, aber stoppen den ACK-Wait.
                    // Dadurch kann sie später bei besserer Route erneut versucht werden.
                    m->awaiting_ack=false;
                    m->retries=0;
                    continue;
                }

                // Retry
                m->retries++;
                m->awaiting_ack=false;

                // Backoff gegen gleichzeitige Retries
                uint32_t bo = (esp_random() % (ACK_BACKOFF_MS+1));
                xSemaphoreGive(g_mutex);
                vTaskDelay(pdMS_TO_TICKS(bo));
                xSemaphoreTake(g_mutex,portMAX_DELAY);

                store_send_locked(i);
                continue;
            }

            // 3) Nicht awaiting_ack:
            //    -> "später erneut übertragen" wenn vorher keine Route / keine ACK-Phase.
            //    Wir versuchen periodisch (STORE_RETRY_INTERVAL_MS), aber nur wenn genug Zeit.
            if(t - m->last_try_ms < STORE_RETRY_INTERVAL_MS) continue;

            // Versuchen zu senden (wenn Route vorhanden)
            store_send_locked(i);
        }

        xSemaphoreGive(g_mutex);
    }
}

// ============================================================================
// FORWARD (Kapitel 25, mit ACK-Option)
// ----------------------------------------------------------------------------
// Für 25D wichtig: wenn wir forwarden und ACKREQ gesetzt ist,
// speichern wir das Paket ebenfalls kurz, damit Retries möglich sind.
// ============================================================================
static void forward_data(uint8_t *buf, size_t len)
{
    if(len < sizeof(mr_hdr_v3_t)) return;

    mr_hdr_v3_t *h=(mr_hdr_v3_t*)buf;
    if(h->ttl <= 1) return;

    // TTL runter
    h->ttl--;

    // last_hop = ME
    call7_set(h->last_hop, MR_CALLSIGN);

    // next_hop neu berechnen
    xSemaphoreTake(g_mutex,portMAX_DELAY);
    char nh[7];
    bool has = route_lookup_locked(h->final_dst, nh);
    xSemaphoreGive(g_mutex);

    if(has) memcpy(h->next_hop, nh, 7);
    else call7_set(h->next_hop, g_bc_fallback ? "*" : "NONE");

    // Wenn NONE -> nicht senden
    if(h->next_hop[0]=='N' && h->next_hop[1]=='O') return;

    lora_send(buf, len);

    // Hop-by-Hop ACK/Retry: nur wenn next_hop nicht wildcard ist
    if((h->flags & MR_FLAG_ACKREQ) && !call7_is_wild(h->next_hop)){
        // In Store aufnehmen (für Retries), aber mit kurzer Expiry
        // (Forward-Cache ist kein Messenger, eher Zuverlässigkeit)
        uint32_t exp_sec = 60; // 1 Minute für forward retrys

        xSemaphoreTake(g_mutex,portMAX_DELAY);

        int idx = store_alloc_slot_locked();
        store_msg_t *m=&store_q[idx];
        memset(m,0,sizeof(*m));

        m->used=true;
        memcpy(m->final_dst, h->final_dst, 7);

        // kompletten Frame kopieren
        if(len > sizeof(m->frame)) len = sizeof(m->frame);
        memcpy(m->frame, buf, len);
        m->frame_len = (uint16_t)len;

        uint32_t t=now_ms();
        m->created_ms=t;
        m->expiry_ms=t + (exp_sec*1000UL);
        m->last_try_ms=t;

        m->ackreq=true;
        m->awaiting_ack=true;
        memcpy(m->expect_ack_from, h->next_hop, 7);
        m->ack_deadline_ms=t + ACK_TIMEOUT_MS;
        m->retries=0;

        xSemaphoreGive(g_mutex);
    }
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
    if(len < sizeof(mr_hdr_v3_t)) return;

    mr_hdr_v3_t *h=(mr_hdr_v3_t*)buf;
    if(h->magic[0]!='M'||h->magic[1]!='R') return;
    if(h->version!=MR_PROTO_VERSION) return;

    if(seen_before(h->src,h->msg_id)) return;
    remember_msg(h->src,h->msg_id);

    int rssi=(int)lora_rd(REG_PKT_RSSI_VALUE)-157;

    // Nachbarpflege
    xSemaphoreTake(g_mutex,portMAX_DELAY);
    neighbor_update_locked(h->last_hop, rssi);
    xSemaphoreGive(g_mutex);

    // Strings fürs Logging
    char srcs[8], dsts[8], nhs[8], lhs[8];
    call7_to_str(srcs, h->src);
    call7_to_str(dsts, h->final_dst);
    call7_to_str(nhs, h->next_hop);
    call7_to_str(lhs, h->last_hop);

    // ------------------------------------------------------------------------
    // BEACON
    // ------------------------------------------------------------------------
    if(h->flags & MR_FLAG_BEACON){
        ESP_LOGI(TAG,"RX BEACON src=%s via=%s rssi=%d", srcs, lhs, rssi);

        xSemaphoreTake(g_mutex,portMAX_DELAY);
        route_update_locked(h->src, h->last_hop, rssi);
        xSemaphoreGive(g_mutex);
        return;
    }

    // ------------------------------------------------------------------------
    // ACK
    // ------------------------------------------------------------------------
    if(h->flags & MR_FLAG_ACK){
        // Hop-by-Hop: ACK kommt vom next_hop
        xSemaphoreTake(g_mutex,portMAX_DELAY);
        int idx = store_find_by_msgid_locked(h->msg_id, h->src);
        if(idx >= 0){
            ESP_LOGI(TAG,"ACK OK msg=%u from=%s -> delete store", h->msg_id, srcs);
            store_delete_locked(idx);
        }
        xSemaphoreGive(g_mutex);
        return;
    }

    // ------------------------------------------------------------------------
    // DATA
    // ------------------------------------------------------------------------
    if(h->flags & MR_FLAG_DATA){
        char my7[7]; call7_set(my7, MR_CALLSIGN);

        bool iam_dst = call7_eq(h->final_dst, my7);
        bool iam_nexthop = call7_eq(h->next_hop, my7);
        bool nh_wild = call7_is_wild(h->next_hop);
        bool nh_none = (h->next_hop[0]=='N' && h->next_hop[1]=='O'); // "NONE..."

        char txt[MAX_PAYLOAD+1]={0};
        if(h->payload_len>0 && h->payload_len<=MAX_PAYLOAD){
            memcpy(txt, buf+sizeof(mr_hdr_v3_t), h->payload_len);
            txt[h->payload_len]=0;
        }

        ESP_LOGI(TAG,"RX DATA src=%s dst=%s next=%s via=%s rssi=%d ttl=%u ack=%u txt=\"%s\"",
                 srcs,dsts,nhs,lhs,rssi,h->ttl,(h->flags&MR_FLAG_ACKREQ)?1:0,txt);

        // Endziel: konsumieren + ggf. ACK zurück
        if(iam_dst){
            ESP_LOGI(TAG,"DATA delivered ✅ (dst=%s)", dsts);

            if(h->flags & MR_FLAG_ACKREQ){
                send_ack(h);
            }
            return;
        }

        if(nh_none){
            ESP_LOGI(TAG,"DROP: next_hop=NONE");
            return;
        }

        // Weiterleiten nur wenn wir dran sind oder Broadcast-Fallback aktiv ist
        if((iam_nexthop || nh_wild) && h->ttl>1){
            forward_data(buf, len);
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
// HTTP helpers (Form decoding – aus Kapitel 25, robust genug)
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

// FIXED: restores '=' also on match
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
// HTTP UI + API
// ============================================================================
static const char *INDEX_HTML =
"<!doctype html><html><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>MeshRadio Kapitel 25D</title>"
"<style>"
"body{font-family:system-ui;margin:16px;max-width:900px}"
"button{padding:10px 14px;margin:6px 4px;font-size:16px}"
"input{padding:10px;margin:6px 0;font-size:16px;width:100%}"
"code{background:#eee;padding:2px 4px}"
"</style></head><body>"
"<h2>MeshRadio Kapitel 25D – Store-and-Forward</h2>"
"<p>AP: <code>" MR_WIFI_AP_SSID "</code> URL: <code>http://192.168.4.1</code></p>"
"<div id='m'></div>"
"<h3>DATA senden</h3>"
"<label>Ziel (dst)</label><input id='dst' value='DJ1ABC'>"
"<label>Text</label><input id='msg' value='Hallooooo'>"
"<label>ACK (0/1)</label><input id='ack' value='1'>"
"<label>Expiry (sec, optional)</label><input id='exp' value='600'>"
"<button onclick='send()'>SEND</button>"
"<h3>Broadcast Fallback</h3>"
"<button onclick='setBC(1)'>BC fallback ON</button>"
"<button onclick='setBC(0)'>BC fallback OFF</button>"
"<p><button onclick='routes()'>Routes anzeigen</button> "
"<button onclick='store()'>Store anzeigen</button></p>"
"<pre id='st'></pre>"
"<script>"
"function m(t){document.getElementById('m').innerHTML='<b>'+t+'</b>'}"
"async function post(url,body){"
" let r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:body});"
" let t=await r.text(); m(t); }"
"async function send(){"
" let dst=document.getElementById('dst').value;"
" let msg=document.getElementById('msg').value;"
" let ack=document.getElementById('ack').value;"
" let exp=document.getElementById('exp').value;"
" await post('/api/send','dst='+encodeURIComponent(dst)+'&msg='+encodeURIComponent(msg)"
" +'&ack='+encodeURIComponent(ack)+'&exp='+encodeURIComponent(exp));"
"}"
"async function setBC(v){ await post('/api/bcfallback','enable='+encodeURIComponent(v)); }"
"async function routes(){ let r=await fetch('/api/routes'); document.getElementById('st').textContent=await r.text(); }"
"async function store(){ let r=await fetch('/api/store'); document.getElementById('st').textContent=await r.text(); }"
"routes();"
"</script></body></html>";

static esp_err_t index_get(httpd_req_t *req)
{
    httpd_resp_set_type(req,"text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t api_bcfallback_post(httpd_req_t *req)
{
    char body[64];
    if(!http_read_body(req,body,sizeof(body))) return http_send_text(req,"ERR body");

    char v[8]={0};
    if(!form_get(body,"enable",v,sizeof(v))) return http_send_text(req,"ERR missing enable");

    int en = (v[0]=='1');

    xSemaphoreTake(g_mutex,portMAX_DELAY);
    g_bc_fallback = en;
    xSemaphoreGive(g_mutex);

    return http_send_text(req, en ? "OK bc_fallback=1" : "OK bc_fallback=0");
}

static esp_err_t api_send_post(httpd_req_t *req)
{
    char body[256];
    if(!http_read_body(req,body,sizeof(body))) return http_send_text(req,"ERR body");

    char dst[16]={0}, msg[160]={0}, ackv[8]={0}, expv[16]={0};
    if(!form_get(body,"dst",dst,sizeof(dst))) return http_send_text(req,"ERR missing dst");
    if(!form_get(body,"msg",msg,sizeof(msg))) return http_send_text(req,"ERR missing msg");
    form_get(body,"ack",ackv,sizeof(ackv));
    form_get(body,"exp",expv,sizeof(expv));

    bool ack = (ackv[0] != '0'); // default on
    uint32_t exp_sec = (uint32_t)atoi(expv);
    if(exp_sec == 0) exp_sec = STORE_DEFAULT_EXP_SEC;

    enqueue_store_message(dst, msg, ack, exp_sec);
    return http_send_text(req,"OK queued");
}

static esp_err_t api_routes_get(httpd_req_t *req)
{
    char out[1400];
    size_t off=0;

    xSemaphoreTake(g_mutex,portMAX_DELAY);
    off += snprintf(out+off,sizeof(out)-off,
                    "dst -> next (rssi, age_ms)\n"
                    "bc_fallback=%u\n\n", g_bc_fallback?1:0);

    uint32_t t=now_ms();
    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used) continue;
        uint32_t age = t - routes[i].t_ms;

        char d[8], n[8];
        call7_to_str(d, routes[i].dst);
        call7_to_str(n, routes[i].next);

        off += snprintf(out+off,sizeof(out)-off,
                        "%s -> %s  (rssi=%d age=%" PRIu32 ")\n",
                        d,n,routes[i].rssi,age);
        if(off > sizeof(out)-120) break;
    }
    xSemaphoreGive(g_mutex);

    return http_send_text(req,out);
}

static esp_err_t api_store_get(httpd_req_t *req)
{
    char out[1800];
    size_t off=0;

    xSemaphoreTake(g_mutex,portMAX_DELAY);

    uint32_t t=now_ms();
    off += snprintf(out+off,sizeof(out)-off,
                    "STORE (used=%d/%d)\n"
                    "idx msg_id dst exp_in_ms state retries expect\n\n",
                    0, STORE_MAX_MSG);

    int used=0;
    for(int i=0;i<STORE_MAX_MSG;i++) if(store_q[i].used) used++;
    // patch used count line:
    // (einfach neu schreiben, ohne fancy cursor)
    off = 0;
    off += snprintf(out+off,sizeof(out)-off,
                    "STORE (used=%d/%d)\n"
                    "idx msg_id dst exp_in_ms state retries expect\n\n",
                    used, STORE_MAX_MSG);

    for(int i=0;i<STORE_MAX_MSG;i++){
        if(!store_q[i].used) continue;

        store_msg_t *m=&store_q[i];
        mr_hdr_v3_t *h=(mr_hdr_v3_t*)m->frame;

        char d[8], e[8];
        call7_to_str(d, h->final_dst);
        call7_to_str(e, m->expect_ack_from);

        uint32_t exp_in = (t < m->expiry_ms) ? (m->expiry_ms - t) : 0;

        const char *state = m->awaiting_ack ? "WAIT_ACK" : "STORED";

        off += snprintf(out+off,sizeof(out)-off,
                        "%d  %u  %s  %" PRIu32 "  %s  %u  %s\n",
                        i, h->msg_id, d, exp_in, state, (unsigned)m->retries,
                        m->awaiting_ack ? e : "-");

        if(off > sizeof(out)-160) break;
    }

    xSemaphoreGive(g_mutex);
    return http_send_text(req,out);
}

static void http_start(void)
{
    httpd_config_t cfg=HTTPD_DEFAULT_CONFIG();
    cfg.stack_size=8192;
    cfg.max_uri_handlers=16;

    ESP_ERROR_CHECK(httpd_start(&g_http,&cfg));

    httpd_uri_t u0={.uri="/",.method=HTTP_GET,.handler=index_get};
    httpd_register_uri_handler(g_http,&u0);

    httpd_uri_t s0={.uri="/api/send",.method=HTTP_POST,.handler=api_send_post};
    httpd_register_uri_handler(g_http,&s0);

    httpd_uri_t r0={.uri="/api/routes",.method=HTTP_GET,.handler=api_routes_get};
    httpd_register_uri_handler(g_http,&r0);

    httpd_uri_t st0={.uri="/api/store",.method=HTTP_GET,.handler=api_store_get};
    httpd_register_uri_handler(g_http,&st0);

    httpd_uri_t b0={.uri="/api/bcfallback",.method=HTTP_POST,.handler=api_bcfallback_post};
    httpd_register_uri_handler(g_http,&b0);

    ESP_LOGI(TAG,"HTTP server started");
}

// ============================================================================
// MAIN
// ============================================================================
void app_main(void)
{
    ESP_LOGI(TAG,"MeshRadio Kapitel 25D start (Store-and-Forward)");

    g_mutex=xSemaphoreCreateMutex();
    if(!g_mutex){ ESP_LOGE(TAG,"mutex failed"); abort(); }

    ESP_ERROR_CHECK(nvs_flash_init());

    init_spi();
    lora_reset();

    ESP_LOGI(TAG,"SX1276 RegVersion=0x%02X", lora_rd(REG_VERSION));

    xSemaphoreTake(g_mutex,portMAX_DELAY);
    lora_apply_locked();
    xSemaphoreGive(g_mutex);

    // DIO0
    dio_q=xQueueCreate(10,sizeof(uint32_t));
    gpio_config_t io={
        .intr_type=GPIO_INTR_POSEDGE,
        .mode=GPIO_MODE_INPUT,
        .pin_bit_mask=(1ULL<<PIN_NUM_DIO0),
        .pull_up_en=1
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_NUM_DIO0,dio_isr,(void*)PIN_NUM_DIO0));

    xTaskCreate(dio_task,"dio",4096,NULL,10,NULL);
    xTaskCreate(store_task,"store",4096,NULL,5,NULL);

    // WiFi + HTTP
    wifi_ap();
    http_start();

    ESP_LOGI(TAG,"CALL=%s", MR_CALLSIGN);
    ESP_LOGI(TAG,"Open http://192.168.4.1");
    ESP_LOGI(TAG,"BC fallback = %u", g_bc_fallback?1:0);

    // Main loop: Cleanup (neighbors/routes) – Store läuft in store_task
    while(1){
        vTaskDelay(pdMS_TO_TICKS(500));

        xSemaphoreTake(g_mutex,portMAX_DELAY);
        neighbor_cleanup_locked();
        route_cleanup_locked();
        xSemaphoreGive(g_mutex);
    }
}