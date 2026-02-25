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
// MeshRadio – Kapitel 25B (Route-Advertisement) – KOMPLETTE SOURCEN
// ----------------------------------------------------------------------------
// Baut auf Kapitel 25 (Destination Routing) auf und erweitert um:
//
// Kapitel 25B Features:
//   - Neuer Frame-Typ: ROUTE_ADV (Route Advertisement)
//   - Aktives Verteilen von Routing-Informationen:
//       "Ich kenne einen Weg zu Ziel X."
//   - Lernen aus ROUTE_ADV:
//       Wenn ich ROUTE_ADV von Neighbor N bekomme:
//         dst = advertised_dst
//         next = N (last_hop)
//         hops = advertised_hops + 1
//         metric = advertised_metric + 1
//   - Einfaches Metric-System:
//       * kleiner metric => besser
//       * bei gleichen metric: höherer RSSI => besser
//   - Web UI + API:
//       * /api/send        (dst,msg)
//       * /api/routes      (Tabelle dst->next inkl rssi/age/metric/hops)
//       * /api/bcfallback  (enable=0/1)
//       * /api/radv        (enable=0/1)
//       * /api/radvint     (ms=5000..300000)
//
// Hinweise/Design-Entscheidungen:
//   - ROUTE_ADV sendet eine Liste bekannter Ziele aus der Routing-Tabelle.
//   - Der Empfänger setzt als next-hop immer den Werber (last_hop),
//     NICHT den "next" aus dessen Tabelle. Der Werber sagt: "geh über mich".
//   - TTL bleibt Pflicht.
//   - Timeout/Aging bleibt Pflicht.
//   - Loop-Bremse (einfach): ignoriere adv_dst == last_hop oder adv_dst == ME.
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
#define MR_PROTO_VERSION 3

#define MR_FLAG_DATA      0x10
#define MR_FLAG_BEACON    0x20
#define MR_FLAG_ROUTE_ADV 0x40   // Kapitel 25B: neuer Frame-Typ

#define DATA_TTL      4
#define BEACON_TTL    2
#define ROUTEADV_TTL  2          // bewusst kurz: Routen-Info soll nicht weit "diffundieren"

#define MAX_PAYLOAD 120

// ============================================================================
// LIMITS / TABLES
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
// Kapitel 25B: ROUTE_ADV Payload-Format
// ----------------------------------------------------------------------------
// Wir senden eine Liste von Einträgen. Jeder Eintrag beschreibt ein Ziel, das
// der Sender erreichen kann, inklusive grober "Kosten".
// - metric: kleiner ist besser
// - hops:   Hopcount bis zum Ziel aus Sicht des Senders
//
// Der Empfänger macht daraus:
//   dst = adv.dst
//   next = last_hop (der Werber)
//   hops = adv.hops + 1
//   metric = adv.metric + 1
// ============================================================================
#pragma pack(push,1)
typedef struct {
    char dst[7];        // advertised destination
    uint8_t hops;       // hops from advertiser to dst
    uint8_t metric;     // cost from advertiser to dst (smaller=better)
} mr_adv_entry_t;
#pragma pack(pop)

// ============================================================================
// STRUCTS
// ============================================================================
typedef struct { bool used; char src[7]; uint16_t id; } seen_t;

typedef struct { bool used; char call[7]; int rssi; uint32_t t_ms; } neighbor_t;

// Kapitel 25B: Route enthält zusätzlich hops + metric
typedef struct {
    bool used;
    char dst[7];
    char next[7];
    int rssi;           // RSSI des Links zu "next" (last measured)
    uint8_t hops;       // hops to dst (aus meiner Sicht)
    uint8_t metric;     // cost to dst (kleiner=besser)
    uint32_t t_ms;      // last update
} route_t;

typedef struct {
    char call[8];
    uint32_t freq_hz;
    uint8_t sf;
    uint8_t bw;
    uint8_t crc;
} cfg_t;

// ============================================================================
// GLOBALS
// ============================================================================
static const char *TAG="MR25B";

static spi_device_handle_t lora_spi;
static QueueHandle_t dio_q;
static SemaphoreHandle_t g_mutex;
static httpd_handle_t g_http=NULL;

static cfg_t g_cfg;
static uint16_t g_msg_id=1;

static seen_t seen_cache[SEEN_CACHE_SIZE];
static neighbor_t neighbors[MAX_NEIGHBORS];
static route_t routes[MAX_ROUTES];

static bool g_beacon_enabled=true;
static uint32_t g_beacon_interval_ms=15000;
static uint32_t g_next_beacon_ms=0;

// Kapitel 25: Wenn keine Route bekannt:
//  - true  => next_hop="*" (Broadcast-Fallback)
//  - false => next_hop="NONE" (Drop)
static bool g_bc_fallback=true;

// Kapitel 25B: Route Advertisement Schalter + Intervall
static bool     g_radv_enabled=true;
static uint32_t g_radv_interval_ms=60000;   // 60s: konservativ für LoRa
static uint32_t g_next_radv_ms=0;

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
// ROUTES (Destination-based) – Kapitel 25B: metric + hops
// ----------------------------------------------------------------------------
// Update-Regel (einfach und "buch-kompatibel"):
//   1) Wenn Eintrag abgelaufen -> übernehmen
//   2) Wenn new_metric < old_metric -> übernehmen
//   3) Wenn metric gleich: wenn RSSI besser -> übernehmen
//   4) Timestamp immer aktualisieren (sonst Ageing killt aktive Einträge)
//
// Hinweis:
//   - metric ist ein grobes "Kostenmaß". Für unser Kapitel reicht das völlig.
//   - RSSI wirkt nur als Tie-Breaker bei gleicher metric.
// ============================================================================
static void route_update_locked(const char dst[7], const char next[7],
                                int rssi, uint8_t hops, uint8_t metric)
{
    uint32_t t=now_ms();

    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used) continue;
        if(!call7_eq(routes[i].dst,dst)) continue;

        bool expired = (t - routes[i].t_ms) > ROUTE_TIMEOUT_MS;

        bool better = false;
        if(metric < routes[i].metric) better = true;
        else if(metric == routes[i].metric && rssi > routes[i].rssi) better = true;

        if(expired || better){
            memcpy(routes[i].next,next,7);
            routes[i].rssi=rssi;
            routes[i].hops=hops;
            routes[i].metric=metric;
        }

        routes[i].t_ms=t;
        return;
    }

    // neu anlegen
    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used){
            routes[i].used=true;
            memcpy(routes[i].dst,dst,7);
            memcpy(routes[i].next,next,7);
            routes[i].rssi=rssi;
            routes[i].hops=hops;
            routes[i].metric=metric;
            routes[i].t_ms=t;
            return;
        }
    }

    // voll -> ältesten überschreiben
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
    routes[oldest].hops=hops;
    routes[oldest].metric=metric;
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
// Apply Config
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

    uint32_t frf=hz_to_frf(g_cfg.freq_hz);
    lora_wr(REG_FRF_MSB,frf>>16);
    lora_wr(REG_FRF_MID,frf>>8);
    lora_wr(REG_FRF_LSB,frf);

    uint8_t bw=0x70;
    if(g_cfg.bw==250) bw=0x80;
    if(g_cfg.bw==500) bw=0x90;

    uint8_t mc1=bw|(1<<1);
    uint8_t mc2=(g_cfg.sf<<4)|((g_cfg.crc?1:0)<<2)|0x03;
    uint8_t mc3=(1<<2);

    lora_wr(REG_MODEM_CONFIG_1,mc1);
    lora_wr(REG_MODEM_CONFIG_2,mc2);
    lora_wr(REG_MODEM_CONFIG_3,mc3);

    lora_wr(REG_PA_CONFIG,0x8E);

    lora_wr(REG_FIFO_RX_BASE_ADDR,0);
    lora_wr(REG_FIFO_ADDR_PTR,0);
    lora_wr(REG_OP_MODE,0x85);
}

// ============================================================================
// BEACON TX
// ============================================================================
static void beacon_schedule_next(void)
{
    g_next_beacon_ms = now_ms() + g_beacon_interval_ms + (esp_random()%2000);
}

static void send_beacon(void)
{
    mr_hdr_v3_t h={0};
    h.magic[0]='M'; h.magic[1]='R';
    h.version=MR_PROTO_VERSION;
    h.flags=MR_FLAG_BEACON;
    h.ttl=BEACON_TTL;
    h.msg_id=g_msg_id++;

    xSemaphoreTake(g_mutex,portMAX_DELAY);
    call7_set(h.src,g_cfg.call);
    call7_set(h.last_hop,g_cfg.call);
    xSemaphoreGive(g_mutex);

    call7_set(h.final_dst,"*");
    call7_set(h.next_hop,"*");
    h.payload_len=0;

    ESP_LOGI(TAG,"TX BEACON id=%u call=%s", h.msg_id, g_cfg.call);
    lora_send((uint8_t*)&h,sizeof(h));
    beacon_schedule_next();
}

// ============================================================================
// Kapitel 25B: ROUTE_ADV TX
// ----------------------------------------------------------------------------
// Wir werben mit den Zielen aus unserer Routing-Tabelle.
// Damit das auch ohne große Netzgraphen funktioniert, gilt:
//   - Advertiser sagt: "Ich kann dst erreichen mit hops/metric."
//   - Empfänger setzt next-hop = Advertiser (last_hop).
//
// Airtime-Schutz:
//   - Wir packen mehrere Einträge in ein Frame, solange MAX_PAYLOAD reicht.
//   - Wir senden nur periodisch (default 60s) + kleiner Jitter.
// ============================================================================
static void radv_schedule_next(void)
{
    g_next_radv_ms = now_ms() + g_radv_interval_ms + (esp_random()%2000);
}

static void send_route_adv(void)
{
    uint8_t buf[256]={0};
    mr_hdr_v3_t *h=(mr_hdr_v3_t*)buf;

    h->magic[0]='M'; h->magic[1]='R';
    h->version=MR_PROTO_VERSION;
    h->flags=MR_FLAG_ROUTE_ADV;
    h->ttl=ROUTEADV_TTL;
    h->msg_id=g_msg_id++;

    char my7[7];
    xSemaphoreTake(g_mutex,portMAX_DELAY);
    call7_set(h->src, g_cfg.call);
    call7_set(h->last_hop, g_cfg.call);
    call7_set(my7, g_cfg.call);
    xSemaphoreGive(g_mutex);

    // ROUTE_ADV ist Broadcast – jeder Nachbar darf lernen
    call7_set(h->final_dst,"*");
    call7_set(h->next_hop,"*");

    // Payload füllen: Liste der bekannten Ziele
    mr_adv_entry_t *e=(mr_adv_entry_t*)(buf + sizeof(mr_hdr_v3_t));
    size_t max_entries = MAX_PAYLOAD / sizeof(mr_adv_entry_t);
    size_t n_entries = 0;

    xSemaphoreTake(g_mutex,portMAX_DELAY);
    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used) continue;

        // Nicht uns selbst bewerben (unnötig) und kein Broadcast-Ziel
        if(call7_eq(routes[i].dst, my7)) continue;
        if(call7_is_wild(routes[i].dst)) continue;

        // Eintrag schreiben
        memcpy(e[n_entries].dst, routes[i].dst, 7);
        e[n_entries].hops = routes[i].hops;
        e[n_entries].metric = routes[i].metric;

        n_entries++;
        if(n_entries >= max_entries) break;
    }
    xSemaphoreGive(g_mutex);

    h->payload_len = (uint8_t)(n_entries * sizeof(mr_adv_entry_t));

    ESP_LOGI(TAG,"TX ROUTE_ADV id=%u entries=%u", h->msg_id, (unsigned)n_entries);
    lora_send(buf, sizeof(mr_hdr_v3_t) + h->payload_len);

    radv_schedule_next();
}

// ============================================================================
// DATA TX (Destination-based)
// ============================================================================
static void send_data_to(const char *dst_str, const char *txt)
{
    uint8_t buf[256]={0};
    mr_hdr_v3_t *h=(mr_hdr_v3_t*)buf;

    h->magic[0]='M'; h->magic[1]='R';
    h->version=MR_PROTO_VERSION;
    h->flags=MR_FLAG_DATA;
    h->ttl=DATA_TTL;
    h->msg_id=g_msg_id++;

    char dst7[7]; call7_set(dst7, dst_str);

    xSemaphoreTake(g_mutex,portMAX_DELAY);
    call7_set(h->src, g_cfg.call);
    call7_set(h->last_hop, g_cfg.call);
    xSemaphoreGive(g_mutex);

    memcpy(h->final_dst, dst7, 7);

    char nh[7]; bool has=false;
    xSemaphoreTake(g_mutex,portMAX_DELAY);
    has = route_lookup_locked(dst7, nh);
    bool bc = g_bc_fallback;
    xSemaphoreGive(g_mutex);

    if(has){
        memcpy(h->next_hop, nh, 7);
    }else{
        call7_set(h->next_hop, bc ? "*" : "NONE");
    }

    size_t n=strlen(txt);
    if(n>MAX_PAYLOAD) n=MAX_PAYLOAD;
    h->payload_len=(uint8_t)n;
    memcpy(buf+sizeof(mr_hdr_v3_t),txt,n);

    char nhs[8]; call7_to_str(nhs, h->next_hop);
    ESP_LOGI(TAG,"TX DATA id=%u dst=%s next=%s txt=%s", h->msg_id, dst_str, nhs, txt);

    lora_send(buf, sizeof(mr_hdr_v3_t)+n);
}

// ============================================================================
// FORWARD (Destination-based)
// ============================================================================
static void forward_data(uint8_t *buf, size_t len)
{
    if(len < sizeof(mr_hdr_v3_t)) return;
    mr_hdr_v3_t *h=(mr_hdr_v3_t*)buf;

    if(h->ttl <= 1) return;
    h->ttl--;

    char my7[7];
    xSemaphoreTake(g_mutex,portMAX_DELAY);
    call7_set(my7, g_cfg.call);
    xSemaphoreGive(g_mutex);

    memcpy(h->last_hop, my7, 7);

    char nh[7]; bool has=false;
    bool bc=false;
    xSemaphoreTake(g_mutex,portMAX_DELAY);
    has = route_lookup_locked(h->final_dst, nh);
    bc  = g_bc_fallback;
    xSemaphoreGive(g_mutex);

    if(has){
        memcpy(h->next_hop, nh, 7);
    }else{
        call7_set(h->next_hop, bc ? "*" : "NONE");
    }

    char dsts[8], nhs[8];
    call7_to_str(dsts, h->final_dst);
    call7_to_str(nhs, h->next_hop);

    ESP_LOGI(TAG,"FWD DATA id=%u dst=%s next=%s ttl=%u", h->msg_id, dsts, nhs, h->ttl);
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
    if(len < sizeof(mr_hdr_v3_t)) return;

    mr_hdr_v3_t *h=(mr_hdr_v3_t*)buf;
    if(h->magic[0]!='M'||h->magic[1]!='R') return;
    if(h->version!=MR_PROTO_VERSION) return;

    if(seen_before(h->src,h->msg_id)) return;
    remember_msg(h->src,h->msg_id);

    int rssi=(int)lora_rd(REG_PKT_RSSI_VALUE)-157;

    // Nachbar-Pflege: last_hop ist immer der direkte Sender auf Funkebene
    xSemaphoreTake(g_mutex,portMAX_DELAY);
    neighbor_update_locked(h->last_hop, rssi);
    xSemaphoreGive(g_mutex);

    char srcs[8], dsts[8], nhs[8], lhs[8];
    call7_to_str(srcs, h->src);
    call7_to_str(dsts, h->final_dst);
    call7_to_str(nhs, h->next_hop);
    call7_to_str(lhs, h->last_hop);

    // ------------------------------------------------------------------------
    // BEACON: route learning (Kapitel 25)
    // ------------------------------------------------------------------------
    if(h->flags & MR_FLAG_BEACON){
        ESP_LOGI(TAG,"RX BEACON src=%s via=%s rssi=%d", srcs, lhs, rssi);

        // Learn direct route to SRC via last_hop
        // - hops=1 (direkter Nachbar bzw. direkt gehört via last_hop)
        // - metric=1 (Basis-Kosten)
        xSemaphoreTake(g_mutex,portMAX_DELAY);
        route_update_locked(h->src, h->last_hop, rssi, 1, 1);
        xSemaphoreGive(g_mutex);

        return;
    }

    // ------------------------------------------------------------------------
    // ROUTE_ADV: Kapitel 25B
    // ------------------------------------------------------------------------
    if(h->flags & MR_FLAG_ROUTE_ADV){
        ESP_LOGI(TAG,"RX ROUTE_ADV src=%s via=%s rssi=%d bytes=%u", srcs, lhs, rssi, h->payload_len);

        // Payload prüfen
        if(h->payload_len == 0) return;
        if((sizeof(mr_hdr_v3_t) + h->payload_len) > len) return;
        if((h->payload_len % sizeof(mr_adv_entry_t)) != 0) return;

        char my7[7];
        xSemaphoreTake(g_mutex,portMAX_DELAY);
        call7_set(my7, g_cfg.call);
        xSemaphoreGive(g_mutex);

        size_t n_entries = h->payload_len / sizeof(mr_adv_entry_t);
        mr_adv_entry_t *e = (mr_adv_entry_t*)(buf + sizeof(mr_hdr_v3_t));

        for(size_t i=0;i<n_entries;i++){
            // Loop-Bremse (einfach):
            //  - ignoriere Werbung "zu mir selbst"
            //  - ignoriere Werbung "zu dem, von dem es kam" (häufiger Kurzschluss)
            if(call7_eq(e[i].dst, my7)) continue;
            if(call7_eq(e[i].dst, h->last_hop)) continue;

            // Neue Route aus Sicht des Empfängers:
            // - next ist der Werber (last_hop)
            // - hops/metric jeweils +1 (ein Hop mehr als beim Werber)
            uint8_t new_hops = (uint8_t)(e[i].hops + 1);
            uint8_t new_metric = (uint8_t)(e[i].metric + 1);

            xSemaphoreTake(g_mutex,portMAX_DELAY);
            route_update_locked(e[i].dst, h->last_hop, rssi, new_hops, new_metric);
            xSemaphoreGive(g_mutex);
        }

        return;
    }

    // ------------------------------------------------------------------------
    // DATA: Kapitel 25
    // ------------------------------------------------------------------------
    if(h->flags & MR_FLAG_DATA){
        char my7[7];
        xSemaphoreTake(g_mutex,portMAX_DELAY);
        call7_set(my7, g_cfg.call);
        xSemaphoreGive(g_mutex);

        bool iam_dst = call7_eq(h->final_dst, my7);
        bool iam_nexthop = call7_eq(h->next_hop, my7);
        bool nh_wild = call7_is_wild(h->next_hop);
        bool nh_none = (h->next_hop[0]=='N' && h->next_hop[1]=='O'); // "NONE..."

        char txt[MAX_PAYLOAD+1]={0};
        if(h->payload_len>0 && h->payload_len<=MAX_PAYLOAD){
            memcpy(txt, buf+sizeof(mr_hdr_v3_t), h->payload_len);
            txt[h->payload_len]=0;
        }

        ESP_LOGI(TAG,"RX DATA src=%s dst=%s next=%s via=%s rssi=%d ttl=%u txt=\"%s\"",
                 srcs,dsts,nhs,lhs,rssi,h->ttl,txt);

        if(iam_dst){
            ESP_LOGI(TAG,"DATA delivered ✅ (dst=%s)", dsts);
            return;
        }
        if(nh_none){
            ESP_LOGI(TAG,"DROP: next_hop=NONE");
            return;
        }
        if((iam_nexthop || nh_wild) && h->ttl>1){
            forward_data(buf,len);
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
    strcpy((char*)ap.ap.ssid,"MeshRadio-Setup");
    ap.ap.authmode=WIFI_AUTH_OPEN;
    ap.ap.max_connection=4;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP,&ap));
    ESP_ERROR_CHECK(esp_wifi_start());
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

// FIXED: restores '=' also on match (wie Kapitel 25)
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
"<title>MeshRadio Kapitel 25B</title>"
"<style>"
"body{font-family:system-ui;margin:16px;max-width:900px}"
"button{padding:10px 14px;margin:6px 4px;font-size:16px}"
"input{padding:10px;margin:6px 0;font-size:16px;width:100%}"
"code{background:#eee;padding:2px 4px}"
"</style></head><body>"
"<h2>MeshRadio Kapitel 25B – Route Advertisement</h2>"
"<p>AP: <code>MeshRadio-Setup2</code>  URL: <code>http://192.168.4.1</code></p>"
"<div id='m'></div>"

"<h3>DATA senden</h3>"
"<label>Ziel (dst)</label><input id='dst' value='DJ1ABC'>"
"<label>Text</label><input id='msg' value='Hallooooo'>"
"<button onclick='send()'>SEND</button>"

"<h3>Broadcast Fallback</h3>"
"<button onclick='setBC(1)'>BC fallback ON</button>"
"<button onclick='setBC(0)'>BC fallback OFF</button>"

"<h3>Route Advertisement</h3>"
"<button onclick='setRA(1)'>RAdv ON</button>"
"<button onclick='setRA(0)'>RAdv OFF</button>"
"<label>RAdv Intervall (ms)</label><input id='rai' value='60000'>"
"<button onclick='setRAI()'>Set RAdv Intervall</button>"

"<p><button onclick='routes()'>Routes anzeigen</button></p>"
"<pre id='st'></pre>"

"<script>"
"function m(t){document.getElementById('m').innerHTML='<b>'+t+'</b>'}"
"async function post(url,body){"
" let r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:body});"
" let t=await r.text(); m(t); }"
"async function send(){"
" let dst=document.getElementById('dst').value;"
" let msg=document.getElementById('msg').value;"
" await post('/api/send','dst='+encodeURIComponent(dst)+'&msg='+encodeURIComponent(msg));"
"}"
"async function setBC(v){ await post('/api/bcfallback','enable='+encodeURIComponent(v)); }"
"async function setRA(v){ await post('/api/radv','enable='+encodeURIComponent(v)); }"
"async function setRAI(){"
" let ms=document.getElementById('rai').value;"
" await post('/api/radvint','ms='+encodeURIComponent(ms));"
"}"
"async function routes(){ let r=await fetch('/api/routes'); document.getElementById('st').textContent=await r.text(); }"
"routes();"
"</script></body></html>";

static esp_err_t index_get(httpd_req_t *req)
{
    httpd_resp_set_type(req,"text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t api_routes_get(httpd_req_t *req)
{
    char out[1600];
    size_t off=0;

    xSemaphoreTake(g_mutex,portMAX_DELAY);
    off += snprintf(out+off,sizeof(out)-off,
                    "dst -> next (metric, hops, rssi, age_ms)\n"
                    "bc_fallback=%u  radv=%u  radv_interval_ms=%" PRIu32 "\n\n",
                    g_bc_fallback?1:0, g_radv_enabled?1:0, g_radv_interval_ms);

    uint32_t t=now_ms();
    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used) continue;
        uint32_t age = t - routes[i].t_ms;

        char d[8], n[8];
        call7_to_str(d, routes[i].dst);
        call7_to_str(n, routes[i].next);

        off += snprintf(out+off,sizeof(out)-off,
                        "%s -> %s  (m=%u h=%u rssi=%d age=%" PRIu32 ")\n",
                        d,n,(unsigned)routes[i].metric,(unsigned)routes[i].hops,routes[i].rssi,age);
        if(off > sizeof(out)-120) break;
    }
    xSemaphoreGive(g_mutex);

    return http_send_text(req,out);
}

static esp_err_t api_send_post(httpd_req_t *req)
{
    char body[256];
    if(!http_read_body(req,body,sizeof(body))) return http_send_text(req,"ERR body");

    char dst[16]={0}, msg[160]={0};
    if(!form_get(body,"dst",dst,sizeof(dst))) return http_send_text(req,"ERR missing dst");
    if(!form_get(body,"msg",msg,sizeof(msg))) return http_send_text(req,"ERR missing msg");

    send_data_to(dst,msg);
    return http_send_text(req,"OK send");
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

// Kapitel 25B: Route Advertisement ON/OFF
static esp_err_t api_radv_post(httpd_req_t *req)
{
    char body[64];
    if(!http_read_body(req,body,sizeof(body))) return http_send_text(req,"ERR body");

    char v[8]={0};
    if(!form_get(body,"enable",v,sizeof(v))) return http_send_text(req,"ERR missing enable");

    int en = (v[0]=='1');

    xSemaphoreTake(g_mutex,portMAX_DELAY);
    g_radv_enabled = en;
    if(en) radv_schedule_next();
    xSemaphoreGive(g_mutex);

    return http_send_text(req, en ? "OK radv=1" : "OK radv=0");
}

// Kapitel 25B: Route Advertisement Intervall setzen
static esp_err_t api_radvint_post(httpd_req_t *req)
{
    char body[64];
    if(!http_read_body(req,body,sizeof(body))) return http_send_text(req,"ERR body");

    char v[16]={0};
    if(!form_get(body,"ms",v,sizeof(v))) return http_send_text(req,"ERR missing ms");

    int ms = atoi(v);
    if(ms < 5000) ms = 5000;          // min 5s (für Tests)
    if(ms > 300000) ms = 300000;      // max 5min (Airtime sparen)

    xSemaphoreTake(g_mutex,portMAX_DELAY);
    g_radv_interval_ms = (uint32_t)ms;
    radv_schedule_next();
    xSemaphoreGive(g_mutex);

    char out[64];
    snprintf(out,sizeof(out),"OK radv_interval_ms=%d", ms);
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

    httpd_uri_t r0={.uri="/api/routes",.method=HTTP_GET,.handler=api_routes_get};
    httpd_register_uri_handler(g_http,&r0);

    httpd_uri_t s0={.uri="/api/send",.method=HTTP_POST,.handler=api_send_post};
    httpd_register_uri_handler(g_http,&s0);

    httpd_uri_t b0={.uri="/api/bcfallback",.method=HTTP_POST,.handler=api_bcfallback_post};
    httpd_register_uri_handler(g_http,&b0);

    httpd_uri_t a0={.uri="/api/radv",.method=HTTP_POST,.handler=api_radv_post};
    httpd_register_uri_handler(g_http,&a0);

    httpd_uri_t i0={.uri="/api/radvint",.method=HTTP_POST,.handler=api_radvint_post};
    httpd_register_uri_handler(g_http,&i0);

    ESP_LOGI(TAG,"HTTP server started");
}

// ============================================================================
// MAIN
// ============================================================================
void app_main(void)
{
    ESP_LOGI(TAG,"MeshRadio Kapitel 25B start (Route Advertisement)");

    g_mutex=xSemaphoreCreateMutex();
    if(!g_mutex){ ESP_LOGE(TAG,"mutex failed"); abort(); }

    ESP_ERROR_CHECK(nvs_flash_init());

    // Defaults
    strcpy(g_cfg.call,"DJ2RF");
    g_cfg.freq_hz=433775000;
    g_cfg.sf=7;
    g_cfg.bw=125;
    g_cfg.crc=1;

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

    // WiFi + HTTP
    wifi_ap();
    http_start();

    beacon_schedule_next();
    radv_schedule_next();

    ESP_LOGI(TAG,"Open http://192.168.4.1");
    ESP_LOGI(TAG,"BC fallback = %u", g_bc_fallback?1:0);
    ESP_LOGI(TAG,"RAdv        = %u", g_radv_enabled?1:0);
    ESP_LOGI(TAG,"RAdv int ms = %" PRIu32, g_radv_interval_ms);

    while(1){
        vTaskDelay(pdMS_TO_TICKS(50));

        // Beacons
        if(g_beacon_enabled && now_ms()>g_next_beacon_ms)
            send_beacon();

        // Route Advertisements
        if(g_radv_enabled && now_ms()>g_next_radv_ms)
            send_route_adv();

        // Cleanup
        xSemaphoreTake(g_mutex,portMAX_DELAY);
        neighbor_cleanup_locked();
        route_cleanup_locked();
        xSemaphoreGive(g_mutex);
    }
}