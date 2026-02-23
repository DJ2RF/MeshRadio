// ============================================================================
// MeshRadio – Kapitel 23A (Sourcen komplett, sauber dokumentiert)
// ----------------------------------------------------------------------------
// Basis: Kapitel 23 (Web-Setup + Neighbors/Routes Sichtbarkeit)
//
// NEU in Kapitel 23A: Neighbor-basierter Auto-Relay Filter
//   Ziel:
//     - Flooding/Forwarding nur dann erlauben, wenn der Empfänger "ein guter
//       Nachbar" ist (RSSI-Schwelle).
//     - Dadurch weniger unnötige Relays / weniger Kollisionen.
//
// Umsetzung (einfach & praxisnah):
//   ✔ Wir führen einen Forward-Filter ein, der pro eingehendem Frame prüft:
//        - kennen wir den last_hop als Neighbor?
//        - ist RSSI vom last_hop >= threshold?
//     -> Nur dann forwarden wir (TTL--, last_hop setzen, next_hop="*").
//
//   ✔ Web-API (neue Endpoints):
//        GET  /api/relay               -> JSON Status (enabled, threshold)
//        POST /api/relay               -> enabled=0|1
//        POST /api/relaythr            -> rssi=-120..-30 (dBm)
//
//   ✔ Web-UI erweitert:
//        - Relay enabled (0/1)
//        - Relay threshold (dBm)
//        - Buttons "Set Relay" / "Set Threshold"
//
// Hinweis:
//   - Wir forwarden hier exemplarisch nur BEACON Frames (MR_FLAG_BEACON),
//     damit man das Verhalten sicher testen kann.
//   - Für Chat/Datenframes wird das gleiche Muster später übernommen.
//
// ESP-IDF v5.5.x
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
#include "nvs.h"

// WiFi / Netif / HTTP server (IDF built-in)
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"

// ============================================================================
// PROTOKOLL / BEACON
// ============================================================================

#define MR_PROTO_VERSION 3
#define MR_FLAG_BEACON  0x20
#define BEACON_TTL 2   // Kapitel 23A: TTL etwas höher, damit Forward sichtbar wird

#define DEFAULT_BEACON_INTERVAL_MS 15000
#define DEFAULT_BEACON_JITTER_MS   2000

// ============================================================================
// TABELLEN / LIMITS
// ============================================================================

#define SEEN_CACHE_SIZE     32
#define MAX_ROUTES          20
#define ROUTE_TIMEOUT_MS    120000
#define MAX_NEIGHBORS       20
#define NEIGHBOR_TIMEOUT_MS 60000

// ============================================================================
// LORA PINS (TTGO T-Beam V1.1 / SX1276 Boards)
// ============================================================================

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 27
#define PIN_NUM_CLK  5
#define PIN_NUM_CS   18
#define PIN_NUM_RST  23
#define PIN_NUM_DIO0 26

#define LORA_SPI_HOST VSPI_HOST

// ============================================================================
// SX1276 Register (minimal)
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
// HEADER V3 (Kapitel 19A)
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
// Tabellen
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
    char destination[7];
    char next_hop[7];
    int rssi_dbm;
    uint32_t last_seen_ms;
} route_entry_t;

// ============================================================================
// Persistente Config (NVS)
// ============================================================================
typedef struct {
    char     call[8];
    uint32_t freq_hz;
    uint8_t  sf;
    uint8_t  bw_khz;
    uint8_t  crc_on;
} mr_config_t;

// ============================================================================
// Globals
// ============================================================================

static const char *TAG="MR23A";

static spi_device_handle_t lora_spi;
static QueueHandle_t dio0_evt_queue;

static uint16_t g_msg_id=1;
static uint32_t next_beacon_ms=0;

static bool     g_beacon_enabled     = true;
static uint32_t g_beacon_interval_ms = DEFAULT_BEACON_INTERVAL_MS;
static uint32_t g_beacon_jitter_ms   = DEFAULT_BEACON_JITTER_MS;

static seen_msg_t    seen_cache[SEEN_CACHE_SIZE];
static neighbor_t    neighbors[MAX_NEIGHBORS];
static route_entry_t routes[MAX_ROUTES];

static mr_config_t g_cfg;

// EIN Mutex schützt alle shared states
static SemaphoreHandle_t g_mutex;

// HTTP Server
static httpd_handle_t g_http = NULL;

// --------------------------------------------------------------------
// 23A Relay-Filter Settings (per Web)
// --------------------------------------------------------------------
static bool g_relay_enabled = true;
// Wenn last_hop RSSI < threshold, dann kein Forward
static int  g_relay_rssi_threshold_dbm = -95;

// ============================================================================
// Helpers
// ============================================================================

static uint32_t now_ms(void){ return (uint32_t)(xTaskGetTickCount()*portTICK_PERIOD_MS); }

static void call7_set(char out7[7], const char *s)
{
    memset(out7,' ',7);
    size_t n=strlen(s); if(n>7)n=7;
    memcpy(out7,s,n);
}

static void call7_to_cstr(char out8[8], const char in7[7])
{
    memcpy(out8,in7,7); out8[7]=0;
    for(int i=6;i>=0;i--){
        if(out8[i]==' ') out8[i]=0;
        else break;
    }
}

static bool call7_eq(const char a[7], const char b[7]){ return memcmp(a,b,7)==0; }

static uint32_t hz_to_frf(uint32_t freq_hz)
{
    uint64_t frf = ((uint64_t)freq_hz << 19) / 32000000ULL;
    return (uint32_t)frf;
}

// ============================================================================
// NVS (wie vorher, unverändert)
// ============================================================================
static esp_err_t config_save(const mr_config_t *cfg)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open("mr", NVS_READWRITE, &h);
    if(err != ESP_OK) return err;

    err = nvs_set_str(h, "call", cfg->call); if(err!=ESP_OK){nvs_close(h);return err;}
    err = nvs_set_u32(h, "freq", cfg->freq_hz); if(err!=ESP_OK){nvs_close(h);return err;}
    err = nvs_set_u8(h, "sf", cfg->sf); if(err!=ESP_OK){nvs_close(h);return err;}
    err = nvs_set_u8(h, "bw", cfg->bw_khz); if(err!=ESP_OK){nvs_close(h);return err;}
    err = nvs_set_u8(h, "crc", cfg->crc_on); if(err!=ESP_OK){nvs_close(h);return err;}

    err = nvs_commit(h);
    nvs_close(h);
    return err;
}

static esp_err_t config_load_or_defaults(mr_config_t *cfg)
{
    memset(cfg,0,sizeof(*cfg));
    strncpy(cfg->call, "DJ2RF", sizeof(cfg->call)-1);
    cfg->freq_hz = 433775000UL;
    cfg->sf      = 7;
    cfg->bw_khz  = 125;
    cfg->crc_on  = 1;

    nvs_handle_t h;
    esp_err_t err = nvs_open("mr", NVS_READWRITE, &h);
    if(err != ESP_OK) return err;

    size_t len = sizeof(cfg->call);
    err = nvs_get_str(h, "call", cfg->call, &len);
    if(err == ESP_ERR_NVS_NOT_FOUND){
        ESP_LOGW(TAG,"NVS: empty -> write defaults");
        nvs_close(h);
        return config_save(cfg);
    }else if(err != ESP_OK){
        nvs_close(h);
        return err;
    }

    uint32_t f=0; if(nvs_get_u32(h,"freq",&f)==ESP_OK) cfg->freq_hz=f;
    uint8_t v=0;  if(nvs_get_u8(h,"sf",&v)==ESP_OK) cfg->sf=v;
    if(nvs_get_u8(h,"bw",&v)==ESP_OK) cfg->bw_khz=v;
    if(nvs_get_u8(h,"crc",&v)==ESP_OK) cfg->crc_on=v;

    nvs_close(h);
    return ESP_OK;
}

// ============================================================================
// Neighbor/Route update (locked)
// ============================================================================
static void route_update_locked(const char dst[7], const char hop[7], int rssi)
{
    uint32_t t=now_ms();

    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used) continue;
        if(call7_eq(routes[i].destination,dst)){
            memcpy(routes[i].next_hop,hop,7);
            routes[i].rssi_dbm=rssi;
            routes[i].last_seen_ms=t;
            return;
        }
    }
    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used){
            routes[i].used=true;
            memcpy(routes[i].destination,dst,7);
            memcpy(routes[i].next_hop,hop,7);
            routes[i].rssi_dbm=rssi;
            routes[i].last_seen_ms=t;
            return;
        }
    }
}

static void neighbor_update_locked(const char call[7], int rssi)
{
    uint32_t t=now_ms();

    for(int i=0;i<MAX_NEIGHBORS;i++){
        if(!neighbors[i].used) continue;
        if(call7_eq(neighbors[i].call,call)){
            neighbors[i].rssi_dbm=rssi;
            neighbors[i].last_seen_ms=t;
            return;
        }
    }
    for(int i=0;i<MAX_NEIGHBORS;i++){
        if(!neighbors[i].used){
            neighbors[i].used=true;
            memcpy(neighbors[i].call,call,7);
            neighbors[i].rssi_dbm=rssi;
            neighbors[i].last_seen_ms=t;
            return;
        }
    }
}

static void neighbor_cleanup_locked(void)
{
    uint32_t t=now_ms();
    for(int i=0;i<MAX_NEIGHBORS;i++){
        if(!neighbors[i].used) continue;
        if(t - neighbors[i].last_seen_ms > NEIGHBOR_TIMEOUT_MS)
            neighbors[i].used=false;
    }
}

static void route_cleanup_locked(void)
{
    uint32_t t=now_ms();
    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used) continue;
        if(t - routes[i].last_seen_ms > ROUTE_TIMEOUT_MS)
            routes[i].used=false;
    }
}

// ============================================================================
// 23A: Neighbor lookup (für Relay Filter)
// ----------------------------------------------------------------------------
// Gibt true zurück, wenn call als Neighbor bekannt und RSSI >= threshold.
// ============================================================================

static bool neighbor_is_good_locked(const char call[7], int threshold_dbm)
{
    uint32_t t=now_ms();
    for(int i=0;i<MAX_NEIGHBORS;i++){
        if(!neighbors[i].used) continue;
        if(!call7_eq(neighbors[i].call, call)) continue;

        // Optional: nur wenn nicht zu alt
        if(t - neighbors[i].last_seen_ms > NEIGHBOR_TIMEOUT_MS) return false;

        return (neighbors[i].rssi_dbm >= threshold_dbm);
    }
    return false;
}

// ============================================================================
// SPI / LoRa low-level
// ============================================================================
static void lora_write_reg(uint8_t reg,uint8_t val)
{
    uint8_t tx[2]={ (uint8_t)(reg|0x80), val };
    spi_transaction_t t={.length=16,.tx_buffer=tx};
    spi_device_transmit(lora_spi,&t);
}

static uint8_t lora_read_reg(uint8_t reg)
{
    uint8_t tx[2]={ (uint8_t)(reg&0x7F), 0 };
    uint8_t rx[2]={0};
    spi_transaction_t t={.length=16,.tx_buffer=tx,.rx_buffer=rx};
    spi_device_transmit(lora_spi,&t);
    return rx[1];
}

static void lora_clear_irqs(void){ lora_write_reg(REG_IRQ_FLAGS,0xFF); }

static void lora_send_packet(uint8_t *data,size_t len)
{
    lora_clear_irqs();

    lora_write_reg(REG_FIFO_TX_BASE_ADDR,0);
    lora_write_reg(REG_FIFO_ADDR_PTR,0);
    for(size_t i=0;i<len;i++) lora_write_reg(REG_FIFO,data[i]);
    lora_write_reg(REG_PAYLOAD_LENGTH,(uint8_t)len);

    lora_write_reg(REG_OP_MODE,0x83); // TX
    while(!(lora_read_reg(REG_IRQ_FLAGS)&IRQ_TX_DONE))
        vTaskDelay(pdMS_TO_TICKS(5));
    lora_clear_irqs();
    lora_write_reg(REG_OP_MODE,0x85); // RX cont
}

static void lora_reset(void)
{
    if(!GPIO_IS_VALID_OUTPUT_GPIO(PIN_NUM_RST)){
        ESP_LOGE(TAG, "LoRa RST GPIO %d invalid -> reset skipped", PIN_NUM_RST);
        return;
    }
    gpio_set_direction(PIN_NUM_RST,GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_RST,0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_NUM_RST,1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

// ============================================================================
// SX1276 Apply (22A)
// ============================================================================
static uint8_t bw_to_reg(uint8_t bw_khz)
{
    switch(bw_khz){
        case 125: return 0x70;
        case 250: return 0x80;
        case 500: return 0x90;
        default:  return 0x70;
    }
}

static bool need_ldo(uint8_t sf, uint8_t bw_khz){ return (bw_khz==125 && sf>=11); }

static void lora_apply_config_locked(void)
{
    ESP_LOGI(TAG,"APPLY: call=%s freq=%" PRIu32 " sf=%u bw=%u crc=%u",
             g_cfg.call, g_cfg.freq_hz, g_cfg.sf, g_cfg.bw_khz, g_cfg.crc_on);

    lora_write_reg(REG_OP_MODE,0x80); // sleep
    vTaskDelay(pdMS_TO_TICKS(10));

    uint32_t frf = hz_to_frf(g_cfg.freq_hz);
    lora_write_reg(REG_FRF_MSB,(uint8_t)(frf>>16));
    lora_write_reg(REG_FRF_MID,(uint8_t)(frf>>8));
    lora_write_reg(REG_FRF_LSB,(uint8_t)(frf));

    uint8_t bw_bits = bw_to_reg(g_cfg.bw_khz);
    uint8_t cr_bits = (1<<1);      // CR 4/5
    uint8_t mc1     = bw_bits | cr_bits;

    uint8_t sf = g_cfg.sf; if(sf<7)sf=7; if(sf>12)sf=12;
    uint8_t crc_bit = (g_cfg.crc_on ? (1<<2) : 0);
    uint8_t mc2 = (sf<<4) | crc_bit | 0x03;

    uint8_t ldo = need_ldo(sf, g_cfg.bw_khz) ? (1<<3) : 0;
    uint8_t agc = (1<<2);
    uint8_t mc3 = ldo | agc;

    lora_write_reg(REG_MODEM_CONFIG_1, mc1);
    lora_write_reg(REG_MODEM_CONFIG_2, mc2);
    lora_write_reg(REG_MODEM_CONFIG_3, mc3);

    lora_write_reg(REG_PA_CONFIG, 0x8E);

    lora_write_reg(REG_FIFO_RX_BASE_ADDR,0);
    lora_write_reg(REG_FIFO_ADDR_PTR,0);
    lora_write_reg(REG_OP_MODE,0x85); // RX cont
}

// ============================================================================
// Beacon scheduler
// ============================================================================
static void beacon_schedule_next_locked(void)
{
    uint32_t jitter = (g_beacon_jitter_ms ? (esp_random()%g_beacon_jitter_ms) : 0);
    next_beacon_ms = now_ms() + g_beacon_interval_ms + jitter;
}

static void beacon_schedule_next(void)
{
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    beacon_schedule_next_locked();
    xSemaphoreGive(g_mutex);
}

static void send_beacon(void)
{
    mr_hdr_v3_t b;
    memset(&b,0,sizeof(b));

    b.magic[0]='M';
    b.magic[1]='R';
    b.version=MR_PROTO_VERSION;
    b.flags=MR_FLAG_BEACON;
    b.ttl=BEACON_TTL;
    b.msg_id=g_msg_id++;

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    call7_set(b.src, g_cfg.call);
    call7_set(b.last_hop, g_cfg.call);
    xSemaphoreGive(g_mutex);

    call7_set(b.final_dst,"*");
    call7_set(b.next_hop,"*");
    b.payload_len=0;

    ESP_LOGI(TAG,"TX BEACON id=%u", b.msg_id);
    lora_send_packet((uint8_t*)&b,sizeof(b));

    beacon_schedule_next();
}

// ============================================================================
// Seen cache
// ============================================================================
static bool seen_before(const char src[7], uint16_t id)
{
    for(int i=0;i<SEEN_CACHE_SIZE;i++){
        if(!seen_cache[i].used) continue;
        if(call7_eq(seen_cache[i].src,src) && seen_cache[i].msg_id==id)
            return true;
    }
    return false;
}

static void remember_msg(const char src[7], uint16_t id)
{
    static int idx=0;
    memcpy(seen_cache[idx].src,src,7);
    seen_cache[idx].msg_id=id;
    seen_cache[idx].used=true;
    idx=(idx+1)%SEEN_CACHE_SIZE;
}

// ============================================================================
// 23A: Forwarding von Beacons (Neighbor Filter)
// ----------------------------------------------------------------------------
// Bedingungen:
//   - relay enabled
//   - TTL > 1
//   - last_hop ist "good neighbor" (RSSI >= threshold)
// Action:
//   - TTL--
//   - last_hop = MY_CALL
//   - next_hop="*"
//   - re-TX
// ============================================================================
static void maybe_forward_beacon(uint8_t *buf, size_t len, int rssi_last_hop)
{
    if(len < sizeof(mr_hdr_v3_t)) return;

    mr_hdr_v3_t *h = (mr_hdr_v3_t*)buf;

    // nur Beacons forwarden (für Kapitel 23A übersichtlich)
    if(!(h->flags & MR_FLAG_BEACON)) return;

    // TTL muss noch >1 sein (wenn wir decrementen, darf es nicht 0 werden)
    if(h->ttl <= 1) return;

    // Prüfen ob Relay enabled und neighbor quality passt
    bool relay_enabled=false;
    int thr=-95;
    char my_call[8]={0};

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    relay_enabled = g_relay_enabled;
    thr = g_relay_rssi_threshold_dbm;
    strncpy(my_call, g_cfg.call, sizeof(my_call)-1);

    bool good = neighbor_is_good_locked(h->last_hop, thr);
    xSemaphoreGive(g_mutex);

    if(!relay_enabled){
        ESP_LOGI(TAG,"RELAY: disabled -> no forward");
        return;
    }

    if(!good){
        char hop8[8]; call7_to_cstr(hop8, h->last_hop);
        ESP_LOGI(TAG,"RELAY: drop (hop=%s rssi=%d < thr=%d)", hop8, rssi_last_hop, thr);
        return;
    }

    // Forward vorbereiten
    h->ttl -= 1;
    call7_set(h->last_hop, my_call);
    call7_set(h->next_hop, "*");

    ESP_LOGI(TAG,"RELAY: forward beacon id=%u ttl=%u", h->msg_id, h->ttl);
    lora_send_packet(buf, len);
}

// ============================================================================
// RX Handler
// ============================================================================
static void handle_rx_packet(void)
{
    uint8_t irq=lora_read_reg(REG_IRQ_FLAGS);
    if(!(irq & IRQ_RX_DONE)) return;

    uint8_t len=lora_read_reg(REG_RX_NB_BYTES);

    uint8_t addr=lora_read_reg(REG_FIFO_RX_CURRENT_ADDR);
    lora_write_reg(REG_FIFO_ADDR_PTR,addr);

    uint8_t buf[256];
    for(int i=0;i<len;i++) buf[i]=lora_read_reg(REG_FIFO);

    lora_clear_irqs();

    if(len < sizeof(mr_hdr_v3_t)) return;

    mr_hdr_v3_t *h=(mr_hdr_v3_t*)buf;

    if(h->magic[0]!='M'||h->magic[1]!='R') return;
    if(h->version!=MR_PROTO_VERSION) return;

    if(seen_before(h->src,h->msg_id)) return;
    remember_msg(h->src,h->msg_id);

    int rssi = (int)lora_read_reg(REG_PKT_RSSI_VALUE) - 157;

    if(h->flags & MR_FLAG_BEACON){
        // Update neighbor + route (immer)
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        neighbor_update_locked(h->last_hop, rssi);
        route_update_locked(h->src, h->last_hop, rssi);
        xSemaphoreGive(g_mutex);

        char ssrc[8], shop[8];
        call7_to_cstr(ssrc, h->src);
        call7_to_cstr(shop, h->last_hop);

        ESP_LOGI(TAG,"BEACON RX from=%s via=%s rssi=%d ttl=%u", ssrc, shop, rssi, h->ttl);

        // 23A: optional forward
        maybe_forward_beacon(buf, len, rssi);
    }
}

// ============================================================================
// ISR / Task
// ============================================================================
static void IRAM_ATTR dio0_isr(void*arg)
{
    uint32_t n=(uint32_t)arg;
    xQueueSendFromISR(dio0_evt_queue,&n,NULL);
}

static void dio0_task(void*arg)
{
    uint32_t io;
    while(1){
        if(xQueueReceive(dio0_evt_queue,&io,portMAX_DELAY))
            handle_rx_packet();
    }
}

// ============================================================================
// SPI init
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
// WiFi AP
// ============================================================================
static void wifi_init_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap = {0};
    strncpy((char*)ap.ap.ssid, "MeshRadio-Setup", sizeof(ap.ap.ssid));
    ap.ap.channel = 1;
    ap.ap.max_connection = 4;
    ap.ap.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// ============================================================================
// HTTP helpers
// ============================================================================
static esp_err_t http_send_json(httpd_req_t *req, const char *json)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t http_send_text(httpd_req_t *req, const char *txt)
{
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, txt, HTTPD_RESP_USE_STRLEN);
}

// URL decode
static void url_decode_inplace(char *s)
{
    char *p=s, *o=s;
    while(*p){
        if(*p=='+'){ *o++=' '; p++; continue; }
        if(*p=='%' && p[1] && p[2]){
            char hex[3]={p[1],p[2],0};
            *o++ = (char)strtoul(hex,NULL,16);
            p+=3;
            continue;
        }
        *o++=*p++;
    }
    *o=0;
}

static bool form_get(char *body, const char *key, char *out, size_t out_sz)
{
    size_t klen=strlen(key);
    char *p=body;
    while(p && *p){
        char *amp=strchr(p,'&'); if(amp)*amp=0;
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
                if(amp)*amp='&';
                return true;
            }
            *eq='=';
        }
        if(amp){ *amp='&'; p=amp+1; } else break;
    }
    return false;
}

static bool parse_u32(const char *s, uint32_t *out)
{
    if(!s||!*s) return false;
    char *end=NULL;
    unsigned long v=strtoul(s,&end,10);
    if(end==s || *end!=0) return false;
    *out=(uint32_t)v;
    return true;
}

static bool parse_i32(const char *s, int *out)
{
    if(!s||!*s) return false;
    char *end=NULL;
    long v=strtol(s,&end,10);
    if(end==s || *end!=0) return false;
    *out=(int)v;
    return true;
}

static bool parse_u8_range(const char *s, uint8_t *out, uint8_t minv, uint8_t maxv)
{
    uint32_t tmp=0;
    if(!parse_u32(s,&tmp)) return false;
    if(tmp<minv||tmp>maxv) return false;
    *out=(uint8_t)tmp;
    return true;
}

static bool validate_call(const char *s)
{
    size_t n=s?strlen(s):0;
    return (n>=1 && n<=7);
}

static bool parse_bw(const char *s, uint8_t *out)
{
    uint32_t tmp=0;
    if(!parse_u32(s,&tmp)) return false;
    if(tmp!=125 && tmp!=250 && tmp!=500) return false;
    *out=(uint8_t)tmp;
    return true;
}

static bool http_read_body(httpd_req_t *req, char *buf, size_t buf_sz)
{
    if(req->content_len <= 0 || req->content_len >= (int)buf_sz) return false;
    int r=httpd_req_recv(req, buf, req->content_len);
    if(r<=0) return false;
    buf[r]=0;
    return true;
}

// ============================================================================
// Web UI (23A: Relay Controls)
// ============================================================================
static const char *INDEX_HTML =
"<!doctype html><html><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>MeshRadio Setup</title>"
"<style>"
"body{font-family:system-ui;margin:16px;max-width:900px}"
"input{width:100%;padding:10px;margin:6px 0;font-size:16px}"
"button{padding:10px 14px;margin:6px 4px;font-size:16px}"
".row{display:flex;gap:8px;flex-wrap:wrap}"
".row button{flex:1}"
"code{background:#eee;padding:2px 4px}"
"</style></head><body>"
"<h2>MeshRadio Setup (WLAN)</h2>"
"<p>Verbunden mit <code>MeshRadio-Setup</code>, öffne <code>http://192.168.4.1</code></p>"
"<div id='msg'></div>"

"<h3>LoRa Config</h3>"
"<label>Call</label><input id='call' placeholder='DJ2RF'>"
"<label>Frequenz (Hz)</label><input id='freq' placeholder='433775000'>"
"<label>SF (7..12)</label><input id='sf' placeholder='7'>"
"<label>BW (125/250/500)</label><input id='bw' placeholder='125'>"
"<label>CRC (0/1)</label><input id='crc' placeholder='1'>"
"<div class='row'>"
"<button onclick='doShow()'>Show</button>"
"<button onclick='doSet()'>Set (RAM)</button>"
"<button onclick='doApply()'>Apply</button>"
"</div><div class='row'>"
"<button onclick='doSave()'>Save (NVS)</button>"
"<button onclick='doLoad()'>Load+Apply</button>"
"<button onclick='doReboot()'>Reboot</button>"
"</div>"

"<h3>Beacon</h3>"
"<label>Beacon enabled (0/1)</label><input id='beacon' placeholder='1'>"
"<label>Beacon Interval (ms)</label><input id='beaconint' placeholder='15000'>"
"<div class='row'>"
"<button onclick='doBeaconSet()'>Set Beacon</button>"
"<button onclick='doBeaconInt()'>Set Interval</button>"
"</div>"

"<h3>Relay Filter (Kapitel 23A)</h3>"
"<label>Relay enabled (0/1)</label><input id='relay' placeholder='1'>"
"<label>Relay RSSI threshold (dBm, z.B. -95)</label><input id='relaythr' placeholder='-95'>"
"<div class='row'>"
"<button onclick='doRelaySet()'>Set Relay</button>"
"<button onclick='doRelayThr()'>Set Threshold</button>"
"</div>"

"<script>"
"function q(id){return document.getElementById(id)}"
"function msg(t){q('msg').innerHTML='<p><b>'+t+'</b></p>'}"
"async function doShow(){"
" let r=await fetch('/api/show'); let j=await r.json();"
" q('call').value=j.call; q('freq').value=j.freq; q('sf').value=j.sf;"
" q('bw').value=j.bw; q('crc').value=j.crc;"
" q('beacon').value=j.beacon_enabled; q('beaconint').value=j.beacon_interval;"
" let rr=await fetch('/api/relay'); let rj=await rr.json();"
" q('relay').value=rj.enabled; q('relaythr').value=rj.threshold;"
" msg('OK: show');"
"}"
"function body_cfg(){"
" return 'call='+encodeURIComponent(q('call').value)"
"+'&freq='+encodeURIComponent(q('freq').value)"
"+'&sf='+encodeURIComponent(q('sf').value)"
"+'&bw='+encodeURIComponent(q('bw').value)"
"+'&crc='+encodeURIComponent(q('crc').value);"
"}"
"async function post(url, body){"
" let r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:body});"
" let t=await r.text(); msg(t);"
"}"
"async function doSet(){ await post('/api/set', body_cfg()); }"
"async function doApply(){ await post('/api/apply', body_cfg()); }"
"async function doSave(){ await post('/api/save', body_cfg()); }"
"async function doLoad(){ let r=await fetch('/api/load',{method:'POST'}); msg(await r.text()); await doShow(); }"
"async function doReboot(){ await fetch('/api/reboot',{method:'POST'}); msg('Reboot...'); }"
"async function doBeaconSet(){ await post('/api/beacon', 'enabled='+encodeURIComponent(q('beacon').value)); }"
"async function doBeaconInt(){ await post('/api/beaconint', 'ms='+encodeURIComponent(q('beaconint').value)); }"
"async function doRelaySet(){ await post('/api/relay', 'enabled='+encodeURIComponent(q('relay').value)); }"
"async function doRelayThr(){ await post('/api/relaythr', 'rssi='+encodeURIComponent(q('relaythr').value)); }"
"doShow();"
"</script></body></html>";

static esp_err_t http_index_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

// ============================================================================
// /api/show (config + beacon)
// ============================================================================
static esp_err_t http_api_show(httpd_req_t *req)
{
    char json[320];

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    snprintf(json,sizeof(json),
             "{"
             "\"call\":\"%s\","
             "\"freq\":%" PRIu32 ","
             "\"sf\":%u,"
             "\"bw\":%u,"
             "\"crc\":%u,"
             "\"beacon_enabled\":%u,"
             "\"beacon_interval\":%" PRIu32
             "}",
             g_cfg.call, g_cfg.freq_hz, g_cfg.sf, g_cfg.bw_khz, g_cfg.crc_on,
             g_beacon_enabled?1:0,
             g_beacon_interval_ms);
    xSemaphoreGive(g_mutex);

    return http_send_json(req,json);
}

// ============================================================================
// 23A: /api/relay (GET) -> status JSON
// ============================================================================
static esp_err_t http_api_relay_get(httpd_req_t *req)
{
    char json[128];
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    snprintf(json,sizeof(json),
             "{\"enabled\":%u,\"threshold\":%d}",
             g_relay_enabled?1:0, g_relay_rssi_threshold_dbm);
    xSemaphoreGive(g_mutex);
    return http_send_json(req,json);
}

// ============================================================================
// 23A: /api/relay (POST) body: enabled=0|1
// ============================================================================
static esp_err_t http_api_relay_post(httpd_req_t *req)
{
    char body[128];
    if(!http_read_body(req, body, sizeof(body)))
        return http_send_text(req,"ERR: body too large/empty");

    char v_en[8]={0};
    if(!form_get(body,"enabled",v_en,sizeof(v_en)))
        return http_send_text(req,"ERR: missing enabled");

    uint8_t en=0;
    if(!parse_u8_range(v_en,&en,0,1))
        return http_send_text(req,"ERR: enabled must be 0 or 1");

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_relay_enabled = (en==1);
    xSemaphoreGive(g_mutex);

    return http_send_text(req, g_relay_enabled ? "OK: relay enabled=1" : "OK: relay enabled=0");
}

// ============================================================================
// 23A: /api/relaythr (POST) body: rssi=-120..-30
// ============================================================================
static esp_err_t http_api_relaythr_post(httpd_req_t *req)
{
    char body[128];
    if(!http_read_body(req, body, sizeof(body)))
        return http_send_text(req,"ERR: body too large/empty");

    char v_rssi[16]={0};
    if(!form_get(body,"rssi",v_rssi,sizeof(v_rssi)))
        return http_send_text(req,"ERR: missing rssi");

    int rssi=0;
    if(!parse_i32(v_rssi,&rssi) || rssi < -120 || rssi > -30)
        return http_send_text(req,"ERR: rssi must be -120..-30");

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_relay_rssi_threshold_dbm = rssi;
    xSemaphoreGive(g_mutex);

    return http_send_text(req,"OK: relay threshold updated");
}

// ============================================================================
// /api/set (RAM only) – wie Kapitel 23 (kompakt)
// ============================================================================
static esp_err_t http_api_set(httpd_req_t *req)
{
    char body[256];
    if(!http_read_body(req, body, sizeof(body)))
        return http_send_text(req, "ERR: body too large/empty");

    char v_call[16]={0}, v_freq[32]={0}, v_sf[8]={0}, v_bw[8]={0}, v_crc[8]={0};
    bool has_call = form_get(body, "call", v_call, sizeof(v_call));
    bool has_freq = form_get(body, "freq", v_freq, sizeof(v_freq));
    bool has_sf   = form_get(body, "sf",   v_sf,   sizeof(v_sf));
    bool has_bw   = form_get(body, "bw",   v_bw,   sizeof(v_bw));
    bool has_crc  = form_get(body, "crc",  v_crc,  sizeof(v_crc));

    xSemaphoreTake(g_mutex, portMAX_DELAY);

    if(has_call){
        if(!validate_call(v_call)){ xSemaphoreGive(g_mutex); return http_send_text(req,"ERR: call (1..7)"); }
        memset(g_cfg.call,0,sizeof(g_cfg.call));
        strncpy(g_cfg.call, v_call, sizeof(g_cfg.call)-1);
    }
    if(has_freq){
        uint32_t f=0;
        if(!parse_u32(v_freq,&f) || f<100000000 || f>1000000000){
            xSemaphoreGive(g_mutex); return http_send_text(req,"ERR: freq invalid");
        }
        g_cfg.freq_hz=f;
    }
    if(has_sf){
        uint8_t sf=0;
        if(!parse_u8_range(v_sf,&sf,7,12)){
            xSemaphoreGive(g_mutex); return http_send_text(req,"ERR: sf 7..12");
        }
        g_cfg.sf=sf;
    }
    if(has_bw){
        uint8_t bw=0;
        if(!parse_bw(v_bw,&bw)){
            xSemaphoreGive(g_mutex); return http_send_text(req,"ERR: bw 125|250|500");
        }
        g_cfg.bw_khz=bw;
    }
    if(has_crc){
        uint8_t c=0;
        if(!parse_u8_range(v_crc,&c,0,1)){
            xSemaphoreGive(g_mutex); return http_send_text(req,"ERR: crc 0|1");
        }
        g_cfg.crc_on=c;
    }

    xSemaphoreGive(g_mutex);
    return http_send_text(req,"OK: set (RAM)");
}

static esp_err_t http_api_apply(httpd_req_t *req)
{
    (void)req;
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    lora_apply_config_locked();
    xSemaphoreGive(g_mutex);
    return http_send_text(req,"OK: applied");
}

static esp_err_t http_api_save(httpd_req_t *req)
{
    (void)req;
    mr_config_t tmp;
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    tmp=g_cfg;
    xSemaphoreGive(g_mutex);

    esp_err_t e=config_save(&tmp);
    if(e==ESP_OK) return http_send_text(req,"OK: saved (NVS)");
    char msg[96]; snprintf(msg,sizeof(msg),"ERR: save failed: %s", esp_err_to_name(e));
    return http_send_text(req,msg);
}

static esp_err_t http_api_load(httpd_req_t *req)
{
    (void)req;
    mr_config_t tmp;
    esp_err_t e=config_load_or_defaults(&tmp);
    if(e!=ESP_OK){
        char msg[96]; snprintf(msg,sizeof(msg),"ERR: load failed: %s", esp_err_to_name(e));
        return http_send_text(req,msg);
    }

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_cfg=tmp;
    lora_apply_config_locked();
    xSemaphoreGive(g_mutex);
    return http_send_text(req,"OK: loaded (NVS) + applied");
}

static esp_err_t http_api_reboot(httpd_req_t *req)
{
    http_send_text(req,"OK: rebooting");
    vTaskDelay(pdMS_TO_TICKS(150));
    esp_restart();
    return ESP_OK;
}

static esp_err_t http_api_beacon(httpd_req_t *req)
{
    char body[128];
    if(!http_read_body(req, body, sizeof(body)))
        return http_send_text(req, "ERR: body too large/empty");

    char v_en[8]={0};
    if(!form_get(body, "enabled", v_en, sizeof(v_en)))
        return http_send_text(req, "ERR: missing enabled");

    uint8_t en=0;
    if(!parse_u8_range(v_en, &en, 0, 1))
        return http_send_text(req, "ERR: enabled must be 0 or 1");

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_beacon_enabled = (en==1);
    if(g_beacon_enabled) beacon_schedule_next_locked();
    xSemaphoreGive(g_mutex);

    return http_send_text(req, g_beacon_enabled ? "OK: beacon enabled=1" : "OK: beacon enabled=0");
}

static esp_err_t http_api_beaconint(httpd_req_t *req)
{
    char body[128];
    if(!http_read_body(req, body, sizeof(body)))
        return http_send_text(req, "ERR: body too large/empty");

    char v_ms[16]={0};
    if(!form_get(body, "ms", v_ms, sizeof(v_ms)))
        return http_send_text(req, "ERR: missing ms");

    uint32_t ms=0;
    if(!parse_u32(v_ms, &ms) || ms < 1000 || ms > 600000)
        return http_send_text(req, "ERR: ms must be 1000..600000");

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_beacon_interval_ms = ms;
    if(g_beacon_enabled) beacon_schedule_next_locked();
    xSemaphoreGive(g_mutex);

    return http_send_text(req, "OK: beacon interval updated");
}

// ============================================================================
// HTTP Server
// ============================================================================
static void http_server_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 8192;
    cfg.max_uri_handlers = 24;

    ESP_ERROR_CHECK(httpd_start(&g_http, &cfg));

    httpd_uri_t u0={.uri="/", .method=HTTP_GET, .handler=http_index_get};
    httpd_register_uri_handler(g_http,&u0);

    httpd_uri_t u1={.uri="/api/show", .method=HTTP_GET, .handler=http_api_show};
    httpd_register_uri_handler(g_http,&u1);

    httpd_uri_t u2={.uri="/api/set", .method=HTTP_POST, .handler=http_api_set};
    httpd_register_uri_handler(g_http,&u2);

    httpd_uri_t u3={.uri="/api/apply", .method=HTTP_POST, .handler=http_api_apply};
    httpd_register_uri_handler(g_http,&u3);

    httpd_uri_t u4={.uri="/api/save", .method=HTTP_POST, .handler=http_api_save};
    httpd_register_uri_handler(g_http,&u4);

    httpd_uri_t u5={.uri="/api/load", .method=HTTP_POST, .handler=http_api_load};
    httpd_register_uri_handler(g_http,&u5);

    httpd_uri_t u6={.uri="/api/reboot", .method=HTTP_POST, .handler=http_api_reboot};
    httpd_register_uri_handler(g_http,&u6);

    httpd_uri_t u7={.uri="/api/beacon", .method=HTTP_POST, .handler=http_api_beacon};
    httpd_register_uri_handler(g_http,&u7);

    httpd_uri_t u8={.uri="/api/beaconint", .method=HTTP_POST, .handler=http_api_beaconint};
    httpd_register_uri_handler(g_http,&u8);

    // 23A relay endpoints
    httpd_uri_t r0={.uri="/api/relay", .method=HTTP_GET, .handler=http_api_relay_get};
    httpd_register_uri_handler(g_http,&r0);

    httpd_uri_t r1={.uri="/api/relay", .method=HTTP_POST, .handler=http_api_relay_post};
    httpd_register_uri_handler(g_http,&r1);

    httpd_uri_t r2={.uri="/api/relaythr", .method=HTTP_POST, .handler=http_api_relaythr_post};
    httpd_register_uri_handler(g_http,&r2);

    ESP_LOGI(TAG,"HTTP server started");
}

// ============================================================================
// MAIN
// ============================================================================
void app_main(void)
{
    ESP_LOGI(TAG,"MeshRadio Kapitel 23A start (Neighbor Relay Filter)");

    g_mutex = xSemaphoreCreateMutex();
    if(!g_mutex){ ESP_LOGE(TAG,"mutex create failed"); abort(); }

    // NVS init
    esp_err_t err=nvs_flash_init();
    if(err==ESP_ERR_NVS_NO_FREE_PAGES || err==ESP_ERR_NVS_NEW_VERSION_FOUND){
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }else{
        ESP_ERROR_CHECK(err);
    }

    // Load config
    ESP_ERROR_CHECK(config_load_or_defaults(&g_cfg));
    ESP_LOGI(TAG,"CFG: call=%s freq=%" PRIu32 " sf=%u bw=%u crc=%u",
             g_cfg.call, g_cfg.freq_hz, g_cfg.sf, g_cfg.bw_khz, g_cfg.crc_on);

    // SPI + Reset + Apply
    init_spi();
    lora_reset();
    ESP_LOGI(TAG,"SX1276 RegVersion=0x%02X", lora_read_reg(REG_VERSION));

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    lora_apply_config_locked();
    xSemaphoreGive(g_mutex);

    // DIO0 IRQ
    dio0_evt_queue=xQueueCreate(10,sizeof(uint32_t));

    gpio_config_t io={
        .intr_type=GPIO_INTR_POSEDGE,
        .mode=GPIO_MODE_INPUT,
        .pin_bit_mask=(1ULL<<PIN_NUM_DIO0),
        .pull_up_en=1
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_NUM_DIO0,dio0_isr,(void*)PIN_NUM_DIO0));
    xTaskCreate(dio0_task,"dio0",4096,NULL,10,NULL);

    // WiFi + HTTP
    wifi_init_ap();
    http_server_start();

    // Beacon schedule
    beacon_schedule_next();

    ESP_LOGI(TAG,"Open browser: http://192.168.4.1/");
    ESP_LOGI(TAG,"Relay filter default: enabled=1 threshold=%d dBm", g_relay_rssi_threshold_dbm);

    // Main loop
    while(1){
        vTaskDelay(pdMS_TO_TICKS(50));

        xSemaphoreTake(g_mutex, portMAX_DELAY);
        bool do_beacon = g_beacon_enabled && (now_ms() > next_beacon_ms);

        // housekeeping
        neighbor_cleanup_locked();
        route_cleanup_locked();

        xSemaphoreGive(g_mutex);

        if(do_beacon) send_beacon();
    }
}
