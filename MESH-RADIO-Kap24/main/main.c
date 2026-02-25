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
// MeshRadio – Kapitel 24 (Sourcen komplett, FINAL korrigiert)
// ----------------------------------------------------------------------------
// Fixes gegenüber vorher:
//   - dio_isr() / dio_task() waren doppelt -> jetzt nur 1x vorhanden.
//   - Web-UI ruft korrekt POST /api/smart mit Form-Body auf (kein 404).
//   - GET /api/relay liefert Status JSON.
//
// Kapitel 24 Features:
//   - Smart Relay: best path pro SRC + Forward Rate Limit + Neighbor threshold
//   - DATA + BEACON RX/TX (Demo)
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

#define MR_FLAG_DATA   0x10
#define MR_FLAG_BEACON 0x20

#define DATA_TTL   3
#define BEACON_TTL 2

#define MAX_PAYLOAD 120

// ============================================================================
// LIMITS / TABLES
// ============================================================================
#define SEEN_CACHE_SIZE 32

#define MAX_NEIGHBORS 20
#define NEIGHBOR_TIMEOUT_MS 60000

#define MAX_ROUTES 20
#define ROUTE_TIMEOUT_MS 120000

// Smart Relay: best route per SOURCE (SRC)
#define MAX_BESTSRC 24

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
typedef struct { bool used; char dst[7]; char next[7]; int rssi; uint32_t t_ms; } route_t;

typedef struct {
    bool used;
    char src[7];

    char best_next[7];
    int  best_rssi;
    uint32_t last_update_ms;

    uint32_t last_forward_ms;
} bestsrc_t;

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
static const char *TAG="MR24";

static spi_device_handle_t lora_spi;
static QueueHandle_t dio_q;
static SemaphoreHandle_t g_mutex;
static httpd_handle_t g_http=NULL;

static cfg_t g_cfg;
static uint16_t g_msg_id=1;

static seen_t seen_cache[SEEN_CACHE_SIZE];
static neighbor_t neighbors[MAX_NEIGHBORS];
static route_t routes[MAX_ROUTES];
static bestsrc_t bestsrc[MAX_BESTSRC];

// Beacon
static bool g_beacon_enabled=true;
static uint32_t g_beacon_interval_ms=15000;
static uint32_t g_next_beacon_ms=0;

// Relay Settings
static bool g_relay_enabled=true;
static int  g_relay_thr_dbm=-95;
static bool g_smart_relay=true;
static uint32_t g_forward_limit_ms=800;

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

static bool neighbor_good_locked(const char call[7], int thr_dbm)
{
    uint32_t t=now_ms();
    for(int i=0;i<MAX_NEIGHBORS;i++){
        if(!neighbors[i].used) continue;
        if(!call7_eq(neighbors[i].call,call)) continue;
        if(t - neighbors[i].t_ms > NEIGHBOR_TIMEOUT_MS) return false;
        return neighbors[i].rssi >= thr_dbm;
    }
    return false;
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
// ROUTES
// ============================================================================
static void route_update_locked(const char dst[7], const char hop[7], int rssi)
{
    uint32_t t=now_ms();
    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used) continue;
        if(call7_eq(routes[i].dst,dst)){
            memcpy(routes[i].next,hop,7);
            routes[i].rssi=rssi;
            routes[i].t_ms=t;
            return;
        }
    }
    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used){
            routes[i].used=true;
            memcpy(routes[i].dst,dst,7);
            memcpy(routes[i].next,hop,7);
            routes[i].rssi=rssi;
            routes[i].t_ms=t;
            return;
        }
    }
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
// BEST ROUTE PER SRC (Kapitel 24)
// ============================================================================
static bestsrc_t* bestsrc_get_or_alloc_locked(const char src[7])
{
    for(int i=0;i<MAX_BESTSRC;i++){
        if(bestsrc[i].used && call7_eq(bestsrc[i].src,src))
            return &bestsrc[i];
    }
    for(int i=0;i<MAX_BESTSRC;i++){
        if(!bestsrc[i].used){
            bestsrc[i].used=true;
            memcpy(bestsrc[i].src,src,7);
            call7_set(bestsrc[i].best_next,"*");
            bestsrc[i].best_rssi=-200;
            bestsrc[i].last_update_ms=now_ms();
            bestsrc[i].last_forward_ms=0;
            return &bestsrc[i];
        }
    }
    // overwrite oldest
    int oldest=0;
    uint32_t ot=bestsrc[0].last_update_ms;
    for(int i=1;i<MAX_BESTSRC;i++){
        if(bestsrc[i].last_update_ms < ot){
            oldest=i; ot=bestsrc[i].last_update_ms;
        }
    }
    bestsrc[oldest].used=true;
    memcpy(bestsrc[oldest].src,src,7);
    call7_set(bestsrc[oldest].best_next,"*");
    bestsrc[oldest].best_rssi=-200;
    bestsrc[oldest].last_update_ms=now_ms();
    bestsrc[oldest].last_forward_ms=0;
    return &bestsrc[oldest];
}

static void bestsrc_update_locked(const char src[7], const char last_hop[7], int rssi)
{
    bestsrc_t *b = bestsrc_get_or_alloc_locked(src);
    b->last_update_ms = now_ms();

    if(rssi > b->best_rssi){
        b->best_rssi = rssi;
        memcpy(b->best_next, last_hop, 7);

        char ssrc[8], shop[8];
        call7_to_str(ssrc, src);
        call7_to_str(shop, last_hop);
        ESP_LOGI(TAG,"SMART: best route src=%s via=%s rssi=%d", ssrc, shop, rssi);
    }
}

static bool smart_should_forward_locked(const mr_hdr_v3_t *h, int rssi_last_hop)
{
    if(!g_relay_enabled) return false;
    if(h->ttl <= 1) return false;

    if(!neighbor_good_locked(h->last_hop, g_relay_thr_dbm)) return false;

    if(!g_smart_relay) return true;

    bestsrc_t *b = bestsrc_get_or_alloc_locked(h->src);

    uint32_t t=now_ms();
    if(g_forward_limit_ms > 0 && (t - b->last_forward_ms) < g_forward_limit_ms)
        return false;

    bool improves = (rssi_last_hop > b->best_rssi);
    bool is_best  = call7_eq(h->last_hop, b->best_next);

    if(!(improves || is_best)) return false;

    b->last_forward_ms = t;
    return true;
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
// Apply Config (minimal demo defaults)
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
// Beacon + Data TX (Demo)
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

    ESP_LOGI(TAG,"TX BEACON id=%u",h.msg_id);
    lora_send((uint8_t*)&h,sizeof(h));
    beacon_schedule_next();
}

static void send_data_text(const char *txt)
{
    uint8_t buf[256]={0};
    mr_hdr_v3_t *h=(mr_hdr_v3_t*)buf;

    h->magic[0]='M'; h->magic[1]='R';
    h->version=MR_PROTO_VERSION;
    h->flags=MR_FLAG_DATA;
    h->ttl=DATA_TTL;
    h->msg_id=g_msg_id++;

    xSemaphoreTake(g_mutex,portMAX_DELAY);
    call7_set(h->src,g_cfg.call);
    call7_set(h->last_hop,g_cfg.call);
    xSemaphoreGive(g_mutex);

    call7_set(h->final_dst,"*");
    call7_set(h->next_hop,"*");

    size_t n=strlen(txt);
    if(n>MAX_PAYLOAD) n=MAX_PAYLOAD;
    h->payload_len=(uint8_t)n;
    memcpy(buf+sizeof(mr_hdr_v3_t),txt,n);

    ESP_LOGI(TAG,"TX DATA id=%u text=%s",h->msg_id,txt);
    lora_send(buf,sizeof(mr_hdr_v3_t)+n);
}

// ============================================================================
// Forward
// ============================================================================
static void forward_packet(uint8_t *buf,size_t len)
{
    if(len<sizeof(mr_hdr_v3_t)) return;
    mr_hdr_v3_t *h=(mr_hdr_v3_t*)buf;

    h->ttl--;

    char my[8]={0};
    xSemaphoreTake(g_mutex,portMAX_DELAY);
    strncpy(my,g_cfg.call,sizeof(my)-1);
    xSemaphoreGive(g_mutex);

    call7_set(h->last_hop,my);
    call7_set(h->next_hop,"*");

    ESP_LOGI(TAG,"FWD flags=0x%02X id=%u ttl=%u",h->flags,h->msg_id,h->ttl);
    lora_send(buf,len);
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

    if(len<sizeof(mr_hdr_v3_t)) return;

    mr_hdr_v3_t *h=(mr_hdr_v3_t*)buf;

    if(h->magic[0]!='M'||h->magic[1]!='R') return;
    if(h->version!=MR_PROTO_VERSION) return;

    if(seen_before(h->src,h->msg_id)) return;
    remember_msg(h->src,h->msg_id);

    int rssi=(int)lora_rd(REG_PKT_RSSI_VALUE)-157;

    // update tables
    xSemaphoreTake(g_mutex,portMAX_DELAY);
    neighbor_update_locked(h->last_hop,rssi);
    route_update_locked(h->src,h->last_hop,rssi);
    bestsrc_update_locked(h->src,h->last_hop,rssi);
    xSemaphoreGive(g_mutex);

    char src[8]; call7_to_str(src,h->src);

    if(h->flags & MR_FLAG_BEACON){
        ESP_LOGI(TAG,"RX BEACON src=%s rssi=%d ttl=%u",src,rssi,h->ttl);
    }
    if(h->flags & MR_FLAG_DATA){
        char txt[MAX_PAYLOAD+1]={0};
        if(h->payload_len>0 && h->payload_len<=MAX_PAYLOAD){
            memcpy(txt,buf+sizeof(mr_hdr_v3_t),h->payload_len);
            txt[h->payload_len]=0;
        }
        ESP_LOGI(TAG,"RX DATA src=%s txt=\"%s\" rssi=%d ttl=%u",src,txt,rssi,h->ttl);
    }

    bool do_fwd=false;
    xSemaphoreTake(g_mutex,portMAX_DELAY);
    do_fwd = smart_should_forward_locked(h, rssi);
    xSemaphoreGive(g_mutex);

    if(do_fwd) forward_packet(buf,len);
}

// ============================================================================
// DIO0 ISR + Task (NUR EINMAL!)
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
// HTTP helpers
// ============================================================================
static esp_err_t http_send_text(httpd_req_t *req, const char *txt)
{
    httpd_resp_set_type(req,"text/plain");
    return httpd_resp_send(req,txt,HTTPD_RESP_USE_STRLEN);
}

static esp_err_t http_send_json(httpd_req_t *req, const char *json)
{
    httpd_resp_set_type(req,"application/json");
    httpd_resp_set_hdr(req,"Cache-Control","no-store");
    return httpd_resp_send(req,json,HTTPD_RESP_USE_STRLEN);
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
        char *amp=strchr(p,'&'); if(amp)*amp=0;
        char *eq=strchr(p,'=');
        if(eq){
            *eq=0;
            char *k=p,*v=eq+1;
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
        if(amp){*amp='&';p=amp+1;} else break;
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

// ============================================================================
// HTTP UI + Endpoints
// ============================================================================
static const char *INDEX_HTML =
"<!doctype html><html><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>MeshRadio Kapitel 24</title>"
"<style>"
"body{font-family:system-ui;margin:16px;max-width:900px}"
"button{padding:10px 14px;margin:6px 4px;font-size:16px}"
"input{padding:10px;margin:6px 0;font-size:16px;width:100%}"
"code{background:#eee;padding:2px 4px}"
"</style></head><body>"
"<h2>MeshRadio Kapitel 24 – Smart Relay</h2>"
"<p>Verbunden mit <code>MeshRadio-Setup</code>, öffne <code>http://192.168.4.1</code></p>"
"<div id='m'></div>"
"<p><button onclick='tx()'>Send DATA</button></p>"
"<h3>Smart Relay</h3>"
"<p><button onclick='setSmart(1)'>Smart ON</button>"
"<button onclick='setSmart(0)'>Smart OFF</button></p>"
"<h3>Forward-Limit (ms)</h3>"
"<input id='fwd' value='800'>"
"<button onclick='setFwd()'>Set fwdlimit</button>"
"<h3>Relay threshold (dBm)</h3>"
"<input id='thr' value='-95'>"
"<button onclick='setThr()'>Set threshold</button>"
"<p><button onclick='refresh()'>Status lesen</button></p>"
"<pre id='st'></pre>"
"<script>"
"function m(t){document.getElementById('m').innerHTML='<b>'+t+'</b>'}"
"async function post(url,body){"
" let r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:body});"
" let t=await r.text(); m(t); }"
"async function tx(){ await fetch('/tx',{method:'POST'}); m('OK tx'); }"
"async function setSmart(v){ await post('/api/smart','smart='+encodeURIComponent(v)); await refresh(); }"
"async function setFwd(){ let v=document.getElementById('fwd').value; await post('/api/fwdlimit','ms='+encodeURIComponent(v)); await refresh(); }"
"async function setThr(){ let v=document.getElementById('thr').value; await post('/api/relaythr','rssi='+encodeURIComponent(v)); await refresh(); }"
"async function refresh(){ let r=await fetch('/api/relay'); document.getElementById('st').textContent=await r.text(); }"
"refresh();"
"</script></body></html>";

// GET /
static esp_err_t index_get(httpd_req_t *req)
{
    httpd_resp_set_type(req,"text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

// POST /tx
static esp_err_t tx_post(httpd_req_t *req)
{
    (void)req;
    send_data_text("Hallo Smart Mesh");
    return http_send_text(req,"OK: tx");
}

// GET /api/relay -> status JSON
static esp_err_t api_relay_get(httpd_req_t *req)
{
    char json[192];
    xSemaphoreTake(g_mutex,portMAX_DELAY);
    snprintf(json,sizeof(json),
             "{"
             "\"relay\":%u,"
             "\"thr\":%d,"
             "\"smart\":%u,"
             "\"fwdlimit\":%" PRIu32
             "}",
             g_relay_enabled?1:0,
             g_relay_thr_dbm,
             g_smart_relay?1:0,
             g_forward_limit_ms);
    xSemaphoreGive(g_mutex);
    return http_send_json(req,json);
}

// POST /api/smart body: smart=0|1
static esp_err_t api_smart_post(httpd_req_t *req)
{
    char body[64];
    if(!http_read_body(req,body,sizeof(body))) return http_send_text(req,"ERR: body");

    char v[8]={0};
    if(!form_get(body,"smart",v,sizeof(v))) return http_send_text(req,"ERR: missing smart");

    uint32_t on=0;
    if(!parse_u32(v,&on) || (on!=0 && on!=1)) return http_send_text(req,"ERR: smart 0|1");

    xSemaphoreTake(g_mutex,portMAX_DELAY);
    g_smart_relay = (on==1);
    xSemaphoreGive(g_mutex);

    return http_send_text(req, g_smart_relay ? "OK: smart=1" : "OK: smart=0");
}

// POST /api/fwdlimit body: ms=0..60000
static esp_err_t api_fwdlimit_post(httpd_req_t *req)
{
    char body[64];
    if(!http_read_body(req,body,sizeof(body))) return http_send_text(req,"ERR: body");

    char v[16]={0};
    if(!form_get(body,"ms",v,sizeof(v))) return http_send_text(req,"ERR: missing ms");

    uint32_t ms=0;
    if(!parse_u32(v,&ms) || ms>60000) return http_send_text(req,"ERR: ms 0..60000");

    xSemaphoreTake(g_mutex,portMAX_DELAY);
    g_forward_limit_ms = ms;
    xSemaphoreGive(g_mutex);

    return http_send_text(req,"OK: fwdlimit updated");
}

// POST /api/relaythr body rssi=-120..-30
static esp_err_t api_relaythr_post(httpd_req_t *req)
{
    char body[64];
    if(!http_read_body(req,body,sizeof(body))) return http_send_text(req,"ERR: body");

    char v[16]={0};
    if(!form_get(body,"rssi",v,sizeof(v))) return http_send_text(req,"ERR: missing rssi");

    int r=0;
    if(!parse_i32(v,&r) || r<-120 || r>-30) return http_send_text(req,"ERR: rssi -120..-30");

    xSemaphoreTake(g_mutex,portMAX_DELAY);
    g_relay_thr_dbm = r;
    xSemaphoreGive(g_mutex);

    return http_send_text(req,"OK: relaythr updated");
}

static void http_start(void)
{
    httpd_config_t cfg=HTTPD_DEFAULT_CONFIG();
    cfg.stack_size=8192;
    cfg.max_uri_handlers=12;

    ESP_ERROR_CHECK(httpd_start(&g_http,&cfg));

    httpd_uri_t u0={.uri="/",.method=HTTP_GET,.handler=index_get};
    httpd_register_uri_handler(g_http,&u0);

    httpd_uri_t u1={.uri="/tx",.method=HTTP_POST,.handler=tx_post};
    httpd_register_uri_handler(g_http,&u1);

    httpd_uri_t a0={.uri="/api/relay",.method=HTTP_GET,.handler=api_relay_get};
    httpd_register_uri_handler(g_http,&a0);

    httpd_uri_t a1={.uri="/api/smart",.method=HTTP_POST,.handler=api_smart_post};
    httpd_register_uri_handler(g_http,&a1);

    httpd_uri_t a2={.uri="/api/fwdlimit",.method=HTTP_POST,.handler=api_fwdlimit_post};
    httpd_register_uri_handler(g_http,&a2);

    httpd_uri_t a3={.uri="/api/relaythr",.method=HTTP_POST,.handler=api_relaythr_post};
    httpd_register_uri_handler(g_http,&a3);

    ESP_LOGI(TAG,"HTTP server started");
}

// ============================================================================
// MAIN
// ============================================================================
void app_main(void)
{
    ESP_LOGI(TAG,"MeshRadio Kapitel 24 start (Smart Relay)");

    g_mutex=xSemaphoreCreateMutex();
    if(!g_mutex){ ESP_LOGE(TAG,"mutex failed"); abort(); }

    ESP_ERROR_CHECK(nvs_flash_init());

    // Demo defaults (fürs Buch okay)
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

    // DIO0 setup
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

    // Beacon schedule
    beacon_schedule_next();

    ESP_LOGI(TAG,"Open http://192.168.4.1");
    ESP_LOGI(TAG,"Smart=%u thr=%d fwdlimit=%" PRIu32 "ms",
             g_smart_relay?1:0, g_relay_thr_dbm, g_forward_limit_ms);

    while(1){
        vTaskDelay(pdMS_TO_TICKS(50));

        if(g_beacon_enabled && now_ms()>g_next_beacon_ms)
            send_beacon();

        xSemaphoreTake(g_mutex,portMAX_DELAY);
        neighbor_cleanup_locked();
        route_cleanup_locked();
        xSemaphoreGive(g_mutex);
    }
}
