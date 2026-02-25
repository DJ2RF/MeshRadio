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
// MeshRadio – Kapitel 23B (Sourcen komplett, sauber dokumentiert)
// ----------------------------------------------------------------------------
// Basis: Kapitel 23A (Neighbor-basierter Relay Filter)
//
// NEU in Kapitel 23B:
//   ✔ Relay funktioniert jetzt NICHT nur für Beacons,
//     sondern auch für DATA / CHAT Frames.
//
//   ✔ Neue Flags:
//        MR_FLAG_DATA
//
//   ✔ Einfaches Text-Payload Beispiel
//        -> Frames können Text enthalten
//
//   ✔ Forward-Logik:
//        - BEACON: wie bisher
//        - DATA:   ebenfalls über Neighbor-Filter
//
//   ✔ RX-Ausgabe:
//        DATA RX src=... text="..."
//
// Ziel:
//   → Jetzt beginnt echtes Mesh-Verhalten.
//
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

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"

// ============================================================================
// PROTOKOLL
// ============================================================================

#define MR_PROTO_VERSION 3

#define MR_FLAG_BEACON 0x20
#define MR_FLAG_DATA   0x10   // Kapitel 23B

#define BEACON_TTL 2
#define DATA_TTL   3

#define DEFAULT_BEACON_INTERVAL_MS 15000
#define DEFAULT_BEACON_JITTER_MS   2000

// ============================================================================
// LIMITS
// ============================================================================

#define MAX_PAYLOAD         120
#define SEEN_CACHE_SIZE     32
#define MAX_NEIGHBORS       20
#define MAX_ROUTES          20

#define NEIGHBOR_TIMEOUT_MS 60000
#define ROUTE_TIMEOUT_MS    120000

// ============================================================================
// LORA PINS
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

typedef struct {
    bool used;
    char src[7];
    uint16_t msg_id;
} seen_msg_t;

typedef struct {
    bool used;
    char call[7];
    int rssi;
    uint32_t t_ms;
} neighbor_t;

typedef struct {
    bool used;
    char dst[7];
    char next[7];
    int rssi;
    uint32_t t_ms;
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

static const char *TAG="MR23B";

static spi_device_handle_t lora_spi;
static QueueHandle_t dio_q;

static SemaphoreHandle_t g_mutex;

static cfg_t g_cfg;
static uint16_t g_msg_id=1;

static bool g_beacon_enabled=true;
static uint32_t g_next_beacon_ms=0;
static uint32_t g_beacon_interval=DEFAULT_BEACON_INTERVAL_MS;

static bool g_relay_enabled=true;
static int  g_relay_thr=-95;

static seen_msg_t seen[SEEN_CACHE_SIZE];
static neighbor_t neighbors[MAX_NEIGHBORS];
static route_t routes[MAX_ROUTES];

// ============================================================================
// HELPERS
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

static bool call7_eq(const char a[7], const char b[7])
{
    return memcmp(a,b,7)==0;
}

static uint32_t hz_to_frf(uint32_t f)
{
    uint64_t frf=((uint64_t)f<<19)/32000000ULL;
    return (uint32_t)frf;
}

// ============================================================================
// NEIGHBOR
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

static bool neighbor_good_locked(const char call[7])
{
    uint32_t t=now_ms();
    for(int i=0;i<MAX_NEIGHBORS;i++){
        if(!neighbors[i].used) continue;
        if(!call7_eq(neighbors[i].call,call)) continue;

        if(t - neighbors[i].t_ms > NEIGHBOR_TIMEOUT_MS) return false;
        return neighbors[i].rssi >= g_relay_thr;
    }
    return false;
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

// ============================================================================
// SEEN
// ============================================================================

static bool seen_before(const char src[7], uint16_t id)
{
    for(int i=0;i<SEEN_CACHE_SIZE;i++){
        if(!seen[i].used) continue;
        if(call7_eq(seen[i].src,src) && seen[i].msg_id==id)
            return true;
    }
    return false;
}

static void remember_msg(const char src[7], uint16_t id)
{
    static int idx=0;
    seen[idx].used=true;
    memcpy(seen[idx].src,src,7);
    seen[idx].msg_id=id;
    idx=(idx+1)%SEEN_CACHE_SIZE;
}

// ============================================================================
// SPI / LORA
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

    lora_wr(REG_PAYLOAD_LENGTH,len);
    lora_wr(REG_OP_MODE,0x83);

    while(!(lora_rd(REG_IRQ_FLAGS)&IRQ_TX_DONE))
        vTaskDelay(pdMS_TO_TICKS(5));

    lora_clear();
    lora_wr(REG_OP_MODE,0x85);
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
// APPLY CONFIG
// ============================================================================

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
    g_next_beacon_ms = now_ms() + g_beacon_interval +
                       (esp_random()%DEFAULT_BEACON_JITTER_MS);
}

static void send_beacon(void)
{
    mr_hdr_v3_t h={0};

    h.magic[0]='M';
    h.magic[1]='R';
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

// ============================================================================
// DATA TX (NEU 23B)
// ============================================================================

static void send_data_text(const char *txt)
{
    uint8_t buf[256];
    mr_hdr_v3_t *h=(mr_hdr_v3_t*)buf;

    memset(buf,0,sizeof(buf));

    h->magic[0]='M';
    h->magic[1]='R';
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

    h->payload_len=n;
    memcpy(buf+sizeof(mr_hdr_v3_t),txt,n);

    ESP_LOGI(TAG,"TX DATA id=%u text=%s",h->msg_id,txt);

    lora_send(buf,sizeof(mr_hdr_v3_t)+n);
}

// ============================================================================
// FORWARD (23B)
// ============================================================================

static void maybe_forward(uint8_t *buf,size_t len,int rssi)
{
    if(len<sizeof(mr_hdr_v3_t)) return;

    mr_hdr_v3_t *h=(mr_hdr_v3_t*)buf;

    if(h->ttl<=1) return;

    xSemaphoreTake(g_mutex,portMAX_DELAY);

    bool relay=g_relay_enabled;
    bool good=neighbor_good_locked(h->last_hop);

    char my[8];
    strncpy(my,g_cfg.call,sizeof(my)-1);

    xSemaphoreGive(g_mutex);

    if(!relay) return;
    if(!good){
        ESP_LOGI(TAG,"RELAY drop (RSSI<thr)");
        return;
    }

    h->ttl--;

    call7_set(h->last_hop,my);
    call7_set(h->next_hop,"*");

    ESP_LOGI(TAG,"RELAY forward flags=0x%02X ttl=%u",h->flags,h->ttl);

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
    xSemaphoreGive(g_mutex);

    char src[8];
    call7_to_str(src,h->src);

    // --------------------------------------------------------
    // BEACON
    // --------------------------------------------------------
    if(h->flags & MR_FLAG_BEACON){
        ESP_LOGI(TAG,"RX BEACON src=%s rssi=%d ttl=%u",src,rssi,h->ttl);
        maybe_forward(buf,len,rssi);
        return;
    }

    // --------------------------------------------------------
    // DATA (NEU)
    // --------------------------------------------------------
    if(h->flags & MR_FLAG_DATA){
        char txt[MAX_PAYLOAD+1]={0};

        if(h->payload_len>0 && h->payload_len<=MAX_PAYLOAD){
            memcpy(txt,buf+sizeof(mr_hdr_v3_t),h->payload_len);
            txt[h->payload_len]=0;
        }

        ESP_LOGI(TAG,"RX DATA src=%s text=\"%s\" ttl=%u",src,txt,h->ttl);

        maybe_forward(buf,len,rssi);
        return;
    }
}

// ============================================================================
// ISR
// ============================================================================

static void IRAM_ATTR dio_isr(void*arg)
{
    uint32_t n=(uint32_t)arg;
    xQueueSendFromISR(dio_q,&n,NULL);
}

static void dio_task(void*arg)
{
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
// WIFI (minimal AP)
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
// VERY SIMPLE HTTP (TEST BUTTON FOR DATA TX)
// ============================================================================

static esp_err_t index_get(httpd_req_t *req)
{
    const char *html=
        "<h2>MeshRadio 23B</h2>"
        "<button onclick=\"fetch('/tx',{method:'POST'})\">Send DATA</button>";
    httpd_resp_set_type(req,"text/html");
    return httpd_resp_send(req,html,HTTPD_RESP_USE_STRLEN);
}

static esp_err_t tx_post(httpd_req_t *req)
{
    send_data_text("Hallo Mesh");
    return httpd_resp_send(req,"OK",HTTPD_RESP_USE_STRLEN);
}

static void http_start(void)
{
    httpd_config_t cfg=HTTPD_DEFAULT_CONFIG();
    httpd_handle_t s=NULL;
    ESP_ERROR_CHECK(httpd_start(&s,&cfg));

    httpd_uri_t u0={.uri="/",.method=HTTP_GET,.handler=index_get};
    httpd_register_uri_handler(s,&u0);

    httpd_uri_t u1={.uri="/tx",.method=HTTP_POST,.handler=tx_post};
    httpd_register_uri_handler(s,&u1);
}

// ============================================================================
// MAIN
// ============================================================================

void app_main(void)
{
    ESP_LOGI(TAG,"MeshRadio Kapitel 23B start (DATA Relay)");

    g_mutex=xSemaphoreCreateMutex();

    ESP_ERROR_CHECK(nvs_flash_init());

    // defaults
    strcpy(g_cfg.call,"DJ2RF");
    g_cfg.freq_hz=433775000;
    g_cfg.sf=7;
    g_cfg.bw=125;
    g_cfg.crc=1;

    init_spi();
    lora_reset();

    ESP_LOGI(TAG,"SX1276 RegVersion=0x%02X",lora_rd(REG_VERSION));

    xSemaphoreTake(g_mutex,portMAX_DELAY);
    lora_apply_locked();
    xSemaphoreGive(g_mutex);

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

    wifi_ap();
    http_start();

    beacon_schedule_next();

    ESP_LOGI(TAG,"Open http://192.168.4.1");

    while(1){

        vTaskDelay(pdMS_TO_TICKS(50));

        if(g_beacon_enabled && now_ms()>g_next_beacon_ms)
            send_beacon();
    }
}
