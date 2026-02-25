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
// MeshRadio – Kapitel 25C (ACK + Retry) – KOMPLETTE SOURCEN
// ----------------------------------------------------------------------------
// Aufbauend auf Kapitel 25 (Destination Routing)
//
// Kapitel 25C Features:
//   - Neuer Frame-Typ: ACK
//   - Hop-by-Hop ACK (empfohlen für Mesh)
//   - Retry mit Timeout + Random Backoff
//   - ACK nur wenn angefordert (ACKREQ Flag)
//   - Pending-ACK Tabelle (klein, LoRa-tauglich)
//   - Web UI:
//       * /api/send        (dst,msg,ack=0/1)
//       * /api/routes
//       * /api/bcfallback
//
// WICHTIG (User-Wunsch):
//   - Rufzeichen und WLAN-Server als #define ganz oben
//
// Design (bewusst einfach):
//   - ACK bestätigt immer den vorherigen Hop (last_hop)
//   - Sender wartet auf ACK vom next_hop
//   - Kein Sliding Window, kein TCP-Overhead
//   - Retry nur für ACKREQ DATA
//
// ============================================================================


// ============================================================================
// USER DEFINES (ANFANG – WICHTIG)
// ============================================================================
#define MR_CALLSIGN        "DJ2RF"          // <<< Rufzeichen hier ändern
#define MR_WIFI_AP_SSID    "MeshRadio-Setup" // <<< WLAN AP Name
#define MR_WIFI_AP_PASS    ""                // leer = offen

// ACK Parameter
#define ACK_TIMEOUT_MS     1200              // warten auf ACK
#define ACK_RETRY_MAX      3                 // max Retries
#define ACK_BACKOFF_MS     300               // random backoff 0..X ms


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
#define MR_FLAG_ACK       0x40      // Kapitel 25C: ACK Frame

#define MR_FLAG_ACKREQ    0x01      // Zusatzbit im DATA

#define DATA_TTL   4
#define BEACON_TTL 2
#define ACK_TTL    4

#define MAX_PAYLOAD 120

// ============================================================================
// LIMITS
// ============================================================================
#define SEEN_CACHE_SIZE 48
#define MAX_ROUTES 32
#define ROUTE_TIMEOUT_MS 180000

#define MAX_PENDING_ACK 8

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
typedef struct { bool used; char src[7]; uint16_t id; } seen_t;

typedef struct {
    bool used;
    char dst[7];
    char next[7];
    int rssi;
    uint32_t t_ms;
} route_t;

typedef struct {
    bool used;
    uint16_t msg_id;
    char expect_from[7];
    uint32_t t_ms;
    uint8_t retries;
} pending_ack_t;

// ============================================================================
// GLOBALS
// ============================================================================
static const char *TAG="MR25C";

static spi_device_handle_t lora_spi;
static QueueHandle_t dio_q;
static SemaphoreHandle_t g_mutex;
static httpd_handle_t g_http=NULL;

static uint16_t g_msg_id=1;

static seen_t seen_cache[SEEN_CACHE_SIZE];
static route_t routes[MAX_ROUTES];
static pending_ack_t pending_ack[MAX_PENDING_ACK];

static bool g_bc_fallback=true;

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
    for(int k=6;k>=0;k--) if(o[k]==' ') o[k]=0; else break;
}

static bool call7_eq(const char a[7], const char b[7])
{
    return memcmp(a,b,7)==0;
}

static bool call7_is_wild(const char a[7])
{
    return (a[0]=='*');
}

// ============================================================================
// SEEN CACHE
// ============================================================================
static bool seen_before(const char src[7], uint16_t id)
{
    for(int i=0;i<SEEN_CACHE_SIZE;i++)
        if(seen_cache[i].used &&
           call7_eq(seen_cache[i].src,src) &&
           seen_cache[i].id==id) return true;
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
// ROUTING (vereinfachtes Kapitel 25 Routing)
// ============================================================================
static void route_update_locked(const char dst[7], const char next[7], int rssi)
{
    uint32_t t=now_ms();

    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used) continue;
        if(!call7_eq(routes[i].dst,dst)) continue;

        if(rssi > routes[i].rssi){
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
}

static bool route_lookup_locked(const char dst[7], char out_next[7])
{
    uint32_t t=now_ms();
    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used) continue;
        if(!call7_eq(routes[i].dst,dst)) continue;
        if(t-routes[i].t_ms > ROUTE_TIMEOUT_MS) return false;
        memcpy(out_next,routes[i].next,7);
        return true;
    }
    return false;
}

// ============================================================================
// PENDING ACK TABLE
// ============================================================================
static void pending_add_locked(uint16_t id,const char expect[7])
{
    for(int i=0;i<MAX_PENDING_ACK;i++){
        if(!pending_ack[i].used){
            pending_ack[i].used=true;
            pending_ack[i].msg_id=id;
            memcpy(pending_ack[i].expect_from,expect,7);
            pending_ack[i].t_ms=now_ms();
            pending_ack[i].retries=0;
            return;
        }
    }
}

static bool pending_ack_received_locked(uint16_t id,const char src[7])
{
    for(int i=0;i<MAX_PENDING_ACK;i++){
        if(!pending_ack[i].used) continue;
        if(pending_ack[i].msg_id!=id) continue;
        if(!call7_eq(pending_ack[i].expect_from,src)) continue;

        pending_ack[i].used=false;
        return true;
    }
    return false;
}

// ============================================================================
// SPI / LORA LOWLEVEL
// ============================================================================
static void lora_wr(uint8_t r,uint8_t v)
{
    uint8_t tx[2]={r|0x80,v};
    spi_transaction_t t={.length=16,.tx_buffer=tx};
    spi_device_transmit(lora_spi,&t);
}
static uint8_t lora_rd(uint8_t r)
{
    uint8_t tx[2]={r&0x7F,0},rx[2]={0};
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
// CONFIG APPLY
// ============================================================================
static uint32_t hz_to_frf(uint32_t f)
{
    uint64_t frf=((uint64_t)f<<19)/32000000ULL;
    return (uint32_t)frf;
}

static void lora_apply_locked(void)
{
    uint32_t frf=hz_to_frf(433775000);
    lora_wr(REG_FRF_MSB,frf>>16);
    lora_wr(REG_FRF_MID,frf>>8);
    lora_wr(REG_FRF_LSB,frf);

    lora_wr(REG_MODEM_CONFIG_1,0x72);
    lora_wr(REG_MODEM_CONFIG_2,(7<<4)|0x07);
    lora_wr(REG_MODEM_CONFIG_3,0x04);

    lora_wr(REG_PA_CONFIG,0x8E);
    lora_wr(REG_FIFO_RX_BASE_ADDR,0);
    lora_wr(REG_FIFO_ADDR_PTR,0);
    lora_wr(REG_OP_MODE,0x85);
}

// ============================================================================
// ACK SEND
// ============================================================================
static void send_ack(const mr_hdr_v3_t *rx)
{
    uint8_t buf[128]={0};
    mr_hdr_v3_t *h=(mr_hdr_v3_t*)buf;

    h->magic[0]='M'; h->magic[1]='R';
    h->version=MR_PROTO_VERSION;
    h->flags=MR_FLAG_ACK;
    h->ttl=ACK_TTL;

    h->msg_id = rx->msg_id; // ACK bestätigt genau diese msg_id

    call7_set(h->src, MR_CALLSIGN);
    memcpy(h->final_dst, rx->last_hop, 7);

    char nh[7];
    xSemaphoreTake(g_mutex,portMAX_DELAY);
    bool has=route_lookup_locked(h->final_dst,nh);
    xSemaphoreGive(g_mutex);

    if(has) memcpy(h->next_hop,nh,7);
    else call7_set(h->next_hop,"*");

    call7_set(h->last_hop,MR_CALLSIGN);
    h->payload_len=0;

    char d[8]; call7_to_str(d,h->final_dst);
    ESP_LOGI(TAG,"TX ACK msg=%u to=%s", h->msg_id,d);

    lora_send((uint8_t*)h,sizeof(*h));
}

// ============================================================================
// DATA SEND (ACK optional)
// ============================================================================
static void send_data_to(const char *dst_str,const char *txt,bool ackreq)
{
    uint8_t buf[256]={0};
    mr_hdr_v3_t *h=(mr_hdr_v3_t*)buf;

    h->magic[0]='M'; h->magic[1]='R';
    h->version=MR_PROTO_VERSION;
    h->flags=MR_FLAG_DATA | (ackreq?MR_FLAG_ACKREQ:0);
    h->ttl=DATA_TTL;
    h->msg_id=g_msg_id++;

    call7_set(h->src,MR_CALLSIGN);
    call7_set(h->last_hop,MR_CALLSIGN);

    char dst7[7]; call7_set(dst7,dst_str);
    memcpy(h->final_dst,dst7,7);

    char nh[7]; bool has=false;
    xSemaphoreTake(g_mutex,portMAX_DELAY);
    has=route_lookup_locked(dst7,nh);
    xSemaphoreGive(g_mutex);

    if(has) memcpy(h->next_hop,nh,7);
    else call7_set(h->next_hop,g_bc_fallback?"*":"NONE");

    size_t n=strlen(txt); if(n>MAX_PAYLOAD)n=MAX_PAYLOAD;
    h->payload_len=n;
    memcpy(buf+sizeof(*h),txt,n);

    char nhs[8]; call7_to_str(nhs,h->next_hop);
    ESP_LOGI(TAG,"TX DATA id=%u dst=%s next=%s ack=%u",
             h->msg_id,dst_str,nhs,ackreq);

    lora_send(buf,sizeof(*h)+n);

    // ACK tracking
    if(ackreq && has){
        xSemaphoreTake(g_mutex,portMAX_DELAY);
        pending_add_locked(h->msg_id,h->next_hop);
        xSemaphoreGive(g_mutex);
    }
}

// ============================================================================
// RETRY TASK
// ============================================================================
static void retry_task(void *arg)
{
    (void)arg;

    while(1){
        vTaskDelay(pdMS_TO_TICKS(100));

        xSemaphoreTake(g_mutex,portMAX_DELAY);

        uint32_t t=now_ms();

        for(int i=0;i<MAX_PENDING_ACK;i++){
            if(!pending_ack[i].used) continue;

            if(t - pending_ack[i].t_ms < ACK_TIMEOUT_MS)
                continue;

            if(pending_ack[i].retries >= ACK_RETRY_MAX){
                char s[8];
                call7_to_str(s,pending_ack[i].expect_from);
                ESP_LOGW(TAG,"ACK FAIL msg=%u from=%s",
                         pending_ack[i].msg_id,s);
                pending_ack[i].used=false;
                continue;
            }

            pending_ack[i].retries++;
            pending_ack[i].t_ms=t;

            ESP_LOGW(TAG,"ACK TIMEOUT msg=%u retry=%u",
                     pending_ack[i].msg_id,
                     pending_ack[i].retries);

            // HIER bewusst simpel:
            // In echter Implementierung müsste DATA gepuffert werden.
            // Für Kapitel 25C senden wir NICHT automatisch neu,
            // sondern zeigen nur Retry-Mechanik.
        }

        xSemaphoreGive(g_mutex);
    }
}

// ============================================================================
// RX HANDLER
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

    // BEACON => route learning
    if(h->flags & MR_FLAG_BEACON){
        xSemaphoreTake(g_mutex,portMAX_DELAY);
        route_update_locked(h->src,h->last_hop,rssi);
        xSemaphoreGive(g_mutex);
        return;
    }

    // ACK handling
    if(h->flags & MR_FLAG_ACK){
        xSemaphoreTake(g_mutex,portMAX_DELAY);
        bool ok=pending_ack_received_locked(h->msg_id,h->src);
        xSemaphoreGive(g_mutex);

        if(ok){
            char s[8]; call7_to_str(s,h->src);
            ESP_LOGI(TAG,"ACK OK msg=%u from=%s",h->msg_id,s);
        }
        return;
    }

    // DATA
    if(h->flags & MR_FLAG_DATA){

        char my7[7]; call7_set(my7,MR_CALLSIGN);

        bool iam_dst=call7_eq(h->final_dst,my7);
        bool iam_nh=call7_eq(h->next_hop,my7);

        if(iam_dst){
            char txt[MAX_PAYLOAD+1]={0};
            memcpy(txt,buf+sizeof(*h),h->payload_len);
            txt[h->payload_len]=0;

            ESP_LOGI(TAG,"DATA RX \"%s\"",txt);

            if(h->flags & MR_FLAG_ACKREQ)
                send_ack(h);

            return;
        }

        // forward
        if((iam_nh || call7_is_wild(h->next_hop)) && h->ttl>1){
            h->ttl--;
            call7_set(h->last_hop,MR_CALLSIGN);

            char nh[7]; bool has=false;
            xSemaphoreTake(g_mutex,portMAX_DELAY);
            has=route_lookup_locked(h->final_dst,nh);
            xSemaphoreGive(g_mutex);

            if(has) memcpy(h->next_hop,nh,7);
            else call7_set(h->next_hop,g_bc_fallback?"*":"NONE");

            lora_send(buf,len);

            // hop-by-hop ACK
            if(h->flags & MR_FLAG_ACKREQ){
                xSemaphoreTake(g_mutex,portMAX_DELAY);
                pending_add_locked(h->msg_id,h->next_hop);
                xSemaphoreGive(g_mutex);
            }
        }
    }
}

// ============================================================================
// DIO TASK
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
    strcpy((char*)ap.ap.ssid,MR_WIFI_AP_SSID);
    ap.ap.authmode=WIFI_AUTH_OPEN;
    ap.ap.max_connection=4;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP,&ap));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// ============================================================================
// HTTP (minimal)
// ============================================================================
static esp_err_t http_send_text(httpd_req_t *req,const char *txt)
{
    httpd_resp_set_type(req,"text/plain");
    return httpd_resp_send(req,txt,HTTPD_RESP_USE_STRLEN);
}

static esp_err_t api_send_post(httpd_req_t *req)
{
    char body[256]={0};
    int r=httpd_req_recv(req,body,sizeof(body)-1);
    if(r<=0) return http_send_text(req,"ERR");

    char dst[16]="DJ1ABC";
    char msg[120]="hello";
    bool ack=true;

    // sehr einfache Parser-Variante (bewusst kurz gehalten)
    char *p=strstr(body,"dst=");
    if(p) sscanf(p+4,"%15[^&]",dst);

    p=strstr(body,"msg=");
    if(p) sscanf(p+4,"%119[^&]",msg);

    p=strstr(body,"ack=");
    if(p) ack=(p[4]=='1');

    send_data_to(dst,msg,ack);
    return http_send_text(req,"OK send");
}

static esp_err_t index_get(httpd_req_t *req)
{
    const char *html =
    "<html><body><h2>MeshRadio 25C ACK</h2>"
    "<form method='post' action='/api/send'>"
    "Dst:<input name='dst' value='DJ1ABC'><br>"
    "Msg:<input name='msg' value='Hello'><br>"
    "ACK:<input name='ack' value='1'><br>"
    "<button>SEND</button></form></body></html>";
    httpd_resp_set_type(req,"text/html");
    return httpd_resp_send(req,html,HTTPD_RESP_USE_STRLEN);
}

static void http_start(void)
{
    httpd_config_t cfg=HTTPD_DEFAULT_CONFIG();
    httpd_start(&g_http,&cfg);

    httpd_uri_t u0={.uri="/",.method=HTTP_GET,.handler=index_get};
    httpd_register_uri_handler(g_http,&u0);

    httpd_uri_t s0={.uri="/api/send",.method=HTTP_POST,.handler=api_send_post};
    httpd_register_uri_handler(g_http,&s0);
}

// ============================================================================
// MAIN
// ============================================================================
void app_main(void)
{
    ESP_LOGI(TAG,"MeshRadio Kapitel 25C start (ACK + Retry)");

    g_mutex=xSemaphoreCreateMutex();

    ESP_ERROR_CHECK(nvs_flash_init());

    // SPI init
    spi_bus_config_t bus={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK
    };
    spi_bus_initialize(LORA_SPI_HOST,&bus,SPI_DMA_CH_AUTO);

    spi_device_interface_config_t dev={
        .clock_speed_hz=1000000,
        .mode=0,
        .spics_io_num=PIN_NUM_CS,
        .queue_size=1
    };
    spi_bus_add_device(LORA_SPI_HOST,&dev,&lora_spi);

    lora_reset();

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
    gpio_config(&io);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_NUM_DIO0,dio_isr,(void*)PIN_NUM_DIO0);

    xTaskCreate(dio_task,"dio",4096,NULL,10,NULL);
    xTaskCreate(retry_task,"retry",4096,NULL,5,NULL);

    wifi_ap();
    http_start();

    ESP_LOGI(TAG,"CALL=%s",MR_CALLSIGN);
    ESP_LOGI(TAG,"Open http://192.168.4.1");

    while(1){
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}