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
// MeshRadio – Kapitel 20
// ----------------------------------------------------------------------------
// BEACONS + ROUTE REFRESH
//
// Basierend auf Kapitel 19A:
//
//   ✔ Header V3 (src, final_dst, next_hop, last_hop)
//   ✔ Routing + ACK + Store&Forward
//
// NEU:
//
//   ✔ MR_FLAG_BEACON
//   ✔ periodische Beacons mit Jitter
//   ✔ Route Refresh über Beacons
//   ✔ Neighbor + Route Update ohne Chatverkehr
//
// Ziel:
//   Netz lebt auch ohne Traffic.
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

#define MY_CALL "DJ2RF"

#define MR_PROTO_VERSION 3

#define MR_FLAG_CHAT    0x08
#define MR_FLAG_ACK     0x10
#define MR_FLAG_BEACON  0x20   // NEU Kapitel 20

#define MR_MAX_PAYLOAD 64

// ---------- Beacon ----------
#define BEACON_INTERVAL_MS 15000
#define BEACON_JITTER_MS   2000
#define BEACON_TTL         1

// ---------- Routing ----------
#define MAX_ROUTES 20
#define ROUTE_TIMEOUT_MS 120000

// ---------- Neighbor ----------
#define MAX_NEIGHBORS 20
#define NEIGHBOR_TIMEOUT_MS 60000

// ---------- Seen ----------
#define SEEN_CACHE_SIZE 32

// ---------- Pins ----------
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
// HEADER V3
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
// Globals
// ============================================================================

static const char *TAG="MR20";

static spi_device_handle_t lora_spi;
static QueueHandle_t dio0_evt_queue;

static uint16_t g_msg_id=1;

static seen_msg_t seen_cache[SEEN_CACHE_SIZE];
static neighbor_t neighbors[MAX_NEIGHBORS];
static route_entry_t routes[MAX_ROUTES];

static uint32_t next_beacon_ms=0;

// ============================================================================
// Helper
// ============================================================================

static uint32_t now_ms(void)
{
    return xTaskGetTickCount()*portTICK_PERIOD_MS;
}

static void call7_set(char out7[7], const char *s)
{
    memset(out7,' ',7);
    size_t n=strlen(s);
    if(n>7)n=7;
    memcpy(out7,s,n);
}

static bool call7_eq(const char a[7], const char b[7])
{
    return memcmp(a,b,7)==0;
}

// ============================================================================
// ROUTING
// ============================================================================

static void route_update(const char dst[7], const char hop[7], int rssi)
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

static void route_cleanup(void)
{
    uint32_t t=now_ms();
    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used) continue;
        if(t-routes[i].last_seen_ms > ROUTE_TIMEOUT_MS)
            routes[i].used=false;
    }
}

// ============================================================================
// SPI
// ============================================================================

static void lora_write_reg(uint8_t reg,uint8_t val)
{
    uint8_t tx[2]={reg|0x80,val};
    spi_transaction_t t={.length=16,.tx_buffer=tx};
    spi_device_transmit(lora_spi,&t);
}

static uint8_t lora_read_reg(uint8_t reg)
{
    uint8_t tx[2]={reg&0x7F,0};
    uint8_t rx[2]={0};
    spi_transaction_t t={.length=16,.tx_buffer=tx,.rx_buffer=rx};
    spi_device_transmit(lora_spi,&t);
    return rx[1];
}

static void lora_clear_irqs(void)
{
    lora_write_reg(REG_IRQ_FLAGS,0xFF);
}

// ============================================================================
// TX
// ============================================================================

static void lora_send_packet(uint8_t *data,size_t len)
{
    lora_clear_irqs();

    lora_write_reg(REG_FIFO_TX_BASE_ADDR,0);
    lora_write_reg(REG_FIFO_ADDR_PTR,0);

    for(size_t i=0;i<len;i++)
        lora_write_reg(REG_FIFO,data[i]);

    lora_write_reg(REG_PAYLOAD_LENGTH,len);

    lora_write_reg(REG_OP_MODE,0x83);

    while(!(lora_read_reg(REG_IRQ_FLAGS)&IRQ_TX_DONE))
        vTaskDelay(pdMS_TO_TICKS(5));

    lora_clear_irqs();
    lora_write_reg(REG_OP_MODE,0x85);
}

// ============================================================================
// BEACON SEND
// ============================================================================

static void send_beacon(void)
{
    mr_hdr_v3_t b={0};

    b.magic[0]='M';
    b.magic[1]='R';
    b.version=MR_PROTO_VERSION;
    b.flags=MR_FLAG_BEACON;

    b.ttl=BEACON_TTL;
    b.msg_id=g_msg_id++;

    call7_set(b.src,MY_CALL);
    call7_set(b.final_dst,"*");
    call7_set(b.next_hop,"*");
    call7_set(b.last_hop,MY_CALL);

    b.payload_len=0;

    ESP_LOGI(TAG,"TX BEACON id=%u",b.msg_id);

    lora_send_packet((uint8_t*)&b,sizeof(b));

    next_beacon_ms = now_ms()
                   + BEACON_INTERVAL_MS
                   + (esp_random()%BEACON_JITTER_MS);
}

// ============================================================================
// RX
// ============================================================================

static bool seen_before(const char src[7], uint16_t id)
{
    for(int i=0;i<SEEN_CACHE_SIZE;i++){
        if(!seen_cache[i].used) continue;
        if(call7_eq(seen_cache[i].src,src)
           && seen_cache[i].msg_id==id)
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

static void handle_rx_packet(void)
{
    uint8_t irq=lora_read_reg(REG_IRQ_FLAGS);
    if(!(irq & IRQ_RX_DONE)) return;

    uint8_t len=lora_read_reg(REG_RX_NB_BYTES);

    uint8_t addr=lora_read_reg(REG_FIFO_RX_CURRENT_ADDR);
    lora_write_reg(REG_FIFO_ADDR_PTR,addr);

    uint8_t buf[256];
    for(int i=0;i<len;i++)
        buf[i]=lora_read_reg(REG_FIFO);

    lora_clear_irqs();

    if(len<sizeof(mr_hdr_v3_t)) return;

    mr_hdr_v3_t *h=(mr_hdr_v3_t*)buf;

    if(seen_before(h->src,h->msg_id))
        return;

    remember_msg(h->src,h->msg_id);

    int rssi=lora_read_reg(REG_PKT_RSSI_VALUE)-157;

    // --------- BEACON RX ----------
    if(h->flags & MR_FLAG_BEACON){

        ESP_LOGI(TAG,"BEACON RX (RSSI=%d)",rssi);

        // Neighbor update
        for(int i=0;i<MAX_NEIGHBORS;i++){
            if(!neighbors[i].used){
                neighbors[i].used=true;
                memcpy(neighbors[i].call,h->last_hop,7);
                neighbors[i].rssi_dbm=rssi;
                neighbors[i].last_seen_ms=now_ms();
                break;
            }
        }

        // Route refresh:
        route_update(h->src,h->last_hop,rssi);

        return;
    }
}

// ============================================================================
// ISR
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
// INIT
// ============================================================================

static void init_spi(void)
{
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
}

static void lora_reset(void)
{
    gpio_set_direction(PIN_NUM_RST,GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_RST,0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_NUM_RST,1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void lora_init(void)
{
    lora_write_reg(REG_OP_MODE,0x80);
    vTaskDelay(pdMS_TO_TICKS(10));

    uint32_t frf=0x6C8000;
    lora_write_reg(REG_FRF_MSB,frf>>16);
    lora_write_reg(REG_FRF_MID,frf>>8);
    lora_write_reg(REG_FRF_LSB,frf);

    lora_write_reg(REG_MODEM_CONFIG_1,0x72);
    lora_write_reg(REG_MODEM_CONFIG_2,0x74);
    lora_write_reg(REG_MODEM_CONFIG_3,0x04);

    lora_write_reg(REG_PA_CONFIG,0x8E);

    lora_write_reg(REG_FIFO_RX_BASE_ADDR,0);
    lora_write_reg(REG_FIFO_ADDR_PTR,0);
    lora_write_reg(REG_OP_MODE,0x85);
}

// ============================================================================
// MAIN
// ============================================================================

void app_main(void)
{
    ESP_LOGI(TAG,"Kapitel 20 start (%s)",MY_CALL);

    init_spi();
    lora_reset();

    ESP_LOGI(TAG,"RegVersion=0x%02X",lora_read_reg(REG_VERSION));

    lora_init();

    dio0_evt_queue=xQueueCreate(10,sizeof(uint32_t));

    gpio_config_t io={
        .intr_type=GPIO_INTR_POSEDGE,
        .mode=GPIO_MODE_INPUT,
        .pin_bit_mask=(1ULL<<PIN_NUM_DIO0),
        .pull_up_en=1
    };
    gpio_config(&io);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_NUM_DIO0,dio0_isr,(void*)PIN_NUM_DIO0);

    xTaskCreate(dio0_task,"dio0",4096,NULL,10,NULL);

    next_beacon_ms=now_ms()+2000;

    while(1){

        vTaskDelay(pdMS_TO_TICKS(50));

        if(now_ms() > next_beacon_ms)
            send_beacon();

        route_cleanup();
    }
}
