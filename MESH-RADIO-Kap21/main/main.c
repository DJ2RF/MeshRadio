// ============================================================================
// MeshRadio 4.0 – Kapitel 21 (Sourcen komplett, sauber dokumentiert)
// ----------------------------------------------------------------------------
// PERSISTENZ MIT NVS (Non-Volatile Storage)
//
// Basierend auf Kapitel 20 (Beacons) + Kapitel 19A Header V3:
//
//   ✔ Protokoll V3 (src, final_dst, next_hop, last_hop)
//   ✔ Beacon TX + Beacon RX -> Route Refresh
//
// NEU in Kapitel 21:
//
//   ✔ NVS init (nvs_flash_init)
//   ✔ Konfigurationsstruktur mr_config_t
//   ✔ config_load_or_defaults(): lädt aus NVS oder setzt Defaults
//   ✔ config_save(): speichert Config in NVS
//   ✔ MY_CALL kommt nun aus cfg.call (kein #define MY_CALL mehr)
//
// Absicht (didaktisch):
//   - Wir speichern erst einmal nur DEVICE-CONFIG (Call + LoRa Parameter).
//   - Routen werden NICHT persistent gespeichert (zu volatil, schreiben zu oft).
//     Beacons bauen Routen nach Reboot schnell wieder auf.
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

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_random.h"

#include "nvs_flash.h"
#include "nvs.h"

// ============================================================================
// KONFIGURATION / DEFAULTS
// ============================================================================

// Protokoll
#define MR_PROTO_VERSION 3

// Flags
#define MR_FLAG_BEACON  0x20

// Beacon (Kapitel 20)
#define BEACON_INTERVAL_MS 15000
#define BEACON_JITTER_MS   2000
#define BEACON_TTL         1

// Tabellen
#define SEEN_CACHE_SIZE 32
#define MAX_ROUTES 20
#define ROUTE_TIMEOUT_MS 120000
#define MAX_NEIGHBORS 20
#define NEIGHBOR_TIMEOUT_MS 60000

// Pins (TTGO T-Beam V1.1)
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 27
#define PIN_NUM_CLK  5
#define PIN_NUM_CS   18
#define PIN_NUM_RST  23
#define PIN_NUM_DIO0 26

#define LORA_SPI_HOST VSPI_HOST

// LoRa (Default Parameter) – diese Werte können später per CLI geändert werden
#define DEFAULT_FREQ_HZ 433775000UL   // Beispiel: 433.775 MHz
#define DEFAULT_SF      7
#define DEFAULT_BW      125           // kHz (hier nur symbolisch)
#define DEFAULT_CRC_ON  1

// ============================================================================
// SX1276 Register (minimal für Beacon Demo)
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
// Protokoll Header V3 (Kapitel 19A)
// ============================================================================

#pragma pack(push,1)
typedef struct {
    uint8_t magic[2];      // "MR"
    uint8_t version;       // 3
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
// Tabellen (Seen / Neighbor / Route)
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
// Kapitel 21: Persistente Config (NVS)
// ----------------------------------------------------------------------------
// call: null-terminated (max 7 Zeichen), im Code nutzen wir call7 (space padded)
// freq_hz: wird später in FRF umgerechnet
// sf/bw/crc_on: für spätere Kapitel (CLI / Modem Config)
// ============================================================================
typedef struct {
    char     call[8];      // "DJ2RF" + '\0'
    uint32_t freq_hz;      // Hz
    uint8_t  sf;           // spreading factor (7..12)
    uint8_t  bw_khz;       // 125/250/500 (symbolisch)
    uint8_t  crc_on;       // 0/1
} mr_config_t;

// ============================================================================
// Globals
// ============================================================================

static const char *TAG="MR21";

static spi_device_handle_t lora_spi;
static QueueHandle_t dio0_evt_queue;

static uint16_t g_msg_id=1;
static uint32_t next_beacon_ms=0;

static seen_msg_t    seen_cache[SEEN_CACHE_SIZE];
static neighbor_t    neighbors[MAX_NEIGHBORS];
static route_entry_t routes[MAX_ROUTES];

static mr_config_t g_cfg;   // geladene Config aus NVS

// ============================================================================
// Helper
// ============================================================================

static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
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

// Hz -> FRF (SX1276): FRF = freq_hz * 2^19 / 32e6
static uint32_t hz_to_frf(uint32_t freq_hz)
{
    // 64-bit, um Overflow zu vermeiden
    uint64_t frf = ((uint64_t)freq_hz << 19) / 32000000ULL;
    return (uint32_t)frf;
}

// ============================================================================
// Kapitel 21: NVS Config Load/Save
// ----------------------------------------------------------------------------
// Namespace: "mr"
// Keys:
//   - "call"   (string)
//   - "freq"   (u32)
//   - "sf"     (u8)
//   - "bw"     (u8)
//   - "crc"    (u8)
// ============================================================================
static esp_err_t config_save(const mr_config_t *cfg)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open("mr", NVS_READWRITE, &h);
    if(err != ESP_OK) return err;

    // String
    err = nvs_set_str(h, "call", cfg->call);
    if(err != ESP_OK){ nvs_close(h); return err; }

    // Zahlen
    err = nvs_set_u32(h, "freq", cfg->freq_hz);
    if(err != ESP_OK){ nvs_close(h); return err; }

    err = nvs_set_u8(h, "sf", cfg->sf);
    if(err != ESP_OK){ nvs_close(h); return err; }

    err = nvs_set_u8(h, "bw", cfg->bw_khz);
    if(err != ESP_OK){ nvs_close(h); return err; }

    err = nvs_set_u8(h, "crc", cfg->crc_on);
    if(err != ESP_OK){ nvs_close(h); return err; }

    // Commit (wichtig!)
    err = nvs_commit(h);
    nvs_close(h);
    return err;
}

static esp_err_t config_load_or_defaults(mr_config_t *cfg)
{
    // Defaults setzen
    memset(cfg,0,sizeof(*cfg));
    strncpy(cfg->call, "DJ2RF", sizeof(cfg->call)-1);   // Default Call
    cfg->freq_hz = DEFAULT_FREQ_HZ;
    cfg->sf      = DEFAULT_SF;
    cfg->bw_khz  = DEFAULT_BW;
    cfg->crc_on  = DEFAULT_CRC_ON;

    nvs_handle_t h;
    esp_err_t err = nvs_open("mr", NVS_READWRITE, &h);
    if(err != ESP_OK) return err;

    // call: wenn nicht vorhanden -> Defaults bleiben + speichern
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

    // freq
    uint32_t f=0;
    err = nvs_get_u32(h, "freq", &f);
    if(err == ESP_OK) cfg->freq_hz = f;

    // sf/bw/crc (falls Keys fehlen -> Default bleibt)
    uint8_t v=0;
    if(nvs_get_u8(h, "sf", &v)  == ESP_OK) cfg->sf=v;
    if(nvs_get_u8(h, "bw", &v)  == ESP_OK) cfg->bw_khz=v;
    if(nvs_get_u8(h, "crc", &v) == ESP_OK) cfg->crc_on=v;

    nvs_close(h);
    return ESP_OK;
}

// ============================================================================
// Routing (wie Kap 20)
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

static void neighbor_cleanup(void)
{
    uint32_t t=now_ms();
    for(int i=0;i<MAX_NEIGHBORS;i++){
        if(!neighbors[i].used) continue;
        if(t-neighbors[i].last_seen_ms > NEIGHBOR_TIMEOUT_MS)
            neighbors[i].used=false;
    }
}

// ============================================================================
// SPI / LoRa Low-Level
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

static void lora_clear_irqs(void)
{
    lora_write_reg(REG_IRQ_FLAGS,0xFF);
}

static void lora_send_packet(uint8_t *data,size_t len)
{
    lora_clear_irqs();

    lora_write_reg(REG_FIFO_TX_BASE_ADDR,0);
    lora_write_reg(REG_FIFO_ADDR_PTR,0);

    for(size_t i=0;i<len;i++)
        lora_write_reg(REG_FIFO,data[i]);

    lora_write_reg(REG_PAYLOAD_LENGTH,(uint8_t)len);

    // TX
    lora_write_reg(REG_OP_MODE,0x83);

    // Poll TX_DONE
    while(!(lora_read_reg(REG_IRQ_FLAGS)&IRQ_TX_DONE))
        vTaskDelay(pdMS_TO_TICKS(5));

    lora_clear_irqs();

    // Back to RX continuous
    lora_write_reg(REG_OP_MODE,0x85);
}

static void lora_reset(void)
{
    gpio_set_direction(PIN_NUM_RST,GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_RST,0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_NUM_RST,1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

// Sehr einfache Init: setzt nur Freq + Modem Defaults
static void lora_init_from_cfg(const mr_config_t *cfg)
{
    // LoRa sleep
    lora_write_reg(REG_OP_MODE,0x80);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Frequency
    uint32_t frf = hz_to_frf(cfg->freq_hz);
    lora_write_reg(REG_FRF_MSB,(uint8_t)(frf>>16));
    lora_write_reg(REG_FRF_MID,(uint8_t)(frf>>8));
    lora_write_reg(REG_FRF_LSB,(uint8_t)(frf));

    // Modem defaults (wie vorher)
    // (SF/BW/CRC werden wir im nächsten Kapitel sauber ableiten)
    lora_write_reg(REG_MODEM_CONFIG_1,0x72);
    lora_write_reg(REG_MODEM_CONFIG_2,0x74);
    lora_write_reg(REG_MODEM_CONFIG_3,0x04);

    // TX power
    lora_write_reg(REG_PA_CONFIG,0x8E);

    // RX continuous
    lora_write_reg(REG_FIFO_RX_BASE_ADDR,0);
    lora_write_reg(REG_FIFO_ADDR_PTR,0);
    lora_write_reg(REG_OP_MODE,0x85);
}

// ============================================================================
// Beacon Send (Kap 20 + Call aus cfg)
// ============================================================================

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

    // Callsign aus NVS-Config
    call7_set(b.src, g_cfg.call);

    call7_set(b.final_dst,"*");
    call7_set(b.next_hop,"*");

    // last_hop = wir
    call7_set(b.last_hop, g_cfg.call);

    b.payload_len=0;

    ESP_LOGI(TAG,"TX BEACON id=%u call=%s freq=%" PRIu32,
             b.msg_id, g_cfg.call, g_cfg.freq_hz);

    lora_send_packet((uint8_t*)&b,sizeof(b));

    next_beacon_ms = now_ms()
                   + BEACON_INTERVAL_MS
                   + (esp_random() % BEACON_JITTER_MS);
}

// ============================================================================
// RX Handling (nur Beacon Demo für Kap 21)
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

static void neighbor_update(const char call[7], int rssi)
{
    uint32_t t=now_ms();

    // update existing
    for(int i=0;i<MAX_NEIGHBORS;i++){
        if(!neighbors[i].used) continue;
        if(call7_eq(neighbors[i].call,call)){
            neighbors[i].rssi_dbm=rssi;
            neighbors[i].last_seen_ms=t;
            return;
        }
    }
    // insert
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

    if(h->magic[0] != 'M' || h->magic[1] != 'R') return;
    if(h->version != MR_PROTO_VERSION) return;

    if(seen_before(h->src,h->msg_id))
        return;
    remember_msg(h->src,h->msg_id);

    int rssi = (int)lora_read_reg(REG_PKT_RSSI_VALUE) - 157;

    if(h->flags & MR_FLAG_BEACON){

        // Route Refresh: destination=src über next_hop=last_hop
        neighbor_update(h->last_hop, rssi);
        route_update(h->src, h->last_hop, rssi);

        ESP_LOGI(TAG,"BEACON RX rssi=%d (route refresh)", rssi);
        return;
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
// Init SPI Bus
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
// MAIN
// ============================================================================

void app_main(void)
{
    ESP_LOGI(TAG,"Kapitel 21 start (NVS Persistenz)");

    // 1) NVS init (wichtig: auch handle ESP_ERR_NVS_NO_FREE_PAGES)
    esp_err_t err = nvs_flash_init();
    if(err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND){
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }else{
        ESP_ERROR_CHECK(err);
    }

    // 2) Config laden oder Defaults schreiben
    ESP_ERROR_CHECK(config_load_or_defaults(&g_cfg));

    ESP_LOGI(TAG,"CFG: call=%s freq=%" PRIu32 " sf=%u bw=%u crc=%u",
             g_cfg.call, g_cfg.freq_hz, g_cfg.sf, g_cfg.bw_khz, g_cfg.crc_on);

    // 3) LoRa init
    init_spi();
    lora_reset();

    ESP_LOGI(TAG,"SX1276 RegVersion=0x%02X", lora_read_reg(REG_VERSION));

    lora_init_from_cfg(&g_cfg);

    // 4) DIO0 IRQ
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

    // 5) Beacon schedule
    next_beacon_ms = now_ms() + 2000;

    // 6) Main loop
    while(1){
        vTaskDelay(pdMS_TO_TICKS(50));

        if(now_ms() > next_beacon_ms)
            send_beacon();

        neighbor_cleanup();
        route_cleanup();
    }
}
