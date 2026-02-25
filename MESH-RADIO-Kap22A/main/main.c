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
// MeshRadio – Kapitel 22A (Sourcen komplett, sauber dokumentiert)
// ----------------------------------------------------------------------------
// CLI/UART KONFIGURATION + NVS PERSISTENZ + KORREKTE SF/BW/CRC REGISTER
//
// WICHTIG: Wir verwenden ab jetzt konsequent nur noch "MeshRadio" (ohne "4.0").
//
// Basierend auf Kapitel 22:
//   ✔ NVS Persistenz (call, freq, sf, bw, crc)
//   ✔ Beacon TX/RX (Route Refresh)
//   ✔ CLI (help/show/call/freq/sf/bw/crc/apply/save/load/beacon/beaconint/reboot)
//
// NEU in Kapitel 22A:
//   ✔ lora_apply_config() setzt SF/BW/CRC korrekt in SX1276 Register:
//        REG_MODEM_CONFIG_1 (BW + CR + Explicit)
//        REG_MODEM_CONFIG_2 (SF + CRC)
//        REG_MODEM_CONFIG_3 (LDO + AGC)
//   ✔ LDO automatisch: BW=125 & SF>=11 => LDO=1
//
// Terminal:
//   - PuTTY / Screen / idf.py monitor
//   - 115200 baud
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

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_random.h"
#include "esp_system.h"

#include "nvs_flash.h"
#include "nvs.h"

// ============================================================================
// PROTOKOLL / BEACON
// ============================================================================

#define MR_PROTO_VERSION 3
#define MR_FLAG_BEACON  0x20

#define BEACON_TTL 1

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
// LORA PINS (TTGO T-Beam V1.1)
// ============================================================================

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 27
#define PIN_NUM_CLK  5
#define PIN_NUM_CS   18
#define PIN_NUM_RST  23
#define PIN_NUM_DIO0 26

#define LORA_SPI_HOST SPI3_HOST // VSPI_HOST

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
// Persistente Config (Kapitel 21/22)
// ============================================================================
typedef struct {
    char     call[8];      // max 7 Zeichen + '\0'
    uint32_t freq_hz;      // Hz
    uint8_t  sf;           // 7..12
    uint8_t  bw_khz;       // 125/250/500
    uint8_t  crc_on;       // 0/1
} mr_config_t;

// ============================================================================
// Globals
// ============================================================================

static const char *TAG="MR22A";

static spi_device_handle_t lora_spi;
static QueueHandle_t dio0_evt_queue;

static uint16_t g_msg_id=1;
static uint32_t next_beacon_ms=0;

// Beacon runtime params (CLI änderbar)
static bool     g_beacon_enabled     = true;
static uint32_t g_beacon_interval_ms = DEFAULT_BEACON_INTERVAL_MS;
static uint32_t g_beacon_jitter_ms   = DEFAULT_BEACON_JITTER_MS;

static seen_msg_t    seen_cache[SEEN_CACHE_SIZE];
static neighbor_t    neighbors[MAX_NEIGHBORS];
static route_entry_t routes[MAX_ROUTES];

static mr_config_t g_cfg;

// CLI input buffer
static char cli_line[128];
static int  cli_pos=0;

// ============================================================================
// Time/Call helpers
// ============================================================================

static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount()*portTICK_PERIOD_MS);
}

static void call7_set(char out7[7], const char *s)
{
    memset(out7,' ',7);
    size_t n=strlen(s);
    if(n>7)n=7;
    memcpy(out7,s,n);
}

static void call7_to_cstr(char out8[8], const char in7[7])
{
    memcpy(out8,in7,7);
    out8[7]=0;
    for(int i=6;i>=0;i--){
        if(out8[i]==' ') out8[i]=0;
        else break;
    }
}

static bool call7_eq(const char a[7], const char b[7])
{
    return memcmp(a,b,7)==0;
}

// ============================================================================
// Frequency helper (Hz -> FRF)
// ============================================================================
static uint32_t hz_to_frf(uint32_t freq_hz)
{
    uint64_t frf = ((uint64_t)freq_hz << 19) / 32000000ULL;
    return (uint32_t)frf;
}

// ============================================================================
// NVS Config Load/Save (Kapitel 21)
// ============================================================================
static esp_err_t config_save(const mr_config_t *cfg)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open("mr", NVS_READWRITE, &h);
    if(err != ESP_OK) return err;

    err = nvs_set_str(h, "call", cfg->call);
    if(err != ESP_OK){ nvs_close(h); return err; }

    err = nvs_set_u32(h, "freq", cfg->freq_hz);
    if(err != ESP_OK){ nvs_close(h); return err; }

    err = nvs_set_u8(h, "sf", cfg->sf);
    if(err != ESP_OK){ nvs_close(h); return err; }

    err = nvs_set_u8(h, "bw", cfg->bw_khz);
    if(err != ESP_OK){ nvs_close(h); return err; }

    err = nvs_set_u8(h, "crc", cfg->crc_on);
    if(err != ESP_OK){ nvs_close(h); return err; }

    err = nvs_commit(h);
    nvs_close(h);
    return err;
}

static esp_err_t config_load_or_defaults(mr_config_t *cfg)
{
    // Defaults
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

    uint32_t f=0;
    if(nvs_get_u32(h, "freq", &f) == ESP_OK) cfg->freq_hz = f;

    uint8_t v=0;
    if(nvs_get_u8(h, "sf", &v)  == ESP_OK) cfg->sf=v;
    if(nvs_get_u8(h, "bw", &v)  == ESP_OK) cfg->bw_khz=v;
    if(nvs_get_u8(h, "crc", &v) == ESP_OK) cfg->crc_on=v;

    nvs_close(h);
    return ESP_OK;
}

// ============================================================================
// Routing + Neighbor cleanup (wie Kapitel 22)
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
        if(t - routes[i].last_seen_ms > ROUTE_TIMEOUT_MS)
            routes[i].used=false;
    }
}

static void neighbor_update(const char call[7], int rssi)
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

static void neighbor_cleanup(void)
{
    uint32_t t=now_ms();
    for(int i=0;i<MAX_NEIGHBORS;i++){
        if(!neighbors[i].used) continue;
        if(t - neighbors[i].last_seen_ms > NEIGHBOR_TIMEOUT_MS)
            neighbors[i].used=false;
    }
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

    while(!(lora_read_reg(REG_IRQ_FLAGS)&IRQ_TX_DONE))
        vTaskDelay(pdMS_TO_TICKS(5));

    lora_clear_irqs();

    // back to RX continuous
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

// ============================================================================
// Kapitel 22A: SF/BW/CRC korrekt setzen (SX1276)
// ----------------------------------------------------------------------------
// Mapping für BW (RegModemConfig1 bits 7..4):
//   125 kHz -> 0x70
//   250 kHz -> 0x80
//   500 kHz -> 0x90
//
// SF (RegModemConfig2 bits 7..4): 7..12
// CRC (RegModemConfig2 bit 2): 1=on
//
// LDO (RegModemConfig3 bit 3): BW125 + SF>=11 -> on
// AGC (RegModemConfig3 bit 2): on
//
// Coding Rate:
//   Wir fixieren CR=4/5 (RegModemConfig1 bits 3..1 = 001).
// ----------------------------------------------------------------------------
static uint8_t bw_to_reg(uint8_t bw_khz)
{
    switch(bw_khz){
        case 125: return 0x70;
        case 250: return 0x80;
        case 500: return 0x90;
        default:  return 0x70; // fallback
    }
}

static bool need_ldo(uint8_t sf, uint8_t bw_khz)
{
    // pragmatische Faustregel (funktioniert in der Praxis sehr gut):
    // bei langen Symbolzeiten -> LDO an
    return (bw_khz==125 && sf>=11);
}

static void lora_apply_config(const mr_config_t *cfg)
{
    ESP_LOGI(TAG,"APPLY: call=%s freq=%" PRIu32 " sf=%u bw=%u crc=%u",
             cfg->call, cfg->freq_hz, cfg->sf, cfg->bw_khz, cfg->crc_on);

    // 1) Sleep + LoRa
    lora_write_reg(REG_OP_MODE,0x80);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 2) Frequency
    uint32_t frf = hz_to_frf(cfg->freq_hz);
    lora_write_reg(REG_FRF_MSB,(uint8_t)(frf>>16));
    lora_write_reg(REG_FRF_MID,(uint8_t)(frf>>8));
    lora_write_reg(REG_FRF_LSB,(uint8_t)(frf));

    // 3) ModemConfig1: BW + CR(4/5) + explicit header
    uint8_t bw_bits = bw_to_reg(cfg->bw_khz);
    uint8_t cr_bits = (1 << 1);    // CR=4/5 => bits 3..1 = 001 => 0b00000010
    uint8_t implicit = 0;          // explicit header => bit0=0
    uint8_t mc1 = bw_bits | cr_bits | implicit;

    // 4) ModemConfig2: SF + CRC
    uint8_t sf = cfg->sf;
    if(sf<7) sf=7;
    if(sf>12) sf=12;

    uint8_t crc_bit = (cfg->crc_on ? (1<<2) : 0);
    uint8_t mc2 = (sf<<4) | crc_bit | 0x03; // keep timeout MSB bits

    // 5) ModemConfig3: LDO + AGC
    uint8_t ldo = need_ldo(sf,cfg->bw_khz) ? (1<<3) : 0;
    uint8_t agc = (1<<2);
    uint8_t mc3 = ldo | agc;

    lora_write_reg(REG_MODEM_CONFIG_1, mc1);
    lora_write_reg(REG_MODEM_CONFIG_2, mc2);
    lora_write_reg(REG_MODEM_CONFIG_3, mc3);

    // TX power (wie gehabt)
    lora_write_reg(REG_PA_CONFIG, 0x8E);

    // RX continuous
    lora_write_reg(REG_FIFO_RX_BASE_ADDR,0);
    lora_write_reg(REG_FIFO_ADDR_PTR,0);
    lora_write_reg(REG_OP_MODE,0x85);

    ESP_LOGI(TAG,"APPLY done: MC1=0x%02X MC2=0x%02X MC3=0x%02X", mc1, mc2, mc3);
}

static void lora_init_hw(void)
{
    lora_apply_config(&g_cfg);
}

// ============================================================================
// Beacon (TX/RX)
// ============================================================================
static void beacon_schedule_next(void)
{
    uint32_t jitter = (g_beacon_jitter_ms ? (esp_random()%g_beacon_jitter_ms) : 0);
    next_beacon_ms = now_ms() + g_beacon_interval_ms + jitter;
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

    call7_set(b.src, g_cfg.call);
    call7_set(b.final_dst,"*");
    call7_set(b.next_hop,"*");
    call7_set(b.last_hop, g_cfg.call);
    b.payload_len=0;

    ESP_LOGI(TAG,"TX BEACON id=%u call=%s", b.msg_id, g_cfg.call);
    lora_send_packet((uint8_t*)&b,sizeof(b));

    beacon_schedule_next();
}

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

    if(len < sizeof(mr_hdr_v3_t)) return;

    mr_hdr_v3_t *h=(mr_hdr_v3_t*)buf;

    if(h->magic[0]!='M'||h->magic[1]!='R') return;
    if(h->version!=MR_PROTO_VERSION) return;

    if(seen_before(h->src,h->msg_id)) return;
    remember_msg(h->src,h->msg_id);

    int rssi = (int)lora_read_reg(REG_PKT_RSSI_VALUE) - 157;

    if(h->flags & MR_FLAG_BEACON){
        neighbor_update(h->last_hop, rssi);
        route_update(h->src, h->last_hop, rssi);

        char ssrc[8], shop[8];
        call7_to_cstr(ssrc, h->src);
        call7_to_cstr(shop, h->last_hop);

        ESP_LOGI(TAG,"BEACON RX from=%s via=%s rssi=%d", ssrc, shop, rssi);
    }
}

// ============================================================================
// DIO0 ISR / Task
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
// CLI (Kapitel 22)
// ============================================================================
static void cli_print_help(void)
{
    printf(
        "\nMeshRadio CLI (Kapitel 22A)\n"
        "----------------------------------------\n"
        "help                 - diese Hilfe\n"
        "show                 - aktuelle Config anzeigen\n"
        "call <CALL>          - Callsign setzen (max 7)\n"
        "freq <HZ>            - Frequenz in Hz (z.B. 433775000)\n"
        "sf <7..12>           - Spreading Factor setzen\n"
        "bw <125|250|500>     - Bandbreite (kHz)\n"
        "crc <0|1>            - CRC aus/an\n"
        "apply                - Config auf LoRa anwenden\n"
        "save                 - Config in NVS speichern\n"
        "load                 - Config aus NVS laden + apply\n"
        "beacon <0|1>         - Beacon senden aus/an\n"
        "beaconint <ms>       - Beacon-Intervall setzen\n"
        "reboot               - Neustart\n"
        "\nBeispiel:\n"
        "  call DL1XYZ\n"
        "  freq 433900000\n"
        "  sf 12\n"
        "  bw 125\n"
        "  crc 1\n"
        "  apply\n"
        "  save\n\n"
    );
}

static void cli_print_show(void)
{
    printf("\nCFG:\n");
    printf("  call=%s\n", g_cfg.call);
    printf("  freq=%" PRIu32 " Hz\n", g_cfg.freq_hz);
    printf("  sf=%u\n", g_cfg.sf);
    printf("  bw=%u kHz\n", g_cfg.bw_khz);
    printf("  crc=%u\n", g_cfg.crc_on);
    printf("Beacon:\n");
    printf("  enabled=%u\n", g_beacon_enabled ? 1 : 0);
    printf("  interval=%" PRIu32 " ms\n", g_beacon_interval_ms);
    printf("\n");
}

static bool cli_set_call(const char *arg)
{
    if(!arg || !*arg) return false;
    size_t n=strlen(arg);
    if(n==0 || n>7) return false;

    memset(g_cfg.call,0,sizeof(g_cfg.call));
    strncpy(g_cfg.call,arg,sizeof(g_cfg.call)-1);
    return true;
}

static bool cli_set_u32(const char *arg, uint32_t *out)
{
    if(!arg || !*arg) return false;
    char *end=NULL;
    unsigned long v=strtoul(arg,&end,10);
    if(end==arg || *end!=0) return false;
    *out=(uint32_t)v;
    return true;
}

static bool cli_set_u8_range(const char *arg, uint8_t *out, uint8_t minv, uint8_t maxv)
{
    uint32_t tmp=0;
    if(!cli_set_u32(arg,&tmp)) return false;
    if(tmp<minv || tmp>maxv) return false;
    *out=(uint8_t)tmp;
    return true;
}

static bool cli_set_bw(const char *arg, uint8_t *out)
{
    uint32_t tmp=0;
    if(!cli_set_u32(arg,&tmp)) return false;
    if(tmp!=125 && tmp!=250 && tmp!=500) return false;
    *out=(uint8_t)tmp;
    return true;
}

static void cli_execute(char *line)
{
    while(*line==' ') line++;
    if(*line==0) return;

    char *cmd=line;
    char *arg=strchr(line,' ');
    if(arg){
        *arg=0; arg++;
        while(*arg==' ') arg++;
        if(*arg==0) arg=NULL;
    }

    if(strcmp(cmd,"help")==0){ cli_print_help(); return; }
    if(strcmp(cmd,"show")==0){ cli_print_show(); return; }

    if(strcmp(cmd,"call")==0){
        if(cli_set_call(arg)) printf("OK: call=%s (apply+save optional)\n", g_cfg.call);
        else printf("ERR: call <CALL> (max 7)\n");
        return;
    }

    if(strcmp(cmd,"freq")==0){
        uint32_t f=0;
        if(cli_set_u32(arg,&f) && f>100000000 && f<1000000000){
            g_cfg.freq_hz=f;
            printf("OK: freq=%" PRIu32 " (apply+save optional)\n", g_cfg.freq_hz);
        }else printf("ERR: freq <HZ>\n");
        return;
    }

    if(strcmp(cmd,"sf")==0){
        uint8_t v=0;
        if(cli_set_u8_range(arg,&v,7,12)){
            g_cfg.sf=v;
            printf("OK: sf=%u (apply+save optional)\n", g_cfg.sf);
        }else printf("ERR: sf <7..12>\n");
        return;
    }

    if(strcmp(cmd,"bw")==0){
        uint8_t v=0;
        if(cli_set_bw(arg,&v)){
            g_cfg.bw_khz=v;
            printf("OK: bw=%u (apply+save optional)\n", g_cfg.bw_khz);
        }else printf("ERR: bw <125|250|500>\n");
        return;
    }

    if(strcmp(cmd,"crc")==0){
        uint8_t v=0;
        if(cli_set_u8_range(arg,&v,0,1)){
            g_cfg.crc_on=v;
            printf("OK: crc=%u (apply+save optional)\n", g_cfg.crc_on);
        }else printf("ERR: crc <0|1>\n");
        return;
    }

    if(strcmp(cmd,"apply")==0){
        lora_apply_config(&g_cfg);
        printf("OK: applied\n");
        return;
    }

    if(strcmp(cmd,"save")==0){
        esp_err_t e=config_save(&g_cfg);
        if(e==ESP_OK) printf("OK: saved to NVS\n");
        else printf("ERR: save failed: %s\n", esp_err_to_name(e));
        return;
    }

    if(strcmp(cmd,"load")==0){
        esp_err_t e=config_load_or_defaults(&g_cfg);
        if(e==ESP_OK){
            lora_apply_config(&g_cfg);
            printf("OK: loaded from NVS and applied\n");
        }else printf("ERR: load failed: %s\n", esp_err_to_name(e));
        return;
    }

    if(strcmp(cmd,"beacon")==0){
        uint8_t v=0;
        if(cli_set_u8_range(arg,&v,0,1)){
            g_beacon_enabled = (v==1);
            printf("OK: beacon enabled=%u\n", g_beacon_enabled?1:0);
            if(g_beacon_enabled) beacon_schedule_next();
        }else printf("ERR: beacon <0|1>\n");
        return;
    }

    if(strcmp(cmd,"beaconint")==0){
        uint32_t ms=0;
        if(cli_set_u32(arg,&ms) && ms>=1000 && ms<=600000){
            g_beacon_interval_ms = ms;
            printf("OK: beacon interval=%" PRIu32 " ms\n", g_beacon_interval_ms);
            if(g_beacon_enabled) beacon_schedule_next();
        }else printf("ERR: beaconint <ms> (1000..600000)\n");
        return;
    }

    if(strcmp(cmd,"reboot")==0){
        printf("Reboot...\n");
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
        return;
    }

    printf("ERR: unknown command '%s' (type: help)\n", cmd);
}

static void cli_poll(void)
{
    int c=getchar();
    if(c==EOF) return;

    if(c=='\r') return;
    if(c=='\n'){
        cli_line[cli_pos]=0;
        cli_execute(cli_line);
        cli_pos=0;
        return;
    }
    if(cli_pos < (int)sizeof(cli_line)-1)
        cli_line[cli_pos++] = (char)c;
}

// ============================================================================
// MAIN
// ============================================================================
void app_main(void)
{
    ESP_LOGI(TAG,"MeshRadio Kapitel 22A start (CLI + NVS + SF/BW/CRC)");

    // NVS init
    esp_err_t err = nvs_flash_init();
    if(err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND){
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }else{
        ESP_ERROR_CHECK(err);
    }

    // Load config
    ESP_ERROR_CHECK(config_load_or_defaults(&g_cfg));
    ESP_LOGI(TAG,"CFG: call=%s freq=%" PRIu32 " sf=%u bw=%u crc=%u",
             g_cfg.call, g_cfg.freq_hz, g_cfg.sf, g_cfg.bw_khz, g_cfg.crc_on);

    // SPI + Reset
    init_spi();
    lora_reset();
    ESP_LOGI(TAG,"SX1276 RegVersion=0x%02X", lora_read_reg(REG_VERSION));

    // Apply config to LoRa (now real SF/BW/CRC)
    lora_init_hw();

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

    // Beacon schedule
    beacon_schedule_next();

    // CLI banner
    printf("\nMeshRadio CLI ready (115200). Type 'help'.\n");

    // Main loop
    while(1){
        vTaskDelay(pdMS_TO_TICKS(20));

        // CLI poll
        cli_poll();

        // Beacon
        if(g_beacon_enabled && now_ms() > next_beacon_ms)
            send_beacon();

        // Maintenance
        neighbor_cleanup();
        route_cleanup();
    }
}
