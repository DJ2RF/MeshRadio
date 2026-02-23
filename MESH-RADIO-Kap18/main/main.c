// ============================================================================
// MeshRadio 4.0 – Kapitel 18 (Sourcen komplett)
// ----------------------------------------------------------------------------
// ROUTE LEARNING (Next-Hop Routing) + Fallback Flooding
//
// Basierend auf Kapitel 17:
//   ✔ Store-and-Forward (Outbox + TX Worker)
//   ✔ Directed Messages (@CALL text)
//   ✔ ACK + Retry für Directed Messages
//   ✔ Neighbor Ranking / Preferred Forwarding
//
// NEU in Kapitel 18:
//   ✔ Route Table (destination -> next_hop)
//   ✔ Route Learning beim Empfang (src -> sender als next_hop)
//   ✔ Directed Forwarding über Next-Hop statt Broadcast
//   ✔ Fallback Flooding wenn Route unbekannt / ACK fails
//   ✔ Route Aging (Timeout Cleanup)
//
// Hinweis (wichtig, bewusst einfach):
//   - Wir modellieren "Next-Hop" als Callsign.
//   - Für "gezieltes Senden" nutzen wir in diesem einfachen Mesh-Protokoll
//     weiterhin LoRa Broadcast, aber mit einem Next-Hop Feld im Header (dst),
//     so dass nur der gewünschte Next-Hop forwarded.
//   - Das reduziert Flooding stark, obwohl physisch broadcast gesendet wird.
//
// Amateurfunk:
//   - kein Crypto
//   - Callsigns sichtbar
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

// Beacon
#define BEACON_INTERVAL_S 10

// Duplicate Cache
#define SEEN_CACHE_SIZE 32

// Neighbor Tabelle
#define MAX_NEIGHBORS          20
#define NEIGHBOR_TIMEOUT_MS    60000

// Preferred Forwarding
#define PREFERRED_TOP_N        3
#define FORWARD_MIN_RSSI       (-105)

// UART Statusausgabe
#define UART_STATUS_INTERVAL_S 5

// ---------- Kapitel 18 Routing ----------
#define MAX_ROUTES          20
#define ROUTE_TIMEOUT_MS    120000   // Route nach 120s ohne Update verwerfen
#define ROUTE_MIN_RSSI      (-115)   // Route nur speichern, wenn Link nicht extrem schlecht

// Wenn ein directed Chat trotz Retries kein ACK bekommt,
// löschen wir die Route und fallen auf Flooding zurück.
#define ACK_FAIL_DROP_ROUTE 1

// ---------- Flags ----------
#define MR_FLAG_BEACON 0x04
#define MR_FLAG_CHAT   0x08
#define MR_FLAG_ACK    0x10

// ---------- Outbox ----------
#define OUTBOX_SIZE            16
#define OUTBOX_MAX_RETRIES     5
#define OUTBOX_EXPIRE_MS       60000
#define OUTBOX_BACKOFF_BASE_MS 800
#define OUTBOX_BACKOFF_MAX_MS  10000
#define OUTBOX_JITTER_MS       200
#define TX_WORKER_TICK_MS      50

// ---------- Payload ----------
#define MR_MAX_PAYLOAD 64

// ============================================================================
// LoRa Pins (TTGO T-Beam)
// ============================================================================

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
// Frame Definition
// ============================================================================

#pragma pack(push,1)
typedef struct {
    uint8_t  magic[2];
    uint8_t  version;
    uint8_t  flags;

    // TTL = Hop-Limit
    uint8_t  ttl;

    // msg_id muss pro SRC eindeutig sein
    uint16_t msg_id;

    // src = ursprünglicher Sender
    char     src[7];

    // dst = Next-Hop oder Ziel (siehe Kapitel 18)
    //   "*" => Broadcast
    //   CALL => "nur dieser Next-Hop soll weiterleiten / reagieren"
    char     dst[7];

    uint8_t  payload_len;
} mr_hdr_t;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct {
    mr_hdr_t h;
    char payload[MR_MAX_PAYLOAD];
} mr_chat_frame_t;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct {
    mr_hdr_t h;
    uint16_t ack_id;
} mr_ack_frame_t;
#pragma pack(pop)

// ============================================================================
// Seen + Neighbor + Route Table
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

// Kapitel 18 Route Entry: destination -> next_hop
typedef struct {
    bool used;
    char destination[7];
    char next_hop[7];
    int  rssi_dbm;          // RSSI beim Lernen
    uint32_t last_seen_ms;  // wann Route zuletzt aktualisiert wurde
} route_entry_t;

// ============================================================================
// Outbox Job
// ============================================================================

typedef enum {
    JOB_CHAT_LOCAL = 0,
    JOB_FORWARD    = 1,
    JOB_BEACON     = 2,
    JOB_ACK        = 3
} outbox_kind_t;

typedef struct {
    bool used;
    outbox_kind_t kind;

    uint8_t frame[256];
    uint16_t frame_len;

    uint8_t retries_left;
    uint32_t next_try_ms;
    uint32_t expires_ms;

    uint16_t msg_id;
    char src[7];

    // Kapitel 17/18
    bool needs_ack;

    // Kapitel 18: Für directed Jobs merken wir das eigentliche Endziel.
    // Damit wir beim ACK-Fail Route löschen können.
    char final_dst[7]; // nur sinnvoll, wenn needs_ack=true
} outbox_job_t;

// ============================================================================
// Globals
// ============================================================================

static const char *TAG="MR18";

static spi_device_handle_t lora_spi;
static QueueHandle_t dio0_evt_queue;

static uint16_t g_msg_id=1;

static seen_msg_t   seen_cache[SEEN_CACHE_SIZE];
static neighbor_t   neighbors[MAX_NEIGHBORS];
static route_entry_t routes[MAX_ROUTES];

static outbox_job_t outbox[OUTBOX_SIZE];
static SemaphoreHandle_t outbox_mutex;

static char uart_line[80];
static int uart_pos=0;

// ============================================================================
// Prototypen
// ============================================================================

static void lora_send_packet(uint8_t *data,size_t len);
static bool lora_wait_tx_done_polling(int timeout_ms);

static void outbox_process_due_jobs(void);
static void outbox_ack_received(uint16_t ack_id);
static void outbox_fail_cleanup_route_if_needed(outbox_job_t *job);

static void route_update(const char destination[7], const char next_hop[7], int rssi_dbm);
static bool route_lookup_next_hop(const char destination[7], char out_next_hop[7]);
static void route_cleanup(void);

// ============================================================================
// Helper
// ============================================================================

static uint32_t now_ms(void){ return xTaskGetTickCount()*portTICK_PERIOD_MS; }

static void call7_set(char out7[7], const char *call)
{
    memset(out7,' ',7);
    size_t n=strlen(call);
    if(n>7)n=7;
    memcpy(out7,call,n);
}

static void call7_to_cstr(char out8[8], const char in7[7])
{
    memcpy(out8,in7,7);
    out8[7]=0;
}

static bool call7_eq(const char a[7], const char b[7])
{
    return memcmp(a,b,7)==0;
}

// ============================================================================
// Duplicate Detection
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
// Neighbor
// ============================================================================

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
        if(t-neighbors[i].last_seen_ms > NEIGHBOR_TIMEOUT_MS)
            neighbors[i].used=false;
    }
}

static bool neighbor_is_preferred(const char call[7])
{
    int my=-9999, better=0;

    for(int i=0;i<MAX_NEIGHBORS;i++){
        if(!neighbors[i].used) continue;
        if(call7_eq(neighbors[i].call,call))
            my=neighbors[i].rssi_dbm;
    }
    if(my==-9999) return false;

    for(int i=0;i<MAX_NEIGHBORS;i++){
        if(!neighbors[i].used) continue;
        if(neighbors[i].rssi_dbm > my)
            better++;
    }
    return (better < PREFERRED_TOP_N);
}

// ============================================================================
// Kapitel 18: ROUTE TABLE
// ----------------------------------------------------------------------------
// Route Learning:
//
// Wenn wir ein Paket mit src=DL1XYZ empfangen, das über einen Nachbarn kam,
// dann wissen wir:
//   destination=DL1XYZ ist über next_hop=<Nachbar> erreichbar.
//
// In diesem einfachen Protokoll ist "Nachbar" ebenfalls die src des Frames,
// weil wir physisch Broadcast senden. Praktisch bedeutet das:
//   - Direkt gehört = direkter Nachbar
//   - Bei Multi-Hop werden Routen über ACK / regelmäßige Seen Updates stabiler.
//
// (Später könnte man ein extra Feld "via" einführen; hier bleiben wir minimal.)
// ============================================================================

static void route_update(const char destination[7], const char next_hop[7], int rssi_dbm)
{
    if(rssi_dbm < ROUTE_MIN_RSSI)
        return;

    uint32_t t=now_ms();

    // update existing
    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used) continue;
        if(call7_eq(routes[i].destination,destination)){
            memcpy(routes[i].next_hop,next_hop,7);
            routes[i].rssi_dbm=rssi_dbm;
            routes[i].last_seen_ms=t;
            return;
        }
    }

    // new slot
    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used){
            routes[i].used=true;
            memcpy(routes[i].destination,destination,7);
            memcpy(routes[i].next_hop,next_hop,7);
            routes[i].rssi_dbm=rssi_dbm;
            routes[i].last_seen_ms=t;
            return;
        }
    }

    // table full -> overwrite oldest
    int oldest=0;
    for(int i=1;i<MAX_ROUTES;i++){
        if(!routes[i].used){ oldest=i; break; }
        if(routes[i].last_seen_ms < routes[oldest].last_seen_ms)
            oldest=i;
    }
    routes[oldest].used=true;
    memcpy(routes[oldest].destination,destination,7);
    memcpy(routes[oldest].next_hop,next_hop,7);
    routes[oldest].rssi_dbm=rssi_dbm;
    routes[oldest].last_seen_ms=t;
}

static bool route_lookup_next_hop(const char destination[7], char out_next_hop[7])
{
    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used) continue;
        if(call7_eq(routes[i].destination,destination)){
            memcpy(out_next_hop,routes[i].next_hop,7);
            return true;
        }
    }
    return false;
}

static void route_delete(const char destination[7])
{
    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used) continue;
        if(call7_eq(routes[i].destination,destination)){
            routes[i].used=false;
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

// ============================================================================
// SPI Access
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

static void lora_clear_irqs(void){ lora_write_reg(REG_IRQ_FLAGS,0xFF); }

// ============================================================================
// LoRa Init
// ============================================================================

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
// TX
// ============================================================================

static bool lora_wait_tx_done_polling(int timeout_ms)
{
    while(timeout_ms>0){
        uint8_t irq=lora_read_reg(REG_IRQ_FLAGS);
        if(irq & IRQ_TX_DONE){
            lora_clear_irqs();
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout_ms-=10;
    }
    return false;
}

static void lora_send_packet(uint8_t *data,size_t len)
{
    lora_clear_irqs();

    lora_write_reg(REG_FIFO_TX_BASE_ADDR,0);
    lora_write_reg(REG_FIFO_ADDR_PTR,0);

    for(size_t i=0;i<len;i++)
        lora_write_reg(REG_FIFO,data[i]);

    lora_write_reg(REG_PAYLOAD_LENGTH,len);

    lora_write_reg(REG_OP_MODE,0x83);
    lora_wait_tx_done_polling(2000);
    lora_write_reg(REG_OP_MODE,0x85);
}

// ============================================================================
// OUTBOX
// ============================================================================

static void outbox_init(void)
{
    outbox_mutex=xSemaphoreCreateMutex();
    memset(outbox,0,sizeof(outbox));
}

static bool outbox_enqueue(outbox_kind_t kind,
                           const uint8_t *frame,
                           uint16_t len,
                           uint16_t msg_id,
                           const char src[7],
                           bool needs_ack,
                           const char final_dst[7])
{
    if(xSemaphoreTake(outbox_mutex,pdMS_TO_TICKS(100))!=pdTRUE)
        return false;

    int slot=-1;
    for(int i=0;i<OUTBOX_SIZE;i++)
        if(!outbox[i].used){slot=i;break;}

    if(slot<0){
        xSemaphoreGive(outbox_mutex);
        ESP_LOGW(TAG,"OUTBOX FULL");
        return false;
    }

    outbox[slot].used=true;
    outbox[slot].kind=kind;

    memcpy(outbox[slot].frame,frame,len);
    outbox[slot].frame_len=len;

    outbox[slot].retries_left=OUTBOX_MAX_RETRIES;
    outbox[slot].next_try_ms=now_ms();
    outbox[slot].expires_ms=now_ms()+OUTBOX_EXPIRE_MS;

    outbox[slot].msg_id=msg_id;
    memcpy(outbox[slot].src,src,7);

    outbox[slot].needs_ack=needs_ack;
    if(final_dst) memcpy(outbox[slot].final_dst,final_dst,7);
    else call7_set(outbox[slot].final_dst,"*");

    xSemaphoreGive(outbox_mutex);

    return true;
}

// ACK angekommen -> passenden Job löschen
static void outbox_ack_received(uint16_t ack_id)
{
    if(xSemaphoreTake(outbox_mutex,pdMS_TO_TICKS(100))!=pdTRUE)
        return;

    for(int i=0;i<OUTBOX_SIZE;i++){
        if(!outbox[i].used) continue;

        if(outbox[i].needs_ack && outbox[i].msg_id==ack_id){
            ESP_LOGI(TAG,"ACK RX id=%u -> remove job",ack_id);
            outbox[i].used=false;
        }
    }

    xSemaphoreGive(outbox_mutex);
}

// Wenn Retries leer -> optional Route löschen
static void outbox_fail_cleanup_route_if_needed(outbox_job_t *job)
{
    if(!job->needs_ack) return;

#if ACK_FAIL_DROP_ROUTE
    // Route für final_dst löschen -> nächster Versuch fällt auf Flooding zurück
    route_delete(job->final_dst);

    char dst8[8];
    call7_to_cstr(dst8,job->final_dst);
    ESP_LOGW(TAG,"ACK FAIL -> drop route for %.7s",dst8);
#endif
}

static void outbox_process_due_jobs(void)
{
    uint32_t t=now_ms();

    if(xSemaphoreTake(outbox_mutex,pdMS_TO_TICKS(50))!=pdTRUE)
        return;

    for(int i=0;i<OUTBOX_SIZE;i++){
        if(!outbox[i].used) continue;

        if(t>=outbox[i].expires_ms){
            ESP_LOGW(TAG,"OUTBOX expired id=%u",outbox[i].msg_id);
            outbox_fail_cleanup_route_if_needed(&outbox[i]);
            outbox[i].used=false;
            continue;
        }

        if(t<outbox[i].next_try_ms)
            continue;

        // send
        lora_send_packet(outbox[i].frame,outbox[i].frame_len);

        if(!outbox[i].needs_ack){
            outbox[i].used=false;
            continue;
        }

        if(outbox[i].retries_left==0){
            ESP_LOGW(TAG,"OUTBOX no retries left id=%u",outbox[i].msg_id);
            outbox_fail_cleanup_route_if_needed(&outbox[i]);
            outbox[i].used=false;
            continue;
        }

        outbox[i].retries_left--;

        // Backoff (sehr einfach)
        uint8_t attempt = (uint8_t)(OUTBOX_MAX_RETRIES - outbox[i].retries_left);
        uint32_t backoff = OUTBOX_BACKOFF_BASE_MS << (attempt>5?5:attempt);
        if(backoff>OUTBOX_BACKOFF_MAX_MS) backoff=OUTBOX_BACKOFF_MAX_MS;
        backoff += (esp_random()%OUTBOX_JITTER_MS);

        outbox[i].next_try_ms=t+backoff;
    }

    xSemaphoreGive(outbox_mutex);
}

static void tx_worker_task(void *arg)
{
    (void)arg;
    while(1){
        outbox_process_due_jobs();
        vTaskDelay(pdMS_TO_TICKS(TX_WORKER_TICK_MS));
    }
}

// ============================================================================
// ACK SENDEN
// ============================================================================

static void enqueue_ack(const char dst[7], uint16_t ack_id)
{
    mr_ack_frame_t a={0};

    a.h.magic[0]='M';
    a.h.magic[1]='R';
    a.h.version=1;
    a.h.flags=MR_FLAG_ACK;
    a.h.ttl=2;
    a.h.msg_id=g_msg_id++;

    call7_set(a.h.src,MY_CALL);
    memcpy(a.h.dst,dst,7);

    a.h.payload_len=2;
    a.ack_id=ack_id;

    outbox_enqueue(JOB_ACK,(uint8_t*)&a,sizeof(a),a.h.msg_id,a.h.src,false,NULL);
}

// ============================================================================
// CHAT BUILD: Route Learning Anwendung
// ----------------------------------------------------------------------------
// - Bei directed Chat versuchen wir Route Lookup.
// - Wenn Route bekannt: dst = next_hop (gezieltes Forwarding).
// - Wenn Route unbekannt: dst="*" (Flooding Fallback).
//
// Wichtig:
//   final_dst merkt das eigentliche Ziel (für ACK-Fail -> Route drop).
// ============================================================================

static void enqueue_chat_line(const char *line)
{
    mr_chat_frame_t f={0};

    f.h.magic[0]='M';
    f.h.magic[1]='R';
    f.h.version=1;
    f.h.flags=MR_FLAG_CHAT;
    f.h.ttl=5; // mit Routing kann TTL ruhig etwas höher sein
    f.h.msg_id=g_msg_id++;

    call7_set(f.h.src,MY_CALL);

    const char *text=line;
    bool directed=false;

    char final_dst7[7];
    call7_set(final_dst7,"*");

    if(line[0]=='@'){
        directed=true;
        char call[8]={0};
        int i=1,j=0;
        while(line[i] && line[i]!=' ' && j<7)
            call[j++]=line[i++];
        call7_set(final_dst7,call);
        if(line[i]==' ') text=&line[i+1];

        // Route Lookup
        char nh[7];
        if(route_lookup_next_hop(final_dst7,nh)){
            memcpy(f.h.dst,nh,7); // Next Hop
        }else{
            call7_set(f.h.dst,"*"); // Fallback Flooding
        }
    }else{
        call7_set(f.h.dst,"*");
    }

    size_t len=strlen(text);
    if(len>MR_MAX_PAYLOAD) len=MR_MAX_PAYLOAD;
    memcpy(f.payload,text,len);
    f.h.payload_len=len;

    outbox_enqueue(JOB_CHAT_LOCAL,
                   (uint8_t*)&f,
                   (uint16_t)(sizeof(mr_hdr_t)+len),
                   f.h.msg_id,
                   f.h.src,
                   directed,           // needs_ack
                   directed?final_dst7:NULL);

    ESP_LOGI(TAG,"CHAT queued id=%u directed=%d",f.h.msg_id,directed);
}

// UART
static void uart_poll_input(void)
{
    int c=getchar();
    if(c==EOF) return;

    if(c=='\n'||c=='\r'){
        uart_line[uart_pos]=0;
        if(uart_pos>0)
            enqueue_chat_line(uart_line);
        uart_pos=0;
        return;
    }

    if(uart_pos<79) uart_line[uart_pos++]=(char)c;
}

// ============================================================================
// RX Handler
// ----------------------------------------------------------------------------
// Kapitel 18 Kerngedanke:
//   - Wir forwarden nicht mehr immer.
//   - Bei directed Frames (dst != "*"):
//       -> Nur weiterleiten, wenn dst == MY_CALL (wir sind Next-Hop)
//   - Route Learning:
//       -> route_update(src, next_hop, rssi) bei jedem RX
// ============================================================================

static void maybe_forward(mr_hdr_t *h, uint8_t *buf, uint16_t len, int rssi)
{
    // TTL?
    if(h->ttl<=1) return;

    // RSSI Gate + Preferred Gate
    if(rssi < FORWARD_MIN_RSSI) return;
    if(!neighbor_is_preferred(h->src)) return;

    // Directed Next-Hop?
    char star7[7]; call7_set(star7,"*");

    // Wenn dst != "*" bedeutet: nur bestimmter Next-Hop soll forwarden.
    if(!call7_eq(h->dst,star7)){
        char my7[7]; call7_set(my7,MY_CALL);
        if(!call7_eq(h->dst,my7)){
            // wir sind NICHT Next-Hop -> nicht forwarden
            return;
        }

        // Wir sind Next-Hop -> weiterleiten Richtung Endziel:
        // Wir versuchen Route Lookup für "final destination".
        // In diesem minimalen Protokoll kennen wir das Endziel nicht explizit im Header.
        // Daher: wir leiten weiter als Broadcast ("*"), aber setzen dst auf nächsten Hop,
        // indem wir "dst" auf ein gelerntes Next-Hop setzen, falls wir für (original src?)...
        // -> Vereinfachung: Wir behalten dst="*" ab hier (kleines Mesh).
        // Leserhinweis: echtes Endziel-Feld wäre ein nächstes Kapitel.

        // Hier: Fallback Flooding ab dem Next-Hop (damit Paket weiterkommt).
        call7_set(h->dst,"*");
    }

    // TTL reduzieren und in Outbox
    h->ttl--;
    outbox_enqueue(JOB_FORWARD,buf,len,h->msg_id,h->src,false,NULL);
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

    if(len<sizeof(mr_hdr_t)) return;

    mr_hdr_t *h=(mr_hdr_t*)buf;

    if(h->magic[0]!='M'||h->magic[1]!='R') return;

    if(seen_before(h->src,h->msg_id))
        return;
    remember_msg(h->src,h->msg_id);

    int rssi=lora_read_reg(REG_PKT_RSSI_VALUE)-157;

    // Neighbor + Route Learning
    neighbor_update(h->src,rssi);
    // Route: destination=src, next_hop=src (direkt gehört)
    route_update(h->src,h->src,rssi);

    // ACK?
    if(h->flags & MR_FLAG_ACK){
        char my7[7]; call7_set(my7,MY_CALL);
        if(call7_eq(h->dst,my7)){
            mr_ack_frame_t *a=(mr_ack_frame_t*)buf;
            outbox_ack_received(a->ack_id);
        }
        return;
    }

    // CHAT?
    if(h->flags & MR_FLAG_CHAT){
        char src8[8]; call7_to_cstr(src8,h->src);

        uint8_t pl=h->payload_len;
        if(pl>MR_MAX_PAYLOAD) pl=MR_MAX_PAYLOAD;

        char msg[MR_MAX_PAYLOAD+1]={0};
        uint16_t avail=len-sizeof(mr_hdr_t);
        uint16_t cp=pl;
        if(cp>avail) cp=avail;
        memcpy(msg,buf+sizeof(mr_hdr_t),cp);
        msg[cp]=0;

        ESP_LOGI(TAG,"CHAT %.7s > %s (RSSI=%d)",src8,msg,rssi);

        // Directed an uns? -> ACK senden
        char my7[7]; call7_set(my7,MY_CALL);
        char star7[7]; call7_set(star7,"*");

        if(!call7_eq(h->dst,star7) && call7_eq(h->dst,my7)){
            // Wir sind Next-Hop oder Endziel (hier gleichbehandelt)
            enqueue_ack(h->src,h->msg_id);
        }

        // Forwarding-Entscheidung (Kapitel 18)
        maybe_forward(h,buf,len,rssi);
    }
}

// ============================================================================
// ISR + TASK
// ============================================================================

static void IRAM_ATTR dio0_isr(void*arg)
{
    uint32_t n=(uint32_t)arg;
    xQueueSendFromISR(dio0_evt_queue,&n,NULL);
}

static void dio0_task(void*arg)
{
    (void)arg;
    uint32_t io;
    while(1){
        if(xQueueReceive(dio0_evt_queue,&io,portMAX_DELAY))
            handle_rx_packet();
    }
}

// ============================================================================
// SPI Init
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

// ============================================================================
// MAIN
// ============================================================================

void app_main(void)
{
    ESP_LOGI(TAG,"Kapitel 18 start (%s)",MY_CALL);

    init_spi();
    lora_reset();

    uint8_t v=lora_read_reg(REG_VERSION);
    ESP_LOGI(TAG,"SX1276 version=0x%02X",v);

    lora_init();

    outbox_init();
    xTaskCreate(tx_worker_task,"txw",4096,NULL,9,NULL);

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

    uint32_t last_beacon=now_ms();
    uint32_t last_status=now_ms();

    while(1){
        vTaskDelay(pdMS_TO_TICKS(50));

        uart_poll_input();

        neighbor_cleanup();
        route_cleanup();

        if(now_ms()-last_beacon > BEACON_INTERVAL_S*1000){
            last_beacon=now_ms();
            // Beacon optional (Kap16)
        }

        if(UART_STATUS_INTERVAL_S>0 &&
           now_ms()-last_status > UART_STATUS_INTERVAL_S*1000){
            last_status=now_ms();
            // optional: neighbor/route status print
        }
    }
}
