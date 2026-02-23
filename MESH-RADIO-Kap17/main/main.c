// ============================================================================
// MeshRadio 4.0 – Kapitel 17
// ----------------------------------------------------------------------------
// ACK + Retry für gerichtete Nachrichten (Directed Messages)
//
// Basierend auf Kapitel 16 (Store-and-Forward):
//   ✔ Outbox + TX Worker
//   ✔ Duplicate Detection
//   ✔ Neighbor Ranking / Preferred Forwarding
//   ✔ UART Chat
//
// NEU in Kapitel 17:
//   ✔ Directed Messages (@CALL text)
//   ✔ ACK Frame (MR_FLAG_ACK)
//   ✔ Outbox Jobs können ACK erwarten
//   ✔ Retry + Backoff bis ACK empfangen
//   ✔ ACK entfernt Job aus Outbox
//
// Amateurfunk-konform:
//   - keine Verschlüsselung
//   - Callsigns immer sichtbar
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

#define MY_CALL "DJ1ABC"

#define BEACON_INTERVAL_S 10

#define SEEN_CACHE_SIZE 32

#define MAX_NEIGHBORS          20
#define NEIGHBOR_TIMEOUT_MS    60000

#define PREFERRED_TOP_N        3
#define FORWARD_MIN_RSSI       (-105)

#define UART_STATUS_INTERVAL_S 5

// ---------- Kapitel 17 ACK ----------
#define MR_FLAG_BEACON 0x04
#define MR_FLAG_CHAT   0x08
#define MR_FLAG_ACK    0x10   // NEU

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
    uint8_t  ttl;
    uint16_t msg_id;
    char     src[7];
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

// ACK Frame (Kapitel 17)
#pragma pack(push,1)
typedef struct {
    mr_hdr_t h;
    uint16_t ack_id;
} mr_ack_frame_t;
#pragma pack(pop)

// ============================================================================
// Seen + Neighbor
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

// ============================================================================
// Outbox Job (Kapitel 17 erweitert)
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

    // Kapitel 17
    bool needs_ack;
} outbox_job_t;

// ============================================================================
// Globals
// ============================================================================

static const char *TAG="MR17";

static spi_device_handle_t lora_spi;
static QueueHandle_t dio0_evt_queue;

static uint16_t g_msg_id=1;

static seen_msg_t seen_cache[SEEN_CACHE_SIZE];
static neighbor_t neighbors[MAX_NEIGHBORS];

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

// ============================================================================
// Helper
// ============================================================================

static uint32_t now_ms(void)
{
    return xTaskGetTickCount()*portTICK_PERIOD_MS;
}

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
        if(call7_eq(seen_cache[i].src,src) &&
           seen_cache[i].msg_id==id)
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
    int my=-9999;
    int better=0;

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

static void lora_clear_irqs(void)
{
    lora_write_reg(REG_IRQ_FLAGS,0xFF);
}

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
                           bool needs_ack)
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

    xSemaphoreGive(outbox_mutex);

    ESP_LOGI(TAG,"OUTBOX add id=%u ack=%d",msg_id,needs_ack);
    return true;
}

// ACK angekommen -> passenden Job löschen
static void outbox_ack_received(uint16_t ack_id)
{
    if(xSemaphoreTake(outbox_mutex,pdMS_TO_TICKS(100))!=pdTRUE)
        return;

    for(int i=0;i<OUTBOX_SIZE;i++){
        if(!outbox[i].used) continue;

        if(outbox[i].needs_ack &&
           outbox[i].msg_id==ack_id){

            ESP_LOGI(TAG,"ACK RX id=%u -> remove job",ack_id);
            outbox[i].used=false;
        }
    }

    xSemaphoreGive(outbox_mutex);
}

// TX Worker
static void outbox_process_due_jobs(void)
{
    uint32_t t=now_ms();

    if(xSemaphoreTake(outbox_mutex,pdMS_TO_TICKS(50))!=pdTRUE)
        return;

    for(int i=0;i<OUTBOX_SIZE;i++){

        if(!outbox[i].used) continue;

        if(t>=outbox[i].expires_ms){
            ESP_LOGW(TAG,"OUTBOX expired id=%u",outbox[i].msg_id);
            outbox[i].used=false;
            continue;
        }

        if(t<outbox[i].next_try_ms)
            continue;

        ESP_LOGI(TAG,"TX id=%u retry=%u",
                 outbox[i].msg_id,
                 outbox[i].retries_left);

        lora_send_packet(outbox[i].frame,outbox[i].frame_len);

        // ACK benötigt?
        if(!outbox[i].needs_ack){
            outbox[i].used=false;
            continue;
        }

        // retry scheduling
        if(outbox[i].retries_left>0)
            outbox[i].retries_left--;

        uint32_t backoff=OUTBOX_BACKOFF_BASE_MS <<
            (OUTBOX_MAX_RETRIES - outbox[i].retries_left);

        if(backoff>OUTBOX_BACKOFF_MAX_MS)
            backoff=OUTBOX_BACKOFF_MAX_MS;

        backoff += esp_random()%OUTBOX_JITTER_MS;

        outbox[i].next_try_ms=t+backoff;
    }

    xSemaphoreGive(outbox_mutex);
}

static void tx_worker_task(void *arg)
{
    while(1){
        outbox_process_due_jobs();
        vTaskDelay(pdMS_TO_TICKS(TX_WORKER_TICK_MS));
    }
}

// ============================================================================
// ACK SENDEN
// ============================================================================

static void send_ack(const char dst[7], uint16_t ack_id)
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

    outbox_enqueue(JOB_ACK,
                   (uint8_t*)&a,
                   sizeof(a),
                   a.h.msg_id,
                   a.h.src,
                   false);

    ESP_LOGI(TAG,"ACK queued for msg=%u",ack_id);
}

// ============================================================================
// CHAT SENDEN (UART)
// ----------------------------------------------------------------------------
// Syntax:
//   broadcast: Hello world
//   directed : @DL1XYZ hello
// ============================================================================

static void enqueue_chat_line(const char *line)
{
    mr_chat_frame_t f={0};

    f.h.magic[0]='M';
    f.h.magic[1]='R';
    f.h.version=1;
    f.h.flags=MR_FLAG_CHAT;
    f.h.ttl=3;
    f.h.msg_id=g_msg_id++;

    call7_set(f.h.src,MY_CALL);

    const char *text=line;
    bool directed=false;

    if(line[0]=='@'){
        directed=true;
        char call[8]={0};
        int i=1,j=0;
        while(line[i] && line[i]!=' ' && j<7)
            call[j++]=line[i++];
        call7_set(f.h.dst,call);
        if(line[i]==' ') text=&line[i+1];
    }else{
        call7_set(f.h.dst,"*");
    }

    size_t len=strlen(text);
    if(len>MR_MAX_PAYLOAD) len=MR_MAX_PAYLOAD;

    memcpy(f.payload,text,len);
    f.h.payload_len=len;

    outbox_enqueue(JOB_CHAT_LOCAL,
                   (uint8_t*)&f,
                   sizeof(mr_hdr_t)+len,
                   f.h.msg_id,
                   f.h.src,
                   directed); // ACK nur directed

    ESP_LOGI(TAG,"CHAT queued id=%u directed=%d",
             f.h.msg_id,directed);
}

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

    if(uart_pos<79)
        uart_line[uart_pos++]=(char)c;
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
    for(int i=0;i<len;i++)
        buf[i]=lora_read_reg(REG_FIFO);

    lora_clear_irqs();

    if(len<sizeof(mr_hdr_t)) return;

    mr_hdr_t *h=(mr_hdr_t*)buf;

    if(seen_before(h->src,h->msg_id))
        return;

    remember_msg(h->src,h->msg_id);

    int rssi=lora_read_reg(REG_PKT_RSSI_VALUE)-157;
    neighbor_update(h->src,rssi);

    // ---------------- ACK ----------------
    if(h->flags & MR_FLAG_ACK){

        if(call7_eq(h->dst,(char[7]){'D','J','2','R','F',' ',' '})){
            mr_ack_frame_t *a=(mr_ack_frame_t*)buf;
            outbox_ack_received(a->ack_id);
        }
        return;
    }

    // ---------------- CHAT ----------------
    if(h->flags & MR_FLAG_CHAT){

        char src8[8];
        call7_to_cstr(src8,h->src);

        char msg[65]={0};
        memcpy(msg,buf+sizeof(mr_hdr_t),h->payload_len);

        ESP_LOGI(TAG,"CHAT %.7s > %s",src8,msg);

        // Directed an uns -> ACK senden
        char my7[7];
        call7_set(my7,MY_CALL);

        if(call7_eq(h->dst,my7)){
            send_ack(h->src,h->msg_id);
        }

        // Forward?
        if(h->ttl>1 &&
           rssi>FORWARD_MIN_RSSI &&
           neighbor_is_preferred(h->src))
        {
            h->ttl--;

            outbox_enqueue(JOB_FORWARD,
                           buf,len,
                           h->msg_id,
                           h->src,
                           false);
        }
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
    ESP_LOGI(TAG,"Kapitel 17 start (%s)",MY_CALL);

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

    while(1){

        vTaskDelay(pdMS_TO_TICKS(50));

        uart_poll_input();

        neighbor_cleanup();

        if(now_ms()-last_beacon > BEACON_INTERVAL_S*1000){
            last_beacon=now_ms();
            // optional beacon (wie Kap16)
        }
    }
}
