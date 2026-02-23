// =========================================================
// Kapitel 10: ACK-System, Zielrufzeichen & einfache Chat-Funktion
// Vollständige, sauber dokumentierte SOURCEN (ESP-IDF, SX1276)
// Board: TTGO T-Beam V1.1 (433 MHz, SX1276)
// RX: DIO0 Interrupt (RxDone)
// TX: Polling (TxDone) -> robust, auch wenn DIO0-TxDone zickt
//
// Inhalt:
//  - MR-Frame v1 (Magic/Version/Flags/TTL/MsgID/Src/Dst/Payload)
//  - Flooding light aus Kapitel 9 (TTL--, Dedupe, Forward)
//  - Unicast/Broadcast
//  - ACK Request + ACK Response
//  - "Chat"-Demo: periodisches Senden + Ausgabe empfangener Nachrichten
//
// WICHTIG (Amateurfunk):
//  - Keine Verschlüsselung
//  - Rufzeichen im Frame (src/dst)
// =========================================================

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
#include "esp_err.h"

// ------------------------------
// Node-Identität (anpassen!)
// ------------------------------
#define MY_CALL "DJ1ABC"     // <= hier pro Node anpassen (max 7 Zeichen sinnvoll)
#define DEFAULT_TTL 3

// ------------------------------
// Pins TTGO T-Beam V1.1 (SX1276)
// ------------------------------
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 27
#define PIN_NUM_CLK  5
#define PIN_NUM_CS   18
#define PIN_NUM_RST  23
#define PIN_NUM_DIO0 26

#define LORA_SPI_HOST VSPI_HOST

static const char *TAG = "MR10";
static spi_device_handle_t lora_spi = NULL;
static QueueHandle_t dio0_evt_queue;

// ------------------------------
// SX1276 Register (LoRa)
// ------------------------------
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
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_MODEM_CONFIG_3       0x26
#define REG_VERSION              0x42
#define REG_DIO_MAPPING_1        0x40

// IRQ Bits (RegIrqFlags)
#define IRQ_RX_DONE              0x40
#define IRQ_PAYLOAD_CRC_ERROR    0x20
#define IRQ_TX_DONE              0x08

// ------------------------------
// MeshRadio Frame v1
// ------------------------------
#pragma pack(push, 1)
typedef struct {
    uint8_t magic[2];      // 'M','R'
    uint8_t version;       // 1
    uint8_t flags;         // ACK_REQ / ACK / ...
    uint8_t ttl;           // Hop-Limit
    uint16_t msg_id;       // Message-ID (für Dedupe/ACK)
    char src[7];           // Rufzeichen, mit Spaces aufgefüllt
    char dst[7];           // Zielrufzeichen oder "*" (Broadcast), mit Spaces
    uint8_t payload_len;   // Länge der Nutzdaten
    // payload folgt direkt im FIFO
} mr_hdr_t;
#pragma pack(pop)

// ------------------------------
// Flags (Kapitel 10)
// ------------------------------
#define MR_FLAG_ACK_REQ  0x01
#define MR_FLAG_ACK      0x02

// ------------------------------
// Dedupe Cache (Kapitel 9)
// msg_id + src reicht für Demo.
// ------------------------------
#define DEDUPE_SIZE 30

typedef struct {
    uint16_t msg_id;
    char src[7];
} dedupe_entry_t;

static dedupe_entry_t dedupe_cache[DEDUPE_SIZE];
static int dedupe_index = 0;

static bool is_duplicate(uint16_t msg_id, const char src[7])
{
    for (int i = 0; i < DEDUPE_SIZE; i++) {
        if (dedupe_cache[i].msg_id == msg_id &&
            memcmp(dedupe_cache[i].src, src, 7) == 0) {
            return true;
        }
    }
    return false;
}

static void add_to_dedupe(uint16_t msg_id, const char src[7])
{
    dedupe_cache[dedupe_index].msg_id = msg_id;
    memcpy(dedupe_cache[dedupe_index].src, src, 7);
    dedupe_index = (dedupe_index + 1) % DEDUPE_SIZE;
}

// ------------------------------
// ACK-Wartezustand
// (Nur eine "offene" Nachricht zur Zeit für Demo)
// ------------------------------
static volatile bool ack_received = false;
static volatile uint16_t waiting_ack_id = 0;
static char waiting_ack_dst[7];

// =========================================================
// Low-Level: SPI / Register
// =========================================================
static esp_err_t lora_spi_init(void)
{
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };

    esp_err_t err = spi_bus_initialize(LORA_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
    };

    return spi_bus_add_device(LORA_SPI_HOST, &devcfg, &lora_spi);
}

static esp_err_t lora_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), value };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx };
    return spi_device_transmit(lora_spi, &t);
}

static esp_err_t lora_read_reg(uint8_t reg, uint8_t *value)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), 0x00 };
    uint8_t rx[2] = { 0 };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx, .rx_buffer = rx };
    esp_err_t err = spi_device_transmit(lora_spi, &t);
    if (err == ESP_OK) *value = rx[1];
    return err;
}

static void lora_fifo_write(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        lora_write_reg(REG_FIFO, data[i]);
    }
}

static void lora_fifo_read(uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        lora_read_reg(REG_FIFO, &buf[i]);
    }
}

// =========================================================
// LoRa Helpers
// =========================================================
static void lora_clear_irqs(void)
{
    // Flags löschen durch 1en schreiben
    lora_write_reg(REG_IRQ_FLAGS, 0xFF);
}

static void lora_reset(void)
{
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);

    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void lora_enter_lora_mode_sleep(void)
{
    // Sleep + LoRa aktivieren
    lora_write_reg(REG_OP_MODE, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void lora_set_frequency_433(void)
{
    // FRF für 433 MHz: 0x6C8000
    uint32_t frf = 0x6C8000;
    lora_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    lora_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
    lora_write_reg(REG_FRF_LSB, (uint8_t)(frf));
}

static void lora_set_modem_defaults(void)
{
    // BW=125kHz, CR=4/5, Explicit Header
    lora_write_reg(REG_MODEM_CONFIG_1, 0x72);

    // SF7, CRC on
    lora_write_reg(REG_MODEM_CONFIG_2, 0x74);

    // AGC auto on
    lora_write_reg(REG_MODEM_CONFIG_3, 0x04);
}

static void lora_set_tx_power_dbm_14(void)
{
    // PA_BOOST + ~14dBm
    lora_write_reg(REG_PA_CONFIG, 0x8E);
}

static void lora_set_standby(void)
{
    // LoRa + Standby
    lora_write_reg(REG_OP_MODE, 0x81);
    vTaskDelay(pdMS_TO_TICKS(5));
}

static void lora_start_rx_continuous(void)
{
    // RX FIFO base, pointer
    lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0x00);
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x00);

    // DIO0 mapping: 00 => RxDone in RX
    lora_write_reg(REG_DIO_MAPPING_1, 0x00);

    lora_clear_irqs();

    // RX Continuous: LoRa + RXCONT
    lora_write_reg(REG_OP_MODE, 0x85);
}

static void lora_start_tx_packet(const uint8_t *packet, size_t len)
{
    lora_clear_irqs();

    // TX FIFO base, pointer
    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0x00);
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x00);

    // payload -> FIFO
    lora_fifo_write(packet, len);

    // length
    lora_write_reg(REG_PAYLOAD_LENGTH, (uint8_t)len);

    // TX Mode: LoRa + TX
    lora_write_reg(REG_OP_MODE, 0x83);
}

// Polling auf TxDone (robust)
static bool wait_tx_done_polling(int timeout_ms)
{
    while (timeout_ms > 0) {
        uint8_t irq = 0;
        lora_read_reg(REG_IRQ_FLAGS, &irq);

        if (irq & IRQ_TX_DONE) {
            lora_clear_irqs();
            return true;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
        timeout_ms -= 10;
    }
    return false;
}

// =========================================================
// MR Frame Builder
// =========================================================
static uint16_t g_msg_id = 1;

static void call7_set(char out7[7], const char *call)
{
    memset(out7, ' ', 7);
    size_t n = strlen(call);
    if (n > 7) n = 7;
    memcpy(out7, call, n);
}

static size_t mr_build_frame(uint8_t *out, size_t out_max,
                             const char *src_call,
                             const char *dst_call,
                             uint8_t flags,
                             uint8_t ttl,
                             uint16_t msg_id,
                             const uint8_t *payload,
                             size_t payload_len)
{
    if (out_max < sizeof(mr_hdr_t)) return 0;
    if (payload_len > 200) payload_len = 200;
    if (sizeof(mr_hdr_t) + payload_len > out_max) return 0;

    mr_hdr_t hdr;
    memset(&hdr, 0, sizeof(hdr));

    hdr.magic[0] = 'M';
    hdr.magic[1] = 'R';
    hdr.version  = 1;
    hdr.flags    = flags;
    hdr.ttl      = ttl;
    hdr.msg_id   = msg_id;

    call7_set(hdr.src, src_call);
    call7_set(hdr.dst, dst_call);

    hdr.payload_len = (uint8_t)payload_len;

    memcpy(out, &hdr, sizeof(hdr));
    if (payload_len) memcpy(out + sizeof(hdr), payload, payload_len);

    return sizeof(hdr) + payload_len;
}

// =========================================================
// TX API: Chat + ACK
// =========================================================
static bool mr_send_frame_and_return_to_rx(const uint8_t *packet, size_t len)
{
    lora_start_tx_packet(packet, len);

    bool ok = wait_tx_done_polling(2000);

    // Nach TX wieder in RX (Mesh läuft immer auf RX)
    lora_start_rx_continuous();

    return ok;
}

static bool mr_send_message(const char *dst, const char *text, bool request_ack)
{
    uint8_t pkt[256];

    uint8_t flags = 0;
    if (request_ack) flags |= MR_FLAG_ACK_REQ;

    uint16_t msg_id = g_msg_id++;

    size_t len = mr_build_frame(pkt, sizeof(pkt),
                                MY_CALL,
                                dst,
                                flags,
                                DEFAULT_TTL,
                                msg_id,
                                (const uint8_t*)text,
                                strlen(text));

    if (!len) return false;

    // ACK-Wartezustand setzen
    if (request_ack) {
        waiting_ack_id = msg_id;
        ack_received = false;
        call7_set(waiting_ack_dst, dst);
    }

    ESP_LOGI(TAG, "TX MSG -> dst='%.7s' id=%u ack=%d text=\"%s\"",
             waiting_ack_dst, msg_id, request_ack ? 1 : 0, text);

    bool tx_ok = mr_send_frame_and_return_to_rx(pkt, len);

    if (!request_ack) return tx_ok;

    // Auf ACK warten (3s)
    int timeout_ms = 3000;
    while (!ack_received && timeout_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(50));
        timeout_ms -= 50;
    }

    if (ack_received) {
        ESP_LOGI(TAG, "ACK OK ✅ for id=%u", msg_id);
        return true;
    } else {
        ESP_LOGW(TAG, "ACK TIMEOUT ❌ for id=%u", msg_id);
        return false;
    }
}

static void mr_send_ack(const char dst7[7], uint16_t msg_id)
{
    uint8_t pkt[64];

    // ACK: keine Payload
    size_t len = mr_build_frame(pkt, sizeof(pkt),
                                MY_CALL,
                                (const char*)dst7,    // dst7 enthält ggf. Spaces
                                MR_FLAG_ACK,
                                2,                    // ACK braucht nur kurze TTL
                                msg_id,
                                NULL, 0);

    if (!len) return;

    ESP_LOGI(TAG, "TX ACK -> dst='%.7s' id=%u", dst7, msg_id);

    (void)mr_send_frame_and_return_to_rx(pkt, len);
}

// =========================================================
// RX: Packet Handling (Kapitel 10 Logik)
// =========================================================
static bool call7_is_broadcast(const char dst7[7])
{
    return (dst7[0] == '*');
}

static bool call7_equals(const char a[7], const char *b_call)
{
    char b7[7];
    call7_set(b7, b_call);
    return memcmp(a, b7, 7) == 0;
}

static void handle_rx_packet(void)
{
    uint8_t irq = 0;
    lora_read_reg(REG_IRQ_FLAGS, &irq);

    if (!(irq & IRQ_RX_DONE)) return;

    if (irq & IRQ_PAYLOAD_CRC_ERROR) {
        ESP_LOGW(TAG, "RX CRC error (IRQ=0x%02X)", irq);
        lora_clear_irqs();
        return;
    }

    uint8_t bytes = 0;
    lora_read_reg(REG_RX_NB_BYTES, &bytes);

    uint8_t fifo_current = 0;
    lora_read_reg(REG_FIFO_RX_CURRENT_ADDR, &fifo_current);

    // FIFO pointer auf Paketstart
    lora_write_reg(REG_FIFO_ADDR_PTR, fifo_current);

    uint8_t buf[256];
    if (bytes > sizeof(buf)) bytes = sizeof(buf);

    lora_fifo_read(buf, bytes);

    lora_clear_irqs();

    if (bytes < sizeof(mr_hdr_t)) return;

    mr_hdr_t *hdr = (mr_hdr_t*)buf;

    // Magic + Version prüfen
    if (hdr->magic[0] != 'M' || hdr->magic[1] != 'R') return;
    if (hdr->version != 1) return;

    // Längen prüfen
    size_t header_len = sizeof(mr_hdr_t);
    if (header_len + hdr->payload_len > bytes) return;

    // Dedupe: msg_id + src
    if (is_duplicate(hdr->msg_id, hdr->src)) {
        // Already seen -> ignore fully (no forward, no ACK)
        return;
    }
    add_to_dedupe(hdr->msg_id, hdr->src);

    // Zielentscheidung
    bool for_me = call7_equals(hdr->dst, MY_CALL);
    bool bc     = call7_is_broadcast(hdr->dst);

    // ACK empfangen?
    if (hdr->flags & MR_FLAG_ACK) {
        if (waiting_ack_id != 0 && hdr->msg_id == waiting_ack_id) {
            // Optional: auch prüfen, ob ACK von richtigem Ziel kommt.
            // Für Demo reicht msg_id.
            ack_received = true;
        }
        return; // ACK nicht weiterleiten
    }

    // Nutzdaten als Text ausgeben (Chat)
    char payload[201];
    uint8_t plen = hdr->payload_len;
    if (plen > 200) plen = 200;
    memcpy(payload, buf + header_len, plen);
    payload[plen] = 0;

    ESP_LOGI(TAG, "RX MSG from='%.7s' to='%.7s' ttl=%u id=%u flags=0x%02X text=\"%s\"",
             hdr->src, hdr->dst, hdr->ttl, hdr->msg_id, hdr->flags, payload);

    // Wenn für mich: ACK senden (bei ACK_REQ)
    if (for_me && (hdr->flags & MR_FLAG_ACK_REQ)) {
        // dst für ACK ist der Sender (src)
        mr_send_ack(hdr->src, hdr->msg_id);
    }

    // Forwarding (Flooding light):
    // - broadcast immer forwarden (wenn TTL>0)
    // - unicast forwarden, solange TTL>0 (damit es durchs Netz zum Ziel kommt)
    if (hdr->ttl > 0) {
        hdr->ttl--;

        // Re-Transmit des ORIGINAL-Pakets (mit reduziertem TTL)
        // Hinweis: der Forwarder bleibt transparent; keine neuen msg_id.
        bool ok = mr_send_frame_and_return_to_rx(buf, bytes);
        if (ok) ESP_LOGI(TAG, "Forward OK (ttl now %u)", hdr->ttl);
        else    ESP_LOGW(TAG, "Forward FAIL");
    }
}

// =========================================================
// DIO0 ISR -> Queue -> RX Handler
// =========================================================
static void IRAM_ATTR dio0_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(dio0_evt_queue, &gpio_num, NULL);
}

static void dio0_task(void *arg)
{
    uint32_t io_num;
    while (1) {
        if (xQueueReceive(dio0_evt_queue, &io_num, portMAX_DELAY)) {
            handle_rx_packet();
            // ensure RX continues
            lora_start_rx_continuous();
        }
    }
}

// =========================================================
// app_main
// =========================================================
void app_main(void)
{
    ESP_LOGI(TAG, "Init SPI...");
    ESP_ERROR_CHECK(lora_spi_init());

    ESP_LOGI(TAG, "Reset LoRa chip...");
    lora_reset();

    uint8_t version = 0;
    lora_read_reg(REG_VERSION, &version);
    ESP_LOGI(TAG, "RegVersion=0x%02X (expected 0x12)", version);
    if (version != 0x12) {
        ESP_LOGE(TAG, "No SX1276 detected. Abort.");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // LoRa basic init
    lora_enter_lora_mode_sleep();
    lora_set_frequency_433();
    lora_set_modem_defaults();
    lora_set_tx_power_dbm_14();
    lora_set_standby();

    // RX start (interrupt-driven)
    dio0_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,   // RX-DIO0 funktioniert i.d.R. stabil
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIN_NUM_DIO0),
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_NUM_DIO0, dio0_isr_handler, (void*)PIN_NUM_DIO0);

    xTaskCreate(dio0_task, "dio0_task", 4096, NULL, 10, NULL);

    lora_start_rx_continuous();

    ESP_LOGI(TAG, "MeshRadio Node started. MY_CALL=%s", MY_CALL);

    // ------------------------------
    // Chat-Demo:
    //  - alle 15s Broadcast (CQ)
    //  - alle 20s Unicast mit ACK (Beispiel: DJ1ABC)
    // ------------------------------
    int t = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        t++;

        if (t % 15 == 0) {
            mr_send_message("*", "CQ CQ MeshRadio 4.0", false);
        }

        if (t % 20 == 0) {
            // Beispielziel: anpassen
            mr_send_message("DJ1ABC", "Hallo OM, bitte ACK", true);
        }
    }
}
