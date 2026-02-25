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
//
// ============================================================

// ============================================================================
// MeshRadio – Kapitel 11 (FINAL, robust) | ESP-IDF v5.5.x
//
// ✅ LoRa Beacon senden/empfangen
// ✅ Neighbor-Tabelle (Call + RSSI + Timeout)
// ✅ OLED optional (k0i05/esp_ssd1306)
// ✅ WICHTIG: Schalter USE_OLED
//     - USE_OLED = 0  -> OLED komplett AUS (keine I2C Pins, kein Reset, kein Task)
//     - USE_OLED = 1  -> OLED EIN (ACK-Probe 0x3C/0x3D, fail-open)
//
// Warum der Schalter sinnvoll ist:
//   Ein "defektes" / stark abweichendes OLED oder ein stuck I2C Bus kann einen ESP32
//   so blockieren, dass der WDT auslöst. Für dein Buch: MeshRadio muss trotzdem laufen.
//
// Abhängigkeit (nur wenn USE_OLED=1 nötig):
//   idf.py add-dependency "k0i05/esp_ssd1306"
//
// Funkamateur-Hinweis:
//   - Keine Verschlüsselung
//   - Rufzeichen im Frame
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
#include "esp_err.h"

// ============================
// OLED an/aus (Build-Schalter)
// 0 = OLED komplett deaktiviert (empfohlen fürs Problem-Board)
// 1 = OLED aktiv
// ============================
#define USE_OLED 0

#if USE_OLED
#include "esp_rom_sys.h"      // esp_rom_delay_us()
#include "driver/i2c_master.h"
#include "ssd1306.h"
#endif

// ============================
// Node-Konfiguration
// ============================
#define MY_CALL              "DJ1ABC"
#define BEACON_INTERVAL_S    10
#define OLED_REFRESH_S       2
#define NEIGHBOR_TIMEOUT_MS  60000

// ============================
// LoRa Pins TTGO T-Beam V1.1 (SX1276)
// ============================
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 27
#define PIN_NUM_CLK  5
#define PIN_NUM_CS   18
#define PIN_NUM_RST  23
#define PIN_NUM_DIO0 26

#define LORA_SPI_HOST VSPI_HOST

// ============================
// OLED / I2C Pins (nur relevant wenn USE_OLED=1)
// ============================
#if USE_OLED
#define OLED_SDA      21
#define OLED_SCL      22
#define OLED_RST      16
#define OLED_I2C_HZ   400000

// Für "unlesbar" später testweise:
// #define OLED_OFFSET_X 2
// #define OLED_FLIP     true
#define OLED_OFFSET_X 0
#define OLED_FLIP     false
#endif

// ============================
// SX1276 Register (LoRa)
// ============================
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
#define REG_DIO_MAPPING_1        0x40

#define IRQ_RX_DONE              0x40
#define IRQ_PAYLOAD_CRC_ERROR    0x20
#define IRQ_TX_DONE              0x08

// ============================
// MeshRadio Beacon Frame v1
// ============================
#define MR_FLAG_BEACON 0x04

#pragma pack(push, 1)
typedef struct {
    uint8_t  magic[2];      // 'M','R'
    uint8_t  version;       // 1
    uint8_t  flags;         // BEACON
    uint8_t  ttl;           // 1
    uint16_t msg_id;        // laufende Nummer
    char     src[7];        // Rufzeichen gepadded (spaces)
    char     dst[7];        // "*" gepadded (spaces)
    uint8_t  payload_len;   // 0
} mr_hdr_t;
#pragma pack(pop)

// ============================
// Neighbor Table
// ============================
#define MAX_NEIGHBORS 20

typedef struct {
    bool used;
    char call[7];
    int  rssi_dbm;
    TickType_t last_seen;
} neighbor_t;

// ============================
// Globals
// ============================
static const char *TAG = "MR11_LIB";

static spi_device_handle_t lora_spi = NULL;
static QueueHandle_t dio0_evt_queue = NULL;

static uint16_t g_msg_id = 1;
static neighbor_t neighbors[MAX_NEIGHBORS];

#if USE_OLED
static i2c_master_bus_handle_t i2c_bus = NULL;
static ssd1306_handle_t oled = NULL;
#endif

// ============================================================================
// Helper: Callsign padding / printing
// ============================================================================
static void call7_set(char out7[7], const char *call)
{
    memset(out7, ' ', 7);
    size_t n = strlen(call);
    if (n > 7) n = 7;
    memcpy(out7, call, n);
}

static void call7_to_cstr(char out8[8], const char in7[7])
{
    memcpy(out8, in7, 7);
    out8[7] = 0;
}

// ============================================================================
// SX1276 SPI Low-Level
// ============================================================================
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

static void lora_clear_irqs(void)
{
    (void)lora_write_reg(REG_IRQ_FLAGS, 0xFF);
}

static void lora_reset(void)
{
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

// ============================================================================
// LoRa Init (433 MHz, BW125, SF7, CRC on)
// ============================================================================
static void lora_enter_lora_sleep(void)
{
    lora_write_reg(REG_OP_MODE, 0x80); // Sleep + LoRa
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void lora_set_frequency_433(void)
{
    const uint32_t frf = 0x6C8000; // 433 MHz
    lora_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    lora_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
    lora_write_reg(REG_FRF_LSB, (uint8_t)(frf));
}

static void lora_set_modem_defaults(void)
{
    lora_write_reg(REG_MODEM_CONFIG_1, 0x72); // BW125, CR4/5, Explicit
    lora_write_reg(REG_MODEM_CONFIG_2, 0x74); // SF7, CRC on
    lora_write_reg(REG_MODEM_CONFIG_3, 0x04); // AGC auto
}

static void lora_set_tx_power_dbm_14(void)
{
    lora_write_reg(REG_PA_CONFIG, 0x8E);
}

static void lora_start_rx_continuous(void)
{
    lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0x00);
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x00);

    lora_write_reg(REG_DIO_MAPPING_1, 0x00); // DIO0=RxDone
    lora_clear_irqs();
    lora_write_reg(REG_OP_MODE, 0x85);       // LoRa + RX continuous
}

static void lora_start_tx_packet(const uint8_t *packet, size_t len)
{
    lora_clear_irqs();

    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0x00);
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x00);

    for (size_t i = 0; i < len; i++) lora_write_reg(REG_FIFO, packet[i]);

    lora_write_reg(REG_PAYLOAD_LENGTH, (uint8_t)len);
    lora_write_reg(REG_OP_MODE, 0x83); // LoRa + TX
}

// TX done per Polling (keine DIO0 Abhängigkeit)
static bool lora_wait_tx_done_polling(int timeout_ms)
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

// ============================================================================
// Neighbor Table
// ============================================================================
static void neighbor_update(const char call7[7], int rssi_dbm)
{
    TickType_t now = xTaskGetTickCount();

    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (neighbors[i].used && memcmp(neighbors[i].call, call7, 7) == 0) {
            neighbors[i].rssi_dbm = rssi_dbm;
            neighbors[i].last_seen = now;
            return;
        }
    }

    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!neighbors[i].used) {
            neighbors[i].used = true;
            memcpy(neighbors[i].call, call7, 7);
            neighbors[i].rssi_dbm = rssi_dbm;
            neighbors[i].last_seen = now;
            return;
        }
    }
}

static void neighbor_cleanup(void)
{
    TickType_t now = xTaskGetTickCount();
    TickType_t limit = pdMS_TO_TICKS(NEIGHBOR_TIMEOUT_MS);

    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!neighbors[i].used) continue;
        if ((now - neighbors[i].last_seen) > limit) neighbors[i].used = false;
    }
}

static int neighbor_count(void)
{
    int c = 0;
    for (int i = 0; i < MAX_NEIGHBORS; i++) if (neighbors[i].used) c++;
    return c;
}

#if USE_OLED
// ============================================================================
// I2C Bus Recovery (kurz, ohne wait-loops)
// ============================================================================
static void i2c_bus_recover(gpio_num_t sda, gpio_num_t scl)
{
    gpio_set_pull_mode(sda, GPIO_PULLUP_ONLY);
    gpio_set_direction(sda, GPIO_MODE_INPUT);

    gpio_set_pull_mode(scl, GPIO_PULLUP_ONLY);
    gpio_set_direction(scl, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(scl, 1);
    esp_rom_delay_us(10);

    for (int i = 0; i < 9; i++) {
        gpio_set_level(scl, 0);
        esp_rom_delay_us(10);
        gpio_set_level(scl, 1);
        esp_rom_delay_us(10);
    }

    // STOP: SDA low -> SCL high -> SDA high
    gpio_set_direction(sda, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(sda, 0);
    esp_rom_delay_us(10);
    gpio_set_level(scl, 1);
    esp_rom_delay_us(10);
    gpio_set_level(sda, 1);
    esp_rom_delay_us(10);

    gpio_set_direction(sda, GPIO_MODE_INPUT);
}

// ============================================================================
// I2C Probe (ACK) – korrekt für new i2c_master driver (IDF 5.5)
// ============================================================================
static bool i2c_probe_addr(i2c_master_bus_handle_t bus, uint8_t addr_7bit)
{
    esp_err_t err = i2c_master_probe(bus, addr_7bit, 50 /*ms*/);
    return (err == ESP_OK);
}

// ============================================================================
// OLED Init (fail-open, kein Panic/WDT)
// ============================================================================
static void oled_init_lib(void)
{
    // Reset OLED
    gpio_set_direction((gpio_num_t)OLED_RST, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)OLED_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level((gpio_num_t)OLED_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "OLED: recover I2C bus...");
    i2c_bus_recover(OLED_SDA, OLED_SCL);
    ESP_LOGI(TAG, "OLED: recover done");

    // I2C bus anlegen (nur 1x)
    if (i2c_bus == NULL) {
        i2c_master_bus_config_t bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = OLED_SDA,
            .scl_io_num = OLED_SCL,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };

        esp_err_t err = i2c_new_master_bus(&bus_cfg, &i2c_bus);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "OLED: i2c_new_master_bus failed: %s", esp_err_to_name(err));
            i2c_bus = NULL;
            oled = NULL;
            return;
        }
    }

    // ACK Probe: 0x3C / 0x3D
    bool ack3c = i2c_probe_addr(i2c_bus, 0x3C);
    bool ack3d = i2c_probe_addr(i2c_bus, 0x3D);
    ESP_LOGI(TAG, "OLED: I2C probe 0x3C=%d 0x3D=%d", ack3c, ack3d);

    if (!ack3c && !ack3d) {
        ESP_LOGE(TAG, "OLED: no ACK -> OLED disabled");
        oled = NULL;
        return;
    }

    uint16_t real_addr = ack3c ? 0x3C : 0x3D;

    ssd1306_config_t cfg = {
        .i2c_address     = real_addr,
        .i2c_clock_speed = OLED_I2C_HZ,
        .panel_size      = SSD1306_PANEL_128x64,
        .offset_x        = OLED_OFFSET_X,
        .flip_enabled    = OLED_FLIP,
        .display_enabled = true,
    };

    esp_err_t err = ssd1306_init(i2c_bus, &cfg, &oled);
    if (err != ESP_OK || oled == NULL) {
        ESP_LOGE(TAG, "OLED: ssd1306_init failed: %s (OLED disabled)", esp_err_to_name(err));
        oled = NULL;
        return;
    }

    (void)ssd1306_clear_display(oled, false);
    (void)ssd1306_display_text(oled, 0, "MeshRadio", false);
    (void)ssd1306_display_text(oled, 1, MY_CALL, false);

    ESP_LOGI(TAG, "OLED: init OK (addr=0x%02X)", (unsigned)real_addr);
}

static void oled_show_neighbors(void)
{
    if (oled == NULL) return;

    char line[32];
    if (ssd1306_clear_display(oled, false) != ESP_OK) return;

    snprintf(line, sizeof(line), "%s N=%d", MY_CALL, neighbor_count());
    (void)ssd1306_display_text(oled, 0, line, false);

    int row = 1;
    for (int i = 0; i < MAX_NEIGHBORS && row < 8; i++) {
        if (!neighbors[i].used) continue;

        char call8[8];
        call7_to_cstr(call8, neighbors[i].call);

        snprintf(line, sizeof(line), "%.7s %d", call8, neighbors[i].rssi_dbm);
        (void)ssd1306_display_text(oled, (uint8_t)row, line, false);
        row++;
    }
}

// OLED Task: init + periodisches Update
static void oled_task(void *arg)
{
    (void)arg;

    // später starten, damit Boot/LoRa sicher durch sind
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "OLED task: init...");
    oled_init_lib();
    ESP_LOGI(TAG, "OLED task: init done (oled=%s)", (oled ? "ON" : "OFF"));

    while (1) {
        if (oled) oled_show_neighbors();
        vTaskDelay(pdMS_TO_TICKS(OLED_REFRESH_S * 1000));
    }
}
#endif // USE_OLED

// ============================================================================
// Beacon TX
// ============================================================================
static void send_beacon(void)
{
    mr_hdr_t h;
    memset(&h, 0, sizeof(h));

    h.magic[0] = 'M';
    h.magic[1] = 'R';
    h.version  = 1;
    h.flags    = MR_FLAG_BEACON;
    h.ttl      = 1;
    h.msg_id   = g_msg_id++;

    call7_set(h.src, MY_CALL);
    call7_set(h.dst, "*");
    h.payload_len = 0;

    lora_start_tx_packet((const uint8_t*)&h, sizeof(h));

    if (!lora_wait_tx_done_polling(2000)) {
        ESP_LOGW(TAG, "Beacon TX timeout");
        lora_clear_irqs();
    }

    lora_start_rx_continuous();
}

// ============================================================================
// RX Handler: Beacon -> RSSI -> Neighbor Update
// ============================================================================
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
    lora_write_reg(REG_FIFO_ADDR_PTR, fifo_current);

    uint8_t buf[256];
    if (bytes > sizeof(buf)) bytes = sizeof(buf);

    for (uint8_t i = 0; i < bytes; i++) lora_read_reg(REG_FIFO, &buf[i]);

    lora_clear_irqs();

    if (bytes < sizeof(mr_hdr_t)) return;

    mr_hdr_t *hdr = (mr_hdr_t*)buf;
    if (hdr->magic[0] != 'M' || hdr->magic[1] != 'R') return;
    if (hdr->version != 1) return;

    if (hdr->flags & MR_FLAG_BEACON) {
        uint8_t rssi_raw = 0;
        lora_read_reg(REG_PKT_RSSI_VALUE, &rssi_raw);

        // 433 MHz: RSSI[dBm] = -157 + raw
        int rssi_dbm = -157 + (int)rssi_raw;

        neighbor_update(hdr->src, rssi_dbm);

        char src8[8];
        call7_to_cstr(src8, hdr->src);
        ESP_LOGI(TAG, "Beacon from %.7s RSSI=%d dBm", src8, rssi_dbm);
    }
}

// ============================================================================
// DIO0 Interrupt -> Queue -> Task
// ============================================================================
static void IRAM_ATTR dio0_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(dio0_evt_queue, &gpio_num, NULL);
}

static void dio0_task(void *arg)
{
    uint32_t io_num;
    (void)arg;

    while (1) {
        if (xQueueReceive(dio0_evt_queue, &io_num, portMAX_DELAY)) {
            handle_rx_packet();
            lora_start_rx_continuous();
        }
    }
}

// ============================================================================
// Init: SPI LoRa
// ============================================================================
static void init_spi_lora(void)
{
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LORA_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(LORA_SPI_HOST, &devcfg, &lora_spi));
}

// ============================================================================
// app_main
// ============================================================================
void app_main(void)
{
    ESP_LOGI(TAG, "Kapitel 11 start (MY_CALL=%s)", MY_CALL);

    // --- LoRa init ---
    init_spi_lora();

    ESP_LOGI(TAG, "Reset SX1276...");
    lora_reset();

    uint8_t version = 0;
    lora_read_reg(REG_VERSION, &version);
    ESP_LOGI(TAG, "RegVersion=0x%02X (expected 0x12)", version);
    if (version != 0x12) {
        ESP_LOGE(TAG, "No SX1276 detected -> abort");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    lora_enter_lora_sleep();
    lora_set_frequency_433();
    lora_set_modem_defaults();
    lora_set_tx_power_dbm_14();
    lora_start_rx_continuous();

    // --- OLED: nur wenn USE_OLED=1 ---
#if USE_OLED
    ESP_LOGI(TAG, "Init OLED (optional)...");
    xTaskCreate(oled_task, "oled_task", 4096, NULL, 5, NULL);
#else
    ESP_LOGI(TAG, "OLED disabled by build switch (USE_OLED=0)");
#endif

    // --- DIO0 interrupt ---
    dio0_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIN_NUM_DIO0),
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_NUM_DIO0, dio0_isr_handler, (void*)PIN_NUM_DIO0);
    xTaskCreate(dio0_task, "dio0_task", 4096, NULL, 10, NULL);

    // --- Main loop ---
    int beacon_t = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        beacon_t++;
        if (beacon_t >= BEACON_INTERVAL_S) {
            send_beacon();
            beacon_t = 0;
        }

        neighbor_cleanup();
        // OLED updates laufen im oled_task() (falls USE_OLED=1)
    }
}
