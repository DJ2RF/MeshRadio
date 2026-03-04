#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "LORA_RELIABLE";

// Heltec V3.2 LoRa Pins (SX1262)
#define LORA_BUSY_PIN    (gpio_num_t)13
#define LORA_RST_PIN     (gpio_num_t)12
#define LORA_CS_PIN      (gpio_num_t)8
#define VEXT_CTRL_PIN    (gpio_num_t)36

spi_device_handle_t spi_bus;

// --- SX1262 Basis-Funktionen ---

void sx1262_wait_busy() {
    uint32_t timeout = 5000; 
    while (gpio_get_level(LORA_BUSY_PIN) == 1 && timeout > 0) {
        esp_rom_delay_us(10);
        timeout--;
    }
}

// Status abfragen (0xC0 + 1 Dummy Byte)
uint8_t sx1262_get_status() {
    sx1262_wait_busy();
    uint8_t tx_data[2] = { 0xC0, 0x00 };
    uint8_t rx_data[2] = { 0 };
    
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    gpio_set_level(LORA_CS_PIN, 0);
    spi_device_polling_transmit(spi_bus, &t);
    gpio_set_level(LORA_CS_PIN, 1);
    
    return rx_data[0]; // Das erste Byte ist der Status
}

void sx1262_write_cmd(uint8_t opcode, uint8_t *data, size_t len) {
    sx1262_wait_busy();
    uint8_t buffer[16] = {0}; 
    buffer[0] = opcode;
    if (len > 0 && len < 15) memcpy(&buffer[1], data, len);

    spi_transaction_t t = {
        .length = (1 + len) * 8,
        .tx_buffer = buffer,
    };

    gpio_set_level(LORA_CS_PIN, 0);
    spi_device_polling_transmit(spi_bus, &t);
    gpio_set_level(LORA_CS_PIN, 1);
    sx1262_wait_busy();
}

// --- Radio Setup ---

void init_lora_863() {
    ESP_LOGI(TAG, "Hardware Reset SX1262...");
    gpio_set_level(LORA_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(LORA_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    // 1. Standby & TCXO (Heltec V3 braucht DIO3 Speisung)
    uint8_t stdby = 0x01; 
    sx1262_write_cmd(0x80, &stdby, 1); // Standby XOSC
    
    uint8_t tcxo[] = { 0x01, 0x00, 0x00, 0x64 }; 
    sx1262_write_cmd(0x97, tcxo, 4); // Set TCXO 1.8V
    ESP_LOGI(TAG, "TCXO aktiviert.");

    // 2. Kalibrierung
    uint8_t cal[] = { 0x7F }; 
    sx1262_write_cmd(0x89, cal, 1);

    // 3. Basis-Parameter
    uint8_t pkt = 0x01; sx1262_write_cmd(0x8A, &pkt, 1); // LoRa Mode
    uint8_t freq[] = { 0x35, 0xF0, 0x00, 0x00 }; sx1262_write_cmd(0x86, freq, 4); // 863 MHz
    uint8_t pa[] = { 0x04, 0x07, 0x00, 0x01 }; sx1262_write_cmd(0x95, pa, 4); // PA Config
    uint8_t dio2 = 0x01; sx1262_write_cmd(0x9D, &dio2, 1); // RF-Switch Auto

    // 4. Modulation: SF7, BW 125kHz, CR 4/5
    uint8_t mod[] = { 0x07, 0x04, 0x01, 0x00 }; 
    sx1262_write_cmd(0x8B, mod, 4);

    ESP_LOGI(TAG, "Setup abgeschlossen.");
}

void send_packet_with_debug(const char* text) {
    size_t len = strlen(text);
    // Payload in Buffer schreiben (Offset 0)
    uint8_t buffer_header[2] = { 0x00, 0x00 }; // Offset
    sx1262_write_cmd(0x0E, (uint8_t*)text, len); 

    // Start TX (Timeout 0)
    uint8_t tx_params[] = { 0x00, 0x00, 0x00 }; 
    sx1262_write_cmd(0x83, tx_params, 3); 

    // --- DEBUG ---
    vTaskDelay(pdMS_TO_TICKS(20)); // Kurz warten auf Zustandsänderung
    uint8_t status = sx1262_get_status();
    uint8_t chip_mode = (status & 0x70) >> 4;
    uint8_t cmd_status = (status & 0x0E) >> 1;

    if (chip_mode == 0x06) {
        ESP_LOGI(TAG, "DEBUG: TX aktiv! Mode=0x%02X, Data='%s'", chip_mode, text);
    } else {
        ESP_LOGE(TAG, "DEBUG: Fehler! Mode=0x%02X (0x02=Standby, 0x03=FreqErr), CmdStat=0x%x", chip_mode, cmd_status);
    }
}

// --- Hauptprogramm ---

void app_main(void) {
    // Vext AN (LOW) für Pull-Ups und Board-Peripherie
    gpio_set_direction(VEXT_CTRL_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(VEXT_CTRL_PIN, 0); 
    vTaskDelay(pdMS_TO_TICKS(100));

    // Pin-Konfiguration
    gpio_set_direction(LORA_BUSY_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(LORA_RST_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LORA_CS_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LORA_CS_PIN, 1);

    // SPI Initialisierung
    spi_bus_config_t buscfg = {
        .mosi_io_num = 10, .miso_io_num = 11, .sclk_io_num = 9,
        .quadwp_io_num = -1, .quadhd_io_num = -1,
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 2000000, .mode = 0,
        .spics_io_num = -1, .queue_size = 7,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi_bus));

    init_lora_863();

    int count = 0;
    char msg[32];

    while (1) {
        snprintf(msg, sizeof(msg), "MR34 HELTEC #%d", count++);
        send_packet_with_debug(msg);
        vTaskDelay(pdMS_TO_TICKS(10000)); // Alle 10 Sek
    }
}
