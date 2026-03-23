/******************************************************************************
 *  MeshRadio Project
 *
 *  FILE: <filename>
 *
 *  DESCRIPTION
 *  ---------------------------------------------------------------------------
 *  MeshRadio main – ESP-IDF v5.5.2
 *
 *  AUTHOR
 *  ---------------------------------------------------------------------------
 *  Friedrich Riedhammer (Fritz)
 *  https://nerdverlag.com
 *  fritz@nerdverlag.com
 *
 *  COPYRIGHT
 *  ---------------------------------------------------------------------------
 *  (c) 2026 Friedrich Riedhammer / Nerd-Verlag
 *
 *  This software is provided "as is", without any express or implied warranty.
 *  In no event will the author be held liable for any damages arising from
 *  the use of this software.
 *
 *  Permission is granted to use, modify, and distribute this software for
 *  educational, experimental, and amateur radio purposes, provided that this
 *  copyright notice and this disclaimer remain intact in all copies.
 *
 *  This software is intended for experimentation and research in wireless
 *  mesh networking. The author does not guarantee correctness, reliability,
 *  or suitability for any specific purpose.
 *
 *  Use at your own risk.
 *
 ******************************************************************************/

#include "incl/config_meshradio.h"
#include "incl/board_pins.h"
#include "incl/radio_config.h"
#include "incl/mr_proto_v7.h"
#include "incl/mr_call7.h"
#include "incl/mr_bucket.h"
#include "incl/mr_sec_ccm.h"
#include "incl/mr_wifi_sta.h"
#include "incl/mr_wifi_ota.h"
#include "incl/mr_display.h"
#include "incl/mr_time_net.h"
#include "incl/mr_board.h"
#include "incl/mr_gps.h"
#include "incl/mr_pmu.h"
#include "incl/aprs.h"
#include "incl/aprs_web.h"
#include "incl/bme280.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/adc.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_netif.h"
#include "lwip/ip4_addr.h"

#include "esp_log.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_app_desc.h"
#include "esp_err.h"

#include "nvs_flash.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"

#include "esp_sleep.h"
#include "mbedtls/ccm.h"


// ============================================================================
// ================================ PROTOKOLL =================================
// ============================================================================
#define MR_PROTO_VERSION 8

#define MR_FLAG_DATA      0x10
#define MR_FLAG_BEACON    0x20
#define MR_FLAG_ACK       0x40
#define MR_FLAG_ROUTEADV  0x80

#define MR_FLAG_ACKREQ    0x01
#define MR_FLAG_SEC       0x08

#define DATA_TTL   4
#define BEACON_TTL 2
#define ACK_TTL    4
#define ADV_TTL    3

#define MAX_PAYLOAD      120
#define MAX_PLAINTEXT    (MAX_PAYLOAD - SEC_TAG_LEN)

#define SEEN_CACHE_SIZE 48
#define MAX_NEIGHBORS 24
#define NEIGHBOR_TIMEOUT_MS 60000
#define MAX_ROUTES 32
#define ROUTE_TIMEOUT_MS 180000
#define MAX_PENDING_ACK  10
#define ROUTEADV_PL_LEN   (8+8+2+2)
#define MAX_REPLAY 24

// LoRa TX timeout (für beide Chips)
#define LORA_TX_TIMEOUT_MS 2000
#define GPS_SEND_INTERVAL_MS 60000
#define GPS_TEXT_MAX_LEN     96


// ============================================================================
// ============================== NODE ROLES ==================================
// ============================================================================
typedef enum {
    NODE_RELAY  = 0,
    NODE_EDGE   = 1,
    NODE_SENSOR = 2
} node_mode_t;

static const char* node_mode_str(node_mode_t m)
{
    switch(m){
        case NODE_RELAY:  return "RELAY";
        case NODE_EDGE:   return "EDGE";
        case NODE_SENSOR: return "SENSOR";
        default:          return "UNKNOWN";
    }
}


// ============================================================================
// ================================ HEADER ====================================
// ============================================================================

// ============================================================================
// ================================ STRUCTS ===================================
// ============================================================================
typedef struct { bool used; char src[8]; uint16_t id; } seen_t;

typedef struct {
    bool used;
    char call[8];
    int rssi;
    uint32_t t_ms;
    uint32_t tx_attempts;
    uint32_t ack_ok;
    uint32_t last_decay_ms;
} neighbor_t;

typedef struct {
    bool used;
    char dst[8];
    char next[8];
    uint16_t seq;
    uint16_t etx_x100;
    int last_rssi;
    uint32_t t_ms;
    uint32_t hold_until_ms;
} route_t;

typedef struct {
    bool used;
    uint16_t msg_id;
    char expect_from[8];
    uint32_t deadline_ms;
    uint8_t retries_left;
    uint8_t frame[sizeof(mr_hdr_v7_t) + MAX_PAYLOAD];
    uint16_t frame_len;
} pending_ack_t;

typedef struct {
    bool used;
    bool has_seq;
    char src[8];
    uint16_t last_seq;
    uint32_t t_ms;
} replay_t;


// ============================================================================
// ================================ GLOBALS ===================================
// ============================================================================
static const char *TAG="MR34";

#include "incl/mr_cfg.h"

void wifi_enable_ap_part(void);
void wifi_disable_ap_part(void);

int8_t g_relay_gpio_runtime = RELAY_GPIO;

char g_callsign_rt[9] = MR_CALLSIGN;
char g_relay_callsign_rt[9] = MR_RELAY_CALLSIGN;


// runtime RF (statt nur DEFAULT_RF_FREQ_HZ Makro)
uint32_t g_rf_freq_hz_runtime = DEFAULT_RF_FREQ_HZ;
int8_t   g_tx_power_dbm_runtime = 2;

// ---------------- RTC persistente Zähler (über DeepSleep hinweg) ----------------
// Bleiben bei DeepSleep erhalten. Bei Power-Off sind sie weg (dann wird neu gesät).
RTC_DATA_ATTR static uint16_t rtc_msg_id = 0;
RTC_DATA_ATTR static uint16_t rtc_seq    = 0;
RTC_DATA_ATTR static uint32_t rtc_boots  = 0;

static uint16_t g_msg_id=0;
static uint16_t g_my_seq=0;
// ---- Relay state über DeepSleep behalten ----
RTC_DATA_ATTR static uint8_t rtc_relay_state = 0;
RTC_DATA_ATTR static uint8_t rtc_display_enable = 1;

static spi_device_handle_t lora_spi;
static QueueHandle_t dio_q;

static SemaphoreHandle_t g_mutex;          // schützt RAM-Tabellen/Globals
static SemaphoreHandle_t g_lora_spi_mutex; // schützt LoRa SPI-Zugriffe (beide Chips)

static httpd_handle_t g_http=NULL;
bool g_http_running=false;

static seen_t seen_cache[SEEN_CACHE_SIZE];
static neighbor_t neighbors[MAX_NEIGHBORS];
static route_t routes[MAX_ROUTES];
static pending_ack_t pend[MAX_PENDING_ACK];
static replay_t replay_tab[MAX_REPLAY];

bool g_bc_fallback=true;
bool g_beacon_enabled=true;
bool g_routeadv_enable=DEFAULT_ROUTEADV_ENABLE;
bool g_cad_enable=DEFAULT_CAD_ENABLE;
bool g_display_enabled = true;
bool g_powersave_enable = false;
bool g_bme280_enable = false;

#if (MR_BOARD_PRESET == MR_BOARD_HELTEC_V3)
static void prog_button_init(void);
static void prog_button_poll(void);
#endif

node_mode_t g_node_mode = (node_mode_t)DEFAULT_NODE_MODE;

bool g_crypto_enable=DEFAULT_CRYPTO_ENABLE;
uint8_t g_net_key[SEC_KEY_LEN];

static uint32_t sec_decrypt_ok=0;
static uint32_t sec_decrypt_fail=0;
static uint32_t sec_mac_fail=0;
static uint32_t sec_replay_drop=0;

uint32_t g_beacon_interval_ms=DEFAULT_BEACON_INTERVAL_MS;
uint8_t  g_routeadv_topn=DEFAULT_ROUTEADV_TOPN;
uint16_t g_routeadv_delta_etx=DEFAULT_ROUTEADV_DELTA_ETX;
uint32_t g_holddown_ms=DEFAULT_HOLDDOWN_MS;

static uint32_t g_next_beacon_ms=0;
static uint32_t g_next_routeadv_ms=0;

static uint32_t c_tx_beacon=0, c_tx_routeadv=0, c_tx_ack=0, c_tx_data=0;
static uint32_t c_defer_beacon=0, c_defer_routeadv=0, c_defer_ack=0, c_defer_data=0;
static uint32_t c_drop_routeadv=0, c_drop_data=0;

static bucket_t b_beacon, b_routeadv, b_ack, b_data;

bool g_wifi_enabled = (MR_WIFI_RUNTIME_DEFAULT ? true : false);
static bool g_wifi_inited  = false;
static bool g_ap_part_enabled = false;

static char g_last_rx_from[9]   = "";
static char g_last_rx_text[180] = "";
static uint32_t g_last_rx_ms    = 0;

#if MR_RELAY_ENABLE
static bool g_relay_on=false;
#endif

//void lora_log_chip_info(void);
//void lora_init_radio(void);
void lora_set_rx_continuous(void);
void lora_recover_radio(void);
//void lora_send_frame(const uint8_t *d, size_t len);
//bool lora_try_read_frame(uint8_t *buf, size_t maxlen, uint8_t *out_len, int *out_rssi);
static void send_data_to(const char *dst_str, const char *txt, bool ackreq);

// ADC
#if MR_BATT_ENABLE
static adc_oneshot_unit_handle_t g_adc_unit=NULL;
static adc_cali_handle_t g_adc_cali=NULL;
static bool g_adc_cali_ok=false;
static uint32_t g_batt_mv=0;
static uint32_t g_batt_pct=0;
static uint32_t g_next_batt_ms=0;
#endif

#if (MR_BOARD_PRESET == MR_BOARD_TBEAM_V11_SX1276) || \
    (MR_BOARD_PRESET == MR_BOARD_TBEAM_V12_AXP2101)
static void pwr_button_init(void);
static void pwr_button_poll(void);
#endif

// ============================================================================
// ================================ HELPERS ===================================
// ============================================================================
uint32_t now_ms(void){ return xTaskGetTickCount()*portTICK_PERIOD_MS; }

/*static const char *cfg_board_str(void)
{
#if (MR_BOARD_PRESET == MR_BOARD_HELTEC_V3)
    return "HELTEC_V3";
#elif (MR_BOARD_PRESET == MR_BOARD_LILYGO_SX1276)
    return "LILYGO_SX1276";
#else
    return "UNKNOWN";
#endif
}

static const char *cfg_lora_chip_str(void)
{
#if defined(MR_LORA_CHIP_SX1262)
    return "SX1262";
#elif defined(MR_LORA_CHIP_SX1276)
    return "SX1276";
#else
    return "UNKNOWN";
#endif
}
*/
static const char *cfg_board_str(void)
{
#if (MR_BOARD_PRESET == MR_BOARD_HELTEC_V3)
    return "HELTEC_V3";
#elif (MR_BOARD_PRESET == MR_BOARD_LILYGO_SX1276)
    return "LILYGO_SX1276";
#elif (MR_BOARD_PRESET == MR_BOARD_TBEAM_V11_SX1276)
    return "TBEAM_V11_SX1276";
#elif (MR_BOARD_PRESET == MR_BOARD_TBEAM_V12_AXP2101)
    return "TBEAM_V12_AXP2101";
#else
    return "UNKNOWN";
#endif
}

static const char *cfg_lora_chip_str(void)
{
#if defined(MR_LORA_CHIP_SX1262)
    return "SX1262";
#elif defined(MR_LORA_CHIP_SX1276)
    return "SX1276";
#else
    return "UNKNOWN";
#endif
}

static void config_print(void)
{
    ESP_LOGI(TAG, "CFG: board=%s chip=%s callsign=%s relay_call=%s",
             cfg_board_str(),
             cfg_lora_chip_str(),
             g_callsign_rt,
             g_relay_callsign_rt);

    ESP_LOGI(TAG, "CFG: rf=%lu tx_dbm=%d net_id=0x%02X crypto=%u wifi=%u role=%s powersave=%u",
             (unsigned long)g_rf_freq_hz_runtime,
             (int)g_tx_power_dbm_runtime,
             (unsigned)MR_NET_ID,
             g_crypto_enable ? 1 : 0,
             g_wifi_enabled ? 1 : 0,
             node_mode_str(g_node_mode),
             g_cfg.powersave_enable ? 1 : 0);

    ESP_LOGI(TAG, "CFG: relay_en=%u relay_gpio=%d batt_en=%u bme280=%u display_cfg=%u display_rt=%u cli=%u",
             (unsigned)MR_RELAY_ENABLE,
             (int)g_relay_gpio_runtime,
             (unsigned)MR_BATT_ENABLE,
             (unsigned)MR_BME280_ENABLE,
             g_cfg.display_enable ? 1 : 0,
             g_display_enabled ? 1 : 0,
             (unsigned)MR_CLI_ENABLE);

    ESP_LOGI(TAG, "CFG: ssid=%s beacon=%lu routeadv=%u topn=%u delta=%u hold=%lu cad=%u",
             g_cfg.wifi_ssid,
             (unsigned long)g_beacon_interval_ms,
             g_routeadv_enable ? 1 : 0,
             (unsigned)g_routeadv_topn,
             (unsigned)g_routeadv_delta_etx,
             (unsigned long)g_holddown_ms,
             g_cad_enable ? 1 : 0);
}

static void pending_clear_all_locked(void)
{
    memset(pend, 0, sizeof(pend));
}

static uint16_t mr_next_msg_id(void)
{
    if(rtc_msg_id == 0){
        rtc_msg_id = (uint16_t)(esp_random() & 0xFFFF);
        if(rtc_msg_id == 0) rtc_msg_id = 1;
    }

    uint16_t id = rtc_msg_id++;
    if(rtc_msg_id == 0) rtc_msg_id = 1;   // 0 vermeiden
    g_msg_id = rtc_msg_id;                // Runtime spiegeln
    return id;
}

static uint16_t mr_next_seq(void)
{
    if(rtc_seq == 0){
        rtc_seq = (uint16_t)(esp_random() & 0xFFFF);
        if(rtc_seq == 0) rtc_seq = 1;
    }

    uint16_t seq = rtc_seq++;
    if(rtc_seq == 0) rtc_seq = 1;         // 0 vermeiden
    g_my_seq = rtc_seq;                   // Runtime spiegeln
    return seq;
}

static void mr_init_msg_seq_from_rtc(void)
{
    rtc_boots++;

    if(rtc_msg_id == 0){
        rtc_msg_id = (uint16_t)(esp_random() & 0xFFFF);
        if(rtc_msg_id == 0) rtc_msg_id = 1;
    }

    if(rtc_seq == 0){
        rtc_seq = (uint16_t)(esp_random() & 0xFFFF);
        if(rtc_seq == 0) rtc_seq = 1;
    }

    g_msg_id = rtc_msg_id;
    g_my_seq = rtc_seq;

    ESP_LOGI(TAG, "RTC init: boots=%" PRIu32 " next_msg_id=%u next_seq=%u",
             rtc_boots,
             (unsigned)g_msg_id,
             (unsigned)g_my_seq);
}

static inline void lora_spi_lock(void){ xSemaphoreTake(g_lora_spi_mutex, portMAX_DELAY); }
static inline void lora_spi_unlock(void){ xSemaphoreGive(g_lora_spi_mutex); }

static uint16_t etx_compute_x100(uint32_t tx, uint32_t ack)
{
    if(tx == 0) return 100;
    if(ack == 0) return ETX_MAX_X100;
    uint32_t v = (tx * 100U) / ack;
    if(v < 100) v = 100;
    if(v > ETX_MAX_X100) v = ETX_MAX_X100;
    return (uint16_t)v;
}

static void json_escape_into(char *dst, size_t dst_sz, const char *src)
{
    size_t o=0;
    for(size_t i=0; src && src[i] && o+2 < dst_sz; i++){
        unsigned char c=(unsigned char)src[i];
        if(c=='\\' || c=='"'){ dst[o++]='\\'; dst[o++]=(char)c; }
        else if(c=='\n'){ dst[o++]='\\'; dst[o++]='n'; }
        else if(c=='\r'){ dst[o++]='\\'; dst[o++]='r'; }
        else if(c=='\t'){ dst[o++]='\\'; dst[o++]='t'; }
        else if(c>=32 && c<=126){ dst[o++]=(char)c; }
        else { /* drop */ }
        if(o >= dst_sz-1) break;
    }
    dst[o]=0;
}


#if (MR_BOARD_PRESET == MR_BOARD_HELTEC_V3)
static void prog_button_init(void)
{
    gpio_config_t io = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << 0), // GPIO0 = PROG button
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
}

static void prog_button_poll(void)
{
    static int last = 1;
    static uint32_t last_ms = 0;

    const int level = gpio_get_level((gpio_num_t)0);
    const uint32_t t = now_ms();

    if (level != last) {
        if ((t - last_ms) < 30) return; // debounce
        last_ms = t;

        if (last == 1 && level == 0) {
            xSemaphoreTake(g_mutex, portMAX_DELAY);
            g_cfg.display_enable = !g_cfg.display_enable;
            rtc_display_enable = g_cfg.display_enable ? 1 : 0;
            (void)mr_cfg_save_nvs(&g_cfg);
            mr_cfg_apply(&g_cfg);
            xSemaphoreGive(g_mutex);

            if (g_display_enabled) {
                mr_display_show_status(
                    "DISPLAY ON",
                    cfg_board_str(),
                    cfg_lora_chip_str(),
                    g_callsign_rt
                );
                ESP_LOGI(TAG, "PROG button: display ON (persisted)");
            } else {
                ESP_LOGI(TAG, "PROG button: display OFF (persisted)");
            }
        }

        last = level;
    }
}
#endif

#if (MR_BOARD_PRESET == MR_BOARD_HELTEC_V3)
static void heltec_vext_on(void)
{
    gpio_config_t io = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << PIN_VEXT_CTRL),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    // ✅ Heltec: LOW = VEXT an
    gpio_set_level(PIN_VEXT_CTRL, 0);

    // ✅ Stabilitäts-Delay: Peripherie + Pullups + RF-Switch hochfahren lassen
    vTaskDelay(pdMS_TO_TICKS(100));
}

static void board_power_boot_init(void)
{
#if (MR_BOARD_PRESET == MR_BOARD_HELTEC_V3)
    // VEXT according to last retained display state
    // Heltec: LOW = ON, HIGH = OFF
    gpio_set_direction((gpio_num_t)VEXT_CTRL_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)VEXT_CTRL_PIN, rtc_display_enable ? 0 : 1);

    // Battery divider enable pin init (default OFF!)
    gpio_set_direction((gpio_num_t)BATT_EN_GPIO, GPIO_MODE_OUTPUT);
#if BATT_EN_ACTIVE_LOW
    gpio_set_level((gpio_num_t)BATT_EN_GPIO, 1); // OFF
#else
    gpio_set_level((gpio_num_t)BATT_EN_GPIO, 0); // OFF
#endif
    vTaskDelay(pdMS_TO_TICKS(50));
#endif

}


static inline void batt_path_enable(bool en)
{
#if (MR_BOARD_PRESET == MR_BOARD_HELTEC_V3)
#if BATT_EN_ACTIVE_LOW
    gpio_set_level((gpio_num_t)BATT_EN_GPIO, en ? 0 : 1);
#else
    gpio_set_level((gpio_num_t)BATT_EN_GPIO, en ? 1 : 0);
#endif
#else
    (void)en;
#endif
}

#endif

#if ((MR_BOARD_PRESET == MR_BOARD_TBEAM_V11_SX1276) || \
     (MR_BOARD_PRESET == MR_BOARD_TBEAM_V12_AXP2101))

static void pwr_button_init(void)
{
    gpio_config_t io = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIN_PWR_BUTTON),
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
}

static void pwr_button_poll(void)
{
    static int last_level = 1;
    static uint32_t last_change_ms = 0;

    const int level = gpio_get_level((gpio_num_t)PIN_PWR_BUTTON);
    const uint32_t tnow = now_ms();

    if (level != last_level) {
        if ((tnow - last_change_ms) < 30) return;
        last_change_ms = tnow;

        if (last_level == 1 && level == 0) {
            xSemaphoreTake(g_mutex, portMAX_DELAY);
            g_cfg.display_enable = !g_cfg.display_enable;
            rtc_display_enable = g_cfg.display_enable ? 1 : 0;
            (void)mr_cfg_save_nvs(&g_cfg);
            mr_cfg_apply(&g_cfg);
            xSemaphoreGive(g_mutex);

            if (g_display_enabled) {
                ESP_LOGI(TAG, "PWR button: display ON (persisted)");
                mr_display_show_status("DISPLAY ON",
                           cfg_board_str(),
                           cfg_lora_chip_str(),
                           g_callsign_rt);
            } else {
                ESP_LOGI(TAG, "PWR button: display OFF (persisted)");
            }
        }

        last_level = level;
    }
}
#endif

// ============================================================================
// ================================ SEEN ======================================
// ============================================================================
static bool seen_before(const char src[8], uint16_t id)
{
    for(int i=0;i<SEEN_CACHE_SIZE;i++){
        if(!seen_cache[i].used) continue;
        if(call7_eq(seen_cache[i].src,src) && seen_cache[i].id==id) return true;
    }
    return false;
}
static void remember_msg(const char src[8], uint16_t id)
{
    static int idx=0;
    seen_cache[idx].used=true;
    memcpy(seen_cache[idx].src,src,8);
    seen_cache[idx].id=id;
    idx=(idx+1)%SEEN_CACHE_SIZE;
}


// ============================================================================
// =============================== NEIGHBORS ==================================
// ============================================================================
static int neighbor_find_locked(const char call[8])
{
    for(int i=0;i<MAX_NEIGHBORS;i++){
        if(!neighbors[i].used) continue;
        if(call7_eq(neighbors[i].call, call)) return i;
    }
    return -1;
}
static int neighbor_ensure_locked(const char call[8])
{
    int idx = neighbor_find_locked(call);
    if(idx >= 0) return idx;

    for(int i=0;i<MAX_NEIGHBORS;i++){
        if(!neighbors[i].used){
            neighbors[i].used=true;
            memcpy(neighbors[i].call, call, 8);
            neighbors[i].rssi=-127;
            neighbors[i].t_ms=now_ms();
            neighbors[i].tx_attempts=0;
            neighbors[i].ack_ok=0;
            neighbors[i].last_decay_ms=now_ms();
            return i;
        }
    }
    return -1;
}
static void neighbor_decay_locked(neighbor_t *n)
{
    uint32_t t=now_ms();
    if(t - n->last_decay_ms < ETX_DECAY_MS) return;
    n->tx_attempts = (n->tx_attempts + 1) / 2;
    n->ack_ok      = (n->ack_ok + 1) / 2;
    n->last_decay_ms = t;
}
static void neighbor_update_rssi_locked(const char call[8], int rssi)
{
    int idx = neighbor_ensure_locked(call);
    if(idx < 0) return;
    neighbors[idx].rssi=rssi;
    neighbors[idx].t_ms=now_ms();
}
static void neighbor_tx_attempt_locked(const char call[8])
{
    int idx = neighbor_ensure_locked(call);
    if(idx < 0) return;
    neighbor_decay_locked(&neighbors[idx]);
    neighbors[idx].tx_attempts++;
}
static void neighbor_ack_ok_locked(const char call[8])
{
    int idx = neighbor_ensure_locked(call);
    if(idx < 0) return;
    neighbor_decay_locked(&neighbors[idx]);
    neighbors[idx].ack_ok++;
}
static void neighbor_cleanup_locked(void)
{
    uint32_t t=now_ms();
    for(int i=0;i<MAX_NEIGHBORS;i++){
        if(!neighbors[i].used) continue;
        if(t - neighbors[i].t_ms > NEIGHBOR_TIMEOUT_MS) neighbors[i].used=false;
    }
}


// ============================================================================
// ================================= ROUTES ==================================
// ============================================================================
static bool holddown_active_locked(route_t *r)
{
    uint32_t t=now_ms();
    return (t < r->hold_until_ms);
}
static void route_update_locked(const char dst[8],                                const char next[8],
                                uint16_t seq,
                                uint16_t etx_x100,
                                int last_rssi)
{
    char me[8]; call7_set(me, g_callsign_rt);
    if(call7_eq(next, me)) return;

    uint32_t t=now_ms();

    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used) continue;
        if(!call7_eq(routes[i].dst, dst)) continue;

        bool replace=false;
        if(seq > routes[i].seq){
            replace=true;
        }else if(seq == routes[i].seq){
            if(holddown_active_locked(&routes[i])){
                if(etx_x100 + (2*g_routeadv_delta_etx) < routes[i].etx_x100) replace=true;
            }else{
                if(etx_x100 < routes[i].etx_x100) replace=true;
            }
        }

        if(replace){
            bool next_changed = !call7_eq(routes[i].next, next);
            memcpy(routes[i].next, next, 8);
            routes[i].seq = seq;
            routes[i].etx_x100 = etx_x100;
            routes[i].last_rssi = last_rssi;
            if(next_changed){
                routes[i].hold_until_ms = t + g_holddown_ms;
            }
        }
        routes[i].t_ms = t;
        return;
    }

    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used){
            routes[i].used=true;
            memcpy(routes[i].dst, dst, 8);
            memcpy(routes[i].next, next, 8);
            routes[i].seq=seq;
            routes[i].etx_x100=etx_x100;
            routes[i].last_rssi=last_rssi;
            routes[i].t_ms=t;
            routes[i].hold_until_ms = t + 1000;
            return;
        }
    }
}
static bool route_lookup_locked(const char dst[8], char out_next[8])
{
    uint32_t t=now_ms();
    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used) continue;
        if(!call7_eq(routes[i].dst, dst)) continue;
        if(t - routes[i].t_ms > ROUTE_TIMEOUT_MS) return false;
        memcpy(out_next, routes[i].next, 8);
        return true;
    }
    return false;
}
static void route_cleanup_locked(void)
{
    uint32_t t=now_ms();
    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used) continue;
        if(t - routes[i].t_ms > ROUTE_TIMEOUT_MS) routes[i].used=false;
    }
}


// ============================================================================
// ============================ REPLAY + SEQ SAFE =============================
// ============================================================================
static replay_t* replay_get_locked(const char src[8])
{
    for(int i=0;i<MAX_REPLAY;i++){
        if(!replay_tab[i].used) continue;
        if(call7_eq(replay_tab[i].src, src)) return &replay_tab[i];
    }
    for(int i=0;i<MAX_REPLAY;i++){
        if(!replay_tab[i].used){
            replay_tab[i].used = true;
            replay_tab[i].has_seq = false;
            memcpy(replay_tab[i].src, src, 8);
            replay_tab[i].last_seq = 0;
            replay_tab[i].t_ms = now_ms();
            return &replay_tab[i];
        }
    }
    int oldest=0;
    uint32_t ot=replay_tab[0].t_ms;
    for(int i=1;i<MAX_REPLAY;i++){
        if(!replay_tab[i].used){ oldest=i; break; }
        if(replay_tab[i].t_ms < ot){ oldest=i; ot=replay_tab[i].t_ms; }
    }
    replay_tab[oldest].used = true;
    replay_tab[oldest].has_seq = false;
    memcpy(replay_tab[oldest].src, src, 8);
    replay_tab[oldest].last_seq = 0;
    replay_tab[oldest].t_ms = now_ms();
    return &replay_tab[oldest];
}
static bool seq_is_newer_u16(uint16_t seq, uint16_t last_seq)
{
    return ((int16_t)(seq - last_seq) > 0);
}
static bool replay_check_locked(const char src[8], uint16_t seq)
{
    replay_t *r = replay_get_locked(src);
    r->t_ms = now_ms();

    // Erster SEC-Frame dieses Senders: immer akzeptieren
    if(!r->has_seq){
        return true;
    }

    return seq_is_newer_u16(seq, r->last_seq);
}
static void replay_update_locked(const char src[8], uint16_t seq)
{
    replay_t *r = replay_get_locked(src);
    r->last_seq = seq;
    r->has_seq = true;
    r->t_ms = now_ms();
}

// ============================================================================
// ================================ SPI INIT ==================================
// ============================================================================
#if defined(VSPI_HOST)
  #define LORA_SPI_HOST VSPI_HOST
#elif defined(SPI3_HOST)
  #define LORA_SPI_HOST SPI3_HOST
#else
  #define LORA_SPI_HOST SPI2_HOST
#endif

static void init_spi(void)
{
    spi_bus_config_t bus={
        .miso_io_num=PIN_LORA_MISO,
        .mosi_io_num=PIN_LORA_MOSI,
        .sclk_io_num=PIN_LORA_SCK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LORA_SPI_HOST, &bus, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev={
        .clock_speed_hz=2000000, // 2MHz robust
        .mode=0,
        .spics_io_num=PIN_LORA_NSS, // Hardware CS (ESP-IDF toggles per transaction)
        .queue_size=1
    };
    ESP_ERROR_CHECK(spi_bus_add_device(LORA_SPI_HOST, &dev, &lora_spi));
}


// ============================================================================
// ============================= LoRa Abstraction =============================
// ============================================================================
static void lora_hw_reset(void)
{
    gpio_set_direction(PIN_LORA_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_LORA_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_LORA_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}


// ----------------------------- SX1276 DRIVER ------------------------------
#if defined(MR_LORA_CHIP_SX1276)

#define SX1276_REG_FIFO                 0x00
#define SX1276_REG_OP_MODE              0x01
#define SX1276_REG_FRF_MSB              0x06
#define SX1276_REG_FRF_MID              0x07
#define SX1276_REG_FRF_LSB              0x08
#define SX1276_REG_PA_CONFIG            0x09
#define SX1276_REG_FIFO_ADDR_PTR        0x0D
#define SX1276_REG_FIFO_TX_BASE_ADDR    0x0E
#define SX1276_REG_FIFO_RX_BASE_ADDR    0x0F
#define SX1276_REG_FIFO_RX_CURRENT_ADDR 0x10
#define SX1276_REG_IRQ_FLAGS            0x12
#define SX1276_REG_RX_NB_BYTES          0x13
#define SX1276_REG_PKT_RSSI_VALUE       0x1A
#define SX1276_REG_PAYLOAD_LENGTH       0x22
#define SX1276_REG_MODEM_CONFIG_1       0x1D
#define SX1276_REG_MODEM_CONFIG_2       0x1E
#define SX1276_REG_MODEM_CONFIG_3       0x26
#define SX1276_REG_VERSION              0x42

#define SX1276_IRQ_RX_DONE 0x40
#define SX1276_IRQ_TX_DONE 0x08

static void sx1276_wr(uint8_t r, uint8_t v)
{
    uint8_t tx[2]={ (uint8_t)(r|0x80), v };
    spi_transaction_t t={.length=16,.tx_buffer=tx};
    spi_device_transmit(lora_spi,&t);
}
static uint8_t sx1276_rd(uint8_t r)
{
    uint8_t tx[2]={ (uint8_t)(r&0x7F), 0 };
    uint8_t rx[2]={0};
    spi_transaction_t t={.length=16,.tx_buffer=tx,.rx_buffer=rx};
    spi_device_transmit(lora_spi,&t);
    return rx[1];
}
static void sx1276_clear_irq(void){ sx1276_wr(SX1276_REG_IRQ_FLAGS, 0xFF); }

static uint32_t hz_to_frf(uint32_t f)
{
    uint64_t frf=((uint64_t)f<<19)/32000000ULL;
    return (uint32_t)frf;
}

void lora_init_radio(void)
{
    lora_spi_lock();

    // sleep
    sx1276_wr(SX1276_REG_OP_MODE, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 863.000 MHz / SF7 / BW125 / CR4/5 / CRC on
    // uint32_t frf=hz_to_frf(DEFAULT_RF_FREQ_HZ);
    uint32_t frf=hz_to_frf(g_rf_freq_hz_runtime);
    sx1276_wr(SX1276_REG_FRF_MSB, frf>>16);
    sx1276_wr(SX1276_REG_FRF_MID, frf>>8);
    sx1276_wr(SX1276_REG_FRF_LSB, frf);

    sx1276_wr(SX1276_REG_MODEM_CONFIG_1, 0x72);
    sx1276_wr(SX1276_REG_MODEM_CONFIG_2, (7<<4)|(1<<2)|0x03);
    sx1276_wr(SX1276_REG_MODEM_CONFIG_3, 0x04);

    int8_t p = g_tx_power_dbm_runtime;
    if(p < 2) p = 2;
    if(p > 17) p = 17;
    sx1276_wr(SX1276_REG_PA_CONFIG, (uint8_t)(0x80 | (p - 2)));

    sx1276_wr(SX1276_REG_FIFO_RX_BASE_ADDR, 0);
    sx1276_wr(SX1276_REG_FIFO_ADDR_PTR, 0);

    // RX continuous
    sx1276_wr(SX1276_REG_OP_MODE, 0x85);

    lora_spi_unlock();
}

void lora_set_rx_continuous(void)
{
    lora_spi_lock();
    sx1276_wr(SX1276_REG_OP_MODE, 0x85);
    lora_spi_unlock();
}

void lora_recover_radio(void)
{
    ESP_LOGE(TAG, "LoRa recover (SX1276)");
    lora_init_radio();
}

void lora_send_frame(const uint8_t *d, size_t len)
{
    // optional CAD backoff
    if(g_cad_enable){
        uint32_t w = (esp_random() % (CAD_JITTER_MS+1));
        vTaskDelay(pdMS_TO_TICKS(CAD_WAIT_MS + w));
    }

    lora_spi_lock();

    sx1276_clear_irq();
    sx1276_wr(SX1276_REG_FIFO_TX_BASE_ADDR, 0);
    sx1276_wr(SX1276_REG_FIFO_ADDR_PTR, 0);
    for(size_t i=0;i<len;i++) sx1276_wr(SX1276_REG_FIFO, d[i]);
    sx1276_wr(SX1276_REG_PAYLOAD_LENGTH, (uint8_t)len);

    sx1276_wr(SX1276_REG_OP_MODE, 0x83); // TX

    uint32_t t0 = now_ms();
    while(1){
        uint8_t irq = sx1276_rd(SX1276_REG_IRQ_FLAGS);
        if(irq & SX1276_IRQ_TX_DONE) break;
        if((now_ms() - t0) > LORA_TX_TIMEOUT_MS){
            lora_spi_unlock();
            ESP_LOGE(TAG, "LoRa TX timeout! Recovering radio...");
            lora_recover_radio();
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    sx1276_clear_irq();
    sx1276_wr(SX1276_REG_OP_MODE, 0x85); // RX cont

    lora_spi_unlock();
}

bool lora_try_read_frame(uint8_t *buf, size_t maxlen, uint8_t *out_len, int *out_rssi)
{
    uint8_t len=0;
    int rssi=-127;

    lora_spi_lock();

    uint8_t irq = sx1276_rd(SX1276_REG_IRQ_FLAGS);
    if(!(irq & SX1276_IRQ_RX_DONE)){
        lora_spi_unlock();
        return false;
    }

    len = sx1276_rd(SX1276_REG_RX_NB_BYTES);
    if(len==0 || len>maxlen){
        sx1276_clear_irq();
        lora_spi_unlock();
        return false;
    }

    uint8_t addr = sx1276_rd(SX1276_REG_FIFO_RX_CURRENT_ADDR);
    sx1276_wr(SX1276_REG_FIFO_ADDR_PTR, addr);

    for(int i=0;i<len;i++) buf[i]=sx1276_rd(SX1276_REG_FIFO);

    sx1276_clear_irq();
    rssi = (int)sx1276_rd(SX1276_REG_PKT_RSSI_VALUE) - 157;

    lora_spi_unlock();

    *out_len = len;
    *out_rssi = rssi;
    return true;
}

void lora_log_chip_info(void)
{
    lora_spi_lock();
    uint8_t v = sx1276_rd(SX1276_REG_VERSION);
    lora_spi_unlock();
    ESP_LOGI(TAG,"LoRa chip: SX1276/78 (RegVersion=0x%02X, expect ~0x12)", v);
}

#endif // MR_LORA_CHIP_SX1276


// ----------------------------- SX1262 DRIVER ------------------------------
// ----------------------------- SX1262 DRIVER (COMPLETE) -------------------
// ----------------------------- SX1262 DRIVER (FIXED) -----------------------
#if defined(MR_LORA_CHIP_SX1262)

// SX126x opcodes (SX1261/2)
#define SX126X_SET_STANDBY           0x80
#define SX126X_SET_PACKET_TYPE       0x8A
#define SX126X_SET_RF_FREQUENCY      0x86
#define SX126X_SET_MODULATION        0x8B
#define SX126X_SET_PACKET_PARAMS     0x8C
#define SX126X_SET_BUFFER_BASE       0x8F
#define SX126X_SET_TX                0x83
#define SX126X_SET_RX                0x82
#define SX126X_GET_IRQ_STATUS        0x12           //15   // ✅ korrekt
#define SX126X_CLEAR_IRQ_STATUS      0x02
#define SX126X_SET_DIO_IRQ_PARAMS    0x08
#define SX126X_WRITE_BUFFER          0x0E
#define SX126X_READ_BUFFER           0x1E
#define SX126X_GET_PACKET_STATUS     0x14
#define SX126X_GET_RX_BUFFER_STATUS  0x13

// Heltec/SX1262 spezifisch (aus deinem funktionierenden Test)
#define SX126X_SET_DIO3_TCXO_CTRL    0x97
#define SX126X_CALIBRATE             0x89
#define SX126X_SET_PA_CONFIG         0x95
#define SX126X_SET_TX_PARAMS         0x8E
#define SX126X_SET_DIO2_RFSWITCH     0x9D

// IRQ bits (SX126x)
#define SX126X_IRQ_TX_DONE           (1<<0)
#define SX126X_IRQ_RX_DONE           (1<<1)
#define SX126X_IRQ_TIMEOUT           (1<<9)
#define SX126X_IRQ_CRC_ERR           (1<<6)

static void sx126x_busy_wait(void)
{
    const uint32_t t0 = now_ms();
    while(gpio_get_level(PIN_LORA_BUSY) == 1){
        if(now_ms() - t0 > 1000){
            // BUSY hängt -> nicht endlos blockieren
            break;
        }
        esp_rom_delay_us(10);
    }
}

/*static void sx126x_xfer(const uint8_t *tx, uint8_t *rx, size_t n)
{
    spi_transaction_t t = {0};
    t.length    = n * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    spi_device_transmit(lora_spi, &t); // CS auto (spics_io_num)
}
*/
static void sx126x_xfer(const uint8_t *tx, uint8_t *rx, size_t n)
{
    spi_transaction_t t = {0};
    t.length = n * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    // Polling ist bei LoRa-Kommandos VIEL stabiler
    spi_device_polling_transmit(lora_spi, &t); 
}


// reines Kommando (Opcode+Args) ohne Readback
static void sx126x_cmd(const uint8_t *cmd, size_t n)
{
    uint8_t rx_dummy[64] = {0};
    if(n > sizeof(rx_dummy)) return;

    sx126x_busy_wait();
    sx126x_xfer(cmd, rx_dummy, n);
    sx126x_busy_wait();
}

// Lesen: opcode + dummy + n bytes
static void sx126x_read(uint8_t opcode, uint8_t *out, size_t n)
{
    if(n > 64) return;

    uint8_t tx[2 + 64] = {0};
    uint8_t rx[2 + 64] = {0};

    tx[0] = opcode;
    tx[1] = 0x00; // dummy

    sx126x_busy_wait();
    sx126x_xfer(tx, rx, 2 + n);
    sx126x_busy_wait();

    memcpy(out, rx + 2, n);
}

static void sx126x_clear_irq(uint16_t mask)
{
    uint8_t cmd[3] = {
        SX126X_CLEAR_IRQ_STATUS,
        (uint8_t)(mask >> 8),
        (uint8_t)(mask & 0xFF)
    };
    sx126x_cmd(cmd, sizeof(cmd));
}

static uint16_t sx126x_get_irq(void)
{
    uint8_t b[2] = {0};
    sx126x_read(SX126X_GET_IRQ_STATUS, b, 2);
    return (uint16_t)((b[0] << 8) | b[1]);
}

// WriteBuffer MUSS in EINER Transaktion sein: opcode + payload(offset+data)
static void sx126x_cmd_with_payload(uint8_t opcode, const uint8_t *payload, size_t n)
{
    // n = offset(1) + data(len) => max 256
    if(n > 256) return;

    uint8_t tx[1 + 256] = {0};
    uint8_t rx_dummy[1 + 256] = {0};

    tx[0] = opcode;
    memcpy(&tx[1], payload, n);

    sx126x_busy_wait();
    sx126x_xfer(tx, rx_dummy, 1 + n);
    sx126x_busy_wait();
}

// Heltec/SX1262: PacketParams müssen PayloadLen korrekt setzen
static void sx126x_set_packet_params(uint8_t payload_len)
{
    // LoRa PacketParams:
    // preamble = 8 (0x0008)
    // header   = explicit (0x00)
    // len      = payload_len
    // CRC      = on (0x01)
    // IQ       = standard (0x00)
    uint8_t pkt[7] = {
        SX126X_SET_PACKET_PARAMS,
        0x00, 0x08,
        0x00,
        payload_len,
        0x01,
        0x00
    };
    sx126x_cmd(pkt, sizeof(pkt));
}

static void lora_init_radio(void)
{
    // Nach heltec_vext_on()
    gpio_set_direction(PIN_LORA_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_LORA_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_LORA_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50)); // Warten bis Chip bereit
    // BUSY input
    gpio_set_direction(PIN_LORA_BUSY, GPIO_MODE_INPUT);

    lora_spi_lock();

    // Standby XOSC (Heltec stabil)
    { uint8_t stby[2] = { SX126X_SET_STANDBY, 0x01 }; sx126x_cmd(stby, sizeof(stby)); }

    // TCXO on DIO3 (1.8V) + delay (0x000064)
    { uint8_t tcxo[5] = { SX126X_SET_DIO3_TCXO_CTRL, 0x01, 0x00, 0x00, 0x64 }; sx126x_cmd(tcxo, sizeof(tcxo)); }
    vTaskDelay(pdMS_TO_TICKS(5)); // ✅ echtes Delay hilft Heltec

    // Calibrate all
    { uint8_t cal[2] = { SX126X_CALIBRATE, 0x7F }; sx126x_cmd(cal, sizeof(cal)); }

    // Packet type: LoRa
    { uint8_t ptype[2] = { SX126X_SET_PACKET_TYPE, 0x01 }; sx126x_cmd(ptype, sizeof(ptype)); }

    // DIO2 RF switch auto
    { uint8_t dio2[2] = { SX126X_SET_DIO2_RFSWITCH, 0x01 }; sx126x_cmd(dio2, sizeof(dio2)); }

    // PA config (Heltec example)
    { uint8_t pa[5] = { SX126X_SET_PA_CONFIG, 0x04, 0x07, 0x00, 0x01 }; sx126x_cmd(pa, sizeof(pa)); }

    // TxParams runtime from config + 200us ramp 
    {
    int8_t p = g_tx_power_dbm_runtime;
    if(p < -9) p = -9;
    if(p > 22) p = 22;

        uint8_t txp[3] = {
            SX126X_SET_TX_PARAMS,
            (uint8_t)p,
            0x04
        };
    sx126x_cmd(txp, sizeof(txp));
    }

    // RF frequency:
    /*ESP_LOGW(TAG, "SX1262 init RF=%lu Hz TX=%d dBm",
         (unsigned long)g_rf_freq_hz_runtime,
         (int)g_tx_power_dbm_runtime);
    */
    uint32_t rf = (uint32_t)(((uint64_t)g_rf_freq_hz_runtime << 25) / 32000000ULL);
    uint8_t rfcmd[5] = {
        SX126X_SET_RF_FREQUENCY,
        (uint8_t)(rf >> 24),
        (uint8_t)(rf >> 16),
        (uint8_t)(rf >> 8),
        (uint8_t)(rf)
    };
    sx126x_cmd(rfcmd, sizeof(rfcmd));

    // Modulation: SF7, BW125(0x04), CR4/5(0x01), LDRO=0
    { uint8_t mod[5] = { SX126X_SET_MODULATION, 0x07, 0x04, 0x01, 0x00 }; sx126x_cmd(mod, sizeof(mod)); }

    // Buffer base: TX=0, RX=0
    { uint8_t base[3] = { SX126X_SET_BUFFER_BASE, 0x00, 0x00 }; sx126x_cmd(base, sizeof(base)); }

    // Default PacketParams (payload len wird pro TX gesetzt)
    sx126x_set_packet_params(0xFF);

    // IRQ mapping to DIO1
    uint16_t mask = SX126X_IRQ_RX_DONE | SX126X_IRQ_TX_DONE | SX126X_IRQ_CRC_ERR | SX126X_IRQ_TIMEOUT;
    uint8_t dio[9] = {
        SX126X_SET_DIO_IRQ_PARAMS,
        (uint8_t)(mask >> 8), (uint8_t)(mask & 0xFF), // irq mask
        (uint8_t)(mask >> 8), (uint8_t)(mask & 0xFF), // dio1 mask
        0x00, 0x00, // dio2 mask
        0x00, 0x00  // dio3 mask
    };
    sx126x_cmd(dio, sizeof(dio));

    sx126x_clear_irq(0xFFFF);

    // RX continuous
    { uint8_t rx[4] = { SX126X_SET_RX, 0xFF, 0xFF, 0xFF }; sx126x_cmd(rx, sizeof(rx)); }

    lora_spi_unlock();
}

void lora_set_rx_continuous(void)
{
    lora_spi_lock();
    uint8_t rx[4] = { SX126X_SET_RX, 0xFF, 0xFF, 0xFF };
    sx126x_cmd(rx, sizeof(rx));
    lora_spi_unlock();
}

void lora_recover_radio(void)
{
    ESP_LOGE(TAG, "LoRa recover (SX1262): re-init");
    lora_init_radio();
}

static void lora_send_frame(const uint8_t *d, size_t len)
{
    if(g_cad_enable){
        uint32_t w = (esp_random() % (CAD_JITTER_MS + 1));
        vTaskDelay(pdMS_TO_TICKS(CAD_WAIT_MS + w));
    }

    if(len > 255) len = 255;

    lora_spi_lock();

    // WriteBuffer: opcode + [offset=0] + payload  (EINE Transaktion)
    uint8_t wb[1 + 255];
    wb[0] = 0x00; // offset
    memcpy(&wb[1], d, len);
    sx126x_cmd_with_payload(SX126X_WRITE_BUFFER, wb, 1 + len);

    // PacketParams: echte Länge setzen (Heltec wichtig)
    sx126x_set_packet_params((uint8_t)len);

    // Clear IRQ
    sx126x_clear_irq(0xFFFF);

    // SetTx timeout ~2s (in 15.625us units)
    uint32_t to = 128000;
    uint8_t txcmd[4] = { SX126X_SET_TX, (uint8_t)(to >> 16), (uint8_t)(to >> 8), (uint8_t)to };
    sx126x_cmd(txcmd, sizeof(txcmd));

    // Wait TX_DONE
    uint32_t t0 = now_ms();
    while(1){
        uint16_t irq = sx126x_get_irq();

        if(irq & SX126X_IRQ_TX_DONE){
            sx126x_clear_irq(SX126X_IRQ_TX_DONE);
            break;
        }
        if(irq & SX126X_IRQ_TIMEOUT){
            sx126x_clear_irq(SX126X_IRQ_TIMEOUT);
            lora_spi_unlock();
            ESP_LOGE(TAG, "LoRa TX timeout! Recovering radio...");
            lora_recover_radio();
            return;
        }
        if((now_ms() - t0) > (LORA_TX_TIMEOUT_MS + 200)){
            lora_spi_unlock();
            ESP_LOGE(TAG, "LoRa TX timeout (host)! Recovering radio...");
            lora_recover_radio();
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    // Back to RX continuous
    uint8_t rx[4] = { SX126X_SET_RX, 0xFF, 0xFF, 0xFF };
    sx126x_cmd(rx, sizeof(rx));

    lora_spi_unlock();
}

static bool lora_try_read_frame(uint8_t *buf, size_t maxlen, uint8_t *out_len, int *out_rssi)
{
    lora_spi_lock();

    uint16_t irq = sx126x_get_irq();

    // CRC error -> drop
    if(irq & SX126X_IRQ_CRC_ERR){
        sx126x_clear_irq(SX126X_IRQ_CRC_ERR);
        lora_spi_unlock();
        return false;
    }
    // timeout -> drop
    if(irq & SX126X_IRQ_TIMEOUT){
        sx126x_clear_irq(SX126X_IRQ_TIMEOUT);
        lora_spi_unlock();
        return false;
    }

    if(!(irq & SX126X_IRQ_RX_DONE)){
        lora_spi_unlock();
        return false;
    }

    // Clear RX_DONE early
    sx126x_clear_irq(SX126X_IRQ_RX_DONE);

    // GetRxBufferStatus: returns [payloadLen, startPointer]
    uint8_t info[2] = {0};
    sx126x_read(SX126X_GET_RX_BUFFER_STATUS, info, 2);
    uint8_t payLen = info[0];
    uint8_t start  = info[1];

    if(payLen == 0 || payLen > maxlen){
        lora_spi_unlock();
        return false;
    }

    // ReadBuffer: opcode + offset + dummy + payload (dummy clocks required)
    uint8_t txb[3 + 255] = {0};
    uint8_t rxb[3 + 255] = {0};

    txb[0] = SX126X_READ_BUFFER;
    txb[1] = start;
    txb[2] = 0x00; // dummy
    // rest remains 0x00 to clock out data

    sx126x_busy_wait();
    sx126x_xfer(txb, rxb, 3 + payLen);
    sx126x_busy_wait();

    memcpy(buf, rxb + 3, payLen);

    // RSSI from GetPacketStatus (RSSI_sync in -0.5 dB steps)
    uint8_t st[3] = {0};
    sx126x_read(SX126X_GET_PACKET_STATUS, st, 3);
    int rssi = -(int)(st[0] / 2);

    lora_spi_unlock();

    *out_len  = payLen;
    *out_rssi = rssi;
    return true;
}

// Optional: Status lesen (ohne manuelles CS!)
static uint8_t sx1262_get_status(void)
{
    // GetStatus: opcode 0xC0, returns status in first received byte
    uint8_t tx[2] = { 0xC0, 0x00 };
    uint8_t rx[2] = { 0, 0 };

    sx126x_busy_wait();
    spi_transaction_t t = { .length = 16, .tx_buffer = tx, .rx_buffer = rx };
    spi_device_polling_transmit(lora_spi, &t);
    sx126x_busy_wait();

    return rx[0];
}

static void lora_log_chip_info(void)
{
    uint8_t st = sx1262_get_status();
    ESP_LOGI(TAG, "LoRa chip: SX1262 (Heltec V3.x) status=0x%02X", st);
}

#endif // MR_LORA_CHIP_SX1262


// ============================================================================
// =============================== Relay (GPIO) ===============================
// ============================================================================
#if MR_RELAY_ENABLE
void relay_init(void)
{
    gpio_config_t io = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << g_relay_gpio_runtime),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    // HOLD sicher aus, sonst kannst du initial nicht setzen
    gpio_hold_dis((gpio_num_t)g_relay_gpio_runtime);

    bool on = (rtc_relay_state != 0);
    int level = on ? (RELAY_ACTIVE_LEVEL ? 1 : 0)
                   : (RELAY_ACTIVE_LEVEL ? 0 : 1);
    gpio_set_level((gpio_num_t)g_relay_gpio_runtime, level);
    g_relay_on = on;

    // optional: gleich wieder halten (Reset-Schutz)
    gpio_hold_en((gpio_num_t)g_relay_gpio_runtime);
}

static void relay_set(bool on)
{
    rtc_relay_state = on ? 1 : 0;   // <- merkt sich über Deep Sleep

    // falls hold aktiv war: kurz lösen, ändern, dann (optional) wieder halten
    gpio_hold_dis((gpio_num_t)g_relay_gpio_runtime);

    g_relay_on = on;
    int level = on ? (RELAY_ACTIVE_LEVEL ? 1 : 0)
                   : (RELAY_ACTIVE_LEVEL ? 0 : 1);
    gpio_set_level((gpio_num_t)g_relay_gpio_runtime, level);

    // optional: hält auch über Reset-Events (WDT etc.)
    gpio_hold_en((gpio_num_t)g_relay_gpio_runtime);
}

static void relay_toggle(void){ relay_set(!g_relay_on); }
#endif


// ============================================================================
// =============================== Battery ADC ================================
// ============================================================================
#if MR_BATT_ENABLE
static int batt_adc_gpio_to_chan(gpio_num_t gpio)
{
    // ESP32-S3 (Heltec V3) Mapping
    #if CONFIG_IDF_TARGET_ESP32S3
    switch(gpio){
        case 1:  return ADC_CHANNEL_0; // Heltec V3.2 VBAT
        case 2:  return ADC_CHANNEL_1;
        case 3:  return ADC_CHANNEL_2;
        case 4:  return ADC_CHANNEL_3;
        default: return -1;
    }
    #else
    // Klassisches ESP32 (LILYGO) Mapping
    switch(gpio){
        case 36: return ADC_CHANNEL_0; // ADC1_CH0
        case 39: return ADC_CHANNEL_3; // ADC1_CH3 (falls je genutzt)
        case 34: return ADC_CHANNEL_6; // ADC1_CH6
        case 35: return ADC_CHANNEL_7; // ✅ ADC1_CH7 (T3 V1.6.1 VBAT sense)
        default: return -1;
    }
    #endif
}
#endif

#if CONFIG_IDF_TARGET_ESP32S3

#if MR_BATT_ENABLE
static void batt_enable_hw(void)
{
#ifdef BATT_EN_GPIO
    gpio_config_t io = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << BATT_EN_GPIO),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    ESP_ERROR_CHECK(gpio_config(&io));

#if BATT_EN_ACTIVE_LOW
    gpio_set_level(BATT_EN_GPIO, 0);   // ✅ enable divider
#else
    gpio_set_level(BATT_EN_GPIO, 1);
#endif
    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_LOGI(TAG, "VBAT divider enabled (GPIO%d=%d)",
             BATT_EN_GPIO,
#if BATT_EN_ACTIVE_LOW
             0
#else
             1
#endif
    );
#endif
}
#endif

static void batt_init(void)
{
    if (mr_board_has_pmu()) {
        ESP_LOGI(TAG, "Battery via PMU on board %s -> ADC init skipped", mr_board_name());
        return;
    }

#if !defined(BATT_ADC_GPIO) || (BATT_ADC_GPIO < 0)
    ESP_LOGI(TAG, "Battery ADC disabled for board %s", mr_board_name());
    return;
#endif

    batt_enable_hw();

#ifdef BATT_EN_GPIO
    gpio_reset_pin((gpio_num_t)BATT_EN_GPIO);
    gpio_set_direction((gpio_num_t)BATT_EN_GPIO, GPIO_MODE_OUTPUT);

#if BATT_EN_ACTIVE_LOW
    gpio_set_level((gpio_num_t)BATT_EN_GPIO, 1); // OFF
#else
    gpio_set_level((gpio_num_t)BATT_EN_GPIO, 0); // OFF
#endif

    vTaskDelay(pdMS_TO_TICKS(50));
#endif

    int ch = batt_adc_gpio_to_chan((gpio_num_t)BATT_ADC_GPIO);
    if(ch < 0){
        ESP_LOGE(TAG, "Battery ADC: GPIO%d auf S3 nicht unterstützt!", BATT_ADC_GPIO);
        return;
    }

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &g_adc_unit));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc_unit, (adc_channel_t)ch, &chan_cfg));

    g_adc_cali_ok = false;
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if(adc_cali_create_scheme_line_fitting(&cali_cfg, &g_adc_cali) == ESP_OK){
        g_adc_cali_ok = true;
    }
#endif

    g_next_batt_ms = now_ms() + 500;
}
#endif

#ifndef CONFIG_IDF_TARGET_ESP32S3
static void batt_init(void)
{
    static uint32_t dbg_last_ms = 0;
    (void)dbg_last_ms;

    if (mr_board_has_pmu()) {
        ESP_LOGI(TAG, "Battery via PMU on board %s -> ADC init skipped", mr_board_name());
        return;
    }

#if !defined(BATT_ADC_GPIO) || (BATT_ADC_GPIO < 0)
    ESP_LOGI(TAG, "Battery ADC disabled for board %s", mr_board_name());
    return;
#endif

    int ch = batt_adc_gpio_to_chan((gpio_num_t)BATT_ADC_GPIO);
    if(ch < 0){
        ESP_LOGW(TAG,"Battery ADC: GPIO%d not supported in mapping -> disabled", BATT_ADC_GPIO);
        return;
    }

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &g_adc_unit));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_11,     // wide range
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc_unit, (adc_channel_t)ch, &chan_cfg));

    // calibration (optional)
    g_adc_cali_ok=false;
    if(MR_ADC_CALI_MODE){
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        adc_cali_curve_fitting_config_t cali_cfg = {
            .unit_id  = ADC_UNIT_1,
            .chan     = (adc_channel_t)ch,
            .atten    = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT
        };
        if(adc_cali_create_scheme_curve_fitting(&cali_cfg, &g_adc_cali) == ESP_OK){
            g_adc_cali_ok=true;
        }else{
            // ESP_LOGW(TAG,"ADC cali (curve fitting) failed -> using raw fallback");
        }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        adc_cali_line_fitting_config_t cali_cfg = {
            .unit_id  = ADC_UNIT_1,
            .atten    = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT
        };
        if(adc_cali_create_scheme_line_fitting(&cali_cfg, &g_adc_cali) == ESP_OK){
            g_adc_cali_ok=true;
        }else{
            ESP_LOGW(TAG,"ADC cali (line fitting) failed -> using raw fallback");
        }
#else
        ESP_LOGW(TAG,"ADC calibration not supported in this ESP-IDF build -> using raw fallback");
#endif
    }

    g_next_batt_ms = now_ms() + 500;
}

#endif

static uint32_t batt_mv_to_pct(uint32_t mv)
{
    if(mv <= BATT_EMPTY_MV) return 0;
    if(mv >= BATT_FULL_MV)  return 100;
    uint32_t span = (uint32_t)(BATT_FULL_MV - BATT_EMPTY_MV);
    return (uint32_t)(((mv - BATT_EMPTY_MV) * 100U) / span);
}

#if MR_BATT_ENABLE
static void batt_force_read_once(void)
{
#if !defined(BATT_ADC_GPIO) || (BATT_ADC_GPIO < 0)
    return;
#endif
    if(!g_adc_unit) return;

#ifdef BATT_EN_GPIO
#if BATT_EN_ACTIVE_LOW
    gpio_set_level(BATT_EN_GPIO, 0);
#else
    gpio_set_level(BATT_EN_GPIO, 1);
#endif
    vTaskDelay(pdMS_TO_TICKS(20));
#endif

    adc_unit_t unit;
    adc_channel_t ch;
    if(adc_oneshot_io_to_channel((gpio_num_t)BATT_ADC_GPIO, &unit, &ch) != ESP_OK){
        goto done;
    }

    int raw_dummy = 0;
    (void)adc_oneshot_read(g_adc_unit, ch, &raw_dummy);

    int raw = 0;
    if(adc_oneshot_read(g_adc_unit, ch, &raw) != ESP_OK){
        goto done;
    }

    int mv_adc = 0;
    if(g_adc_cali_ok && g_adc_cali){
        if(adc_cali_raw_to_voltage(g_adc_cali, raw, &mv_adc) != ESP_OK){
#if (MR_BOARD_PRESET == MR_BOARD_HELTEC_V3)
            mv_adc = (raw * 3300) / 4095;
#else
            mv_adc = 0;
#endif
        }
    }else{
        mv_adc = (raw * 3300) / 4095;
    }

    float scale = (BATT_DIV_RTOP_OHMS + BATT_DIV_RBOT_OHMS) / BATT_DIV_RBOT_OHMS;
    uint32_t vbat = (uint32_t)((float)mv_adc * scale * BATT_CAL_FACTOR);

    /*ESP_LOGW(TAG, "BATT DBG raw=%d mv_adc=%d scale=%.3f cal=%.3f vbat=%lu",
             raw,
             mv_adc,
             (double)scale,
             (double)BATT_CAL_FACTOR,
             (unsigned long)vbat);
    */
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_batt_mv  = vbat;
    g_batt_pct = batt_mv_to_pct(vbat);
    xSemaphoreGive(g_mutex);

done:
#ifdef BATT_EN_GPIO
#if BATT_EN_ACTIVE_LOW
    gpio_set_level(BATT_EN_GPIO, 1);
#else
    gpio_set_level(BATT_EN_GPIO, 0);
#endif
#endif
}
#endif

static void batt_poll(void)
{
#if MR_BATT_ENABLE
    const mr_board_info_t *b = mr_board_get();
    if (!b) {
        g_batt_mv = 0;
        g_batt_pct = 0;
        return;
    }

    uint32_t t = now_ms();
    if (t < g_next_batt_ms) {
        return;
    }
    g_next_batt_ms = t + BATT_MEASURE_INTERVAL_MS;

    // =====================================================================
    // ======================= PMU BOARDS (T-BEAM) ==========================
    // =====================================================================
    if (b->has_pmu) {
        if (!mr_pmu_is_available()) {
            g_batt_mv = 0;
            g_batt_pct = 0;
            return;
        }

        mr_pmu_status_t st;
        if (mr_pmu_poll() == ESP_OK && mr_pmu_get_status(&st)) {
            if (!st.battery_present || st.battery_mv < 1000) {
    // kein Akku erkannt
    g_batt_mv  = 0;
    g_batt_pct = 0;

    //ESP_LOGI("BATT", "kein Akku");
    } else {
    g_batt_mv  = st.battery_mv;
    g_batt_pct = st.battery_percent;

    }
    } else {
        g_batt_mv = 0;
        g_batt_pct = 0;
    }
    return;
}

    // =====================================================================
    // ========================== ADC DISABLED ==============================
    // =====================================================================
#if !defined(BATT_ADC_GPIO) || (BATT_ADC_GPIO < 0)
    g_batt_mv = 0;
    g_batt_pct = 0;
    return;
#endif

    if (!g_adc_unit) {
        g_batt_mv = 0;
        g_batt_pct = 0;
        return;
    }

    // =====================================================================
    // ======================= HELTEC SPECIAL PATH ==========================
    // =====================================================================
#if (MR_BOARD_PRESET == MR_BOARD_HELTEC_V3)
    batt_force_read_once();
    return;
#endif

    // =====================================================================
    // ======================= ADC BOARDS (LILYGO etc.) =====================
    // =====================================================================
#ifdef BATT_EN_GPIO
#if BATT_EN_ACTIVE_LOW
    gpio_set_level(BATT_EN_GPIO, 0);
#else
    gpio_set_level(BATT_EN_GPIO, 1);
#endif
    vTaskDelay(pdMS_TO_TICKS(20));
#endif

    adc_unit_t unit;
    adc_channel_t ch;
    esp_err_t e = adc_oneshot_io_to_channel((gpio_num_t)BATT_ADC_GPIO, &unit, &ch);
    if (e != ESP_OK) {
#ifdef BATT_EN_GPIO
#if BATT_EN_ACTIVE_LOW
        gpio_set_level(BATT_EN_GPIO, 1);
#else
        gpio_set_level(BATT_EN_GPIO, 0);
#endif
#endif
        g_batt_mv = 0;
        g_batt_pct = 0;
        return;
    }

    int raw_dummy = 0;
    (void)adc_oneshot_read(g_adc_unit, ch, &raw_dummy);

    int raw = 0;
    if (adc_oneshot_read(g_adc_unit, ch, &raw) != ESP_OK) {
#ifdef BATT_EN_GPIO
#if BATT_EN_ACTIVE_LOW
        gpio_set_level(BATT_EN_GPIO, 1);
#else
        gpio_set_level(BATT_EN_GPIO, 0);
#endif
#endif
        g_batt_mv = 0;
        g_batt_pct = 0;
        return;
    }

#ifdef BATT_EN_GPIO
#if BATT_EN_ACTIVE_LOW
    gpio_set_level(BATT_EN_GPIO, 1);
#else
    gpio_set_level(BATT_EN_GPIO, 0);
#endif
#endif

    int mv_adc = 0;
    if (g_adc_cali_ok && g_adc_cali) {
        if (adc_cali_raw_to_voltage(g_adc_cali, raw, &mv_adc) != ESP_OK) {
            mv_adc = 0;
        }
    } else {
        mv_adc = (raw * 3300) / 4095;
    }

    float scale = (BATT_DIV_RTOP_OHMS + BATT_DIV_RBOT_OHMS) / BATT_DIV_RBOT_OHMS;
    uint32_t vbat = (uint32_t)((float)mv_adc * scale * BATT_CAL_FACTOR);

    g_batt_mv = vbat;

    if (g_batt_mv <= BATT_EMPTY_MV) {
        g_batt_pct = 0;
    } else if (g_batt_mv >= BATT_FULL_MV) {
        g_batt_pct = 100;
    } else {
        g_batt_pct = (uint32_t)(
            (g_batt_mv - BATT_EMPTY_MV) * 100U /
            (BATT_FULL_MV - BATT_EMPTY_MV)
        );
    }

    /*ESP_LOGI("BATT", "ADC: raw=%d mv_adc=%d vbat=%lu => %lu%%",
             raw,
             mv_adc,
             (unsigned long)g_batt_mv,
             (unsigned long)g_batt_pct);*/
#endif
}

// ============================================================================
// =============================== Pending ACK ================================
// ============================================================================
static void pending_clear_slot_locked(int i){ pend[i].used=false; }

static int pending_alloc_slot_locked(void)
{
    for(int i=0;i<MAX_PENDING_ACK;i++){
        if(!pend[i].used) return i;
    }
    int oldest=0;
    uint32_t od=pend[0].deadline_ms;
    for(int i=1;i<MAX_PENDING_ACK;i++){
        if(!pend[i].used) return i;
        if(pend[i].deadline_ms < od){ oldest=i; od=pend[i].deadline_ms; }
    }
    return oldest;
}

static void pending_add_locked(uint16_t msg_id,
                               const char expect_from[8],
                               const uint8_t *frame,
                               uint16_t frame_len)
{
    int i = pending_alloc_slot_locked();
    pend[i].used=true;
    pend[i].msg_id=msg_id;
    memcpy(pend[i].expect_from, expect_from, 8);
    pend[i].deadline_ms = now_ms() + ACK_TIMEOUT_MS;
    pend[i].retries_left = ACK_RETRY_MAX;

    if(frame_len > sizeof(pend[i].frame)) frame_len = sizeof(pend[i].frame);
    memcpy(pend[i].frame, frame, frame_len);
    pend[i].frame_len = frame_len;
}

static bool pending_mark_acked_locked(uint16_t msg_id, const char src[8])
{
    for(int i=0;i<MAX_PENDING_ACK;i++){
        if(!pend[i].used) continue;
        if(pend[i].msg_id != msg_id) continue;
        if(!call7_eq(pend[i].expect_from, src)) continue;
        pending_clear_slot_locked(i);
        return true;
    }
    return false;
}


// ============================================================================
// ============================ WiFi + HTTP ===================================
// ============================================================================
static void wifi_init_once(void)
{
    if(g_wifi_inited) return;

    ESP_ERROR_CHECK(esp_netif_init());
    esp_err_t e = esp_event_loop_create_default();
    if(e != ESP_OK && e != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(e);

    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap = {0};

    strncpy((char*)ap.ap.ssid, g_cfg.wifi_ssid, sizeof(ap.ap.ssid)-1);
    ap.ap.ssid[sizeof(ap.ap.ssid)-1] = 0;
    ap.ap.ssid_len = 0;
    ap.ap.channel = 1;
    ap.ap.max_connection = 8;   // <--- HIER
    ap.ap.ssid_hidden = 0;
    ap.ap.beacon_interval = 100;

    if(strlen(g_cfg.wifi_pass) >= 8){
        strncpy((char*)ap.ap.password, g_cfg.wifi_pass, sizeof(ap.ap.password)-1);
        ap.ap.password[sizeof(ap.ap.password)-1] = 0;
        ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
    }else{
        ap.ap.authmode = WIFI_AUTH_OPEN;
        ap.ap.password[0] = 0;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));

    g_wifi_inited = true;
}

void wifi_start_ap(void)
{
    wifi_init_once();

    esp_err_t err = esp_wifi_start();
    if (err != ESP_OK && err != ESP_ERR_WIFI_CONN && err != ESP_ERR_INVALID_STATE) {
        // ESP_LOGW(TAG, "esp_wifi_start failed: %s", esp_err_to_name(err));
        return;
    }

    wifi_enable_ap_part();
    ESP_LOGI(TAG,"WiFi AP started: %s (http://192.168.4.1)", MR_WIFI_AP_SSID);
}

void wifi_disable_ap_part(void)
{
    if (!g_ap_part_enabled) return;

    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        // ESP_LOGW(TAG, "wifi_disable_ap_part set_mode(STA) failed: %s", esp_err_to_name(err));
        return;
    }

    g_ap_part_enabled = false;
    ESP_LOGI(TAG, "SoftAP really disabled, STA-only mode active");
}

void wifi_enable_ap_part(void)
{
    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (err != ESP_OK) {
        // ESP_LOGW(TAG, "wifi_enable_ap_part set_mode(APSTA) failed: %s", esp_err_to_name(err));
        return;
    }

    wifi_config_t ap = {0};

    strncpy((char*)ap.ap.ssid, g_cfg.wifi_ssid, sizeof(ap.ap.ssid)-1);
    ap.ap.ssid[sizeof(ap.ap.ssid)-1] = 0;
    ap.ap.ssid_len = strlen((char*)ap.ap.ssid);

    strncpy((char*)ap.ap.password, g_cfg.wifi_pass, sizeof(ap.ap.password)-1);
    ap.ap.password[sizeof(ap.ap.password)-1] = 0;

    ap.ap.channel = 1;
    ap.ap.max_connection = 4;
    ap.ap.ssid_hidden = 0;
    ap.ap.beacon_interval = 100;
    ap.ap.authmode = (strlen((char*)ap.ap.password) >= 8) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

    err = esp_wifi_set_config(WIFI_IF_AP, &ap);
    if (err != ESP_OK) {
        // ESP_LOGW(TAG, "wifi_enable_ap_part set_config failed: %s", esp_err_to_name(err));
        return;
    }

    g_ap_part_enabled = true;
    ESP_LOGI(TAG, "SoftAP enabled: %s (http://192.168.4.1)", (char*)ap.ap.ssid);
}

void wifi_stop_ap(void)
{
    wifi_disable_ap_part();
    ESP_LOGI(TAG,"WiFi AP hidden");
}

static bool wifi_sta_cfg_present(void)
{
    if(strlen(g_cfg.wifi_sta_ssid) == 0)
        return false;

    return true;
}

static void http_stop(void)
{
    if(g_http_running && g_http){
        httpd_stop(g_http);
        g_http=NULL;
        g_http_running=false;
        ESP_LOGI(TAG,"HTTP server stopped");
    }
}

static void http_start_if_needed(void); // forward

void set_wifi_enabled(bool en)
{
    g_wifi_enabled = en;

    if (en) {
        /*
         * Grundregel:
         * - AP immer zuerst einschalten, damit das Webinterface erreichbar ist
         * - wenn STA-Credentials vorhanden sind, STA zusätzlich starten
         * - sobald STA oben ist, darf das STA-Modul den AP ausblenden
         *   (hide_ap_when_sta_up = true)
         * - fällt STA später aus, soll das STA-Modul den AP wieder aktivieren
         */
        wifi_start_ap();
        http_start_if_needed();

#if MR_WIFI_STA_ENABLE
        if (wifi_sta_cfg_present()) {
            mr_wifi_sta_cfg_t sta_cfg = {0};

            strncpy(sta_cfg.ssid, g_cfg.wifi_sta_ssid, sizeof(sta_cfg.ssid) - 1);
            sta_cfg.ssid[sizeof(sta_cfg.ssid) - 1] = 0;

            strncpy(sta_cfg.pass, g_cfg.wifi_sta_pass, sizeof(sta_cfg.pass) - 1);
            sta_cfg.pass[sizeof(sta_cfg.pass) - 1] = 0;

            sta_cfg.use_dhcp_hostname = true;
            sta_cfg.retry_max = MR_WIFI_STA_RETRY_MAX;
            sta_cfg.retry_backoff_ms = 2000;

            /*
             * Entscheidend für das gewünschte Verhalten:
             * - STA oben => AP aus
             * - STA weg  => AP wieder an
             *
             * Das muss das STA-Modul anhand dieses Flags steuern.
             */
            sta_cfg.hide_ap_when_sta_up = true;

            ESP_LOGI(TAG, "STA SSID from g_cfg: '%s'", g_cfg.wifi_sta_ssid);
            ESP_LOGI(TAG, "STA PASS LEN from g_cfg: %u",
                     (unsigned)strlen(g_cfg.wifi_sta_pass));

            esp_err_t err = mr_wifi_sta_reconfigure(&sta_cfg);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "STA reconfigure failed: %s", esp_err_to_name(err));
            }
        } else {
            ESP_LOGI(TAG, "No STA credentials configured -> AP only");
        }
#endif
    } else {
#if MR_WIFI_STA_ENABLE
        mr_wifi_sta_stop();
#endif
        http_stop();
        wifi_disable_ap_part();
    }
}

// ============================================================================
// =============================== HTTP helpers ===============================
// ============================================================================

// --- forward declarations for HTTP helpers (must be before api_cfg_* uses them) ---
static esp_err_t http_send_text(httpd_req_t *req, const char *txt);
static bool http_read_body(httpd_req_t *req, char *buf, size_t buf_sz);
static bool form_get(char *body,const char *key,char *out,size_t out_sz);

static void aprs_cfg_from_local_web(aprs_cfg_t *out)
{
    if(!out) return;
    memset(out, 0, sizeof(*out));
    out->enabled = aprs_web_enabled();
    snprintf(out->callsign, sizeof(out->callsign), "%s", aprs_web_callsign());
    snprintf(out->passcode, sizeof(out->passcode), "%s", aprs_web_passcode());
    out->symbol_table = aprs_web_symbol_table();
    out->symbol_code  = aprs_web_symbol_code();
    snprintf(out->comment, sizeof(out->comment), "%s", aprs_web_comment());
    snprintf(out->host, sizeof(out->host), "%s", aprs_web_host());
    out->port = aprs_web_port();
    out->use_static_pos = aprs_web_use_static_pos();
    snprintf(out->latitude, sizeof(out->latitude), "%s", aprs_web_latitude());
    snprintf(out->longitude, sizeof(out->longitude), "%s", aprs_web_longitude());
}

static void aprs_format_lat_for_body(double lat, char *out, size_t out_sz, char *out_hemi)
{
    double alat = (lat < 0.0) ? -lat : lat;
    int deg = (int)alat;
    double min = (alat - (double)deg) * 60.0;
    snprintf(out, out_sz, "%02d%05.2f", deg, min);
    *out_hemi = (lat >= 0.0) ? 'N' : 'S';
}

static void aprs_format_lon_for_body(double lon, char *out, size_t out_sz, char *out_hemi)
{
    double alon = (lon < 0.0) ? -lon : lon;
    int deg = (int)alon;
    double min = (alon - (double)deg) * 60.0;
    snprintf(out, out_sz, "%03d%05.2f", deg, min);
    *out_hemi = (lon >= 0.0) ? 'E' : 'W';
}

static bool aprs_build_body_from_gps_text(const char *gps_txt,
                                          const aprs_cfg_t *cfg,
                                          char *out_body,
                                          size_t out_body_sz)
{
    if(!gps_txt || !cfg || !out_body || out_body_sz < 32) return false;

    double lat = 0.0;
    double lon = 0.0;
    if(sscanf(gps_txt, "GPS %lf %lf", &lat, &lon) != 2){
        return false;
    }

    if(lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0){
        return false;
    }

    char aprs_lat[16];
    char aprs_lon[16];
    char lat_hemi = 'N';
    char lon_hemi = 'E';

    aprs_format_lat_for_body(lat, aprs_lat, sizeof(aprs_lat), &lat_hemi);
    aprs_format_lon_for_body(lon, aprs_lon, sizeof(aprs_lon), &lon_hemi);

    char symbol_table = cfg->symbol_table ? cfg->symbol_table : '/';
    char symbol_code  = cfg->symbol_code  ? cfg->symbol_code  : '>';

    int n = snprintf(out_body, out_body_sz,
                     "!%.7s%c%c%.8s%c%c%s",
                     aprs_lat, lat_hemi,
                     symbol_table,
                     aprs_lon, lon_hemi,
                     symbol_code,
                     cfg->comment);
    return (n > 0 && (size_t)n < out_body_sz);
}

static bool aprs_parse_decimal_coord(const char *s, double min_v, double max_v, double *out)
{
    if(!s || !*s || !out) return false;

    char *end = NULL;
    double v = strtod(s, &end);
    if(end == s) return false;

    while(end && *end){
        if(!isspace((unsigned char)*end)) return false;
        end++;
    }

    if(v < min_v || v > max_v) return false;
    *out = v;
    return true;
}

static bool aprs_build_body_from_static_cfg(const aprs_cfg_t *cfg,
                                            char *out_body,
                                            size_t out_body_sz)
{
    if(!cfg || !out_body || out_body_sz < 32) return false;
    if(!cfg->use_static_pos) return false;

    double lat = 0.0;
    double lon = 0.0;
    if(!aprs_parse_decimal_coord(cfg->latitude, -90.0, 90.0, &lat)) return false;
    if(!aprs_parse_decimal_coord(cfg->longitude, -180.0, 180.0, &lon)) return false;

    char aprs_lat[16];
    char aprs_lon[16];
    char lat_hemi = 'N';
    char lon_hemi = 'E';

    aprs_format_lat_for_body(lat, aprs_lat, sizeof(aprs_lat), &lat_hemi);
    aprs_format_lon_for_body(lon, aprs_lon, sizeof(aprs_lon), &lon_hemi);

    char symbol_table = cfg->symbol_table ? cfg->symbol_table : '/';
    char symbol_code  = cfg->symbol_code  ? cfg->symbol_code  : '>';

    int n = snprintf(out_body, out_body_sz,
                     "!%.7s%c%c%.8s%c%c%s",
                     aprs_lat, lat_hemi,
                     symbol_table,
                     aprs_lon, lon_hemi,
                     symbol_code,
                     cfg->comment);
    return (n > 0 && (size_t)n < out_body_sz);
}

static bool aprs_build_direct_msg(const aprs_cfg_t *cfg,
                                  const char *body,
                                  char *out_msg,
                                  size_t out_msg_sz)
{
    if(!cfg || !body || !out_msg || out_msg_sz < 32) return false;
    if(!cfg->enabled) return false;
    if(cfg->callsign[0] == 0 || cfg->passcode[0] == 0 || cfg->host[0] == 0 || cfg->port == 0) return false;

    int n = snprintf(out_msg, out_msg_sz,
                     "APRSD C=%s P=%s H=%s O=%u B=%s",
                     cfg->callsign,
                     cfg->passcode,
                     cfg->host,
                     (unsigned)cfg->port,
                     body);
    return (n > 0 && (size_t)n < out_msg_sz);
}

static bool aprs_direct_parse_msg(const char *msg,
                                  aprs_cfg_t *out,
                                  char *body,
                                  size_t body_sz)
{
    if(!msg || !out || !body || body_sz < 8) return false;
    if(strncmp(msg, "APRSD ", 6) != 0) return false;

    const char *body_tag = strstr(msg, " B=");
    if(!body_tag) return false;

    size_t head_len = (size_t)(body_tag - msg);
    if(head_len >= 192) return false;

    char head[192];
    memcpy(head, msg, head_len);
    head[head_len] = 0;

    memset(out, 0, sizeof(*out));
    out->enabled = true;

    char c[16] = {0};
    char p[16] = {0};
    char h[64] = {0};
    unsigned port = 0;

    int n = sscanf(head,
                   "APRSD C=%15s P=%15s H=%63s O=%u",
                   c, p, h, &port);
    if(n != 4) return false;
    if(port == 0 || port > 65535) return false;

    snprintf(out->callsign, sizeof(out->callsign), "%s", c);
    snprintf(out->passcode, sizeof(out->passcode), "%s", p);
    snprintf(out->host, sizeof(out->host), "%s", h);
    out->port = (uint16_t)port;

    snprintf(body, body_sz, "%s", body_tag + 3);
    return body[0] != 0;
}


const char *wifi_get_current_url(void)
{
    static char url[64];

    esp_netif_ip_info_t ip;

    // zuerst versuchen STA-IP zu holen
    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");

    if (sta && esp_netif_get_ip_info(sta, &ip) == ESP_OK && ip.ip.addr != 0) {
        snprintf(url, sizeof(url), "http://" IPSTR, IP2STR(&ip.ip));
        return url;
    }

    // sonst AP-IP
    esp_netif_t *ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");

    if (ap && esp_netif_get_ip_info(ap, &ip) == ESP_OK) {
        snprintf(url, sizeof(url), "http://" IPSTR, IP2STR(&ip.ip));
        return url;
    }

    return "http://192.168.4.1";
}

static void delayed_factory_reset_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(800));
    // ESP_LOGW(TAG, "Factory reset requested -> erasing NVS and restarting");

    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());

    esp_restart();
    vTaskDelete(NULL);
}

static esp_err_t api_cfg_erase_post(httpd_req_t *req)
{
    http_send_text(req, "OK factory reset");

    BaseType_t ok = xTaskCreate(
        delayed_factory_reset_task,
        "factory_reset_task",
        3072,
        NULL,
        5,
        NULL
    );

    if(ok != pdPASS){
        ESP_LOGE(TAG, "Failed to create factory reset task");
        return ESP_FAIL;
    }

    return ESP_OK;
}


static void delayed_reset_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(800));
    // ESP_LOGW(TAG, "HTTP reset requested -> restarting now");
    esp_restart();
    vTaskDelete(NULL);
}

static esp_err_t api_reset_post(httpd_req_t *req)
{
    http_send_text(req, "OK reset");

    BaseType_t ok = xTaskCreate(
        delayed_reset_task,
        "reset_task",
        2048,
        NULL,
        5,
        NULL
    );

    if(ok != pdPASS){
        ESP_LOGE(TAG, "Failed to create reset task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t api_cfg_defaults_post(httpd_req_t *req)
{
    (void)req;
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    mr_cfg_defaults(&g_cfg);
    (void)parse_key_hex16(MR_NET_KEY_HEX, g_cfg.net_key); // compile-time default key
    mr_cfg_apply(&g_cfg);
    xSemaphoreGive(g_mutex);
    return http_send_text(req, "OK defaults");
}

static esp_err_t api_cfg_apply_post(httpd_req_t *req)
{
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    mr_cfg_apply(&g_cfg);
    xSemaphoreGive(g_mutex);

    return http_send_text(req, "OK applying");
}
static esp_err_t api_cfg_get(httpd_req_t *req)
{
    static char out[768];
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    mr_cfg_to_json(&g_cfg, out, sizeof(out));
    xSemaphoreGive(g_mutex);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, out, HTTPD_RESP_USE_STRLEN);
}


static esp_err_t api_cfg_post(httpd_req_t *req)
{
    char body[512];
    if(!http_read_body(req, body, sizeof(body))) return http_send_text(req, "ERR body");

    char k[48]={0}, v[160]={0};
    if(!form_get(body,"key",k,sizeof(k))) return http_send_text(req,"ERR missing key");
    if(!form_get(body,"val",v,sizeof(v))) return http_send_text(req,"ERR missing val");

    ESP_LOGI(TAG, "WEB CFG POST key='%s' val_len=%u", k, (unsigned)strlen(v));

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    bool ok = mr_cfg_set_kv(&g_cfg, k, v);
    ESP_LOGI(TAG, "WEB CFG POST result=%d sta_ssid='%s' sta_pass_len=%u",
             ok ? 1 : 0,
             g_cfg.wifi_sta_ssid,
             (unsigned)strlen(g_cfg.wifi_sta_pass));
    xSemaphoreGive(g_mutex);

    return http_send_text(req, ok ? "OK" : "ERR invalid");
}

// optional: /api/cfg/save
static esp_err_t api_cfg_save_post(httpd_req_t *req)
{
    (void)req;
    bool ok=false;
    xSemaphoreTake(g_mutex, portMAX_DELAY);

    ESP_LOGI(TAG, "SAVE sta_ssid='%s' sta_pass_len=%u",
             g_cfg.wifi_sta_ssid,
             (unsigned)strlen(g_cfg.wifi_sta_pass));

    ok = mr_cfg_save_nvs(&g_cfg);
    xSemaphoreGive(g_mutex);
    return http_send_text(req, ok ? "OK saved" : "ERR save");
}

// optional: /api/cfg/load
static esp_err_t api_cfg_load_post(httpd_req_t *req)
{
    (void)req;
    bool ok=false;
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    ok = mr_cfg_load_nvs(&g_cfg);
    if(ok) mr_cfg_apply(&g_cfg);
    xSemaphoreGive(g_mutex);
    return http_send_text(req, ok ? "OK loaded" : "ERR load");
}

static esp_err_t http_send_text(httpd_req_t *req, const char *txt)
{
    httpd_resp_set_type(req,"text/plain");
    return httpd_resp_send(req,txt,HTTPD_RESP_USE_STRLEN);
}
static bool http_read_body(httpd_req_t *req, char *buf, size_t buf_sz)
{
    if(req->content_len<=0 || req->content_len >= (int)buf_sz) return false;
    int r=httpd_req_recv(req,buf,req->content_len);
    if(r<=0) return false;
    buf[r]=0;
    return true;
}
static void url_decode_inplace(char *s)
{
    char *p=s,*o=s;
    while(*p){
        if(*p=='+'){*o++=' ';p++;continue;}
        if(*p=='%' && p[1] && p[2]){
            char hex[3]={p[1],p[2],0};
            *o++=(char)strtoul(hex,NULL,16);
            p+=3; continue;
        }
        *o++=*p++;
    }
    *o=0;
}
static bool form_get(char *body,const char *key,char *out,size_t out_sz)
{
    size_t klen=strlen(key);
    char *p=body;

    while(p && *p){
        char *amp=strchr(p,'&');
        if(amp) *amp=0;

        char *eq=strchr(p,'=');
        if(eq){
            *eq=0;
            char *k=p;
            char *v=eq+1;

            url_decode_inplace(k);
            url_decode_inplace(v);

            if(strlen(k)==klen && strcmp(k,key)==0){
                strncpy(out,v,out_sz-1);
                out[out_sz-1]=0;

                *eq='=';
                if(amp) *amp='&';
                return true;
            }
            *eq='=';
        }

        if(amp){
            *amp='&';
            p=amp+1;
        }else{
            break;
        }
    }
    return false;
}


// ============================================================================
// =============================== Web UI HTML ================================
// ============================================================================
static const char *INDEX_HTML =
"<!doctype html><html><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>MeshRadio Bonus Software (c) 2026 Nerd-Verlag https://nerdverlag.com </title>"
"<style>"
"body{font-family:system-ui;margin:16px;max-width:1200px}"
"button{padding:10px 14px;margin:6px 4px;font-size:16px}"
"input:not([type='checkbox']){padding:10px;margin:6px 0;font-size:16px;width:100%;box-sizing:border-box}"
"input[type='checkbox']{width:16px;height:16px;margin:0;padding:0}"
"select{padding:10px;margin:6px 0;font-size:16px;width:100%;box-sizing:border-box}"
".card{border:1px solid #ddd;border-radius:12px;padding:12px;box-shadow:0 1px 3px rgba(0,0,0,.06);margin:10px 0}"
".muted{color:#666}.err{color:#b00020;font-weight:600}.ok{color:#0b7a26;font-weight:600}"
".grid{display:grid;grid-template-columns:1fr;gap:12px}"
"@media(min-width:980px){.grid{grid-template-columns:1fr 1fr 1fr}}"
"table{width:100%;border-collapse:collapse;font-size:14px}"
"th,td{border-bottom:1px solid #eee;padding:6px 8px;text-align:left;vertical-align:top}"
"th{background:#fafafa}"
".pill{display:inline-block;padding:2px 8px;border-radius:999px;border:1px solid #ddd;font-size:12px}"
".amp{display:inline-block;width:12px;height:12px;border-radius:50%;vertical-align:middle;margin-right:6px;border:1px solid #aaa}"
".cfg-checks{display:grid;grid-template-columns:1fr;row-gap:10px;margin-bottom:14px}"
".cfg-checks label{display:grid;grid-template-columns:20px auto;column-gap:10px;align-items:center;justify-content:start;margin:0;white-space:nowrap}"
".cfg-actions{display:flex;flex-direction:column;gap:10px;margin-top:14px}"
".cfg-actions-row{display:flex;flex-wrap:wrap;gap:10px}"
".cfg-actions button{margin:0}"
".cfg-actions-row button{min-width:140px}"
".cfg-three-wide{display:grid;grid-template-columns:1fr 1fr 1fr;gap:12px;margin-top:10px}"
"@media(max-width:980px){.cfg-three-wide{grid-template-columns:1fr 1fr}}"
"@media(max-width:760px){.cfg-three-wide{grid-template-columns:1fr}}"
".pw-wrap{position:relative}"
".pw-wrap input{padding-right:44px}"
".pw-eye{position:absolute;right:8px;top:50%;transform:translateY(-50%);border:1px solid #ccc;background:#fff;border-radius:6px;padding:4px 8px;font-size:16px;line-height:1;cursor:pointer;margin:0}"
".pw-eye:hover{background:#f5f5f5}"
".cfg-three{display:grid;grid-template-columns:1fr 1fr 1fr;gap:12px}"
"@media(max-width:980px){.cfg-three{grid-template-columns:1fr 1fr}}"
"@media(max-width:760px){.cfg-three{grid-template-columns:1fr}}"
"</style></head><body>"

"<h2>MeshRadio Bonus (c) 2026 Nerd-Verlag https://nerdverlag.com – Roles, Security, Relay, WiFi, Dashboard</h2>"
"<p class='muted'>AP: <code>" MR_WIFI_AP_SSID "</code> • URL: <code>__MR_URL__</code></p>"
"<div class='card'>"
"<div><b><span id='amp' class='amp'></span>UI-Ampel:</b> <span id='ampTxt'>loading…</span> <span class='pill' id='agePill'>?</span></div>"
"<div><b>Status:</b> <span id='st'>loading…</span></div>"
"<div><b>Target:</b> <span id='targetinfo'>loading…</span></div>"
"<div id='err' class='err'></div>"
"<button onclick='refreshAll()'>Refresh</button>"
"</div>"

"<div class='grid'>"
" <div class='card'>"
"  <h3>DATA senden</h3>"
"  <label>Ziel (dst)</label><input id='dst'>"
"  <label>ACK (0/1)</label><input id='ack' value='1'>"
"  <label>Text</label><input id='msg' value='CMD:STATUS?'>"
"  <label>Quick Command</label>"
"  <select id='cmdsel' onchange='applyCmdPreset()'>"
"    <option value=''>-- Select Command --</option>"
"    <option value='CMD:STATUS?'>STATUS</option>"
"    <option value='CMD:DISPLAY ON'>DISPLAY ON</option>"
"    <option value='CMD:DISPLAY OFF'>DISPLAY OFF</option>"
"    <option value='CMD:WIFI ON'>WIFI ON</option>"
"    <option value='CMD:WIFI OFF'>WIFI OFF</option>"
"    <option value='CMD:GPS'>GPS</option>"
"    <option value='CMD:BATT'>BATT</option>"
"    <option value='CMD:VERSION'>VERSION</option>"
"    <option value='CMD:REBOOT'>REBOOT</option>"
"    <option value='CMD:RELAY ON'>RELAY ON</option>"
"    <option value='CMD:RELAY OFF'>RELAY OFF</option>"
"    <option value='CMD:RELAY TOGGLE'>RELAY TOGGLE</option>"
"    <option value='CMD:CRYPTO ON'>CRYPTO ON</option>"
"    <option value='CMD:CRYPTO OFF'>CRYPTO OFF</option>"
"  </select>"
"  <button onclick='send()'>SEND</button>"
"  <div id='m' class='muted'></div>"
" </div>"

" <div class='card'>"
"  <h3>Node Role</h3>"
"  <div><b>Mode:</b> <span id='nm' class='ok'>?</span></div>"
"  <button onclick='setRole(0)'>RELAY</button>"
"  <button onclick='setRole(1)'>EDGE</button>"
"  <button onclick='setRole(2)'>SENSOR</button>"
" </div>"

" <div class='card'>"
"  <h3>Security</h3>"
"  <div><b>Crypto:</b> <span id='cst' class='ok'>?</span></div>"
"  <button onclick='setCrypto(1)'>Crypto ON</button>"
"  <button onclick='setCrypto(0)'>Crypto OFF</button>"
" </div>"

" <div class='card'>"
"  <h3>WiFi / Web UI</h3>"
"  <button onclick='setWiFi(1)'>WiFi ON</button>"
"  <button onclick='setWiFi(0)'>WiFi OFF</button>"
" </div>"
" <div class='card'>"
"  <h3>Relay Output</h3>"
"  <button onclick='setRelay(1)'>Relay ON</button>"
"  <button onclick='setRelay(0)'>Relay OFF</button>"
"  <button onclick='toggleRelay()'>Relay TOGGLE</button>"
" </div>"
" __APRS_BUTTON__ "
"</div>"

"<div class='card'><h3>Last RX</h3><div id='lastrx'>loading…</div></div>"
"<div class='card'><h3>Security Stats</h3><pre id='secstats'>loading…</pre></div>"
"<div class='card'><h3>Counters</h3><pre id='counters'>loading…</pre></div>"
"<div class='card'><h3>Neighbors</h3><div id='neigh'>loading…</div></div>"
"<div class='card'><h3>Routes</h3><div id='routes'>loading…</div></div>"

"<div class='card'>"
"<div style='display:flex;justify-content:space-between;align-items:center'>"
"  <h3 style='margin:0'>Configuration</h3>"
"  <a href='https://nerdverlag.com/?page_id=706' target='_blank'>"
"    <button style='padding:8px 20px;font-size:14px;font-weight:bold;color:#fff;background:#d00000;border:1px solid #900;border-radius:6px'>HELP</button>"
"  </a>"
"</div>"
"  <div class='grid'>"
"    <div>"
"      <label>Callsign</label><input id='cfg_callsign'>"
"      <label>Relay Callsign</label><input id='cfg_relay_callsign'>"
"      <label>Node Mode</label>"
"      <select id='cfg_node_mode'>"
"        <option value='0'>RELAY</option>"
"        <option value='1'>EDGE</option>"
"        <option value='2'>SENSOR</option>"
"      </select>"
"      <label>RF Frequency (Hz)</label><input id='cfg_rf_hz'>"
"      <label>TX Power (dBm)</label><input id='cfg_tx_dbm'>"
"      <label>Beacon Interval (ms)</label><input id='cfg_beacon_ms'>"
"      <label>Holddown (ms)</label><input id='cfg_holddown_ms'>"
"    </div>"

"    <div>"
"      <label>WiFi AP SSID</label><input id='cfg_ssid'>"
"      <label>WiFi AP Password</label>"
"      <div class='pw-wrap'><input id='cfg_pass' type='password'><button type='button' class='pw-eye' onclick='togglePwBtn(\"cfg_pass\", this)'>👁</button></div>""      <label>Router STA SSID</label><input id='cfg_sta_ssid'>"
"      <label>Router STA Password</label>"
"      <div class='pw-wrap'><input id='cfg_sta_pass' type='password'><button type='button' class='pw-eye' onclick='togglePwBtn(\"cfg_sta_pass\", this)'>👁</button></div>"
"      <label>Net ID</label><input id='cfg_net_id'>"
"      <label>Net Key (32 hex)</label><input id='cfg_netkey'>"
"      <div class='cfg-three'>"
"        <div><label>RouteAdv TopN</label><input id='cfg_routeadv_topn'></div>"
"        <div><label>Relay GPIO</label><input id='cfg_relay_gpio'></div>"
"        <div></div>"
"      </div>"
"    </div>"

"    <div>"
"      <div class='cfg-checks'>"
"        <label><input type='checkbox' id='cfg_wifi'><span>WiFi enabled</span></label>"
"        <label><input type='checkbox' id='cfg_crypto'><span>Crypto enabled</span></label>"
"        <label><input type='checkbox' id='cfg_beacon'><span>Beacon enabled</span></label>"
"        <label><input type='checkbox' id='cfg_routeadv'><span>RouteAdv enabled</span></label>"
"        <label><input type='checkbox' id='cfg_cad'><span>CAD enabled</span></label>"
"        <label><input type='checkbox' id='cfg_powersave'><span>PowerSave enabled</span></label>"
"        <label><input type='checkbox' id='cfg_bme280'> BME280 enabled</label>"
"        <label><input type='checkbox' id='cfg_display'><span>Display enabled</span></label><br>"
"      </div>"
"      <div class='cfg-actions'>"
"        <div class='cfg-actions-row'>"
"          <button onclick='cfgApplyForm()'>Apply Form</button>"
"          <button onclick='cfgSave()'>Save NVS</button>"
"        </div>"
"        <div class='cfg-actions-row'>"
"          <button onclick='cfgLoad()'>Load NVS</button>"
"          <button onclick='cfgDefaults()'>Defaults</button>"
"        </div>"
"        <div class='cfg-actions-row'>"
"          <button onclick='cfgRefresh()'>Reload Config</button>"
"          <button style='background:#b00020;color:#fff;font-weight:700;box-shadow:0 2px 8px rgba(0,0,0,.18)' onclick='doReset()'>Reset ESP</button>"
"        </div>"
"        <div class='cfg-actions-row'>"
"          <button style='background:#7a0000;color:#fff;font-weight:700;box-shadow:0 2px 8px rgba(0,0,0,.18)' onclick='cfgErase()'>Factory Reset</button>"
"        </div>"
"      </div>"
"      <div id='cfgmsg' class='muted' style='margin-top:10px'></div>"
"    </div>"
"  </div>"
"  <div class='cfg-three-wide'>"
"    <div><label>RouteAdv Delta</label><input id='cfg_routeadv_delta'></div>"
"    <div><label>Sensor Wake (ms)</label><input id='cfg_sensor_wake_ms'></div>"
"    <div><label>Sensor RX Window (ms)</label><input id='cfg_sensor_rxwin_ms'></div>"
"  </div>"
"</div>"

"<script>"
"async function post(url,body){"
"  let r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:body||''});"
"  return await r.text();"
"}"

"function esc(s){"
"  return (s||'').replaceAll('&','&amp;').replaceAll('<','&lt;').replaceAll('>','&gt;');"
"}"

"function togglePwBtn(inputId, btn){"
"  let inp=document.getElementById(inputId);"
"  if(!inp) return;"
"  let show = (inp.type === 'password');"
"  inp.type = show ? 'text' : 'password';"
"  if(btn) btn.textContent = show ? '🙈' : '👁';"
"}"

"function setAmp(color,txt,age){"
"  let a=document.getElementById('amp');"
"  a.style.background=color;"
"  document.getElementById('ampTxt').textContent=txt;"
"  document.getElementById('agePill').textContent=age;"
"}"

"function renderTable(cols, rows){"
"  let h='<table><thead><tr>' + cols.map(c=>'<th>'+esc(c)+'</th>').join('') + '</tr></thead><tbody>';"
"  for(let r of rows){"
"    h+='<tr>' + r.map(v=>'<td>'+esc(String(v))+'</td>').join('') + '</tr>';"
"  }"
"  h+='</tbody></table>';"
"  return h;"
"}"

"function applyCmdPreset(){"
"  let s=document.getElementById('cmdsel');"
"  let m=document.getElementById('msg');"
"  if(!s || !m) return;"
"  if(s.value) m.value=s.value;"
"}"

"async function send(){"
"  let dst=document.getElementById('dst').value;"
"  let msg=document.getElementById('msg').value;"
"  let ack=document.getElementById('ack').value;"
"  localStorage.setItem('mr_last_dst', dst);"
"  let t=await post('/api/send','dst='+encodeURIComponent(dst)+'&msg='+encodeURIComponent(msg)+'&ack='+encodeURIComponent(ack));"
"  document.getElementById('m').textContent=t;"
"  refreshAll();"
"}"

"async function setCrypto(v){"
"  let t=await post('/api/crypto','enable='+encodeURIComponent(v));"
"  document.getElementById('m').textContent=t;"
"  refreshAll();"
"}"

"async function setRole(m){"
"  let t=await post('/api/role','mode='+encodeURIComponent(m));"
"  document.getElementById('m').textContent=t;"
"  refreshAll();"
"}"

"async function setWiFi(v){"
"  let t=await post('/api/wifi','enable='+encodeURIComponent(v));"
"  document.getElementById('m').textContent=t;"
"}"

"async function setRelay(v){"
"  let t=await post('/api/relay','enable='+encodeURIComponent(v));"
"  document.getElementById('m').textContent=t;"
"  refreshAll();"
"}"

"async function toggleRelay(){"
"  let t=await post('/api/relay/toggle','');"
"  document.getElementById('m').textContent=t;"
"  refreshAll();"
"}"
"async function cfgPost(key,val){"
"  return await post('/api/cfg','key='+encodeURIComponent(key)+'&val='+encodeURIComponent(val));"
"}"

"function cfgSetMsg(t){"
"  let el=document.getElementById('cfgmsg');"
"  if(el) el.textContent=t;"
"}"

"async function cfgRefresh(){"
"  try{"
"    let resp = await fetch('/api/cfg');"
"    let txt = await resp.text();"
"    if(!txt.startsWith('{')) throw new Error(txt);"
"    let c = JSON.parse(txt);"
"    document.getElementById('cfg_callsign').value=c.callsign||'';"
"    document.getElementById('cfg_relay_callsign').value=c.relay_callsign||'';"
"    document.getElementById('cfg_node_mode').value=String(c.node_mode);"
"    document.getElementById('cfg_rf_hz').value=c.rf_hz;"
"    document.getElementById('cfg_tx_dbm').value=c.tx_dbm;"
"    document.getElementById('cfg_beacon_ms').value=c.beacon_ms;"
"    document.getElementById('cfg_holddown_ms').value=c.holddown_ms;"
"    document.getElementById('cfg_ssid').value=c.ssid||'';"
"    document.getElementById('cfg_sta_ssid').value=c.sta_ssid || '';"
"    document.getElementById('cfg_net_id').value=c.net_id;"
"    document.getElementById('cfg_netkey').value='';"
"    document.getElementById('cfg_relay_gpio').value=c.relay_gpio;"
"    document.getElementById('cfg_routeadv_topn').value=c.routeadv_topn;"
"    document.getElementById('cfg_routeadv_delta').value=c.routeadv_delta;"
"    document.getElementById('cfg_sensor_wake_ms').value=c.sensor_wake_ms;"
"    document.getElementById('cfg_sensor_rxwin_ms').value=c.sensor_rxwin_ms;"
"    document.getElementById('cfg_wifi').checked=!!c.wifi;"
"    document.getElementById('cfg_crypto').checked=!!c.crypto;"
"    document.getElementById('cfg_beacon').checked=!!c.beacon;"
"    document.getElementById('cfg_routeadv').checked=!!c.routeadv;"
"    document.getElementById('cfg_cad').checked=!!c.cad;"
"    document.getElementById('cfg_powersave').checked=!!c.powersave;"
"    document.getElementById('cfg_bme280').checked=!!c.bme280;"
"    document.getElementById('cfg_display').checked=!!c.display;"
"    cfgSetMsg('Config loaded');"
"  }catch(e){"
"    cfgSetMsg('Config load error: '+e);"
"  }"
"}"

"async function cfgApplyForm(){"
"  try{"
"    let ops=["
"      ['callsign', document.getElementById('cfg_callsign').value],"
"      ['relay_callsign', document.getElementById('cfg_relay_callsign').value],"
"      ['node_mode', document.getElementById('cfg_node_mode').value],"
"      ['rf_hz', document.getElementById('cfg_rf_hz').value],"
"      ['tx_dbm', document.getElementById('cfg_tx_dbm').value],"
"      ['beacon_ms', document.getElementById('cfg_beacon_ms').value],"
"      ['holddown_ms', document.getElementById('cfg_holddown_ms').value],"
"      ['ssid', document.getElementById('cfg_ssid').value],"
"      ['sta_ssid', document.getElementById('cfg_sta_ssid').value],"
"      ['netid', document.getElementById('cfg_net_id').value],"
"      ['relay_gpio', document.getElementById('cfg_relay_gpio').value],"
"      ['routeadv_topn', document.getElementById('cfg_routeadv_topn').value],"
"      ['routeadv_delta', document.getElementById('cfg_routeadv_delta').value],"
"      ['sensor_wake_ms', document.getElementById('cfg_sensor_wake_ms').value],"
"      ['sensor_rxwin_ms', document.getElementById('cfg_sensor_rxwin_ms').value],"
"      ['wifi', document.getElementById('cfg_wifi').checked ? '1' : '0'],"
"      ['crypto', document.getElementById('cfg_crypto').checked ? '1' : '0'],"
"      ['beacon', document.getElementById('cfg_beacon').checked ? '1' : '0'],"
"      ['routeadv', document.getElementById('cfg_routeadv').checked ? '1' : '0'],"
"      ['cad', document.getElementById('cfg_cad').checked ? '1' : '0'],"
"      ['powersave', document.getElementById('cfg_powersave').checked ? '1' : '0'],"
"      ['bme280', document.getElementById('cfg_bme280').checked ? '1':'0'],"
"      ['display', document.getElementById('cfg_display').checked ? '1':'0'],"
"    ];"

"    let pass=document.getElementById('cfg_pass').value;"
"    let sta_pass=document.getElementById('cfg_sta_pass').value;"
"    let netkey=document.getElementById('cfg_netkey').value;"
"    if(pass.length>0) ops.push(['pass', pass]);"
"    if(sta_pass.length>0) ops.push(['sta_pass', sta_pass]);"
"    if(netkey.length>0) ops.push(['netkey', netkey]);"

"    for(let kv of ops){"
"      let r=await cfgPost(kv[0], kv[1]);"
"      if(!String(r).startsWith('OK')){"
"        cfgSetMsg('Error at '+kv[0]+': '+r);"
"        return;"
"      }"
"    }"

"    let ra=await post('/api/cfg/apply','');"
"    cfgSetMsg(ra);"
"    cfgSetMsg(ra + ' - Konfiguration übernommen (Reboot erforderlich für WiFi).');"
"    await refreshAll();"
"  }catch(e){"
"    cfgSetMsg('Apply error: '+e);"
"  }"
"}"
"async function cfgSave(){"
"  let r=await post('/api/cfg/save','');"
"  cfgSetMsg(r);"
"  if(String(r).startsWith('OK')){"
"    document.getElementById('cfg_pass').value='';"
"    document.getElementById('cfg_sta_pass').value='';"
"    document.getElementById('cfg_pass').type='password';"
"    document.getElementById('cfg_sta_pass').type='password';"
"    await cfgRefresh();"
"  }"
"}"

"async function cfgLoad(){"
"  let r=await post('/api/cfg/load','');"
"  cfgSetMsg(r);"
"  await cfgRefresh();"
"  await refreshAll();"
"}"

"async function cfgErase(){"
"  if(!confirm('Wirklich Factory Reset? NVS wird gelöscht und der ESP startet neu.')) return;"
"  try{"
"    let r = await post('/api/cfg/erase','');"
"    cfgSetMsg(r + ' - Gerät startet neu...');"
"  }catch(e){"
"    cfgSetMsg('Factory reset error: ' + e);"
"  }"
"}"

"async function cfgDefaults(){"
"  let r=await post('/api/cfg/defaults','');"
"  cfgSetMsg(r);"
"  await cfgRefresh();"
"  await refreshAll();"
"}"

"async function doReset(){"
"  if(!confirm('ESP wirklich neu starten?')) return;"
"  try{"
"    let r = await post('/api/reset','');"
"    cfgSetMsg(r + ' - Gerät startet neu...');"
"  }catch(e){"
"    cfgSetMsg('Reset error: ' + e);"
"  }"
"}"

"async function refreshAll(){"
"  document.getElementById('err').textContent='';"
"  try{"
"    let j=JSON.parse(await (await fetch('/api/status')).text());"
"    document.getElementById('st').textContent='call='+j.call+' • mode='+j.node_mode_str+' • wifi='+j.wifi+' • relay='+j.relay+' • crypto='+j.crypto_enable+' • batt='+j.batt;"
"    document.getElementById('targetinfo').textContent=(j.board||'?') + ' / ' + (j.chip||'?') + ' / Firmware: V' + (j.fw||'?') + ' / Protokoll: v' + String(j.proto ?? '?');"
"    document.getElementById('nm').textContent=j.node_mode_str;"
"    document.getElementById('cst').textContent=(j.crypto_enable==1)?'ON':'OFF';"
"    document.getElementById('cst').className=(j.crypto_enable==1)?'ok':'err';"
"    document.getElementById('secstats').textContent=j.secstats;"
"    document.getElementById('counters').textContent=j.counters;"

"    let lj=JSON.parse(await (await fetch('/api/lastrx')).text());"
"    let age=lj.age_ms;"
"    if(age<0){"
"      document.getElementById('lastrx').innerHTML='<span class=muted>No RX yet</span>';"
"      setAmp('#bbb','NO RX','—');"
"    }else{"
"      document.getElementById('lastrx').innerHTML='<b>from</b> <code>'+esc(lj.from)+'</code><br><b>text</b> '+esc(lj.text)+'<br><span class=muted>age_ms='+age+'</span>';"
"      if(age <= 6000) setAmp('#19a34a','OK (fresh)',age+' ms');"
"      else if(age <= 20000) setAmp('#f59e0b','STALE',age+' ms');"
"      else setAmp('#dc2626','OLD',age+' ms');"
"    }"

"    let nj=JSON.parse(await (await fetch('/api/neighbors')).text());"
"    let nrows=nj.map(x=>[x.call, x.rssi, x.age_ms, x.tx_attempts, x.ack_ok, x.etx_x100]);"
"    document.getElementById('neigh').innerHTML = nrows.length ? renderTable(['call','rssi','age_ms','tx','ack','etx_x100'], nrows) : '<span class=muted>none</span>';"

"    let rj=JSON.parse(await (await fetch('/api/routes')).text());"
"    let rrows=rj.map(x=>[x.dst, x.next, x.etx_x100, x.seq, x.age_ms, x.hold_ms]);"
"    document.getElementById('routes').innerHTML = rrows.length ? renderTable(['dst','next','etx_x100','seq','age_ms','hold_ms'], rrows) : '<span class=muted>none</span>';"
"  }catch(e){"
"    document.getElementById('st').textContent='ERROR';"
"    document.getElementById('err').textContent='Dashboard error: '+e;"
"    setAmp('#dc2626','UI ERROR','—');"
"  }"
"}"
"let lastDst = localStorage.getItem('mr_last_dst');"
"if(lastDst) document.getElementById('dst').value = lastDst;"
"else document.getElementById('dst').value = 'DJ2RF';"
"cfgRefresh();"
"refreshAll();"
"setInterval(refreshAll, 3000);"
"</script></body></html>";

static esp_err_t index_get(httpd_req_t *req)
{
    static char html[28672];
    const char *needle_url = "__MR_URL__";
    const char *needle_btn = "__APRS_BUTTON__";
    const char *url = wifi_get_current_url();
    const char *btn = "";
    const mr_board_info_t *b = mr_board_get();

    if(b){
        btn = "<div class='card'><h3>APRS</h3><button onclick=\"location.href='/aprs'\">APRS Config</button></div>";
    }

    httpd_resp_set_type(req, "text/html");

    const char *p1 = strstr(INDEX_HTML, needle_url);
    if(!p1){
        return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
    }

    size_t a_len = (size_t)(p1 - INDEX_HTML);
    const char *after_url = p1 + strlen(needle_url);
    const char *p2 = strstr(after_url, needle_btn);
    if(!p2){
        return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
    }

    size_t b_len = (size_t)(p2 - after_url);
    const char *after_btn = p2 + strlen(needle_btn);
    size_t c_len = strlen(after_btn);

    size_t need = a_len + strlen(url) + b_len + strlen(btn) + c_len + 1;
    if(need > sizeof(html)){
        return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
    }

    char *w = html;
    memcpy(w, INDEX_HTML, a_len); w += a_len;
    memcpy(w, url, strlen(url)); w += strlen(url);
    memcpy(w, after_url, b_len); w += b_len;
    memcpy(w, btn, strlen(btn)); w += strlen(btn);
    memcpy(w, after_btn, c_len); w += c_len;
    *w = 0;

    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

// ============================================================================
// ============================ API: /api/status ==============================
// ============================================================================
static esp_err_t api_status_get(httpd_req_t *req)
{
    static char out[3072];
    size_t off = 0;

#if MR_BATT_ENABLE
    batt_poll();
#endif

    char counters[512];
    char secstats[256];
    uint32_t batt_mv = 0, batt_pct = 0;
    
    xSemaphoreTake(g_mutex, portMAX_DELAY);

    snprintf(counters, sizeof(counters),
        "NODE_MODE: %s (%d)\n"
        "TX: beacon=%" PRIu32 " adv=%" PRIu32 " ack=%" PRIu32 " data=%" PRIu32 "\n"
        "DEFER: beacon=%" PRIu32 " adv=%" PRIu32 " ack=%" PRIu32 " data=%" PRIu32 "\n"
        "DROP: adv=%" PRIu32 " data=%" PRIu32 "\n"
        "DISPLAY: %s  WIFI: %s  HTTP: %s\n",
        node_mode_str(g_node_mode), (int)g_node_mode,
        c_tx_beacon, c_tx_routeadv, c_tx_ack, c_tx_data,
        c_defer_beacon, c_defer_routeadv, c_defer_ack, c_defer_data,
        c_drop_routeadv, c_drop_data,
        g_display_enabled ? "ON" : "OFF",
        g_wifi_enabled ? "ON" : "OFF",
        g_http_running ? "ON" : "OFF"
    );

    snprintf(secstats, sizeof(secstats),
        "crypto_enable=%u\n"
        "decrypt_ok=%" PRIu32 "\n"
        "decrypt_fail=%" PRIu32 "\n"
        "mac_fail=%" PRIu32 "\n"
        "replay_drop=%" PRIu32 "\n",
        g_crypto_enable ? 1 : 0,
        sec_decrypt_ok, sec_decrypt_fail, sec_mac_fail, sec_replay_drop
    );

#if MR_BATT_ENABLE
    batt_mv = g_batt_mv;
    batt_pct = g_batt_pct;
#endif

    xSemaphoreGive(g_mutex);

    char batt_str[48];
    snprintf(batt_str, sizeof(batt_str), "%" PRIu32 "mV/%" PRIu32 "%%", batt_mv, batt_pct);
    //P_LOGI("BATT", "batt_mv=%" PRIu32 " batt_pct=%" PRIu32, batt_mv, batt_pct);

    off += snprintf(out+off, sizeof(out)-off,
        "{"
        "\"call\":\"%s\","
        "\"crypto_enable\":%u,"
        "\"node_mode\":%d,"
        "\"node_mode_str\":\"%s\","
        "\"board\":\"%s\","
        "\"chip\":\"%s\","
        "\"fw\":\"%s\","
        "\"proto\":%u,"
        "\"display\":%u,"
        "\"display_cfg\":%u,"
        "\"wifi\":%u,"
        "\"relay\":%u,"
        "\"batt\":\"%s\","
        "\"secstats\":\"",
        g_callsign_rt,
        g_crypto_enable ? 1 : 0,
        (int)g_node_mode,
        node_mode_str(g_node_mode),
        cfg_board_str(),
        cfg_lora_chip_str(),
        esp_app_get_description()->version,
        (unsigned)MR_PROTO_VERSION,
        g_display_enabled ? 1 : 0,
        g_cfg.display_enable ? 1 : 0,
        g_wifi_enabled ? 1 : 0,
#if MR_RELAY_ENABLE
        g_relay_on ? 1 : 0,
#else
        0,
#endif
        batt_str
    );

    for (const char *p = secstats; *p && off < sizeof(out)-8; p++) {
        if (*p == '\n') { out[off++]='\\'; out[off++]='n'; }
        else if (*p == '"') { out[off++]='\\'; out[off++]='"'; }
        else out[off++] = *p;
    }
    out[off++] = '\"';

    off += snprintf(out+off, sizeof(out)-off, ",\"counters\":\"");
    for (const char *p = counters; *p && off < sizeof(out)-8; p++) {
        if (*p == '\n') { out[off++]='\\'; out[off++]='n'; }
        else if (*p == '"') { out[off++]='\\'; out[off++]='"'; }
        else out[off++] = *p;
    }
    out[off++] = '\"';
    out[off++] = '}';

    if (off >= sizeof(out)) out[sizeof(out)-1] = 0;
    else out[off] = 0;

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, out, HTTPD_RESP_USE_STRLEN);
}

// ============================================================================
// ============================ API: /api/lastrx ==============================
// ============================================================================
static esp_err_t api_lastrx_get(httpd_req_t *req)
{
    char out[512];

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    uint32_t t=now_ms();
    uint32_t lr=g_last_rx_ms;
    char from[32]; char text[220];
    json_escape_into(from, sizeof(from), g_last_rx_from);
    json_escape_into(text, sizeof(text), g_last_rx_text);
    xSemaphoreGive(g_mutex);

    int32_t age = (lr==0) ? -1 : (int32_t)(t - lr);

    snprintf(out, sizeof(out),
        "{"
        "\"from\":\"%s\","
        "\"text\":\"%s\","
        "\"age_ms\":%d"
        "}",
        from, text, (int)age
    );

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, out, HTTPD_RESP_USE_STRLEN);
}


// ============================================================================
// ============================ API: /api/neighbors ===========================
// ============================================================================
static esp_err_t api_neighbors_get(httpd_req_t *req)
{
    static char out[1536];
    size_t off=0;

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    uint32_t t=now_ms();

    off += snprintf(out+off, sizeof(out)-off, "[");

    bool first=true;
    for(int i=0;i<MAX_NEIGHBORS;i++){
        if(!neighbors[i].used) continue;

        char call[9]; call7_to_str(call, neighbors[i].call);
        uint32_t age = t - neighbors[i].t_ms;
        uint16_t etx = etx_compute_x100(neighbors[i].tx_attempts, neighbors[i].ack_ok);

        if(!first) off += snprintf(out+off, sizeof(out)-off, ",");
        first=false;

        off += snprintf(out+off, sizeof(out)-off,
            "{"
            "\"call\":\"%s\","
            "\"rssi\":%d,"
            "\"age_ms\":%" PRIu32 ","
            "\"tx_attempts\":%" PRIu32 ","
            "\"ack_ok\":%" PRIu32 ","
            "\"etx_x100\":%u"
            "}",
            call,
            neighbors[i].rssi,
            age,
            neighbors[i].tx_attempts,
            neighbors[i].ack_ok,
            (unsigned)etx
        );

        if(off > sizeof(out)-120) break;
    }

    off += snprintf(out+off, sizeof(out)-off, "]");
    if(off >= sizeof(out)) out[sizeof(out)-1]=0;
    else out[off]=0;

    xSemaphoreGive(g_mutex);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, out, HTTPD_RESP_USE_STRLEN);
}


// ============================================================================
// ============================ API: /api/routes ==============================
// ============================================================================
static esp_err_t api_routes_get(httpd_req_t *req)
{
    static char out[2048];
    size_t off=0;

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    uint32_t t=now_ms();

    off += snprintf(out+off, sizeof(out)-off, "[");
    bool first=true;

    for(int i=0;i<MAX_ROUTES;i++){
        if(!routes[i].used) continue;

        char d[9], n[9];
        call7_to_str(d, routes[i].dst);
        call7_to_str(n, routes[i].next);

        uint32_t age = t - routes[i].t_ms;
        uint32_t hold = (t < routes[i].hold_until_ms) ? (routes[i].hold_until_ms - t) : 0;

        if(!first) off += snprintf(out+off, sizeof(out)-off, ",");
        first=false;

        off += snprintf(out+off, sizeof(out)-off,
            "{"
            "\"dst\":\"%s\","
            "\"next\":\"%s\","
            "\"etx_x100\":%u,"
            "\"seq\":%u,"
            "\"age_ms\":%" PRIu32 ","
            "\"hold_ms\":%" PRIu32
            "}",
            d, n,
            (unsigned)routes[i].etx_x100,
            (unsigned)routes[i].seq,
            age, hold
        );

        if(off > sizeof(out)-160) break;
    }

    off += snprintf(out+off, sizeof(out)-off, "]");
    if(off >= sizeof(out)) out[sizeof(out)-1]=0;
    else out[off]=0;

    xSemaphoreGive(g_mutex);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, out, HTTPD_RESP_USE_STRLEN);
}


// ============================================================================
// ============================== API: POSTs ==================================
// ============================================================================
static void send_data_to(const char *dst_str, const char *txt, bool ackreq); // forward

static esp_err_t api_send_post(httpd_req_t *req)
{
    char body[256];
    if(!http_read_body(req, body, sizeof(body))) return http_send_text(req, "ERR body");

    char dst[16]={0}, msg[160]={0}, ackv[8]={0};
    if(!form_get(body,"dst",dst,sizeof(dst))) return http_send_text(req,"ERR missing dst");
    if(!form_get(body,"msg",msg,sizeof(msg))) return http_send_text(req,"ERR missing msg");
    form_get(body,"ack",ackv,sizeof(ackv));

    bool ack = (ackv[0] != '0');
    send_data_to(dst, msg, ack);

    return http_send_text(req, "OK send");
}

static esp_err_t api_crypto_post(httpd_req_t *req)
{
    char body[64];
    if(!http_read_body(req, body, sizeof(body))) return http_send_text(req, "ERR body");

    char v[8]={0};
    if(!form_get(body,"enable",v,sizeof(v))) return http_send_text(req,"ERR missing enable");
    int en = (v[0]=='1');

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_crypto_enable = en;
    xSemaphoreGive(g_mutex);

    return http_send_text(req, en ? "OK crypto=1" : "OK crypto=0");
}

static esp_err_t api_role_post(httpd_req_t *req)
{
    char body[64];
    if(!http_read_body(req, body, sizeof(body))) return http_send_text(req, "ERR body");

    char m[8]={0};
    if(!form_get(body,"mode",m,sizeof(m))) return http_send_text(req,"ERR missing mode");
    int v = atoi(m);
    if(v < 0 || v > 2) return http_send_text(req, "ERR mode must be 0/1/2");

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_node_mode = (node_mode_t)v;
    xSemaphoreGive(g_mutex);

    return http_send_text(req, node_mode_str(g_node_mode));
}

static esp_err_t api_wifi_post(httpd_req_t *req)
{
    char body[64];
    if(!http_read_body(req, body, sizeof(body))) return http_send_text(req, "ERR body");

    char v[8]={0};
    if(!form_get(body,"enable",v,sizeof(v))) return http_send_text(req,"ERR missing enable");
    int en = (v[0]=='1');
    set_wifi_enabled(en ? true : false);

    return http_send_text(req, en ? "OK wifi=1" : "OK wifi=0");
}

static esp_err_t api_relay_post(httpd_req_t *req)
{
    char body[64];
    if(!http_read_body(req, body, sizeof(body))) return http_send_text(req, "ERR body");

    char v[8]={0};
    if(!form_get(body,"enable",v,sizeof(v))) return http_send_text(req,"ERR missing enable");

    int en = (v[0]=='1');

#if MR_RELAY_ENABLE
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    relay_set(en ? true : false);
    bool st = g_relay_on;
    xSemaphoreGive(g_mutex);
    return http_send_text(req, st ? "OK relay=1" : "OK relay=0");
#else
    return http_send_text(req, "ERR relay disabled");
#endif
}

static esp_err_t api_relay_toggle_post(httpd_req_t *req)
{
    (void)req;

#if MR_RELAY_ENABLE
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    relay_toggle();
    bool st = g_relay_on;
    xSemaphoreGive(g_mutex);
    return http_send_text(req, st ? "OK relay=1" : "OK relay=0");
#else
    return http_send_text(req, "ERR relay disabled");
#endif
}

// ============================================================================
// ============================ SENSOR AWAKE ==================================
// ============================================================================
static void sensor_send_awake(void)
{
    if(g_node_mode != NODE_SENSOR) return;

    // ACKREQ = true -> Relay antwortet sicherer
    send_data_to(g_relay_callsign_rt, "SENSOR1:AWAKE", true);
}

static void gps_send_periodic(void)
{
#if (MR_BOARD_PRESET == MR_BOARD_TBEAM_V11_SX1276) || \
    (MR_BOARD_PRESET == MR_BOARD_TBEAM_V12_AXP2101)

    static uint32_t last_gps_send_ms = 0;
    uint32_t tnow = now_ms();
    uint32_t interval_ms = aprs_web_interval_ms();

    if(interval_ms < 5000UL){
        interval_ms = 60000UL;
    }

    if((tnow - last_gps_send_ms) < interval_ms){
        return;
    }

    last_gps_send_ms = tnow;

    aprs_cfg_t cfg;
    aprs_cfg_from_local_web(&cfg);
    if(!cfg.enabled){
        ESP_LOGW(TAG, "APRSD send skipped: APRS disabled");
        return;
    }

    char gps_txt[64];
    mr_gps_get_text(gps_txt, sizeof(gps_txt));

    if(gps_txt[0] == '\0'){
        ESP_LOGW(TAG, "APRSD send skipped: empty GPS text");
        return;
    }

    if(strstr(gps_txt, "no fix") || strstr(gps_txt, "NO FIX") ||
       strstr(gps_txt, "NOFIX")  || strstr(gps_txt, "---") ||
       strstr(gps_txt, "off")){
        ESP_LOGW(TAG, "APRSD send skipped: %s", gps_txt);
        return;
    }

    char body[80];
    if(!aprs_build_body_from_gps_text(gps_txt, &cfg, body, sizeof(body))){
        ESP_LOGW(TAG, "APRSD send skipped: GPS parse/build failed: %s", gps_txt);
        return;
    }

    char msg[MAX_PLAINTEXT + 1];
    if(!aprs_build_direct_msg(&cfg, body, msg, sizeof(msg))){
        ESP_LOGW(TAG, "APRSD send skipped: direct message too long");
        return;
    }

    send_data_to(g_relay_callsign_rt, msg, false);
    ESP_LOGI(TAG, "APRSD sent to relay %s: %s", g_relay_callsign_rt, msg);
#endif
}

static void aprs_gateway_send_periodic(void)
{
    static uint32_t last_gateway_send_ms = 0;

    aprs_cfg_t cfg;
    aprs_cfg_from_local_web(&cfg);

    if(!cfg.enabled) return;
    if(!cfg.use_static_pos) return;

    uint32_t interval_ms = aprs_web_interval_ms();
    if(interval_ms < 5000UL){
        interval_ms = 60000UL;
    }

    uint32_t tnow = now_ms();
    if((tnow - last_gateway_send_ms) < interval_ms){
        return;
    }

    char body[80];
    if(!aprs_build_body_from_static_cfg(&cfg, body, sizeof(body))){
        ESP_LOGW(TAG, "APRS gateway beacon skipped: invalid static position lat='%s' lon='%s'",
                 cfg.latitude, cfg.longitude);
        return;
    }

    char packet[160];
    if(!aprs_build_packet_from_cfg(&cfg, body, packet, sizeof(packet))){
        ESP_LOGW(TAG, "APRS gateway beacon skipped: packet build failed");
        return;
    }

    last_gateway_send_ms = tnow;

    if(aprs_send_packet_with_cfg(&cfg, packet)){
        ESP_LOGI(TAG, "APRS gateway beacon sent: %s", packet);
    }else{
        ESP_LOGW(TAG, "APRS gateway beacon send failed");
    }
}
// ============================================================================
// =============================== HTTP start =================================
// ============================================================================
static void http_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 20000;
    cfg.max_uri_handlers = 32;

    ESP_ERROR_CHECK(httpd_start(&g_http, &cfg));

    /* ---------------------------------------------------------
   OTA Update API registrieren
   ---------------------------------------------------------*/
    mr_wifi_ota_register(g_http);
    aprs_web_register_http(g_http);

    httpd_uri_t u0 = { .uri="/",                .method=HTTP_GET,  .handler=index_get };
    httpd_register_uri_handler(g_http, &u0);

    httpd_uri_t st0 = { .uri="/api/status",     .method=HTTP_GET,  .handler=api_status_get };
    httpd_register_uri_handler(g_http, &st0);

    httpd_uri_t lr0 = { .uri="/api/lastrx",     .method=HTTP_GET,  .handler=api_lastrx_get };
    httpd_register_uri_handler(g_http, &lr0);

    httpd_uri_t n0 = { .uri="/api/neighbors",   .method=HTTP_GET,  .handler=api_neighbors_get };
    httpd_register_uri_handler(g_http, &n0);

    httpd_uri_t r0 = { .uri="/api/routes",      .method=HTTP_GET,  .handler=api_routes_get };
    httpd_register_uri_handler(g_http, &r0);

    httpd_uri_t s0 = { .uri="/api/send",        .method=HTTP_POST, .handler=api_send_post };
    httpd_register_uri_handler(g_http, &s0);

    httpd_uri_t cr0 = { .uri="/api/crypto",     .method=HTTP_POST, .handler=api_crypto_post };
    httpd_register_uri_handler(g_http, &cr0);

    httpd_uri_t ro0 = { .uri="/api/role",       .method=HTTP_POST, .handler=api_role_post };
    httpd_register_uri_handler(g_http, &ro0);

    httpd_uri_t w0 = { .uri="/api/wifi",        .method=HTTP_POST, .handler=api_wifi_post };
    httpd_register_uri_handler(g_http, &w0);

    httpd_uri_t re0 = { .uri="/api/relay", .method=HTTP_POST, .handler=api_relay_post };
    httpd_register_uri_handler(g_http, &re0);

    httpd_uri_t rt0 = { .uri="/api/relay/toggle", .method=HTTP_POST, .handler=api_relay_toggle_post };
    httpd_register_uri_handler(g_http, &rt0);

    httpd_uri_t cg0 = { .uri="/api/cfg",        .method=HTTP_GET,  .handler=api_cfg_get };
    httpd_register_uri_handler(g_http, &cg0);

    httpd_uri_t cp0 = { .uri="/api/cfg",        .method=HTTP_POST, .handler=api_cfg_post };
    httpd_register_uri_handler(g_http, &cp0);

    httpd_uri_t cs0 = { .uri="/api/cfg/save",   .method=HTTP_POST, .handler=api_cfg_save_post };
    httpd_register_uri_handler(g_http, &cs0);

    httpd_uri_t cl0 = { .uri="/api/cfg/load",   .method=HTTP_POST, .handler=api_cfg_load_post };
    httpd_register_uri_handler(g_http, &cl0);

    httpd_uri_t ca0 = { .uri="/api/cfg/apply",  .method=HTTP_POST, .handler=api_cfg_apply_post };
    httpd_register_uri_handler(g_http, &ca0);

    httpd_uri_t ce0 = {.uri="/api/cfg/erase", .method=HTTP_POST, .handler=api_cfg_erase_post};
    httpd_register_uri_handler(g_http, &ce0);

    httpd_uri_t cd0 = { .uri="/api/cfg/defaults", .method=HTTP_POST, .handler=api_cfg_defaults_post };
    httpd_register_uri_handler(g_http, &cd0);

    httpd_uri_t rs0 = {.uri="/api/reset",        .method=HTTP_POST, .handler=api_reset_post};
    httpd_register_uri_handler(g_http, &rs0);

    g_http_running = true;
    ESP_LOGI(TAG, "HTTP server started");
}

static void http_start_if_needed(void)
{
    if(g_http_running) return;
    http_start();
}


// ============================================================================
// =============================== Schedulers =================================
// ============================================================================
static void beacon_schedule_next(void)
{
    g_next_beacon_ms = now_ms() + g_beacon_interval_ms + (esp_random()%BEACON_JITTER_MS);
}
static void routeadv_schedule_next(void)
{
    g_next_routeadv_ms = now_ms() + 20000 + (esp_random()%2000);
}


// ============================================================================
// ============================= Remote Commands ==============================
// ============================================================================
static void app_send_reply_to_sender(const char from7[8], const char *msg)
{
    char from_str[9];
    call7_to_str(from_str, from7);
    send_data_to(from_str, msg, true);
}

static void app_handle_cmd_if_any(const char from7[8], const char *txt)
{
    if(!txt || !txt[0]) return;
    //if(g_node_mode != NODE_RELAY) return;

    if(strcmp(txt, "CMD:STATUS?")==0 || strcmp(txt, "STATUS?")==0){
        uint32_t batt_mv=0, batt_pct=0;
#if MR_BATT_ENABLE
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        batt_mv = g_batt_mv;
        batt_pct = g_batt_pct;
        xSemaphoreGive(g_mutex);
#endif
        char ans[240];
        snprintf(ans,sizeof(ans),
                 "STATUS: mode=%s relay=%s display=%s wifi=%s http=%s crypto=%s batt=%" PRIu32 "mV/%" PRIu32 "%%",
                 node_mode_str(g_node_mode),
#if MR_RELAY_ENABLE
                 g_relay_on?"ON":"OFF",
#else
                 "N/A",
#endif
                 g_display_enabled?"ON":"OFF",
                 g_wifi_enabled?"ON":"OFF",
                 g_http_running?"ON":"OFF",
                 g_crypto_enable?"ON":"OFF",
                 batt_mv, batt_pct);
        app_send_reply_to_sender(from7, ans);
        return;
    }

    if(strcmp(txt, "CMD:CRYPTO ON")==0 || strcmp(txt, "CRYPTO ON")==0){
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        g_cfg.crypto_enable = true;
        mr_cfg_apply(&g_cfg);
        xSemaphoreGive(g_mutex);
        app_send_reply_to_sender(from7, "CRYPTO: ON");
        return;
    }

    if(strcmp(txt, "CMD:CRYPTO OFF")==0 || strcmp(txt, "CRYPTO OFF")==0){
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        g_cfg.crypto_enable = false;
        mr_cfg_apply(&g_cfg);
        xSemaphoreGive(g_mutex);
        app_send_reply_to_sender(from7, "CRYPTO: OFF");
        return;
    }

    if(strcmp(txt, "CMD:DISPLAY ON")==0){
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        g_cfg.display_enable = true;
        rtc_display_enable = 1;
        (void)mr_cfg_save_nvs(&g_cfg);
        mr_cfg_apply(&g_cfg);
        xSemaphoreGive(g_mutex);
        app_send_reply_to_sender(from7, "DISPLAY: ON");
        return;
    }

    if(strcmp(txt, "CMD:DISPLAY OFF")==0){
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        g_cfg.display_enable = false;
        rtc_display_enable = 0;
        (void)mr_cfg_save_nvs(&g_cfg);
        mr_cfg_apply(&g_cfg);
        xSemaphoreGive(g_mutex);
        app_send_reply_to_sender(from7, "DISPLAY: OFF");
        return;
    }

    if(strcmp(txt, "CMD:WIFI ON")==0){
        bool ok=false;
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        g_cfg.wifi_enable = true;
        ok = mr_cfg_save_nvs(&g_cfg);
        mr_cfg_apply(&g_cfg);
        xSemaphoreGive(g_mutex);
        app_send_reply_to_sender(from7, ok ? "WIFI: ON" : "WIFI: ON (NOT SAVED)");
        return;
    }

    if(strcmp(txt, "CMD:WIFI OFF")==0){
        bool ok=false;
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        g_cfg.wifi_enable = false;
        ok = mr_cfg_save_nvs(&g_cfg);
        mr_cfg_apply(&g_cfg);
        xSemaphoreGive(g_mutex);
        app_send_reply_to_sender(from7, ok ? "WIFI: OFF" : "WIFI: OFF (NOT SAVED)");
        return;
    }

    if(strcmp(txt, "CMD:GPS")==0){
#if (MR_BOARD_PRESET == MR_BOARD_TBEAM_V11_SX1276) || \
    (MR_BOARD_PRESET == MR_BOARD_TBEAM_V12_AXP2101)
        char gps_txt[96];
        mr_gps_get_text(gps_txt, sizeof(gps_txt));
        app_send_reply_to_sender(from7, gps_txt[0] ? gps_txt : "GPS: N/A");
#else
        app_send_reply_to_sender(from7, "GPS: N/A");
#endif
        return;
    }

    if(strcmp(txt, "CMD:BATT")==0){
        uint32_t batt_mv=0, batt_pct=0;
#if MR_BATT_ENABLE
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        batt_mv = g_batt_mv;
        batt_pct = g_batt_pct;
        xSemaphoreGive(g_mutex);
#endif
        char ans[96];
        snprintf(ans, sizeof(ans), "BATT: %" PRIu32 "mV/%" PRIu32 "%%", batt_mv, batt_pct);
        app_send_reply_to_sender(from7, ans);
        return;
    }

    if(strcmp(txt, "CMD:VERSION")==0){
        const esp_app_desc_t *app = esp_app_get_description();
        app_send_reply_to_sender(from7, app->version);
        return;
    }

    if(strcmp(txt, "CMD:REBOOT")==0){
        app_send_reply_to_sender(from7, "REBOOTING");
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
        return;
    }

#if MR_RELAY_ENABLE
    if(strcmp(txt, "CMD:RELAY ON")==0){
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        relay_set(true);
        bool st=g_relay_on;
        xSemaphoreGive(g_mutex);
        app_send_reply_to_sender(from7, st?"RELAY: ON":"RELAY: OFF");
        return;
    }
    if(strcmp(txt, "CMD:RELAY OFF")==0){
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        relay_set(false);
        bool st=g_relay_on;
        xSemaphoreGive(g_mutex);
        app_send_reply_to_sender(from7, st?"RELAY: ON":"RELAY: OFF");
        return;
    }
    if(strcmp(txt, "CMD:RELAY TOGGLE")==0){
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        relay_toggle();
        bool st=g_relay_on;
        xSemaphoreGive(g_mutex);
        app_send_reply_to_sender(from7, st?"RELAY: ON":"RELAY: OFF");
        return;
    }
#endif
}

// ============================================================================
// ============================= TX: Beacon/ACK/DATA ==========================
// ============================================================================
static void send_beacon(void)
{
    mr_hdr_v7_t h={0};
    h.msg_id = mr_next_msg_id();
    h.seq    = mr_next_seq();
    h.magic[0]='M'; h.magic[1]='R';
    h.version=MR_PROTO_VERSION;
    h.flags=MR_FLAG_BEACON;
    h.ttl=BEACON_TTL;
    h.msg_id = mr_next_msg_id();
    h.seq    = mr_next_seq();
    call7_set(h.src, g_callsign_rt);
    call7_set(h.last_hop, g_callsign_rt);
    call7_set(h.final_dst,"*");
    call7_set(h.next_hop,"*");
    h.payload_len=0;

    if(bucket_take(&b_beacon)){
        c_tx_beacon++;
        lora_send_frame((uint8_t*)&h, sizeof(h));
    }else{
        c_defer_beacon++;
    }

    beacon_schedule_next();
}

static void send_ack(const mr_hdr_v7_t *rx)
{
    mr_hdr_v7_t h={0};
    h.magic[0]='M'; h.magic[1]='R';
    h.version=MR_PROTO_VERSION;
    h.flags=MR_FLAG_ACK;
    h.ttl=ACK_TTL;
    h.msg_id = rx->msg_id;
    h.seq = g_my_seq; // ack doesn't increment seq here

    call7_set(h.src, g_callsign_rt);
    call7_set(h.last_hop, g_callsign_rt);
    memcpy(h.final_dst, rx->last_hop, 8);
    call7_set(h.next_hop, "*");
    h.payload_len=0;

    if(bucket_take(&b_ack)){
        c_tx_ack++;
        vTaskDelay(pdMS_TO_TICKS(esp_random()%150));
        lora_send_frame((uint8_t*)&h, sizeof(h));
    }else{
        c_defer_ack++;
    }
}

static void send_data_to(const char *dst_str, const char *txt, bool ackreq)
{
    uint8_t buf[sizeof(mr_hdr_v7_t) + MAX_PAYLOAD]={0};
    mr_hdr_v7_t *h=(mr_hdr_v7_t*)buf;

    char dbg_src[9], dbg_dst[9], dbg_last[9], dbg_next[9];
    call7_to_str(dbg_src, h->src);
    call7_to_str(dbg_dst, h->final_dst);
    call7_to_str(dbg_last, h->last_hop);
    call7_to_str(dbg_next, h->next_hop);

    /* ESP_LOGW(TAG,
         "RX FRAME flags=0x%02X src=%s dst=%s last=%s next=%s msg_id=%u seq=%u len=%u",
         h->flags,
         dbg_src,
         dbg_dst,
         dbg_last,
         dbg_next,
         (unsigned)h->msg_id,
         (unsigned)h->seq,
         (unsigned)h->payload_len);
    */

    h->magic[0]='M'; h->magic[1]='R';
    h->version=MR_PROTO_VERSION;
    h->flags=MR_FLAG_DATA | (ackreq?MR_FLAG_ACKREQ:0);
    h->ttl=DATA_TTL;
    h->msg_id = mr_next_msg_id();
    h->seq    = mr_next_seq();
    call7_set(h->src, g_callsign_rt);
    call7_set(h->last_hop, g_callsign_rt);

    char dst7[8]; call7_set(dst7, dst_str);
    memcpy(h->final_dst, dst7, 8);

    // Für "profi-level stabil": routing kann hier wieder rein (route_lookup_locked).
    call7_set(h->next_hop, "*");

    uint8_t *pl = buf + sizeof(mr_hdr_v7_t);
    size_t n=strlen(txt);
    if(n>MAX_PLAINTEXT) n=MAX_PLAINTEXT;

    bool crypto=g_crypto_enable;
    if(crypto){
        h->flags |= MR_FLAG_SEC;
                
                 /* ESP_LOGW(TAG, "TX SEC SET dst=%s flags=0x%02X msg_id=%u seq=%u payload_plain=%u",
                 dst_str,
                 h->flags,
                 (unsigned)h->msg_id,
                 (unsigned)h->seq,
                 (unsigned)n);
            */  
        h->payload_len = (uint8_t)(n + SEC_TAG_LEN);

        uint8_t tag[SEC_TAG_LEN]={0};
        if(!sec_encrypt_payload(h, (const uint8_t*)txt, n, pl, tag)){
            ESP_LOGE(TAG,
                     "SEC ENCRYPT FAIL dst=%s msg_id=%u seq=%u plain_len=%u key=%02X%02X%02X%02X",
                     dst_str,
                     (unsigned)h->msg_id,
                     (unsigned)h->seq,
                     (unsigned)n,
                     g_net_key[0], g_net_key[1], g_net_key[2], g_net_key[3]);
            c_drop_data++;
            return;
        }
        memcpy(pl + n, tag, SEC_TAG_LEN);
                /*ESP_LOGW(TAG,
                 "SEC ENCRYPT OK dst=%s msg_id=%u seq=%u tag=%02X%02X%02X%02X",
                 dst_str,
                 (unsigned)h->msg_id,
                 (unsigned)h->seq,
                 tag[0], tag[1], tag[2], tag[3]); */
    }else{
        h->payload_len=(uint8_t)n;
        memcpy(pl, txt, n);
                /*ESP_LOGW(TAG, "TX PLAIN dst=%s flags=0x%02X msg_id=%u seq=%u payload_plain=%u",
                 dst_str,
                 h->flags,
                 (unsigned)h->msg_id,
                 (unsigned)h->seq,
                 (unsigned)n); */
    }

    uint16_t frame_len = (uint16_t)(sizeof(mr_hdr_v7_t) + h->payload_len);

        if(bucket_take(&b_data)){
        c_tx_data++;

        /*ESP_LOGW(TAG,
                 "TX AIR dst=%s flags=0x%02X msg_id=%u seq=%u frame_len=%u",
                 dst_str,
                 h->flags,
                 (unsigned)h->msg_id,
                 (unsigned)h->seq,
                 (unsigned)frame_len);
            */
        if (ackreq) {
            xSemaphoreTake(g_mutex, portMAX_DELAY);
            pending_add_locked(h->msg_id, h->final_dst, (const uint8_t*)buf, frame_len);
            neighbor_tx_attempt_locked(h->final_dst);
            xSemaphoreGive(g_mutex);
        }

        lora_send_frame(buf, frame_len);

        /*ESP_LOGW(TAG,
                 "TX AIR DONE dst=%s msg_id=%u seq=%u",
                 dst_str,
                 (unsigned)h->msg_id,
                 (unsigned)h->seq);
            */
    }else{
        c_defer_data++;
        ESP_LOGE(TAG,
                 "TX DEFER bucket empty dst=%s flags=0x%02X msg_id=%u seq=%u",
                 dst_str,
                 h->flags,
                 (unsigned)h->msg_id,
                 (unsigned)h->seq);
    }
}


// ============================================================================
// =============================== RX (unlock-safe) ===========================
// ============================================================================
static void handle_rx(void)
{
    uint8_t buf[256];
    uint8_t len=0;
    int rssi=-127;

    if(!lora_try_read_frame(buf, sizeof(buf), &len, &rssi)) return;
    if(len < sizeof(mr_hdr_v7_t)) return;

    mr_hdr_v7_t *h=(mr_hdr_v7_t*)buf;
    if(h->magic[0]!='M'||h->magic[1]!='R') return;
    if(h->version!=MR_PROTO_VERSION) return;
    char dbg_src[9], dbg_dst[9], dbg_last[9], dbg_next[9];
    call7_to_str(dbg_src, h->src);
    call7_to_str(dbg_dst, h->final_dst);
    call7_to_str(dbg_last, h->last_hop);
    call7_to_str(dbg_next, h->next_hop);

    /*ESP_LOGW(TAG,
             "RX FRAME flags=0x%02X src=%s dst=%s last=%s next=%s msg_id=%u seq=%u len=%u rssi=%d",
             h->flags,
             dbg_src,
             dbg_dst,
             dbg_last,
             dbg_next,
             (unsigned)h->msg_id,
             (unsigned)h->seq,
             (unsigned)h->payload_len,
             rssi); */
    // ACK: not duplicate filtered
    if(h->flags & MR_FLAG_ACK){
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        neighbor_update_rssi_locked(h->last_hop, rssi);
        neighbor_ack_ok_locked(h->src);
        pending_mark_acked_locked(h->msg_id, h->src);
        xSemaphoreGive(g_mutex);
        return;
    }

    if(seen_before(h->src, h->msg_id)){
        ESP_LOGE(TAG, "RX DROP seen_before src=%.8s msg_id=%u",
                 h->src, (unsigned)h->msg_id);
        return;
    }
    remember_msg(h->src, h->msg_id);

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    neighbor_update_rssi_locked(h->last_hop, rssi);
    xSemaphoreGive(g_mutex);

    if(h->flags & MR_FLAG_BEACON){
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        int ni = neighbor_ensure_locked(h->last_hop);
        uint16_t etx=100;
        if(ni>=0){
            neighbor_decay_locked(&neighbors[ni]);
            etx = etx_compute_x100(neighbors[ni].tx_attempts, neighbors[ni].ack_ok);
        }
        route_update_locked(h->src, h->last_hop, h->seq, etx, rssi);
        xSemaphoreGive(g_mutex);
        return;
    }

        if(h->flags & MR_FLAG_DATA){
        char my7[8];
        char my_str[9];
        call7_set(my7, g_callsign_rt);
        call7_to_str(my_str, my7);

        /* ESP_LOGW(TAG, "RX DATA check: dst=%.8s my=%s sec=%u",
                 h->final_dst,
                 my_str,
                 ((h->flags & MR_FLAG_SEC) ? 1 : 0));
            */
        if(call7_eq(h->final_dst, my7)){
            char txt[MAX_PAYLOAD+1]={0};

            if((h->flags & MR_FLAG_SEC) != 0){
                char dbg_from[9];
                call7_to_str(dbg_from, h->src);

                ESP_LOGW(TAG,
                        "SEC RX detected from=%s msg_id=%u seq=%u payload_len=%u",
                        dbg_from,
                        (unsigned)h->msg_id,
                        (unsigned)h->seq,
                        (unsigned)h->payload_len);
                                if(h->payload_len < SEC_TAG_LEN){
                                    xSemaphoreTake(g_mutex, portMAX_DELAY);
                                    sec_decrypt_fail++;
                                    xSemaphoreGive(g_mutex);
                                    return;
                                }

                xSemaphoreTake(g_mutex, portMAX_DELAY);
                bool fresh = replay_check_locked(h->src, h->seq);
                xSemaphoreGive(g_mutex);
                if(!fresh){
                     ESP_LOGE(TAG,"SEC FAIL replay src=%.8s seq=%u",
                        h->src,(unsigned)h->seq);

                    xSemaphoreTake(g_mutex, portMAX_DELAY);
                    sec_replay_drop++;
                    xSemaphoreGive(g_mutex);
                    return;
                }

                size_t ciph_len = (size_t)h->payload_len - SEC_TAG_LEN;
                const uint8_t *pl  = buf + sizeof(mr_hdr_v7_t);
                const uint8_t *tag = pl + ciph_len;

                uint8_t plain[MAX_PLAINTEXT+1];
                memset(plain,0,sizeof(plain));

                if(!sec_decrypt_payload(h, pl, ciph_len, tag, plain)){

                ESP_LOGE(TAG,
                    "SEC FAIL decrypt msg_id=%u seq=%u ciph_len=%u",
                    (unsigned)h->msg_id,
                    (unsigned)h->seq,
                    (unsigned)ciph_len);

                    xSemaphoreTake(g_mutex, portMAX_DELAY);
                    sec_mac_fail++; 
                    sec_decrypt_fail++;
                    xSemaphoreGive(g_mutex);
                return;
                }

                 /*ESP_LOGW(TAG,
                 "SEC RX decrypt OK from=%s msg_id=%u seq=%u plain_len=%u",
                 dbg_from,
                 (unsigned)h->msg_id,
                 (unsigned)h->seq,
                 (unsigned)ciph_len);
                */
                xSemaphoreTake(g_mutex, portMAX_DELAY);
                replay_update_locked(h->src, h->seq);
                sec_decrypt_ok++;
                ESP_LOGI(TAG,
                "SEC OK msg_id=%u seq=%u",
                (unsigned)h->msg_id,
                (unsigned)h->seq);
                xSemaphoreGive(g_mutex);

                size_t copy = (ciph_len < MAX_PAYLOAD) ? ciph_len : MAX_PAYLOAD;
                memcpy(txt, plain, copy);
                txt[copy]=0;
            }else{
                if(h->payload_len>0 && h->payload_len<=MAX_PAYLOAD){
                    memcpy(txt, buf+sizeof(mr_hdr_v7_t), h->payload_len);
                    txt[h->payload_len]=0;
                }
            }

            char from_str[9];
            call7_to_str(from_str, h->src);
            xSemaphoreTake(g_mutex, portMAX_DELAY);
            strncpy(g_last_rx_from, from_str, sizeof(g_last_rx_from)-1);
            g_last_rx_from[sizeof(g_last_rx_from)-1]=0;
            strncpy(g_last_rx_text, txt, sizeof(g_last_rx_text)-1);
            g_last_rx_text[sizeof(g_last_rx_text)-1]=0;
            g_last_rx_ms = now_ms();
            xSemaphoreGive(g_mutex);

            ESP_LOGI(TAG,"DATA delivered ✅ from=%s rssi=%d \"%s\"", from_str, rssi, txt);

            if(g_node_mode == NODE_RELAY && g_wifi_enabled){
                aprs_cfg_t remote_cfg;
                char aprs_body[96];

                if(strncmp(txt, "APRSD ", 6) == 0){
                    if(aprs_direct_parse_msg(txt, &remote_cfg, aprs_body, sizeof(aprs_body))){
                        char packet[256];
                        bool ok = aprs_build_packet_from_cfg(&remote_cfg, aprs_body, packet, sizeof(packet));
                        if(ok){
                            ok = aprs_send_packet_with_cfg(&remote_cfg, packet);
                        }
                        ESP_LOGI(TAG, "APRSD gateway from %s using %s: %s -> %s",
                                 from_str, remote_cfg.callsign, aprs_body, ok ? "OK" : "FAIL");
                    } else {
                        ESP_LOGW(TAG, "APRSD parse failed from %s: %s", from_str, txt);
                    }
                }
            }

            char wx_str[16] = "";
            float temp;

            if (strstr(txt, "WX")) {
                char *p = strstr(txt, "t=");
                if (p && sscanf(p, "t=%f", &temp) == 1) {
                    snprintf(wx_str, sizeof(wx_str), "T=%.2f C", temp);
                }
            }

            char dt[20] = "";
            char dt2[24] = "";
            char line4[24] = "";
            const char *line4_eff = line4;

            mr_time_net_get_datetime(dt, sizeof(dt));

            if (mr_time_net_get_datetime(dt, sizeof(dt))) {
                snprintf(dt2, sizeof(dt2), "   %s", dt); 
                line4_eff = dt2;
            }

            if (g_display_enabled) {
                mr_display_clear();

                char line3[24];

                if (wx_str[0]) {
                    snprintf(line3, sizeof(line3), "rssi=%d %.10s", rssi, wx_str);
                } else {
                    snprintf(line3, sizeof(line3), "rssi=%d", rssi);
                }

                mr_display_show_status8(
                        "LAST RX",
                        "",
                        from_str,
                        line3,
                        "",
                        line4_eff,
                        "",
                        ""
                    );
            }

            app_handle_cmd_if_any(h->src, txt);

            if((h->flags & MR_FLAG_ACKREQ) != 0) send_ack(h);
        }
    }
}

// SENSOR: kleines, robustes RX-Fenster per Polling
static void sensor_awake_window(uint32_t ms)
{
    uint32_t t0 = now_ms();
    while(now_ms() - t0 < ms){
        handle_rx(); // poll
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
// ============================================================================
// ============================ DIO ISR + Task ================================
// ============================================================================
static void IRAM_ATTR dio_isr(void*arg)
{
    uint32_t n=(uint32_t)arg;
    xQueueSendFromISR(dio_q,&n,NULL);
}

static void dio_task(void*arg)
{
    (void)arg;
    uint32_t io;
    while(1){
        if(xQueueReceive(dio_q,&io,portMAX_DELAY)){
            handle_rx();
        }
    }
}

static void init_dio_irq(void)
{
    dio_q=xQueueCreate(10,sizeof(uint32_t));

#if defined(MR_LORA_CHIP_SX1276)
    const gpio_num_t pin_irq = (gpio_num_t)PIN_LORA_DIO0;
#elif defined(MR_LORA_CHIP_SX1262)
    const gpio_num_t pin_irq = (gpio_num_t)PIN_LORA_DIO1;
#else
#error "No LoRa chip defined"
#endif

    gpio_config_t io={
        .intr_type=GPIO_INTR_POSEDGE,
        .mode=GPIO_MODE_INPUT,
        .pin_bit_mask=(1ULL<<pin_irq),
#if defined(MR_LORA_CHIP_SX1262)
        .pull_up_en=0, // Heltec usually has external pullups; keep input clean
#else
        .pull_up_en=1
#endif
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(pin_irq, dio_isr, (void*)pin_irq));

    xTaskCreate(dio_task, "dio", 4096, NULL, 10, NULL);
}


// ============================================================================
// =============================== Retry Task =================================
// ============================================================================
static void retry_task(void *arg)
{
    (void)arg;
    while(1){
        vTaskDelay(pdMS_TO_TICKS(100));

        xSemaphoreTake(g_mutex, portMAX_DELAY);
        uint32_t t=now_ms();

        for(int i=0;i<MAX_PENDING_ACK;i++){
            if(!pend[i].used) continue;
            if(t < pend[i].deadline_ms) continue;

            if(pend[i].retries_left == 0){
                pending_clear_slot_locked(i);
                continue;
            }

            pend[i].retries_left--;
            pend[i].deadline_ms = t + ACK_TIMEOUT_MS;

            uint32_t bo = (esp_random() % (ACK_BACKOFF_MS+1));
            xSemaphoreGive(g_mutex);
            vTaskDelay(pdMS_TO_TICKS(bo));
            xSemaphoreTake(g_mutex, portMAX_DELAY);

            mr_hdr_v7_t *h = (mr_hdr_v7_t*)pend[i].frame;
            if(pend[i].frame_len >= sizeof(mr_hdr_v7_t) && h->ttl > 1) h->ttl--;

            neighbor_tx_attempt_locked(pend[i].expect_from);

            if(bucket_take(&b_data)){
                c_tx_data++;
                lora_send_frame(pend[i].frame, pend[i].frame_len);
            }else{
                c_defer_data++;
            }
        }

        xSemaphoreGive(g_mutex);
    }
}


// ============================================================================
// =============================== CLI (UART0) ================================
// ============================================================================
#if MR_CLI_ENABLE
static void cli_print_help(void)
{
    printf("\n--- MeshRadio CLI ---\n");
    printf("help (h)\n");
    printf("status?\n");
    printf("wifi on|off\n");
    printf("role 0|1|2       (0=RELAY 1=EDGE 2=SENSOR)\n");
    printf("crypto on|off\n");
    printf("cfg get\n");
    printf("cfg set <key> <val>\n");
    printf("cfg save\n");
    printf("cfg load\n");
    printf("cfg defaults\n");
    printf("cfg apply\n");
#if MR_RELAY_ENABLE
    printf("relay on|off|toggle\n");
#endif
    printf("send <DST> <ACK 0|1> <TEXT...>\n");
    printf("cmd  <DST> <ACK 0|1> STATUS|RELAY ON|RELAY OFF|RELAY TOGGLE|CRYPTO ON|CRYPTO OFF|DISPLAY ON|DISPLAY OFF|WIFI ON|WIFI OFF|GPS|BATT|VERSION|REBOOT\n");
    printf("---------------------\n\n");
}

static void trim_line(char *s)
{
    size_t L=strlen(s);
    while(L>0 && (s[L-1]=='\r'||s[L-1]=='\n'||s[L-1]==' '||s[L-1]=='\t')) s[--L]=0;
    size_t i=0;
    while(s[i]==' '||s[i]=='\t') i++;
    if(i>0) memmove(s, s+i, strlen(s+i)+1);
}

static void cli_sanitize_line(char *s)
{
    char out[MR_CLI_LINE_MAX];
    size_t oi=0;

    for(size_t i=0; s[i] && oi < sizeof(out)-1; ){
        unsigned char c = (unsigned char)s[i];

        if(c == 0x1B){
            i++;
            if(s[i] == '['){
                i++;
                while(s[i]){
                    unsigned char x = (unsigned char)s[i++];
                    if(x >= 0x40 && x <= 0x7E) break;
                }
                continue;
            }else if(s[i] == ']'){
                i++;
                while(s[i]){
                    if((unsigned char)s[i] == 0x07){ i++; break; }
                    if((unsigned char)s[i] == 0x1B && s[i+1] == '\\'){ i+=2; break; }
                    i++;
                }
                continue;
            }else{
                if(s[i]) i++;
                continue;
            }
        }

        if(c == 0x08 || c == 0x7F){
            if(oi>0) oi--;
            i++;
            continue;
        }

        if(c >= 32 && c <= 126){
            out[oi++] = (char)c;
        }
        i++;
    }

    out[oi]=0;
    strcpy(s, out);
    trim_line(s);

    while(s[0] == '>'){
        memmove(s, s+1, strlen(s+1)+1);
        trim_line(s);
    }
}

static bool streq_ci(const char *a, const char *b)
{
    while(*a && *b){
        if(tolower((unsigned char)*a) != tolower((unsigned char)*b)) return false;
        a++; b++;
    }
    return (*a==0 && *b==0);
}

static void cli_handle_line(char *line)
{
    cli_sanitize_line(line);
    if(line[0]==0) return;

    char *save=NULL;
    char *cmd=strtok_r(line, " \t", &save);
    if(!cmd) return;

    if(streq_ci(cmd,"h") || streq_ci(cmd,"help")){
        cli_print_help();
        return;
    }

    if(streq_ci(cmd,"status?")){
        uint32_t batt_mv=0, batt_pct=0;
        uint32_t last_rx_age_ms = 0;
        int neigh_cnt = 0;
        int route_cnt = 0;

        xSemaphoreTake(g_mutex, portMAX_DELAY);
#if MR_BATT_ENABLE
        batt_mv = g_batt_mv;
        batt_pct = g_batt_pct;
#endif
        if(g_last_rx_ms != 0){
            last_rx_age_ms = now_ms() - g_last_rx_ms;
        }
        for(int i=0;i<MAX_NEIGHBORS;i++) if(neighbors[i].used) neigh_cnt++;
        for(int i=0;i<MAX_ROUTES;i++)    if(routes[i].used)    route_cnt++;
        xSemaphoreGive(g_mutex);

        printf("CALL:     %s\n", g_callsign_rt);
        printf("MODE:     %s\n", node_mode_str(g_node_mode));
        printf("BOARD:    %s\n", cfg_board_str());
        printf("CHIP:     %s\n", cfg_lora_chip_str());
        printf("PROTO:    %u\n", MR_PROTO_VERSION);
        printf("RF:       %lu Hz\n", (unsigned long)g_rf_freq_hz_runtime);
        printf("TX:       %d dBm\n", (int)g_tx_power_dbm_runtime);
#if MR_RELAY_ENABLE
        printf("RELAY:    %s\n", g_relay_on?"ON":"OFF");
#else
        printf("RELAY:    N/A\n");
#endif
        printf("DISPLAY:  %s (cfg=%s)\n",
               g_display_enabled?"ON":"OFF",
               g_cfg.display_enable?"ON":"OFF");
        printf("WIFI:     %s\n", g_wifi_enabled?"ON":"OFF");
        printf("HTTP:     %s\n", g_http_running?"ON":"OFF");
        printf("CRYPTO:   %s\n", g_crypto_enable?"ON":"OFF");
        printf("BEACON:   %s\n", g_beacon_enabled?"ON":"OFF");
        printf("ROUTEADV: %s\n", g_routeadv_enable?"ON":"OFF");
        printf("CAD:      %s\n", g_cad_enable?"ON":"OFF");
        printf("PWSAVE:   %s\n", g_powersave_enable?"ON":"OFF");
        printf("BME280:   %s\n", g_bme280_enable?"ON":"OFF");
        printf("BATT:     %" PRIu32 " mV / %" PRIu32 "%%\n", batt_mv, batt_pct);
        printf("NEIGH:    %d\n", neigh_cnt);
        printf("ROUTES:   %d\n", route_cnt);
        if(g_last_rx_from[0]){
            printf("LAST RX:  %s / %lu ms / %s\n",
                   g_last_rx_from,
                   (unsigned long)last_rx_age_ms,
                   g_last_rx_text[0] ? g_last_rx_text : "-");
        }else{
            printf("LAST RX:  -\n");
        }
        return;
    }

    if(streq_ci(cmd,"wifi")){
        char *arg=strtok_r(NULL, " \t", &save);
        bool ok=false;
        bool save_ok=false;

        if(!arg){
            printf("ERR wifi on|off\n");
            return;
        }

        xSemaphoreTake(g_mutex, portMAX_DELAY);

        if(streq_ci(arg,"on")){
            g_cfg.wifi_enable = true;
            save_ok = mr_cfg_save_nvs(&g_cfg);
            mr_cfg_apply(&g_cfg);
            ok = true;
        }
        else if(streq_ci(arg,"off")){
            g_cfg.wifi_enable = false;
            save_ok = mr_cfg_save_nvs(&g_cfg);
            mr_cfg_apply(&g_cfg);
            ok = true;
        }

        xSemaphoreGive(g_mutex);

        if(!ok){
            printf("ERR wifi on|off\n");
            return;
        }

        printf(save_ok ? "OK wifi=%s\n" : "OK wifi=%s (NOT SAVED)\n",
               g_cfg.wifi_enable ? "ON" : "OFF");
        return;
    }

    if(streq_ci(cmd,"role")){
        char *arg=strtok_r(NULL, " \t", &save);
        if(!arg){ printf("ERR role 0|1|2\n"); return; }
        int m=atoi(arg);
        if(m<0||m>2){ printf("ERR role 0|1|2\n"); return; }

        xSemaphoreTake(g_mutex, portMAX_DELAY);
        g_node_mode=(node_mode_t)m;
        xSemaphoreGive(g_mutex);

        printf("OK role=%s\n", node_mode_str(g_node_mode));
        return;
    }

    if(streq_ci(cmd,"crypto")){
    char *arg=strtok_r(NULL, " \t", &save);
    if(!arg){ printf("ERR crypto on|off\n"); return; }

    xSemaphoreTake(g_mutex, portMAX_DELAY);

    if(streq_ci(arg,"on")){
        g_cfg.crypto_enable = true;
        mr_cfg_apply(&g_cfg);
    }
    else if(streq_ci(arg,"off")){
        g_cfg.crypto_enable = false;
        mr_cfg_apply(&g_cfg);
    }
    else{
        xSemaphoreGive(g_mutex);
        printf("ERR crypto on|off\n");
        return;
    }

    bool st = g_crypto_enable;
    xSemaphoreGive(g_mutex);

    printf("OK crypto=%s\n", st?"ON":"OFF");
    return;
}


#if MR_RELAY_ENABLE
    if(streq_ci(cmd,"relay")){
    char *arg=strtok_r(NULL, " \t", &save);
    if(!arg){ printf("ERR relay on|off|toggle\n"); return; }

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    if(streq_ci(arg,"on")) relay_set(true);
    else if(streq_ci(arg,"off")) relay_set(false);
    else if(streq_ci(arg,"toggle")) relay_toggle();
    else { xSemaphoreGive(g_mutex); printf("ERR relay on|off|toggle\n"); return; }
    bool st=g_relay_on;
    xSemaphoreGive(g_mutex);

    printf("OK relay=%s\n", st?"ON":"OFF");
    return;
}
#endif

    if(streq_ci(cmd,"send")){
        char *dst=strtok_r(NULL, " \t", &save);
        char *ack=strtok_r(NULL, " \t", &save);
        char *msg=save;
        if(!dst || !ack || !msg){ printf("ERR send <DST> <ACK 0|1> <TEXT...>\n"); return; }
        while(*msg==' '||*msg=='\t') msg++;
        if(!*msg){ printf("ERR missing text\n"); return; }
        int a=atoi(ack);
        send_data_to(dst, msg, (a!=0));
        printf("OK sent\n");
        return;
    }

    if(streq_ci(cmd,"cmd")){
        char *dst=strtok_r(NULL, " \t", &save);
        char *ack=strtok_r(NULL, " \t", &save);
        char *what=strtok_r(NULL, " \t", &save);
        if(!dst || !ack || !what){
            printf("ERR cmd <DST> <ACK> STATUS|RELAY ON|RELAY OFF|RELAY TOGGLE|CRYPTO ON|CRYPTO OFF|DISPLAY ON|DISPLAY OFF|WIFI ON|WIFI OFF|GPS|BATT|VERSION|REBOOT\n");
            return;
        }
        int a=atoi(ack);

        if(streq_ci(what,"STATUS")){
            send_data_to(dst, "CMD:STATUS?", (a!=0));
        }
        else if(streq_ci(what,"RELAY")){
            char *arg=strtok_r(NULL, " \t", &save);
            if(!arg){ printf("ERR cmd <DST> <ACK> RELAY ON|OFF|TOGGLE\n"); return; }

            if(streq_ci(arg,"ON")) send_data_to(dst, "CMD:RELAY ON", (a!=0));
            else if(streq_ci(arg,"OFF")) send_data_to(dst, "CMD:RELAY OFF", (a!=0));
            else if(streq_ci(arg,"TOGGLE")) send_data_to(dst, "CMD:RELAY TOGGLE", (a!=0));
            else { printf("ERR cmd <DST> <ACK> RELAY ON|OFF|TOGGLE\n"); return; }
        }
        else if(streq_ci(what,"CRYPTO")){
            char *arg=strtok_r(NULL, " \t", &save);
            if(!arg){ printf("ERR cmd <DST> <ACK> CRYPTO ON|OFF\n"); return; }

            if(streq_ci(arg,"ON")) send_data_to(dst, "CMD:CRYPTO ON", (a!=0));
            else if(streq_ci(arg,"OFF")) send_data_to(dst, "CMD:CRYPTO OFF", (a!=0));
            else { printf("ERR cmd <DST> <ACK> CRYPTO ON|OFF\n"); return; }
        }
        else if(streq_ci(what,"DISPLAY")){
            char *arg=strtok_r(NULL, " \t", &save);
            if(!arg){ printf("ERR cmd <DST> <ACK> DISPLAY ON|OFF\n"); return; }

            if(streq_ci(arg,"ON")) send_data_to(dst, "CMD:DISPLAY ON", (a!=0));
            else if(streq_ci(arg,"OFF")) send_data_to(dst, "CMD:DISPLAY OFF", (a!=0));
            else { printf("ERR cmd <DST> <ACK> DISPLAY ON|OFF\n"); return; }
        }
        else if(streq_ci(what,"WIFI")){
            char *arg=strtok_r(NULL, " \t", &save);
            if(!arg){ printf("ERR cmd <DST> <ACK> WIFI ON|OFF\n"); return; }

            if(streq_ci(arg,"ON")) send_data_to(dst, "CMD:WIFI ON", (a!=0));
            else if(streq_ci(arg,"OFF")) send_data_to(dst, "CMD:WIFI OFF", (a!=0));
            else { printf("ERR cmd <DST> <ACK> WIFI ON|OFF\n"); return; }
        }
        else if(streq_ci(what,"GPS")){
            send_data_to(dst, "CMD:GPS", (a!=0));
        }
        else if(streq_ci(what,"BATT")){
            send_data_to(dst, "CMD:BATT", (a!=0));
        }
        else if(streq_ci(what,"VERSION")){
            send_data_to(dst, "CMD:VERSION", (a!=0));
        }
        else if(streq_ci(what,"REBOOT")){
            send_data_to(dst, "CMD:REBOOT", (a!=0));
        }
        else{
            printf("ERR unknown cmd\n");
            return;
        }

        printf("OK cmd sent\n");
        return;
    }

        if(streq_ci(cmd,"cfg")){
        char *sub=strtok_r(NULL," \t",&save);
        if(!sub){ printf("ERR cfg get|set|save|load|defaults|apply\n"); return; }

        if(streq_ci(sub,"get")){
            char j[768];
            xSemaphoreTake(g_mutex, portMAX_DELAY);
            mr_cfg_to_json(&g_cfg, j, sizeof(j));
            xSemaphoreGive(g_mutex);
            printf("%s\n", j);
            return;
        }
        if(streq_ci(sub,"set")){
            char *k=strtok_r(NULL," \t",&save);
            char *v=strtok_r(NULL,"",&save); // rest of line
            if(!k || !v){ printf("ERR cfg set <key> <val>\n"); return; }
            while(*v==' '||*v=='\t') v++;
            bool ok=false;
            xSemaphoreTake(g_mutex, portMAX_DELAY);
            ok = mr_cfg_set_kv(&g_cfg, k, v);
            if(ok) mr_cfg_apply(&g_cfg);
            xSemaphoreGive(g_mutex);
            printf(ok ? "OK\n" : "ERR invalid\n");
            return;
        }
        if(streq_ci(sub,"save")){
            bool ok=false;
            xSemaphoreTake(g_mutex, portMAX_DELAY);
            ok = mr_cfg_save_nvs(&g_cfg);
            xSemaphoreGive(g_mutex);
            printf(ok ? "OK saved\n" : "ERR save\n");
            return;
        }
        if(streq_ci(sub,"load")){
            bool ok=false;
            xSemaphoreTake(g_mutex, portMAX_DELAY);
            ok = mr_cfg_load_nvs(&g_cfg);
            if(ok) mr_cfg_apply(&g_cfg);
            xSemaphoreGive(g_mutex);
            printf(ok ? "OK loaded\n" : "ERR load\n");
            return;
        }
        if(streq_ci(sub,"defaults")){
            xSemaphoreTake(g_mutex, portMAX_DELAY);
            mr_cfg_defaults(&g_cfg);
            // keep key from compile-time if you want:
            (void)parse_key_hex16(MR_NET_KEY_HEX, g_cfg.net_key);
            mr_cfg_apply(&g_cfg);
            xSemaphoreGive(g_mutex);
            printf("OK defaults applied\n");
            return;
        }
        if(streq_ci(sub,"apply")){
            xSemaphoreTake(g_mutex, portMAX_DELAY);
            mr_cfg_apply(&g_cfg);
            xSemaphoreGive(g_mutex);
            printf("OK applied\n");
            return;
        }

        printf("ERR cfg get|set|save|load|defaults|apply\n");
        return;
    }

    printf("ERR unknown. type: help\n");
}

static void cli_uart_init_once(void)
{
    static bool inited=false;
    if(inited) return;

    const int rx_buf = 1024;
    const int tx_buf = 0;

    esp_err_t e = uart_driver_install(UART_NUM_0, rx_buf, tx_buf, 0, NULL, 0);
    if(e != ESP_OK && e != ESP_ERR_INVALID_STATE){
        // ESP_LOGW(TAG, "uart_driver_install: %s", esp_err_to_name(e));
    }
    uart_set_rx_timeout(UART_NUM_0, 1);
    inited=true;
}

static void cli_task(void *arg)
{
    (void)arg;
    cli_uart_init_once();
    cli_print_help();

    printf("> "); fflush(stdout);

    char line[MR_CLI_LINE_MAX];
    int n=0;

    while(1){
        uint8_t c=0;
        int r = uart_read_bytes(UART_NUM_0, &c, 1, pdMS_TO_TICKS(50));
        if(r <= 0) continue;

        if(c == '\r' || c == '\n'){
            line[n]=0;
            printf("\n"); fflush(stdout);
            if(n>0) cli_handle_line(line);
            n=0;
            printf("> "); fflush(stdout);
            continue;
        }

        if(c == 0x08 || c == 0x7F){
            if(n>0){
                n--;
                printf("\b \b");
                fflush(stdout);
            }
            continue;
        }

        if(isprint((int)c)){
            if(n < (MR_CLI_LINE_MAX-1)){
                line[n++] = (char)c;
                putchar((int)c);
                fflush(stdout);
            }
        }
    }
}
#endif // MR_CLI_ENABLE

// ============================================================================
// =========================== POWERSAVE / SLEEP ==============================
// ============================================================================

#if MR_POWERSAVE_ENABLE

#if defined(MR_LORA_CHIP_SX1262)
// SX126x sleep opcode
#define SX126X_SET_SLEEP 0x84

static void sx126x_set_sleep(bool warm_start)
{
    // Sleep config:
    // bit2: warm start (keep config) if 1, cold start if 0
    // rest: 0
    uint8_t cfg = warm_start ? 0x04 : 0x00;
    uint8_t cmd[2] = { SX126X_SET_SLEEP, cfg };

    // Important: wait BUSY before, then send, then don't wait forever after.
    sx126x_busy_wait();
    sx126x_xfer(cmd, NULL, sizeof(cmd)); // rx can be NULL for polling_transmit? safer to use dummy:
    // If your spi driver requires rx, use a dummy buffer:
    // uint8_t rx_dummy[2]; sx126x_xfer(cmd, rx_dummy, sizeof(cmd));

    // After SetSleep the chip won't respond; BUSY may drop.
    esp_rom_delay_us(200);
}
#endif

#if defined(MR_LORA_CHIP_SX1276)
static void sx1276_set_sleep(void)
{
    // SX1276 sleep: LoRa mode + sleep
    // Your existing code uses 0x80 for sleep already.
    lora_spi_lock();
    sx1276_wr(SX1276_REG_OP_MODE, 0x80);
    lora_spi_unlock();
    esp_rom_delay_us(200);
}
#endif

static void board_prepare_for_deep_sleep(void)
{
#if (MR_BOARD_PRESET == MR_BOARD_HELTEC_V3)
    // 1) Battery divider OFF
#ifdef BATT_EN_GPIO
#if BATT_EN_ACTIVE_LOW
    gpio_set_level((gpio_num_t)BATT_EN_GPIO, 1); // OFF
#else
    gpio_set_level((gpio_num_t)BATT_EN_GPIO, 0); // OFF
#endif
    gpio_hold_en((gpio_num_t)BATT_EN_GPIO);
#endif

    // 2) VEXT OFF (Heltec: LOW=ON, HIGH=OFF)
    gpio_set_level((gpio_num_t)VEXT_CTRL_PIN, 1); // OFF
    gpio_hold_en((gpio_num_t)VEXT_CTRL_PIN);

    // Hold lines during deep sleep (prevents floating)
    gpio_deep_sleep_hold_en();
#endif
}

static void lora_prepare_for_deep_sleep(void)
{
#if defined(MR_LORA_CHIP_SX1262)
    // Warm start keeps config; cold start is also ok because we re-init after wake anyway.
    // Warm start tends to reduce weird states on some boards.
    lora_spi_lock();
    sx126x_set_sleep(true);
    lora_spi_unlock();
#elif defined(MR_LORA_CHIP_SX1276)
    sx1276_set_sleep();
#endif
}

// Main helper: call this when you want to go to sleep for SENSOR_WAKE_PERIOD_MS
static void mr_enter_deep_sleep(uint32_t sleep_ms)
{
    // Stop WiFi/HTTP if it was enabled (optional but recommended)
    if(g_wifi_enabled){
        set_wifi_enabled(false);
    }

    // Put radio to sleep (prevents BUSY/IRQ oddities, lowers current)
    lora_prepare_for_deep_sleep();

    // Board rails off (Heltec VEXT etc.)
    rtc_display_enable = g_cfg.display_enable ? 1 : 0;

    board_prepare_for_deep_sleep();

    // Wake timer
    esp_sleep_enable_timer_wakeup((uint64_t)sleep_ms * 1000ULL);

    // Go!
    esp_deep_sleep_start();
}


#endif // MR_POWERSAVE_ENABLE


// ============================================================================
// ================================= MAIN ====================================
// ============================================================================
// ============================================================================
// app_main() – clean & robust (RELAY/EDGE Dauerbetrieb, SENSOR optional PowerSave)
// ----------------------------------------------------------------------------
// Was dieses app_main() macht:
//
// 1) Hardware/Peripherie init (wie bei dir funktionierend)
// 2) Tasks starten: IRQ-RX, Retry(ACK), CLI
// 3) Scheduler initialisieren (Beacon/RouteAdv Timer)
// 4) SENSOR-Spezialfall:
//      - direkt nach Boot: "SENSOR:AWAKE" senden (am besten mit ACKREQ)
//      - ein RX-Fenster offen halten, damit Downlink-Kommandos (z.B. CMD:RELAY ON)
//        ankommen können
//      - falls MR_POWERSAVE_ENABLE: danach DeepSleep und später wieder aufwachen
//
// WICHTIG:
// - Damit "AWAKE" zuverlässig ist: send_data_to(..., ackreq=true) + pending_add im send_data_to()
// - Damit Downlink sicher ankommt: RX-Fenster ausreichend groß wählen (z.B. 5–15s)
// - RELAY/EDGE: keine Sleep-Logik, laufen normal weiter
// ============================================================================



// Forward Decl (falls du es schon woanders hast: ok, sonst hier belassen)
//static void sensor_awake_window(uint32_t ms);      // hält RX-Fenster "aktiv" (nur delay-loop)
//static void mr_enter_deep_sleep(uint32_t ms);      // geht schlafen (esp_sleep_enable_timer_wakeup + start)

// Minimal-Implementierung, falls du KEINE eigene hast.
// Wenn du bereits eigene Versionen hast: diese beiden entfernen.
/*static void sensor_awake_window(uint32_t ms)
{
    uint32_t t0 = now_ms();
    while((now_ms() - t0) < ms){
        // RX wird über IRQ/DIO Task verarbeitet; hier nur Zeit verstreichen lassen
        vTaskDelay(pdMS_TO_TICKS(20));
#if MR_BATT_ENABLE
        batt_poll();
#endif
    }
}
*/
/*static void mr_enter_deep_sleep(uint32_t ms)
{
    // Timer wakeup
    esp_sleep_enable_timer_wakeup((uint64_t)ms * 1000ULL);

    // Optional: vor Sleep Funk aus (spart Strom, vermeidet "komische" Zustände)
    // (Wenn du dafür eine Funktion hast, nutze sie – sonst weglassen.)
    // lora_sleep(); // falls implementiert

    ESP_LOGI(TAG, "SENSOR: entering deep sleep for %" PRIu32 " ms", ms);
    esp_deep_sleep_start();
}
*/
void app_main(void)
{
    ESP_LOGI(TAG,
             "MeshRadio start (ESP-IDF 5.5.2) – SX1262/SX1276 switchable @ %lu Hz",
             (unsigned long)DEFAULT_RF_FREQ_HZ);

    // ------------------------------------------------------------------------
    // 1) Board Power / VEXT / Battery Divider init (Heltec)
    // ------------------------------------------------------------------------
#if (MR_BOARD_PRESET == MR_BOARD_HELTEC_V3)
    board_power_boot_init();
#endif

#if (MR_BOARD_PRESET == MR_BOARD_HELTEC_V3)
#ifdef BATT_EN_GPIO
    gpio_deep_sleep_hold_dis();
    gpio_hold_dis((gpio_num_t)BATT_EN_GPIO);
#endif
#endif
#if (MR_BOARD_PRESET == MR_BOARD_HELTEC_V3)
    gpio_deep_sleep_hold_dis();
#ifdef BATT_EN_GPIO
    gpio_hold_dis((gpio_num_t)BATT_EN_GPIO);
#endif
    gpio_hold_dis((gpio_num_t)VEXT_CTRL_PIN);
#endif

    mr_wifi_ota_confirm_running_image();

#if (MR_BOARD_PRESET == MR_BOARD_TBEAM_V11_SX1276) || \
    (MR_BOARD_PRESET == MR_BOARD_TBEAM_V12_AXP2101)
    mr_pmu_init();
    mr_gps_init();
    pwr_button_init();
#endif

    // ------------------------------------------------------------------------
    // 2) Mutex / NVS / Token Buckets / Key
    // ------------------------------------------------------------------------
    g_mutex = xSemaphoreCreateMutex();
    if(!g_mutex){ ESP_LOGE(TAG,"mutex failed"); abort(); }

    g_lora_spi_mutex = xSemaphoreCreateMutex();
    if(!g_lora_spi_mutex){ ESP_LOGE(TAG,"lora spi mutex failed"); abort(); }

    ESP_ERROR_CHECK(nvs_flash_init());
    aprs_web_init();

    mr_init_msg_seq_from_rtc();

    bucket_init(&b_beacon,   RL_BEACON_TPS,   RL_BEACON_BURST);
    bucket_init(&b_routeadv, RL_ROUTEADV_TPS, RL_ROUTEADV_BURST);
    bucket_init(&b_ack,      RL_ACK_TPS,      RL_ACK_BURST);
    bucket_init(&b_data,     RL_DATA_TPS,     RL_DATA_BURST);

    if(!parse_key_hex16(MR_NET_KEY_HEX, g_net_key)){
        ESP_LOGE(TAG,"Invalid MR_NET_KEY_HEX – need 32 hex chars");
        abort();
    }

    // ------------------------------------------------------------------------
    // 2b) Config init + NVS load
    // ------------------------------------------------------------------------
    mr_cfg_defaults(&g_cfg);

    if(!parse_key_hex16(MR_NET_KEY_HEX, g_cfg.net_key)){
        ESP_LOGE(TAG,"Invalid MR_NET_KEY_HEX – need 32 hex chars");
        abort();
    }

    if(mr_cfg_load_nvs(&g_cfg)){
        ESP_LOGI(TAG,"Config loaded from NVS");
    }else{
        ESP_LOGI(TAG,"No NVS config, using defaults");
    }

    rtc_display_enable = g_cfg.display_enable ? 1 : 0;

    // ------------------------------------------------------------------------
    // 3) Station Addons (Relay GPIO, Battery ADC)
    // ------------------------------------------------------------------------
#if MR_RELAY_ENABLE
    relay_init();
    gpio_deep_sleep_hold_dis();
    gpio_hold_dis((gpio_num_t)g_relay_gpio_runtime);
#endif

#if MR_BATT_ENABLE
    batt_init();
#endif

#if MR_BATT_ENABLE
#if (MR_BOARD_PRESET == MR_BOARD_HELTEC_V3)
    batt_force_read_once();
#endif
#endif


// ------------------------------------------------------------------------
// Runtime-Config zuerst anwenden
// ------------------------------------------------------------------------

    g_rf_freq_hz_runtime   = g_cfg.rf_freq_hz;
    g_tx_power_dbm_runtime = g_cfg.tx_power_dbm;
    g_relay_gpio_runtime   = g_cfg.relay_gpio;

    mr_cfg_apply(&g_cfg);

    config_print();

#if (MR_BOARD_PRESET == MR_BOARD_HELTEC_V3)
    prog_button_init();
#endif

#if MR_DISPLAY_ENABLE
    if(g_display_enabled){
        mr_display_show_boot(MR_BOARD_NAME, "DISPLAY OK");
    }
#endif

// ------------------------------------------------------------------------
// 4) LoRa init (SPI + Chip)
// ------------------------------------------------------------------------

    init_spi();
    lora_hw_reset();
    lora_log_chip_info();
    lora_init_radio();

#if MR_RELAY_ENABLE
    // falls relay_gpio aus NVS geladen wurde und sich geändert hat:
    gpio_hold_dis((gpio_num_t)g_relay_gpio_runtime);
#endif

#if MR_BME280_ENABLE
        if(g_bme280_enable){
            (void)bme280_init();
        }
#endif

    // IRQ line -> dio_task() -> handle_rx()
    init_dio_irq();

    // Retry-Task für ACKREQ/Resend
    xTaskCreate(retry_task, "retry", 4096, NULL, 6, NULL);

#if MR_CLI_ENABLE
    xTaskCreate(cli_task, "cli", 4096, NULL, 5, NULL);
#endif

    // ------------------------------------------------------------------------
    // 5) WiFi / HTTP optional
    // ------------------------------------------------------------------------
    if(g_wifi_enabled){
        wifi_start_ap();
        http_start_if_needed();
        ESP_LOGI(TAG,"Open http://192.168.4.1");
    }else{
        ESP_LOGI(TAG,"WiFi/HTTP disabled at boot. Use CLI: wifi on");
    }

    // ------------------------------------------------------------------------
    // 6) Scheduler starten
    // ------------------------------------------------------------------------
    beacon_schedule_next();
    routeadv_schedule_next();

    ESP_LOGI(TAG,
             "CALL=%s  mode=%s  crypto=%u  net_id=0x%02X  wifi=%u",
             g_callsign_rt, node_mode_str(g_node_mode),
             g_crypto_enable?1:0,
             (unsigned)MR_NET_ID,
             g_wifi_enabled?1:0);

    // ------------------------------------------------------------------------
    // 7) SENSOR: Boot-Aktion
    // ------------------------------------------------------------------------
    if(g_node_mode == NODE_SENSOR){

    lora_set_rx_continuous();
    vTaskDelay(pdMS_TO_TICKS(150));

uint32_t batt_mv = 0, batt_pct = 0;

#if MR_BATT_ENABLE
#if (MR_BOARD_PRESET == MR_BOARD_HELTEC_V3)
    uint32_t batt_wait_start = now_ms();

    while((now_ms() - batt_wait_start) < 4500){
        g_next_batt_ms = 0;
        batt_poll();

        xSemaphoreTake(g_mutex, portMAX_DELAY);
        batt_mv  = g_batt_mv;
        batt_pct = g_batt_pct;
        xSemaphoreGive(g_mutex);

        if(batt_mv > 0){
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }

    if(batt_mv == 0){
        vTaskDelay(pdMS_TO_TICKS(1000));
        g_next_batt_ms = 0;
        batt_poll();

        xSemaphoreTake(g_mutex, portMAX_DELAY);
        batt_mv  = g_batt_mv;
        batt_pct = g_batt_pct;
        xSemaphoreGive(g_mutex);
    }
#else
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    batt_mv  = g_batt_mv;
    batt_pct = g_batt_pct;
    xSemaphoreGive(g_mutex);
#endif
#endif

#if MR_DISPLAY_ENABLE
    if(g_display_enabled){
        mr_display_show_status("MeshRadio",
                               cfg_board_str(),
                               cfg_lora_chip_str(),
                               g_callsign_rt);
    }
#endif

    char wx[160];
    strcpy(wx, "SENSOR:AWAKE");

#if MR_BME280_ENABLE
        if(g_bme280_enable){
            int32_t t_x100 = 0;
            uint32_t p_pa = 0;
            uint32_t rh_x1000 = 0;
            //mr_display_sleep(false); // Display an, damit BME280 Werte angezeigt werden können (I2C-Bus aktiv)
            //mr_display_init();

            if(bme280_read_weather(&t_x100, &p_pa, &rh_x1000)){
                int32_t t_int  = t_x100 / 100;
                int32_t t_frac = llabs(t_x100 % 100);

                int32_t p_hpa = (int32_t)(p_pa / 100U) + (int32_t)BME_PRESS_OFFSET_HPA;
                if(p_hpa < 300) p_hpa = 300;
                if(p_hpa > 1100) p_hpa = 1100;

                uint32_t rh_int  = rh_x1000 / 1000U;
                uint32_t rh_frac = (rh_x1000 % 1000U) / 10U;

               snprintf(wx, sizeof(wx),
                    "SENSOR:AWAKE WX t=%" PRId32 ".%02" PRId32 "C "
                    "p=%" PRIu32 "hPa "
                    "rh=%" PRIu32 ".%02" PRIu32 "%% "
                    "bat=%" PRIu32 "mV "
                    "bat=%" PRIu32 "%%",
                    t_int, t_frac,
                    p_hpa,
                    rh_int, rh_frac,
                    batt_mv,
                    batt_pct);
                    #if MR_DISPLAY_ENABLE
                        if(g_display_enabled){
                            
                            char l0[22], l1[22], l2[22], l3[22], l4[22], l5[22], l6[22], l7[22];

                            snprintf(l0, sizeof(l0), "SENSOR AWAKE");
                            snprintf(l1, sizeof(l1), "%s", g_callsign_rt);
                            snprintf(l2, sizeof(l2), "Temp: %" PRId32 ".%02" PRId32 " C", t_int, t_frac);
                            snprintf(l3, sizeof(l3), "Press:%" PRId32 " hPa", p_hpa);
                            snprintf(l4, sizeof(l4), "Hum:  %" PRIu32 ".%02" PRIu32 " %%", rh_int, rh_frac);
                            snprintf(l5, sizeof(l5), "Batt: %" PRIu32 " mV", batt_mv);
                            snprintf(l6, sizeof(l6), "Batt: %" PRIu32 " %%", batt_pct);
                            snprintf(l7, sizeof(l7), "WX OK");

                            mr_display_show_status8(l0, l1, l2, l3, l4, l5, l6, l7);
                        }
                    #endif

            }else{
                ESP_LOGE(TAG, "BME280 read failed, bme_ok=%d", bme280_is_ok() ? 1 : 0);
                snprintf(wx, sizeof(wx), "SENSOR:AWAKE WX ERR");
            
            #if MR_DISPLAY_ENABLE
                if(g_display_enabled){
                    mr_display_show_status8(
                        "SENSOR AWAKE",
                        g_callsign_rt,
                        "WX ERROR",
                        bme280_is_ok() ? "BME read fail" : "BME init fail",
                        "",
                        "Check sensor",
                        "and I2C bus",
                        ""
                    );
                }
            #endif
            }
        }
#endif

send_data_to(g_relay_callsign_rt, wx, false);
sensor_awake_window(SENSOR_BOOT_RX_WINDOW_MS);

#if MR_POWERSAVE_ENABLE
        if(g_powersave_enable){
    #if MR_RELAY_ENABLE
            gpio_hold_en((gpio_num_t)g_relay_gpio_runtime);
            gpio_deep_sleep_hold_en();
    #endif

            // ---- Pending Retry Queue löschen ----
            xSemaphoreTake(g_mutex, portMAX_DELAY);
            pending_clear_all_locked();
            xSemaphoreGive(g_mutex);

            ESP_LOGI(TAG, "SENSOR: powersave runtime=ON -> deep sleep");
            mr_enter_deep_sleep(g_cfg.sensor_wake_period_ms);
        }else{
            ESP_LOGI(TAG, "SENSOR: powersave runtime=OFF -> staying awake");
        }
#else
        ESP_LOGI(TAG, "SENSOR: powersave compile-time disabled -> staying awake");
#endif
    }

    config_print();

    //char dt[20] = "";
    //mr_time_net_get_datetime(dt, sizeof(dt));

    

    // ------------------------------------------------------------------------
    // 8) Main Loop
    // ------------------------------------------------------------------------
    while(1){
        vTaskDelay(pdMS_TO_TICKS(50));

#if MR_BATT_ENABLE
        batt_poll();
        vTaskDelay(pdMS_TO_TICKS(50));
        batt_poll();
#endif

#if (MR_BOARD_PRESET == MR_BOARD_TBEAM_V11_SX1276) || \
    (MR_BOARD_PRESET == MR_BOARD_TBEAM_V12_AXP2101)
        mr_gps_poll();
        gps_send_periodic();
#endif
        aprs_gateway_send_periodic();

#if (MR_BOARD_PRESET == MR_BOARD_TBEAM_V11_SX1276) || \
    (MR_BOARD_PRESET == MR_BOARD_TBEAM_V12_AXP2101)
        {
            static uint32_t last_gps_ui_ms = 0;
            uint32_t tnow = now_ms();

            if((tnow - last_gps_ui_ms) >= 1000){
                last_gps_ui_ms = tnow;

                char gps_txt[64];
                char nmea_dbg[128];

                mr_gps_get_text(gps_txt, sizeof(gps_txt));
                mr_gps_get_nmea_debug(nmea_dbg, sizeof(nmea_dbg));

                //ESP_LOGI("GPS", "%s", gps_txt);
                //ESP_LOGI("GPSRAW", "%s", nmea_dbg);

                /*mr_display_clear();
                mr_display_show_status8(
                    "GPS TEST",
                    mr_board_name(),
                    gps_txt,
                    "",
                    "",
                    "",
                    "",
                    ""
                );*/
            }
        }
#endif

       #if ((MR_BOARD_PRESET == MR_BOARD_TBEAM_V11_SX1276) || \
        (MR_BOARD_PRESET == MR_BOARD_TBEAM_V12_AXP2101))
        pwr_button_poll();
        #endif

#if (MR_BOARD_PRESET == MR_BOARD_HELTEC_V3)
        prog_button_poll();
#endif

        if(g_beacon_enabled && g_node_mode != NODE_SENSOR && now_ms() > g_next_beacon_ms){
            send_beacon();
        }

        xSemaphoreTake(g_mutex, portMAX_DELAY);
        neighbor_cleanup_locked();
        route_cleanup_locked();
        xSemaphoreGive(g_mutex);
    }
}
