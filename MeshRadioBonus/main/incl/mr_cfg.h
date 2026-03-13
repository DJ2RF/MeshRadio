#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // Identity / Mesh
    char     callsign[9];         // 8 chars + 0
    char     relay_callsign[9];   // target for sensor awake
    uint8_t  net_id;
    uint8_t  net_key[16];
    bool     crypto_enable;

    // Node role / behavior
    uint8_t  node_mode;           // 0 RELAY / 1 EDGE / 2 SENSOR
    bool     beacon_enable;
    uint32_t beacon_interval_ms;

    bool     routeadv_enable;
    uint8_t  routeadv_topn;
    uint16_t routeadv_delta_etx;
    uint32_t holddown_ms;

    bool     cad_enable;

    // RF parameters
    uint32_t rf_freq_hz;
    int8_t   tx_power_dbm;        // keep it simple: -9..22 depending on chip/board

    // WiFi/Web
    bool     wifi_enable;
    char     wifi_ssid[32];
    char     wifi_pass[64];

    // Router WLAN (STA)
    char     wifi_sta_ssid[33];
    char     wifi_sta_pass[65];

    // Power save (sensor)
    bool     powersave_enable;
    uint32_t sensor_wake_period_ms;
    uint32_t sensor_boot_rx_window_ms;

    // Relay
    bool     relay_feature_enable;
    int8_t   relay_gpio;
    // Wetterdaten GPIO's Fest / Target
    bool bme280_enable;
} mr_cfg_t;

extern mr_cfg_t g_cfg;
extern char g_callsign_rt[9];
extern char g_relay_callsign_rt[9];

// lifecycle
void mr_cfg_defaults(mr_cfg_t *c);
bool mr_cfg_load_nvs(mr_cfg_t *c);
bool mr_cfg_save_nvs(const mr_cfg_t *c);

// apply
void mr_cfg_apply(const mr_cfg_t *c);

// setters helpers (validate+apply later)
bool mr_cfg_set_kv(mr_cfg_t *c, const char *key, const char *val);

// export json (for /api/cfg)
void mr_cfg_to_json(const mr_cfg_t *c, char *out, size_t out_sz);

#ifdef __cplusplus
}
#endif