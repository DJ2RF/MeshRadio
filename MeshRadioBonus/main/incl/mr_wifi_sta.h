#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MR_WIFI_STA_IDLE = 0,
    MR_WIFI_STA_CONNECTING,
    MR_WIFI_STA_CONNECTED,
    MR_WIFI_STA_BACKOFF,
    MR_WIFI_STA_FAILED
} mr_wifi_sta_state_t;

typedef struct {
    char ssid[33];
    char pass[65];
    bool use_dhcp_hostname;
    uint8_t retry_max;
    uint32_t retry_backoff_ms;
    bool hide_ap_when_sta_up;
} mr_wifi_sta_cfg_t;

esp_err_t mr_wifi_build_hostname(char *out, size_t out_sz);

esp_err_t mr_wifi_sta_init_once(void);
esp_err_t mr_wifi_sta_start(const mr_wifi_sta_cfg_t *cfg);
esp_err_t mr_wifi_sta_stop(void);
esp_err_t mr_wifi_sta_reconfigure(const mr_wifi_sta_cfg_t *cfg);

bool mr_wifi_sta_is_connected(void);
mr_wifi_sta_state_t mr_wifi_sta_get_state(void);
const char *mr_wifi_sta_get_hostname(void);

#ifdef __cplusplus
}
#endif