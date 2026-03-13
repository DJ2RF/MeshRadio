#include "incl/mr_wifi_sta.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_err.h"
#include "esp_check.h"

static const char *TAG = "MR_WIFI_STA";

static esp_netif_t *s_sta_netif = NULL;
static EventGroupHandle_t s_wifi_ev = NULL;

static bool s_wifi_inited = false;
static bool s_connected = false;
static char s_hostname[32] = {0};

static mr_wifi_sta_cfg_t s_cfg = {0};
static mr_wifi_sta_state_t s_state = MR_WIFI_STA_IDLE;
static uint8_t s_retry_count = 0;

extern void wifi_enable_ap_part(void);
extern void wifi_disable_ap_part(void);

#define MR_WIFI_CONNECTED_BIT BIT0

static void mr_wifi_sta_connect_now(void)
{
    if (s_cfg.ssid[0] == 0) {
        s_state = MR_WIFI_STA_FAILED;
        ESP_LOGW(TAG, "No STA SSID configured");
        return;
    }

    esp_err_t err = esp_wifi_connect();
    if (err == ESP_OK || err == ESP_ERR_WIFI_CONN) {
        s_state = MR_WIFI_STA_CONNECTING;
        ESP_LOGI(TAG, "STA connecting to '%s'", s_cfg.ssid);
    } else {
        s_state = MR_WIFI_STA_FAILED;
        ESP_LOGW(TAG, "esp_wifi_connect failed: %s", esp_err_to_name(err));
    }
}

static void mr_wifi_sta_schedule_reconnect(void)
{
    if (s_retry_count >= s_cfg.retry_max) {
        s_state = MR_WIFI_STA_FAILED;
        ESP_LOGW(TAG, "STA retry limit reached (%u)", (unsigned)s_cfg.retry_max);
        wifi_enable_ap_part();
        return;
    }

    s_retry_count++;
    s_state = MR_WIFI_STA_BACKOFF;

    wifi_enable_ap_part();

    uint32_t wait_ms = s_cfg.retry_backoff_ms * s_retry_count;
    ESP_LOGW(TAG, "STA reconnect in %lu ms (%u/%u)",
             (unsigned long)wait_ms,
             (unsigned)s_retry_count,
             (unsigned)s_cfg.retry_max);

    vTaskDelay(pdMS_TO_TICKS(wait_ms));
    mr_wifi_sta_connect_now();
}

static void mr_wifi_event_handler(void *arg,
                                  esp_event_base_t event_base,
                                  int32_t event_id,
                                  void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "STA start event");
                s_retry_count = 0;
                mr_wifi_sta_connect_now();
                break;

            case WIFI_EVENT_STA_DISCONNECTED:
                s_connected = false;
                if (s_wifi_ev) {
                    xEventGroupClearBits(s_wifi_ev, MR_WIFI_CONNECTED_BIT);
                }

                ESP_LOGW(TAG, "STA disconnected");
                mr_wifi_sta_schedule_reconnect();
                break;

            default:
                break;
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t *ev = (ip_event_got_ip_t *)event_data;

            s_connected = true;
            s_retry_count = 0;
            s_state = MR_WIFI_STA_CONNECTED;

            if (s_wifi_ev) {
                xEventGroupSetBits(s_wifi_ev, MR_WIFI_CONNECTED_BIT);
            }

            ESP_LOGI(TAG, "STA got IP: " IPSTR, IP2STR(&ev->ip_info.ip));
            ESP_LOGI(TAG, "DHCP hostname: %s", s_hostname[0] ? s_hostname : "(disabled)");

            if (s_cfg.hide_ap_when_sta_up) {
                wifi_disable_ap_part();
            }
        }
    }
}

esp_err_t mr_wifi_build_hostname(char *out, size_t out_sz)
{
    if (!out || out_sz < 16) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t mac[6] = {0};
    esp_err_t err = esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (err != ESP_OK) {
        return err;
    }

    snprintf(out, out_sz, "MeshRadio-%02X", mac[5]);
    return ESP_OK;
}

esp_err_t mr_wifi_sta_init_once(void)
{
    if (s_wifi_inited) {
        return ESP_OK;
    }

    s_sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!s_sta_netif) {
        ESP_LOGE(TAG, "STA netif not found");
        return ESP_FAIL;
    }

    if (!s_wifi_ev) {
        s_wifi_ev = xEventGroupCreate();
        if (!s_wifi_ev) {
            ESP_LOGE(TAG, "Failed to create wifi event group");
            return ESP_ERR_NO_MEM;
        }
    }

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &mr_wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &mr_wifi_event_handler, NULL));

    s_wifi_inited = true;
    s_state = MR_WIFI_STA_IDLE;
    return ESP_OK;
}

static esp_err_t mr_wifi_sta_apply_cfg(const mr_wifi_sta_cfg_t *cfg)
{
    if (!cfg || cfg->ssid[0] == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(&s_cfg, 0, sizeof(s_cfg));
    memcpy(&s_cfg, cfg, sizeof(s_cfg));

    memset(s_hostname, 0, sizeof(s_hostname));
    if (s_cfg.use_dhcp_hostname) {
        ESP_RETURN_ON_ERROR(mr_wifi_build_hostname(s_hostname, sizeof(s_hostname)),
                            TAG, "build hostname failed");

        ESP_RETURN_ON_ERROR(esp_netif_set_hostname(s_sta_netif, s_hostname),
                            TAG, "set hostname failed");
    }

    wifi_config_t wc = {0};
    strncpy((char *)wc.sta.ssid, s_cfg.ssid, sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, s_cfg.pass, sizeof(wc.sta.password) - 1);

    wc.sta.threshold.authmode = WIFI_AUTH_OPEN;
    wc.sta.pmf_cfg.capable = true;
    wc.sta.pmf_cfg.required = false;

    esp_err_t err = esp_wifi_disconnect();
    if (err != ESP_OK && err != ESP_ERR_WIFI_NOT_CONNECT) {
        ESP_LOGW(TAG, "esp_wifi_disconnect: %s", esp_err_to_name(err));
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    err = esp_wifi_set_config(WIFI_IF_STA, &wc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config failed: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

esp_err_t mr_wifi_sta_start(const mr_wifi_sta_cfg_t *cfg)
{
    ESP_RETURN_ON_ERROR(mr_wifi_sta_init_once(), TAG, "wifi init failed");
    ESP_RETURN_ON_ERROR(mr_wifi_sta_apply_cfg(cfg), TAG, "apply sta cfg failed");

    esp_err_t err = esp_wifi_start();
    if (err != ESP_OK && err != ESP_ERR_WIFI_CONN && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(err));
        return err;
    }

    s_connected = false;
    s_retry_count = 0;
    s_state = MR_WIFI_STA_CONNECTING;

    ESP_LOGI(TAG, "STA start requested for SSID '%s'", s_cfg.ssid);

    err = esp_wifi_connect();
    if (err != ESP_OK && err != ESP_ERR_WIFI_CONN) {
        ESP_LOGE(TAG, "esp_wifi_connect failed: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

esp_err_t mr_wifi_sta_reconfigure(const mr_wifi_sta_cfg_t *cfg)
{
    ESP_RETURN_ON_ERROR(mr_wifi_sta_init_once(), TAG, "wifi init failed");
    ESP_RETURN_ON_ERROR(mr_wifi_sta_apply_cfg(cfg), TAG, "apply sta cfg failed");

    s_connected = false;
    s_retry_count = 0;
    s_state = MR_WIFI_STA_CONNECTING;

    ESP_LOGI(TAG, "STA reconfigure requested for SSID '%s'", s_cfg.ssid);

    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK && err != ESP_ERR_WIFI_CONN) {
        ESP_LOGE(TAG, "esp_wifi_connect failed: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

esp_err_t mr_wifi_sta_stop(void)
{
    if (!s_wifi_inited) {
        return ESP_OK;
    }

    s_connected = false;
    s_retry_count = 0;
    s_state = MR_WIFI_STA_IDLE;

    if (s_wifi_ev) {
        xEventGroupClearBits(s_wifi_ev, MR_WIFI_CONNECTED_BIT);
    }

    return esp_wifi_disconnect();
}

bool mr_wifi_sta_is_connected(void)
{
    return s_connected;
}

mr_wifi_sta_state_t mr_wifi_sta_get_state(void)
{
    return s_state;
}

const char *mr_wifi_sta_get_hostname(void)
{
    return s_hostname;
}