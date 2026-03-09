#include "incl/mr_cfg.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include "incl/config_meshradio.h"
#include "incl/board_pins.h"
#include "incl/mr_sec_ccm.h"

// forward hooks (implemented in main)
extern void set_wifi_enabled(bool en);
extern void lora_recover_radio(void);     // you already have it
extern void lora_init_radio(void);
extern void lora_log_chip_info(void);
extern void lora_set_rx_continuous(void);


// these globals currently exist in your main; we’ll sync them in mr_cfg_apply():
extern bool g_crypto_enable;
extern bool g_beacon_enabled;
extern bool g_routeadv_enable;
extern bool g_cad_enable;
extern bool g_powersave_enable;
extern bool g_bme280_enable;

extern uint32_t g_beacon_interval_ms;
extern uint8_t  g_routeadv_topn;
extern uint16_t g_routeadv_delta_etx;
extern uint32_t g_holddown_ms;
extern uint8_t  g_net_key[16];
extern int8_t g_relay_gpio_runtime;
extern void relay_init(void);
// extern uint8_t  MR_NET_ID; // NOTE: if MR_NET_ID is macro, remove this extern

// you use this:

extern int g_node_mode;   // in main ist es node_mode_t; int passt ABI-mäßig
extern char g_callsign_rt[8];
extern char g_relay_callsign_rt[8];

// RF config is currently macros; we’ll store runtime in g_cfg and add a runtime var in main:
extern uint32_t g_rf_freq_hz_runtime;
extern int8_t   g_tx_power_dbm_runtime;

static const char *TAG="MR_CFG";
mr_cfg_t g_cfg;

static bool parse_bool(const char *v, bool *out){
    if(!v) return false;
    if(strcmp(v,"1")==0 || strcasecmp(v,"true")==0 || strcasecmp(v,"on")==0 || strcasecmp(v,"yes")==0){ *out=true; return true; }
    if(strcmp(v,"0")==0 || strcasecmp(v,"false")==0 || strcasecmp(v,"off")==0 || strcasecmp(v,"no")==0){ *out=false; return true; }
    return false;
}
static bool parse_u32(const char *v, uint32_t *out){
    if(!v || !*v) return false;
    char *e=NULL; unsigned long x=strtoul(v,&e,10);
    if(!e || *e) return false;
    *out=(uint32_t)x; return true;
}
static bool parse_i8(const char *v, int8_t *out){
    if(!v || !*v) return false;
    char *e=NULL; long x=strtol(v,&e,10);
    if(!e || *e) return false;
    if(x < -128 || x > 127) return false;
    *out=(int8_t)x; return true;
}
static void str7_sanitize(char *s){
    // keep max 7 printable non-space; uppercase optional
    if(!s) return;
    size_t n=strlen(s);
    if(n>7) s[7]=0;
    for(size_t i=0;i<strlen(s);i++){
        unsigned char c=(unsigned char)s[i];
        if(c<33 || c>126){ s[i]='_'; }
    }
}

void mr_cfg_defaults(mr_cfg_t *c)
{
    memset(c,0,sizeof(*c));

    strncpy(c->callsign, MR_CALLSIGN, sizeof(c->callsign)-1);
    strncpy(c->relay_callsign, MR_RELAY_CALLSIGN, sizeof(c->relay_callsign)-1);

    c->net_id = MR_NET_ID;

    if(!parse_key_hex16(MR_NET_KEY_HEX, c->net_key)){
        ESP_LOGE(TAG, "Invalid MR_NET_KEY_HEX – need 32 hex chars");
    }

    c->crypto_enable = (DEFAULT_CRYPTO_ENABLE ? true:false);

    c->node_mode = (uint8_t)DEFAULT_NODE_MODE;

    c->beacon_enable = true;
    c->beacon_interval_ms = DEFAULT_BEACON_INTERVAL_MS;

    c->routeadv_enable = (DEFAULT_ROUTEADV_ENABLE ? true:false);
    c->routeadv_topn = DEFAULT_ROUTEADV_TOPN;
    c->routeadv_delta_etx = DEFAULT_ROUTEADV_DELTA_ETX;
    c->holddown_ms = DEFAULT_HOLDDOWN_MS;

    c->cad_enable = (DEFAULT_CAD_ENABLE ? true:false);

    c->rf_freq_hz = DEFAULT_RF_FREQ_HZ;
    c->tx_power_dbm = 2;

    c->wifi_enable = (MR_WIFI_RUNTIME_DEFAULT ? true:false);
    strncpy(c->wifi_ssid, MR_WIFI_AP_SSID, sizeof(c->wifi_ssid)-1);
    strncpy(c->wifi_pass, MR_WIFI_AP_PASS, sizeof(c->wifi_pass)-1);

    c->powersave_enable = false;
    c->sensor_wake_period_ms = SENSOR_WAKE_PERIOD_MS;
    c->sensor_boot_rx_window_ms = SENSOR_BOOT_RX_WINDOW_MS;
    c->bme280_enable = false; //(MR_BME280_ENABLE ? true : false);

    c->relay_feature_enable = (MR_RELAY_ENABLE ? true:false);
    c->relay_gpio = RELAY_GPIO;
}

bool mr_cfg_load_nvs(mr_cfg_t *c)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open("mr_cfg", NVS_READONLY, &h);
    if(e != ESP_OK) return false;

    size_t len = sizeof(*c);
    e = nvs_get_blob(h, "cfg", c, &len);
    nvs_close(h);
    if(e != ESP_OK || len != sizeof(*c)) return false;

    // minimal sanity
    c->callsign[7]=0; c->relay_callsign[7]=0;
    str7_sanitize(c->callsign);
    str7_sanitize(c->relay_callsign);
    if(c->rf_freq_hz < 100000000UL || c->rf_freq_hz > 1000000000UL) c->rf_freq_hz = DEFAULT_RF_FREQ_HZ;
    if(c->beacon_interval_ms < 1000) c->beacon_interval_ms = 1000;
    if(c->routeadv_topn > 32) c->routeadv_topn = 8;

    return true;
}

bool mr_cfg_save_nvs(const mr_cfg_t *c)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open("mr_cfg", NVS_READWRITE, &h);
    if(e != ESP_OK) return false;

    e = nvs_set_blob(h, "cfg", c, sizeof(*c));
    if(e == ESP_OK) e = nvs_commit(h);

    nvs_close(h);
    return (e == ESP_OK);
}

void mr_cfg_apply(const mr_cfg_t *c)
{
    // Prüfen ob RF geändert wurde
    bool rf_changed =
        (g_rf_freq_hz_runtime != c->rf_freq_hz) ||
        (g_tx_power_dbm_runtime != c->tx_power_dbm);

    bool relay_gpio_changed =
        (g_relay_gpio_runtime != c->relay_gpio);

    // ---- Callsigns ----
    strncpy(g_callsign_rt, c->callsign, 7);
    g_callsign_rt[7] = 0;

    strncpy(g_relay_callsign_rt, c->relay_callsign, 7);
    g_relay_callsign_rt[7] = 0;

    // ---- Crypto ----
    g_crypto_enable = c->crypto_enable;
    memcpy(g_net_key, c->net_key, SEC_KEY_LEN);

    // Wetterdaten BME280
    g_bme280_enable = c->bme280_enable;

    // ---- Node Mode ----
    if(c->node_mode <= 2)
        g_node_mode = (int)c->node_mode;

    // ---- Runtime Flags ----
    g_beacon_enabled   = c->beacon_enable;
    g_routeadv_enable  = c->routeadv_enable;
    g_cad_enable       = c->cad_enable;
    g_powersave_enable = c->powersave_enable;

    g_beacon_interval_ms  = c->beacon_interval_ms;
    g_routeadv_topn       = c->routeadv_topn;
    g_routeadv_delta_etx  = c->routeadv_delta_etx;
    g_holddown_ms         = c->holddown_ms;

    // ---- Relay GPIO ----
    g_relay_gpio_runtime = c->relay_gpio;

#if MR_RELAY_ENABLE
    if(relay_gpio_changed){
        ESP_LOGW(TAG,"Relay GPIO changed -> reinit (%d)", g_relay_gpio_runtime);
        relay_init();
    }
#endif

    // ---- RF Runtime ----
    g_rf_freq_hz_runtime   = c->rf_freq_hz;
    g_tx_power_dbm_runtime = c->tx_power_dbm;

    if(rf_changed){
        ESP_LOGW(TAG,
                 "RF changed -> reinit radio (freq=%lu Hz, pwr=%d dBm)",
                 (unsigned long)g_rf_freq_hz_runtime,
                 (int)g_tx_power_dbm_runtime);

        lora_recover_radio();
        lora_set_rx_continuous();
    }

    // ---- WiFi Runtime ----
    set_wifi_enabled(c->wifi_enable);
}

static bool parse_hex_key16(const char *hex, uint8_t out[16])
{
    if(!hex) return false;
    if(strlen(hex) < 32) return false;
    for(int i=0;i<16;i++){
        char a=hex[2*i], b=hex[2*i+1];
        int hi = (a>='0'&&a<='9')?a-'0':(a>='a'&&a<='f')?10+a-'a':(a>='A'&&a<='F')?10+a-'A':-1;
        int lo = (b>='0'&&b<='9')?b-'0':(b>='a'&&b<='f')?10+b-'a':(b>='A'&&b<='F')?10+b-'A':-1;
        if(hi<0||lo<0) return false;
        out[i]=(uint8_t)((hi<<4)|lo);
    }
    return true;
}

bool mr_cfg_set_kv(mr_cfg_t *c, const char *key, const char *val)
{
    if(!key || !val) return false;

    // --- booleans ---
    bool bv=false;
    if(strcmp(key,"crypto")==0){
        if(!parse_bool(val,&bv)) return false;
        c->crypto_enable=bv; return true;
    }
    if(strcmp(key,"wifi")==0){
        if(!parse_bool(val,&bv)) return false;
        c->wifi_enable=bv; return true;
    }
    if(strcmp(key,"beacon")==0){
        if(!parse_bool(val,&bv)) return false;
        c->beacon_enable=bv; return true;
    }
    if(strcmp(key,"routeadv")==0){
        if(!parse_bool(val,&bv)) return false;
        c->routeadv_enable=bv; return true;
    }
    if(strcmp(key,"cad")==0){
        if(!parse_bool(val,&bv)) return false;
        c->cad_enable=bv; return true;
    }
    if(strcmp(key,"powersave")==0){
        if(!parse_bool(val,&bv)) return false;
        c->powersave_enable=bv; return true;
    }
    if(strcmp(key, "bme280") == 0){
    c->bme280_enable = (atoi(val) != 0);
    return true;
    }
    // --- integers ---
    uint32_t u32=0;
    if(strcmp(key,"beacon_ms")==0){
        if(!parse_u32(val,&u32)) return false;
        if(u32 < 1000 || u32 > 3600000UL) return false;
        c->beacon_interval_ms=u32; return true;
    }
    if(strcmp(key,"rf_hz")==0){
        if(!parse_u32(val,&u32)) return false;
        if(u32 < 100000000UL || u32 > 1000000000UL) return false;
        c->rf_freq_hz=u32; return true;
    }
    if(strcmp(key,"holddown_ms")==0){
        if(!parse_u32(val,&u32)) return false;
        if(u32 < 0 || u32 > 600000UL) return false;
        c->holddown_ms=u32; return true;
    }
    if(strcmp(key,"sensor_wake_ms")==0){
        if(!parse_u32(val,&u32)) return false;
        if(u32 < 1000 || u32 > 86400000UL) return false;
        c->sensor_wake_period_ms=u32; return true;
    }
    if(strcmp(key,"sensor_rxwin_ms")==0){
        if(!parse_u32(val,&u32)) return false;
        if(u32 < 100 || u32 > 300000UL) return false;
        c->sensor_boot_rx_window_ms=u32; return true;
    }
    if(strcmp(key,"routeadv_topn")==0){
        if(!parse_u32(val,&u32)) return false;
        if(u32 > 32) return false;
        c->routeadv_topn=(uint8_t)u32; return true;
    }
    if(strcmp(key,"routeadv_delta")==0){
        if(!parse_u32(val,&u32)) return false;
        if(u32 > 2000) return false;
        c->routeadv_delta_etx=(uint16_t)u32; return true;
    }
    if(strcmp(key,"node_mode")==0){
        if(!parse_u32(val,&u32)) return false;
        if(u32 > 2) return false;
        c->node_mode=(uint8_t)u32; return true;
    }

    // --- i8 ---
    int8_t i8=0;
    if(strcmp(key,"tx_dbm")==0){
        if(!parse_i8(val,&i8)) return false;
        if(i8 < -9 || i8 > 22) return false;
        c->tx_power_dbm=i8; return true;
    }
    if(strcmp(key,"relay_gpio")==0){
        if(!parse_i8(val,&i8)) return false;
            if(i8 < 0 || i8 > 48) return false;
        c->relay_gpio = i8;
        return true;
    }
    // --- strings ---
    if(strcmp(key,"ssid")==0){
        if(strlen(val) >= sizeof(c->wifi_ssid)) return false;
        strcpy(c->wifi_ssid, val);
        return true;
    }
    if(strcmp(key,"pass")==0){
        if(strlen(val) >= sizeof(c->wifi_pass)) return false;
        strcpy(c->wifi_pass, val);
        return true;
    }
    if(strcmp(key,"callsign")==0){
        if(strlen(val) < 1 || strlen(val) > 7) return false;
        strncpy(c->callsign, val, 7); c->callsign[7]=0;
        str7_sanitize(c->callsign);
        return true;
    }
    if(strcmp(key,"relay_callsign")==0){
        if(strlen(val) < 1 || strlen(val) > 7) return false;
        strncpy(c->relay_callsign, val, 7); c->relay_callsign[7]=0;
        str7_sanitize(c->relay_callsign);
        return true;
    }
    if(strcmp(key,"netid")==0){
        if(!parse_u32(val,&u32)) return false;
        if(u32 > 255) return false;
        c->net_id=(uint8_t)u32;
        return true;
    }
    if(strcmp(key,"netkey")==0){
        uint8_t k[16];
        if(!parse_hex_key16(val,k)) return false;
        memcpy(c->net_key,k,16);
        return true;
    }

    return false;
}

void mr_cfg_to_json(const mr_cfg_t *c, char *out, size_t out_sz)
{
    // minimal JSON (no fancy escaping for ssid/pass; keep printable)
    snprintf(out,out_sz,
        "{"
        "\"callsign\":\"%s\","
        "\"relay_callsign\":\"%s\","
        "\"net_id\":%u,"
        "\"crypto\":%u,"
        "\"node_mode\":%u,"
        "\"wifi\":%u,"
        "\"ssid\":\"%s\","
        "\"beacon\":%u,"
        "\"beacon_ms\":%lu,"
        "\"routeadv\":%u,"
        "\"routeadv_topn\":%u,"
        "\"routeadv_delta\":%u,"
        "\"holddown_ms\":%lu,"
        "\"cad\":%u,"
        "\"rf_hz\":%lu,"
        "\"tx_dbm\":%d,"
        "\"powersave\":%u,"
        "\"bme280\":%u,"
        "\"relay_gpio\":%d,"
        "\"sensor_wake_ms\":%lu,"
        "\"sensor_rxwin_ms\":%lu"
        "}",
        c->callsign,
        c->relay_callsign,
        (unsigned)c->net_id,
        c->crypto_enable?1:0,
        (unsigned)c->node_mode,
        c->wifi_enable?1:0,
        c->wifi_ssid,
        c->beacon_enable?1:0,
        (unsigned long)c->beacon_interval_ms,
        c->routeadv_enable?1:0,
        (unsigned)c->routeadv_topn,
        (unsigned)c->routeadv_delta_etx,
        (unsigned long)c->holddown_ms,
        c->cad_enable?1:0,
        (unsigned long)c->rf_freq_hz,
        (int)c->tx_power_dbm,
        c->powersave_enable?1:0,
        c->bme280_enable ? 1:0,
        (int)c->relay_gpio,
        (unsigned long)c->sensor_wake_period_ms,
        (unsigned long)c->sensor_boot_rx_window_ms
    );
}