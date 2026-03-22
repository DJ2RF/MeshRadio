#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APRS_WEB_CALLSIGN_MAX  16
#define APRS_WEB_PASSCODE_MAX  16
#define APRS_WEB_COMMENT_MAX   64
#define APRS_WEB_HOST_MAX      64
#define APRS_WEB_LAT_MAX       20
#define APRS_WEB_LON_MAX       20

typedef struct {
    bool enabled;
    char callsign[APRS_WEB_CALLSIGN_MAX];
    char passcode[APRS_WEB_PASSCODE_MAX];
    char symbol_table;
    char symbol_code;
    char comment[APRS_WEB_COMMENT_MAX];
    char host[APRS_WEB_HOST_MAX];
    uint16_t port;
    uint32_t interval_ms;
    bool use_static_pos;
    char latitude[APRS_WEB_LAT_MAX];
    char longitude[APRS_WEB_LON_MAX];
} aprs_web_cfg_t;

void aprs_web_init(void);
void aprs_web_register_http(httpd_handle_t server);
bool aprs_web_has_gps_board(void);

bool aprs_web_enabled(void);
const char *aprs_web_callsign(void);
const char *aprs_web_passcode(void);
char aprs_web_symbol_table(void);
char aprs_web_symbol_code(void);
const char *aprs_web_comment(void);
const char *aprs_web_host(void);
uint16_t aprs_web_port(void);
uint32_t aprs_web_interval_ms(void);
bool aprs_web_use_static_pos(void);
const char *aprs_web_latitude(void);
const char *aprs_web_longitude(void);
void aprs_web_get_cfg(aprs_web_cfg_t *out);

#ifdef __cplusplus
}
#endif
