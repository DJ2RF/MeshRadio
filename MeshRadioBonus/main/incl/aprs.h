#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef APRS_IS_HOST
#define APRS_IS_HOST          "euro.aprs2.net"
#endif

#ifndef APRS_IS_PORT
#define APRS_IS_PORT          14580
#endif

#ifndef APRS_CALLSIGN
#define APRS_CALLSIGN         "DJ2RF-10"
#endif

#ifndef APRS_PASSCODE
#define APRS_PASSCODE         "17402"
#endif

#ifndef APRS_SYMBOL_TABLE
#define APRS_SYMBOL_TABLE     '/'
#endif

#ifndef APRS_SYMBOL_CODE
#define APRS_SYMBOL_CODE      '>'
#endif

#ifndef APRS_COMMENT
#define APRS_COMMENT          "MeshRadio Test Gateway"
#endif

#ifndef APRS_SEND_TIMEOUT_MS
#define APRS_SEND_TIMEOUT_MS  5000
#endif

#define APRS_CALLSIGN_MAX 16
#define APRS_PASSCODE_MAX 16
#define APRS_COMMENT_MAX  64
#define APRS_HOST_MAX     64
#define APRS_LAT_MAX      20
#define APRS_LON_MAX      20

typedef struct {
    bool enabled;
    char callsign[APRS_CALLSIGN_MAX];
    char passcode[APRS_PASSCODE_MAX];
    char symbol_table;
    char symbol_code;
    char comment[APRS_COMMENT_MAX];
    char host[APRS_HOST_MAX];
    uint16_t port;
    bool use_static_pos;
    char latitude[APRS_LAT_MAX];
    char longitude[APRS_LON_MAX];
} aprs_cfg_t;

bool aprs_build_packet_from_cfg(const aprs_cfg_t *cfg,
                                const char *mesh_msg,
                                char *out_packet,
                                size_t out_packet_sz);

bool aprs_send_packet_with_cfg(const aprs_cfg_t *cfg,
                               const char *packet);

#ifdef __cplusplus
}
#endif