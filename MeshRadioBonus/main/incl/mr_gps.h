#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool valid;
    bool has_fix;

    double lat;
    double lon;

    float altitude_m;
    float speed_kmh;
    float course_deg;

    uint8_t sats;
    uint8_t fix_quality;

    char utc_time[16];
    char utc_date[16];

    uint32_t last_update_ms;
} mr_gps_fix_t;

esp_err_t mr_gps_init(void);
void mr_gps_poll(void);

bool mr_gps_is_initialized(void);
bool mr_gps_has_fix(void);
bool mr_gps_get_fix(mr_gps_fix_t *out);

void mr_gps_get_text(char *out, size_t out_sz);
void mr_gps_get_nmea_debug(char *out, size_t out_sz);

#ifdef __cplusplus
}
#endif