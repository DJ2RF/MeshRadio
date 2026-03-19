#include "incl/mr_time_net.h"
#include "incl/mr_wifi_sta.h"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_sntp.h"

static const char *TAG = "MR_TIME_NET";

static bool s_sntp_started = false;

static bool mr_time_net_epoch_valid(time_t t)
{
    /* grob: alles ab 2024 gilt als "gültige Zeit" */
    return (t >= 1704067200);
}

void mr_time_net_init(void)
{
    /* bewusst leer/minimal:
       Start passiert erst dann, wenn STA wirklich verbunden ist */
}

void mr_time_net_poll(void)
{
    if (!mr_wifi_sta_is_connected()) {
        return;
    }

    if (!s_sntp_started) {
        esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
        esp_sntp_setservername(0, "pool.ntp.org");
        esp_sntp_init();

        s_sntp_started = true;
        ESP_LOGI(TAG, "SNTP started");
    }
}

bool mr_time_net_is_valid(void)
{
    time_t now = 0;
    time(&now);
    return mr_time_net_epoch_valid(now);
}

bool mr_time_net_get_datetime(char *out, size_t out_sz)
{
    if (!out || out_sz < 6) {
        return false;
    }

    out[0] = 0;

    mr_time_net_poll();

    time_t now = 0;
    struct tm ti = {0};

    time(&now);
    if (!mr_time_net_epoch_valid(now)) {
        return false;
    }

    localtime_r(&now, &ti);

    snprintf(out, out_sz, "%02d.%02d %02d:%02d",
             ti.tm_mday,
             ti.tm_mon + 1,
             ti.tm_hour,
             ti.tm_min);
    return true;
}

bool mr_time_net_get_time(char *out, size_t out_sz)
{
    if (!out || out_sz < 6) {
        return false;
    }

    out[0] = 0;

    mr_time_net_poll();

    time_t now = 0;
    struct tm ti = {0};

    time(&now);
    if (!mr_time_net_epoch_valid(now)) {
        return false;
    }

    localtime_r(&now, &ti);

    snprintf(out, out_sz, "%02d:%02d", ti.tm_hour, ti.tm_min);
    return true;
}

bool mr_time_net_get_date(char *out, size_t out_sz)
{
    if (!out || out_sz < 6) {
        return false;
    }

    out[0] = 0;

    mr_time_net_poll();

    time_t now = 0;
    struct tm ti = {0};

    time(&now);
    if (!mr_time_net_epoch_valid(now)) {
        return false;
    }

    localtime_r(&now, &ti);

    snprintf(out, out_sz, "%02d.%02d.%02d",
             ti.tm_mday,
             ti.tm_mon + 1,
             (ti.tm_year + 1900) % 100);
    return true;
}