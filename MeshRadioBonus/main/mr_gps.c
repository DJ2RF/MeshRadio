#include "incl/mr_gps.h"
#include "incl/mr_board.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "MR_GPS";

#ifndef MR_GPS_UART_NUM
#define MR_GPS_UART_NUM      UART_NUM_1
#endif

#ifndef MR_GPS_BAUDRATE
#define MR_GPS_BAUDRATE      9600
#endif

#ifndef MR_GPS_RX_BUF_SZ
#define MR_GPS_RX_BUF_SZ     1024
#endif

#ifndef MR_GPS_LINE_MAX
#define MR_GPS_LINE_MAX      128
#endif

static bool s_init = false;
static mr_gps_fix_t s_fix;
static char s_last_nmea[MR_GPS_LINE_MAX];

static char s_line[MR_GPS_LINE_MAX];
static size_t s_line_len = 0;

static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static int split_csv(char *s, char *tok[], int max_tok)
{
    int n = 0;
    char *p = s;
    if (!s || !tok || max_tok <= 0) return 0;

    tok[n++] = p;

    while (*p && n < max_tok) {
        if (*p == ',') {
            *p = 0;
            tok[n++] = p + 1;
        } else if (*p == '*') {
            *p = 0;
            break;
        }
        p++;
    }
    return n;
}

static double parse_degmin_to_decimal(const char *v, const char *hemi)
{
    if (!v || !*v) return 0.0;

    double raw = atof(v);
    int deg = (int)(raw / 100.0);
    double min = raw - ((double)deg * 100.0);
    double dec = (double)deg + (min / 60.0);

    if (hemi && (*hemi == 'S' || *hemi == 'W')) {
        dec = -dec;
    }
    return dec;
}

static bool is_nonempty(const char *s)
{
    return (s && *s);
}

static void safe_copy(char *dst, size_t dst_sz, const char *src)
{
    if (!dst || dst_sz == 0) return;
    dst[0] = 0;

    if (!src) return;

    strncpy(dst, src, dst_sz - 1);
    dst[dst_sz - 1] = 0;
}

static void gps_apply_rmc(char *tok[], int n)
{
    if (n < 10) return;
    /*
     * $GPRMC/$GNRMC:
     * 0 type
     * 1 time
     * 2 status A/V
     * 3 lat
     * 4 N/S
     * 5 lon
     * 6 E/W
     * 7 speed kn
     * 8 course
     * 9 date
     */

    if (is_nonempty(tok[1])) safe_copy(s_fix.utc_time, sizeof(s_fix.utc_time), tok[1]);
    if (is_nonempty(tok[9])) safe_copy(s_fix.utc_date, sizeof(s_fix.utc_date), tok[9]);

    if (tok[2] && tok[2][0] == 'A' &&
        is_nonempty(tok[3]) && is_nonempty(tok[4]) &&
        is_nonempty(tok[5]) && is_nonempty(tok[6])) {

        s_fix.lat = parse_degmin_to_decimal(tok[3], tok[4]);
        s_fix.lon = parse_degmin_to_decimal(tok[5], tok[6]);

        s_fix.speed_kmh = (float)(atof(tok[7]) * 1.852);
        s_fix.course_deg = is_nonempty(tok[8]) ? (float)atof(tok[8]) : 0.0f;

        s_fix.valid = true;
        s_fix.has_fix = true;
        s_fix.last_update_ms = now_ms();
    } else if (tok[2] && tok[2][0] == 'V') {
        s_fix.has_fix = false;
    }
}

static void gps_apply_gga(char *tok[], int n)
{
    if (n < 10) return;
    /*
     * $GPGGA/$GNGGA:
     * 0 type
     * 1 time
     * 2 lat
     * 3 N/S
     * 4 lon
     * 5 E/W
     * 6 quality
     * 7 sats
     * 8 hdop
     * 9 altitude
     */

    if (is_nonempty(tok[1])) safe_copy(s_fix.utc_time, sizeof(s_fix.utc_time), tok[1]);

    s_fix.fix_quality = (uint8_t)(is_nonempty(tok[6]) ? atoi(tok[6]) : 0);
    s_fix.sats = (uint8_t)(is_nonempty(tok[7]) ? atoi(tok[7]) : 0);
    s_fix.altitude_m = is_nonempty(tok[9]) ? (float)atof(tok[9]) : 0.0f;

    if (s_fix.fix_quality > 0 &&
        is_nonempty(tok[2]) && is_nonempty(tok[3]) &&
        is_nonempty(tok[4]) && is_nonempty(tok[5])) {

        s_fix.lat = parse_degmin_to_decimal(tok[2], tok[3]);
        s_fix.lon = parse_degmin_to_decimal(tok[4], tok[5]);

        s_fix.valid = true;
        s_fix.has_fix = true;
        s_fix.last_update_ms = now_ms();
    } else {
        s_fix.has_fix = false;
    }
}

static void parse_nmea_line(char *line)
{
    if (!line || line[0] != '$') return;

    safe_copy(s_last_nmea, sizeof(s_last_nmea), line);

    char *tok[24] = {0};
    int n = split_csv(line, tok, 24);
    if (n <= 0) return;

    if (strcmp(tok[0], "$GPRMC") == 0 || strcmp(tok[0], "$GNRMC") == 0) {
        gps_apply_rmc(tok, n);
    } else if (strcmp(tok[0], "$GPGGA") == 0 || strcmp(tok[0], "$GNGGA") == 0) {
        gps_apply_gga(tok, n);
    }
}

esp_err_t mr_gps_init(void)
{
    const mr_board_info_t *b = mr_board_get();
    if (!b || !b->has_gps) {
        ESP_LOGW(TAG, "GPS not supported on board %s", mr_board_name());
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (b->pin_gps_tx < 0 || b->pin_gps_rx < 0) {
        ESP_LOGE(TAG, "GPS pins invalid: tx=%d rx=%d", b->pin_gps_tx, b->pin_gps_rx);
        return ESP_ERR_INVALID_ARG;
    }

    uart_config_t cfg = {
        .baud_rate = MR_GPS_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t e = uart_driver_install(MR_GPS_UART_NUM, MR_GPS_RX_BUF_SZ, 0, 0, NULL, 0);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(e));
        return e;
    }

    ESP_ERROR_CHECK(uart_param_config(MR_GPS_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(MR_GPS_UART_NUM,
                                 b->pin_gps_tx,
                                 b->pin_gps_rx,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));

    memset(&s_fix, 0, sizeof(s_fix));
    memset(s_last_nmea, 0, sizeof(s_last_nmea));
    memset(s_line, 0, sizeof(s_line));
    s_line_len = 0;
    s_init = true;

    ESP_LOGI(TAG, "GPS init OK board=%s uart=%d tx=%d rx=%d baud=%d",
             mr_board_name(),
             (int)MR_GPS_UART_NUM,
             b->pin_gps_tx,
             b->pin_gps_rx,
             (int)MR_GPS_BAUDRATE);

    return ESP_OK;
}

void mr_gps_poll(void)
{
    if (!s_init) return;

    uint8_t buf[128];
    int n = uart_read_bytes(MR_GPS_UART_NUM, buf, sizeof(buf), 10 / portTICK_PERIOD_MS);
    if (n <= 0) return;

    for (int i = 0; i < n; i++) {
        char c = (char)buf[i];

        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            s_line[s_line_len] = 0;
            if (s_line_len > 6) {
                char tmp[MR_GPS_LINE_MAX];
                strncpy(tmp, s_line, sizeof(tmp) - 1);
                tmp[sizeof(tmp) - 1] = 0;
                parse_nmea_line(tmp);
            }
            s_line_len = 0;
            s_line[0] = 0;
            continue;
        }

        if (isprint((unsigned char)c) && s_line_len < (sizeof(s_line) - 1)) {
            s_line[s_line_len++] = c;
        }
    }
}

bool mr_gps_is_initialized(void)
{
    return s_init;
}

bool mr_gps_has_fix(void)
{
    return (s_init && s_fix.valid && s_fix.has_fix);
}

bool mr_gps_get_fix(mr_gps_fix_t *out)
{
    if (!out) return false;
    *out = s_fix;
    return s_fix.valid;
}

void mr_gps_get_text(char *out, size_t out_sz)
{
    if (!out || out_sz == 0) return;

    if (!s_init) {
        snprintf(out, out_sz, "GPS: off");
        return;
    }

    if (!mr_gps_has_fix()) {
        snprintf(out, out_sz, "GPS: no fix");
        return;
    }

    snprintf(out, out_sz, "GPS %.5f %.5f %usat",
             s_fix.lat, s_fix.lon, (unsigned)s_fix.sats);
}

void mr_gps_get_nmea_debug(char *out, size_t out_sz)
{
    if (!out || out_sz == 0) return;
    out[0] = 0;

    if (!s_init) {
        snprintf(out, out_sz, "GPS not initialized");
        return;
    }

    snprintf(out, out_sz, "%s", s_last_nmea[0] ? s_last_nmea : "<no nmea yet>");
}