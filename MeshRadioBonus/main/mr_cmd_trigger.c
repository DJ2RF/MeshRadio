#include "incl/mr_cmd_trigger.h"
#include "incl/mr_cfg.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/*
 * Externe Runtime aus main.c
 */
extern mr_cfg_t g_cfg;
extern uint32_t now_ms(void);
extern void send_data_to(const char *dst_str, const char *txt, bool ackreq);

static const char *TAG = "CMDTRIG";

static bool   s_cfg_loaded           = false;
static bool   s_enabled              = false;
static int8_t s_trigger_gpio         = -1;
static char   s_dst[9]               = {0};
static char   s_cmd[97]              = {0};

static bool   s_trigger_gpio_ready   = false;

static int    s_last_trigger_level   = 1;
static uint32_t s_last_edge_ms       = 0;

/*
 * ------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------
 */

static bool gpio_num_valid_cfg(int8_t gpio_num)
{
    return (gpio_num >= 0 && gpio_num <= 48);
}

/*
 * Konservative Safe-List.
 * Die ist absichtlich klein gehalten.
 * Später können wir pro Board sauber erweitern.
 */
static bool gpio_allowed_for_user_io(int8_t gpio_num)
{
    switch (gpio_num) {
        case 13:
        case 14:
        case 26:
        case 48:
            return true;
        default:
            return false;
    }
}

static void trigger_gpio_init_if_needed(void)
{
    s_trigger_gpio_ready = false;

    if (!gpio_num_valid_cfg(s_trigger_gpio)) {
        ESP_LOGW(TAG, "Trigger GPIO invalid: %d", (int)s_trigger_gpio);
        return;
    }

    if (!gpio_allowed_for_user_io(s_trigger_gpio)) {
        ESP_LOGW(TAG, "Trigger GPIO %d not in safe allow-list", (int)s_trigger_gpio);
        return;
    }

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << s_trigger_gpio),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };

    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config(trigger=%d) failed: %s",
                 (int)s_trigger_gpio, esp_err_to_name(err));
        return;
    }

    s_last_trigger_level = gpio_get_level((gpio_num_t)s_trigger_gpio);
    s_trigger_gpio_ready = true;

    ESP_LOGI(TAG, "Trigger GPIO ready: %d initial_level=%d", (int)s_trigger_gpio, s_last_trigger_level);
}

static bool cfg_is_complete(void)
{
    if (!s_enabled) return false;
    if (!s_trigger_gpio_ready) return false;
    if (s_dst[0] == 0) return false;
    if (s_cmd[0] == 0) return false;
    return true;
}

/*
 * ------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------
 */

void mr_cmd_trigger_init(void)
{
    s_cfg_loaded         = false;
    s_enabled            = false;
    s_trigger_gpio       = -1;
    s_dst[0]             = 0;
    s_cmd[0]             = 0;
    s_trigger_gpio_ready = false;
    s_last_trigger_level = 1;
    s_last_edge_ms       = 0;
}

void mr_cmd_trigger_apply_cfg(void)
{
    s_enabled           = g_cfg.cmd_enable;
    s_trigger_gpio      = g_cfg.cmd_trigger_gpio;

    strncpy(s_dst, g_cfg.cmd_dst, sizeof(s_dst) - 1);
    s_dst[sizeof(s_dst) - 1] = 0;

    strncpy(s_cmd, g_cfg.cmd_text, sizeof(s_cmd) - 1);
    s_cmd[sizeof(s_cmd) - 1] = 0;

    ESP_LOGI(TAG,
             "apply_cfg: enable=%d trig=%d dst='%s' cmd='%s'",
             s_enabled ? 1 : 0,
             (int)s_trigger_gpio,
             s_dst,
             s_cmd);

    trigger_gpio_init_if_needed();

    s_cfg_loaded = true;

    if (!cfg_is_complete()) {
        ESP_LOGW(TAG,
                 "Command trigger config incomplete or disabled: enable=%d ready=%d dst_len=%d cmd_len=%d",
                 s_enabled ? 1 : 0,
                 s_trigger_gpio_ready ? 1 : 0,
                 (int)strlen(s_dst),
                 (int)strlen(s_cmd));
        return;
    }

    ESP_LOGI(TAG,
             "Command trigger armed: trig=%d dst='%s' cmd='%s'",
             (int)s_trigger_gpio,
             s_dst,
             s_cmd);
}

bool mr_cmd_trigger_is_enabled(void)
{
    return cfg_is_complete();
}

void mr_cmd_trigger_mark_sent(void)
{
}

void mr_cmd_trigger_mark_confirmed(const char *from_callsign)
{
    (void)from_callsign;
}

void mr_cmd_trigger_poll(void)
{
    if (!s_cfg_loaded) return;

    if (!cfg_is_complete()) return;

    int level = gpio_get_level((gpio_num_t)s_trigger_gpio);
    uint32_t tnow = now_ms();


    if (level != s_last_trigger_level) {
        ESP_LOGI(TAG, "Trigger edge gpio=%d %d->%d at %lu ms",
                 (int)s_trigger_gpio,
                 s_last_trigger_level,
                 level,
                 (unsigned long)tnow);
        /*
         * Debounce
         */
        if ((tnow - s_last_edge_ms) < 40U) {
            ESP_LOGW(TAG, "Trigger edge ignored by debounce (%lu ms)",
                     (unsigned long)(tnow - s_last_edge_ms));
            return;
        }

        s_last_edge_ms = tnow;

        /*
         * Fallende Flanke: Taster gegen GND
         */
        if (s_last_trigger_level == 1 && level == 0) {
            ESP_LOGI(TAG, "Trigger fired -> dst='%s' cmd='%s'", s_dst, s_cmd);
            send_data_to(s_dst, s_cmd, true);
        }

        s_last_trigger_level = level;
    }
}