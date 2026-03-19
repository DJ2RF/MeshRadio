#include "incl/mr_i2c_bus.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

static const char *TAG = "MR_I2C_BUS";

typedef struct {
    bool used;
    i2c_port_num_t port;
    int sda_io_num;
    int scl_io_num;
    i2c_master_bus_handle_t handle;
} mr_i2c_bus_slot_t;

static mr_i2c_bus_slot_t s_slots[2];
static SemaphoreHandle_t s_lock = NULL;

static void mr_i2c_bus_lock_init(void)
{
    if (!s_lock) {
        s_lock = xSemaphoreCreateMutex();
    }
}

static int mr_i2c_bus_port_to_index(i2c_port_num_t port)
{
    switch (port) {
        case I2C_NUM_0: return 0;
#if SOC_HP_I2C_NUM > 1
        case I2C_NUM_1: return 1;
#endif
        default: return -1;
    }
}

esp_err_t mr_i2c_bus_get(i2c_port_num_t port,
                         int sda_io_num,
                         int scl_io_num,
                         i2c_master_bus_handle_t *out_bus)
{
    if (!out_bus) {
        return ESP_ERR_INVALID_ARG;
    }
    if (sda_io_num < 0 || scl_io_num < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    mr_i2c_bus_lock_init();
    if (!s_lock) {
        return ESP_ERR_NO_MEM;
    }

    int idx = mr_i2c_bus_port_to_index(port);
    if (idx < 0) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);

    mr_i2c_bus_slot_t *slot = &s_slots[idx];

    if (slot->used) {
        if (slot->port != port ||
            slot->sda_io_num != sda_io_num ||
            slot->scl_io_num != scl_io_num) {
            ESP_LOGE(TAG,
                     "I2C port %d already in use with SDA=%d SCL=%d, requested SDA=%d SCL=%d",
                     (int)slot->port,
                     slot->sda_io_num,
                     slot->scl_io_num,
                     sda_io_num,
                     scl_io_num);
            xSemaphoreGive(s_lock);
            return ESP_ERR_INVALID_STATE;
        }

        *out_bus = slot->handle;
        xSemaphoreGive(s_lock);
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = port,
        .sda_io_num = sda_io_num,
        .scl_io_num = scl_io_num,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus = NULL;
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG,
                 "i2c_new_master_bus failed port=%d sda=%d scl=%d: %s",
                 (int)port, sda_io_num, scl_io_num, esp_err_to_name(err));
        xSemaphoreGive(s_lock);
        return err;
    }

    memset(slot, 0, sizeof(*slot));
    slot->used = true;
    slot->port = port;
    slot->sda_io_num = sda_io_num;
    slot->scl_io_num = scl_io_num;
    slot->handle = bus;

    *out_bus = bus;

    ESP_LOGI(TAG, "I2C shared bus ready: port=%d sda=%d scl=%d",
             (int)port, sda_io_num, scl_io_num);

    xSemaphoreGive(s_lock);
    return ESP_OK;
}

esp_err_t mr_i2c_bus_release(i2c_port_num_t port)
{
    mr_i2c_bus_lock_init();
    if (!s_lock) {
        return ESP_ERR_NO_MEM;
    }

    int idx = mr_i2c_bus_port_to_index(port);
    if (idx < 0) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);

    mr_i2c_bus_slot_t *slot = &s_slots[idx];
    if (!slot->used || !slot->handle) {
        xSemaphoreGive(s_lock);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = i2c_del_master_bus(slot->handle);
    if (err == ESP_OK) {
        memset(slot, 0, sizeof(*slot));
        ESP_LOGI(TAG, "I2C shared bus released: port=%d", (int)port);
    } else {
        ESP_LOGE(TAG, "i2c_del_master_bus failed port=%d: %s",
                 (int)port, esp_err_to_name(err));
    }

    xSemaphoreGive(s_lock);
    return err;
}

bool mr_i2c_bus_is_ready(i2c_port_num_t port)
{
    int idx = mr_i2c_bus_port_to_index(port);
    if (idx < 0) {
        return false;
    }
    return s_slots[idx].used && (s_slots[idx].handle != NULL);
}
