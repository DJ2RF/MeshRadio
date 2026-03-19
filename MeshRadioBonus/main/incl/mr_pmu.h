#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MR_PMU_CHIP_NONE = 0,
    MR_PMU_CHIP_AXP192,
    MR_PMU_CHIP_AXP2101
} mr_pmu_chip_t;

typedef struct {
    bool present;
    bool initialized;

    bool battery_present;
    bool usb_present;
    bool charging;

    uint16_t battery_mv;
    uint8_t  battery_percent;

    mr_pmu_chip_t chip;
} mr_pmu_status_t;

esp_err_t mr_pmu_init(void);
esp_err_t mr_pmu_poll(void);

bool mr_pmu_is_available(void);
bool mr_pmu_get_status(mr_pmu_status_t *out);

esp_err_t mr_pmu_set_gps_power(bool en);
esp_err_t mr_pmu_set_display_power(bool en);

#ifdef __cplusplus
}
#endif
