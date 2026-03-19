#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Init (inkl. I2C Setup)
bool bme280_init(void);

// Messung lesen
bool bme280_read_weather(int32_t *temp_x100,
                         uint32_t *press_pa,
                         uint32_t *hum_x1000);

// Status
bool bme280_is_ok(void);