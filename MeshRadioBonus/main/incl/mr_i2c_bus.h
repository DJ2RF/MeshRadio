#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Gemeinsamer I2C-Bus für MeshRadio-Module
 *
 * Ziel:
 *  - Display, PMU und optionale Sensoren teilen sich denselben I2C-Master-Bus
 *  - doppeltes i2c_new_master_bus() auf demselben Port wird verhindert
 *
 * Typische Nutzung:
 *
 *   i2c_master_bus_handle_t bus = NULL;
 *   ESP_ERROR_CHECK(mr_i2c_bus_get(I2C_NUM_0, PIN_I2C_SDA, PIN_I2C_SCL, &bus));
 *
 * Danach wie gewohnt Device hinzufügen:
 *
 *   i2c_device_config_t dev_cfg = {
 *       .dev_addr_length = I2C_ADDR_BIT_LEN_7,
 *       .device_address  = 0x3C,
 *       .scl_speed_hz    = 400000,
 *   };
 *   ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &dev_handle));
 */

/* Holt einen gemeinsamen I2C-Master-Bus oder legt ihn einmalig an. */
esp_err_t mr_i2c_bus_get(i2c_port_num_t port,
                         int sda_io_num,
                         int scl_io_num,
                         i2c_master_bus_handle_t *out_bus);

/* Optional: trennt den Bus wieder. Nur verwenden, wenn du das wirklich brauchst. */
esp_err_t mr_i2c_bus_release(i2c_port_num_t port);

/* Debug/Status */
bool mr_i2c_bus_is_ready(i2c_port_num_t port);

#ifdef __cplusplus
}
#endif
