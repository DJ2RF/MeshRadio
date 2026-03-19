#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Fallback/Kompatibilität:
 * Wenn config_meshradio.h die neuen Board-IDs noch nicht kennt,
 * erzeugen wir hier sinnvolle Aliase, damit das Modul schon baubar bleibt.
 */
#ifndef MR_BOARD_TBEAM_V11_SX1276
#ifdef MR_BOARD_LILYGO_SX1276
#define MR_BOARD_TBEAM_V11_SX1276  MR_BOARD_LILYGO_SX1276
#else
#define MR_BOARD_TBEAM_V11_SX1276  1
#endif
#endif

#ifndef MR_BOARD_TBEAM_V12_AXP2101
#define MR_BOARD_TBEAM_V12_AXP2101  3
#endif

#ifndef MR_BOARD_HELTEC_V3
#define MR_BOARD_HELTEC_V3          2
#endif

typedef enum {
    MR_BOARD_ID_UNKNOWN = 0,
    MR_BOARD_ID_TBEAM_V11_SX1276,
    MR_BOARD_ID_TBEAM_V12_AXP2101,
    MR_BOARD_ID_HELTEC_V3
} mr_board_id_t;

typedef struct {
    mr_board_id_t board_id;
    const char   *name;

    bool has_gps;
    bool has_pmu;
    bool has_display;
    bool has_batt_adc;

    bool lora_is_sx1276;
    bool lora_is_sx1262;

    int pin_lora_nss;
    int pin_lora_sck;
    int pin_lora_mosi;
    int pin_lora_miso;
    int pin_lora_rst;
    int pin_lora_dio0;
    int pin_lora_dio1;
    int pin_lora_busy;

    int pin_i2c_sda;
    int pin_i2c_scl;

    int pin_gps_tx;     /* ESP32 TX -> GPS RX */
    int pin_gps_rx;     /* ESP32 RX <- GPS TX */
    int pin_gps_pps;

    int pin_batt_adc;
    int batt_en_gpio;
    int batt_en_active_low;

    int vext_ctrl_gpio;
    int vext_active_low;

    int pmu_i2c_addr;
    int pmu_irq_gpio;
} mr_board_info_t;

const mr_board_info_t *mr_board_get(void);
mr_board_id_t mr_board_get_id(void);
const char *mr_board_name(void);

bool mr_board_has_gps(void);
bool mr_board_has_pmu(void);
bool mr_board_has_display(void);
bool mr_board_has_batt_adc(void);

#ifdef __cplusplus
}
#endif