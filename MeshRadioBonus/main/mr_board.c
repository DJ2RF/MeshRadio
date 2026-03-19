#include "incl/mr_board.h"
#include "incl/config_meshradio.h"

#include "esp_log.h"

static const char *TAG = "MR_BOARD";

/*
 * V1.2:
 * Es gibt Berichte, dass GPS TX/RX gegenüber V1.1 geändert wurden.
 * Deshalb bewusst als overridable Defaults.
 */
#ifndef MR_TBEAM_V12_GPS_TX
#define MR_TBEAM_V12_GPS_TX  12
#endif

#ifndef MR_TBEAM_V12_GPS_RX
#define MR_TBEAM_V12_GPS_RX  34
#endif

#ifndef MR_TBEAM_V12_GPS_PPS
#define MR_TBEAM_V12_GPS_PPS (-1)
#endif

#ifndef MR_TBEAM_V11_GPS_TX
#define MR_TBEAM_V11_GPS_TX  12
#endif

#ifndef MR_TBEAM_V11_GPS_RX
#define MR_TBEAM_V11_GPS_RX  34
#endif

#ifndef MR_TBEAM_V11_GPS_PPS
#define MR_TBEAM_V11_GPS_PPS 37
#endif

#ifndef MR_AXP_I2C_ADDR
#define MR_AXP_I2C_ADDR      0x34
#endif

/*
 * LILYGO / TTGO SX1276
 * Eigenständige Boardbeschreibung.
 * Wichtig: bleibt ADC-basiert, weil dieses Board bei dir bereits korrekt funktioniert.
 */
static const mr_board_info_t s_board_lilygo_sx1276 = {
    .board_id = MR_BOARD_ID_UNKNOWN,
    .name = "LILYGO SX1276",

    .has_gps = false,
    .has_pmu = false,
    .has_display = true,
    .has_batt_adc = true,

    .lora_is_sx1276 = true,
    .lora_is_sx1262 = false,

    .pin_lora_nss  = 18,
    .pin_lora_sck  = 5,
    .pin_lora_mosi = 27,
    .pin_lora_miso = 19,
    .pin_lora_rst  = 23,
    .pin_lora_dio0 = 26,
    .pin_lora_dio1 = -1,
    .pin_lora_busy = -1,

    .pin_i2c_sda = 21,
    .pin_i2c_scl = 22,

    .pin_gps_tx  = -1,
    .pin_gps_rx  = -1,
    .pin_gps_pps = -1,

    .pin_batt_adc = 35,
    .batt_en_gpio = -1,
    .batt_en_active_low = 0,

    .vext_ctrl_gpio = -1,
    .vext_active_low = 0,

    .pmu_i2c_addr = -1,
    .pmu_irq_gpio = -1
};

/*
 * T-Beam V1.1
 * Umgestellt auf PMU / AXP192.
 */
static const mr_board_info_t s_board_tbeam_v11 = {
    .board_id = MR_BOARD_ID_TBEAM_V11_SX1276,
    .name = "T-Beam V1.1 SX1276",

    .has_gps = true,
    .has_pmu = true,
    .has_display = true,
    .has_batt_adc = false,

    .lora_is_sx1276 = true,
    .lora_is_sx1262 = false,

    .pin_lora_nss  = 18,
    .pin_lora_sck  = 5,
    .pin_lora_mosi = 27,
    .pin_lora_miso = 19,
    .pin_lora_rst  = 23,
    .pin_lora_dio0 = 26,
    .pin_lora_dio1 = -1,
    .pin_lora_busy = -1,

    .pin_i2c_sda = 21,
    .pin_i2c_scl = 22,

    .pin_gps_tx  = MR_TBEAM_V11_GPS_TX,
    .pin_gps_rx  = MR_TBEAM_V11_GPS_RX,
    .pin_gps_pps = MR_TBEAM_V11_GPS_PPS,

    .pin_batt_adc = -1,
    .batt_en_gpio = -1,
    .batt_en_active_low = 0,

    .vext_ctrl_gpio = -1,
    .vext_active_low = 0,

    .pmu_i2c_addr = MR_AXP_I2C_ADDR,
    .pmu_irq_gpio = -1
};

/*
 * T-Beam V1.2
 * PMU / AXP2101.
 */
static const mr_board_info_t s_board_tbeam_v12 = {
    .board_id = MR_BOARD_ID_TBEAM_V12_AXP2101,
    .name = "T-Beam V1.2 AXP2101",

    .has_gps = true,
    .has_pmu = true,
    .has_display = true,
    .has_batt_adc = false,

    .lora_is_sx1276 = true,
    .lora_is_sx1262 = false,

    .pin_lora_nss  = 18,
    .pin_lora_sck  = 5,
    .pin_lora_mosi = 27,
    .pin_lora_miso = 19,
    .pin_lora_rst  = 23,
    .pin_lora_dio0 = 26,
    .pin_lora_dio1 = -1,
    .pin_lora_busy = -1,

    .pin_i2c_sda = 21,
    .pin_i2c_scl = 22,

    .pin_gps_tx  = MR_TBEAM_V12_GPS_TX,
    .pin_gps_rx  = MR_TBEAM_V12_GPS_RX,
    .pin_gps_pps = MR_TBEAM_V12_GPS_PPS,

    .pin_batt_adc = -1,
    .batt_en_gpio = -1,
    .batt_en_active_low = 0,

    .vext_ctrl_gpio = -1,
    .vext_active_low = 0,

    .pmu_i2c_addr = MR_AXP_I2C_ADDR,
    .pmu_irq_gpio = -1
};

static const mr_board_info_t s_board_heltec_v3 = {
    .board_id = MR_BOARD_ID_HELTEC_V3,
    .name = "Heltec V3",

    .has_gps = false,
    .has_pmu = false,
    .has_display = true,
    .has_batt_adc = true,

    .lora_is_sx1276 = false,
    .lora_is_sx1262 = true,

    .pin_lora_nss  = 8,
    .pin_lora_sck  = 9,
    .pin_lora_mosi = 10,
    .pin_lora_miso = 11,
    .pin_lora_rst  = 12,
    .pin_lora_dio0 = -1,
    .pin_lora_dio1 = 14,
    .pin_lora_busy = 13,

    .pin_i2c_sda = 41,
    .pin_i2c_scl = 42,

    .pin_gps_tx  = -1,
    .pin_gps_rx  = -1,
    .pin_gps_pps = -1,

    .pin_batt_adc = 1,
    .batt_en_gpio = 37,
    .batt_en_active_low = 0,

    .vext_ctrl_gpio = 36,
    .vext_active_low = 1,

    .pmu_i2c_addr = -1,
    .pmu_irq_gpio = -1
};

static const mr_board_info_t s_board_unknown = {
    .board_id = MR_BOARD_ID_UNKNOWN,
    .name = "Unknown",

    .has_gps = false,
    .has_pmu = false,
    .has_display = false,
    .has_batt_adc = false,

    .lora_is_sx1276 = false,
    .lora_is_sx1262 = false,

    .pin_lora_nss  = -1,
    .pin_lora_sck  = -1,
    .pin_lora_mosi = -1,
    .pin_lora_miso = -1,
    .pin_lora_rst  = -1,
    .pin_lora_dio0 = -1,
    .pin_lora_dio1 = -1,
    .pin_lora_busy = -1,

    .pin_i2c_sda = -1,
    .pin_i2c_scl = -1,

    .pin_gps_tx  = -1,
    .pin_gps_rx  = -1,
    .pin_gps_pps = -1,

    .pin_batt_adc = -1,
    .batt_en_gpio = -1,
    .batt_en_active_low = 0,

    .vext_ctrl_gpio = -1,
    .vext_active_low = 0,

    .pmu_i2c_addr = -1,
    .pmu_irq_gpio = -1
};

const mr_board_info_t *mr_board_get(void)
{
    switch (MR_BOARD_PRESET) {
        case MR_BOARD_TBEAM_V11_SX1276:
            return &s_board_tbeam_v11;

        case MR_BOARD_TBEAM_V12_AXP2101:
            return &s_board_tbeam_v12;

        case MR_BOARD_HELTEC_V3:
            return &s_board_heltec_v3;

#ifdef MR_BOARD_LILYGO_SX1276
        case MR_BOARD_LILYGO_SX1276:
            return &s_board_lilygo_sx1276;
#endif

        default:
            ESP_LOGW(TAG, "Unknown MR_BOARD_PRESET=%d -> fallback", (int)MR_BOARD_PRESET);
            return &s_board_unknown;
    }
}

mr_board_id_t mr_board_get_id(void)
{
    return mr_board_get()->board_id;
}

const char *mr_board_name(void)
{
    return mr_board_get()->name;
}

bool mr_board_has_gps(void)
{
    return mr_board_get()->has_gps;
}

bool mr_board_has_pmu(void)
{
    return mr_board_get()->has_pmu;
}

bool mr_board_has_display(void)
{
    return mr_board_get()->has_display;
}

bool mr_board_has_batt_adc(void)
{
    return mr_board_get()->has_batt_adc;
}
