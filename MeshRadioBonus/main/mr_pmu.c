#include "incl/mr_pmu.h"
#include "incl/mr_board.h"
#include "incl/config_meshradio.h"
#include "incl/mr_i2c_bus.h"

#include <string.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "MR_PMU";

#ifndef MR_AXP_I2C_ADDR
#define MR_AXP_I2C_ADDR 0x34
#endif

/* --------------------------- Known registers --------------------------- */
#define AXP192_REG_STATUS1                 0x00
#define AXP192_REG_CHARGE_STATUS           0x01
#define AXP192_REG_ADC_ENABLE1             0x82
#define AXP192_REG_BAT_VOLT_H              0x78
#define AXP192_REG_BAT_VOLT_L              0x79

#define AXP2101_REG_STATUS1                0x00
#define AXP2101_REG_STATUS2                0x01
#define AXP2101_REG_FG_CHG_WDT_CTRL        0x18
#define AXP2101_REG_ADC_CH_EN              0x30
#define AXP2101_REG_VBAT_H                 0x34
#define AXP2101_REG_VBAT_L                 0x35
#define AXP2101_REG_BAT_PERCENT            0xA4

static bool s_ready = false;
static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_pmu_dev = NULL;
static mr_pmu_status_t s_status;

static esp_err_t pmu_read_reg(uint8_t reg, uint8_t *out, size_t n)
{
    if (!s_pmu_dev || !out || n == 0) return ESP_ERR_INVALID_STATE;
    return i2c_master_transmit_receive(s_pmu_dev, &reg, 1, out, n, 100);
}

static esp_err_t pmu_write_reg8(uint8_t reg, uint8_t val)
{
    if (!s_pmu_dev) return ESP_ERR_INVALID_STATE;
    uint8_t tx[2] = { reg, val };
    return i2c_master_transmit(s_pmu_dev, tx, sizeof(tx), 100);
}

static esp_err_t pmu_update_bits(uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t v = 0;
    esp_err_t e = pmu_read_reg(reg, &v, 1);
    if (e != ESP_OK) return e;
    v = (uint8_t)((v & (uint8_t)(~mask)) | (value & mask));
    return pmu_write_reg8(reg, v);
}

static uint8_t clamp_pct_from_mv(uint16_t mv)
{
    if (mv <= BATT_EMPTY_MV) return 0;
    if (mv >= BATT_FULL_MV) return 100;
    return (uint8_t)(((uint32_t)(mv - BATT_EMPTY_MV) * 100U) /
                     (uint32_t)(BATT_FULL_MV - BATT_EMPTY_MV));
}

static uint16_t decode_axp192_vbat_mv(uint8_t hi, uint8_t lo)
{
    uint16_t raw = (uint16_t)(((uint16_t)hi << 4) | (lo & 0x0F));
    return (uint16_t)((raw * 11U) / 10U); /* 1.1mV / LSB */
}

static uint16_t decode_axp2101_vbat_mv(uint8_t hi, uint8_t lo)
{
    /* Testweise gleiche 12-bit/1.1mV-Logik wie beim AXP192 */
    uint16_t raw = (uint16_t)(((uint16_t)hi << 4) | (lo & 0x0F));
    return (uint16_t)((raw * 11U) / 10U);
}

static esp_err_t axp192_enable_measurements(void)
{
    /* Enable ADC blocks conservatively. */
    return pmu_write_reg8(AXP192_REG_ADC_ENABLE1, 0xFF);
}

static esp_err_t axp2101_enable_measurements(void)
{
    /* REG30 bit0 = battery voltage ADC channel enable
       REG18 bit3 = fuel gauge enable */
    esp_err_t e = pmu_update_bits(AXP2101_REG_ADC_CH_EN, 0x01, 0x01);
    if (e != ESP_OK) return e;
    return pmu_update_bits(AXP2101_REG_FG_CHG_WDT_CTRL, 0x08, 0x08);
}

static esp_err_t axp192_read_status(mr_pmu_status_t *st)
{
    uint8_t s0 = 0, s1 = 0, vh = 0, vl = 0;

    esp_err_t e0 = pmu_read_reg(AXP192_REG_STATUS1, &s0, 1);
    esp_err_t e1 = pmu_read_reg(AXP192_REG_CHARGE_STATUS, &s1, 1);
    esp_err_t e2 = pmu_read_reg(AXP192_REG_BAT_VOLT_H, &vh, 1);
    esp_err_t e3 = pmu_read_reg(AXP192_REG_BAT_VOLT_L, &vl, 1);
    if (e0 != ESP_OK || e1 != ESP_OK || e2 != ESP_OK || e3 != ESP_OK) {
        return ESP_FAIL;
    }

    st->chip = MR_PMU_CHIP_AXP192;
    st->usb_present = ((s0 & 0x10U) != 0U) || ((s0 & 0x20U) != 0U);
    st->charging = ((s1 & 0x40U) != 0U);
    st->battery_mv = decode_axp192_vbat_mv(vh, vl);
    st->battery_present = (st->battery_mv > 2500U);
    st->battery_percent = clamp_pct_from_mv(st->battery_mv);

    return ESP_OK;
}

static esp_err_t axp2101_read_status(mr_pmu_status_t *st)
{
    uint8_t s0 = 0, s1 = 0, vh = 0, vl = 0, soc = 0xFF;

    esp_err_t e0 = pmu_read_reg(AXP2101_REG_STATUS1, &s0, 1);
    esp_err_t e1 = pmu_read_reg(AXP2101_REG_STATUS2, &s1, 1);
    esp_err_t e2 = pmu_read_reg(AXP2101_REG_VBAT_H, &vh, 1);
    esp_err_t e3 = pmu_read_reg(AXP2101_REG_VBAT_L, &vl, 1);

    if (e0 != ESP_OK || e1 != ESP_OK || e2 != ESP_OK || e3 != ESP_OK) {
        return ESP_FAIL;
    }

    (void)pmu_read_reg(AXP2101_REG_BAT_PERCENT, &soc, 1);

    st->chip = MR_PMU_CHIP_AXP2101;
    st->usb_present = ((s0 & 0x20U) != 0U);
    st->battery_present = ((s0 & 0x08U) != 0U);

    /* status2[2:0]: 001 pre-charge, 010 CC, 011 CV => charging */
    st->charging = (((s1 & 0x07U) == 0x01U) ||
                    ((s1 & 0x07U) == 0x02U) ||
                    ((s1 & 0x07U) == 0x03U));

    /* AXP2101: laut Doku 14-bit ADC in reg34/reg35 */
    uint16_t raw14 = (uint16_t)((((uint16_t)vh & 0x3FU) << 8) | vl);

    /* Debug: Rohdaten sichtbar machen */
    //ESP_LOGI(TAG,
    //         "AXP2101 RAW: s0=0x%02X s1=0x%02X vh=0x%02X vl=0x%02X raw14=%u soc=%u",
    //         s0, s1, vh, vl, raw14, soc);

    /*
     * Nur plausible Spannungen akzeptieren.
     * Der bisherige direkte 1mV/LSB-Pfad liefert bei dir unplausible Werte
     * (17mV, 55mV, 141mV ...), deshalb markieren wir das vorerst als unbekannt.
     */
    if (raw14 >= 3000U && raw14 <= 4600U) {
        st->battery_mv = raw14;
    } else {
        st->battery_mv = 0;
    }

    /*
     * Prozent bevorzugt aus Fuel Gauge (regA4), falls plausibel.
     * Doku: Battery percentage über regA4H.
     */
    if (soc <= 100U) {
        st->battery_percent = soc;
    } else if (st->battery_mv >= BATT_EMPTY_MV && st->battery_mv <= BATT_FULL_MV) {
        st->battery_percent = clamp_pct_from_mv(st->battery_mv);
    } else {
        st->battery_percent = 0;
    }

    return ESP_OK;
}

static mr_pmu_chip_t pmu_detect_chip(void)
{
    const mr_board_info_t *b = mr_board_get();
    uint8_t probe = 0;

    if (!b) return MR_PMU_CHIP_NONE;
    if (pmu_read_reg(0x00, &probe, 1) != ESP_OK) return MR_PMU_CHIP_NONE;

    switch (b->board_id) {
        case MR_BOARD_ID_TBEAM_V11_SX1276:
            return MR_PMU_CHIP_AXP192;
        case MR_BOARD_ID_TBEAM_V12_AXP2101:
            return MR_PMU_CHIP_AXP2101;
        default:
            return MR_PMU_CHIP_NONE;
    }
}

esp_err_t mr_pmu_init(void)
{
    const mr_board_info_t *b = mr_board_get();
    if (!b || !b->has_pmu) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (b->pin_i2c_sda < 0 || b->pin_i2c_scl < 0 || b->pmu_i2c_addr < 0) {
        ESP_LOGE(TAG, "Invalid PMU I2C configuration");
        return ESP_ERR_INVALID_ARG;
    }

    memset(&s_status, 0, sizeof(s_status));
    s_status.present = true;
    s_status.chip = MR_PMU_CHIP_NONE;

    if (!s_i2c_bus) {
        esp_err_t e = mr_i2c_bus_get(I2C_NUM_0,
                                     b->pin_i2c_sda,
                                     b->pin_i2c_scl,
                                     &s_i2c_bus);
        if (e != ESP_OK) {
            ESP_LOGE(TAG, "mr_i2c_bus_get failed: %s", esp_err_to_name(e));
            return e;
        }
    }

    if (!s_pmu_dev) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = (uint16_t)b->pmu_i2c_addr,
            .scl_speed_hz = 400000,
        };

        esp_err_t e = i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_pmu_dev);
        if (e != ESP_OK) {
            ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(e));
            return e;
        }
    }

    s_status.chip = pmu_detect_chip();
    if (s_status.chip == MR_PMU_CHIP_NONE) {
        ESP_LOGE(TAG, "PMU not detected on board=%s addr=0x%02X",
                 mr_board_name(), (unsigned)b->pmu_i2c_addr);
        return ESP_FAIL;
    }

    if (s_status.chip == MR_PMU_CHIP_AXP192) {
        (void)axp192_enable_measurements();
        ESP_LOGI(TAG, "Detected AXP192");
    } else if (s_status.chip == MR_PMU_CHIP_AXP2101) {
        (void)axp2101_enable_measurements();
        ESP_LOGI(TAG, "Detected AXP2101");
    }

    s_ready = true;
    s_status.initialized = true;

    ESP_LOGI(TAG, "PMU init OK board=%s addr=0x%02X sda=%d scl=%d",
             mr_board_name(),
             (unsigned)b->pmu_i2c_addr,
             b->pin_i2c_sda,
             b->pin_i2c_scl);

    return mr_pmu_poll();
}

esp_err_t mr_pmu_poll(void)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;

    mr_pmu_status_t tmp = s_status;
    esp_err_t e = ESP_ERR_NOT_SUPPORTED;

    if (s_status.chip == MR_PMU_CHIP_AXP192) {
        e = axp192_read_status(&tmp);
    } else if (s_status.chip == MR_PMU_CHIP_AXP2101) {
        e = axp2101_read_status(&tmp);
    }

    if (e == ESP_OK) {
        tmp.present = true;
        tmp.initialized = true;
        s_status = tmp;
    }

    return e;
}

bool mr_pmu_is_available(void)
{
    return s_ready;
}

bool mr_pmu_get_status(mr_pmu_status_t *out)
{
    if (!out) return false;
    *out = s_status;
    return s_ready;
}

esp_err_t mr_pmu_set_gps_power(bool en)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    ESP_LOGI(TAG, "GPS power -> %s (stub/no-op)", en ? "ON" : "OFF");
    return ESP_OK;
}

esp_err_t mr_pmu_set_display_power(bool en)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    ESP_LOGI(TAG, "Display power -> %s (stub/no-op)", en ? "ON" : "OFF");
    return ESP_OK;
}