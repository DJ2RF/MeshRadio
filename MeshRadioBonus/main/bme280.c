#include "incl/bme280.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "incl/board_pins.h"
#include "incl/mr_i2c_bus.h"

/******************************************************************************
 *  BME280 Sensor Driver (I2C)
 *
 *  DATEI
 *  ---------------------------------------------------------------------------
 *  bme280.c
 *
 *  BESCHREIBUNG
 *  ---------------------------------------------------------------------------
 *  Dieses Modul kapselt den Zugriff auf einen BME280 über I2C.
 *
 *  Exportierte API:
 *    - bme280_init()
 *    - bme280_read_weather()
 *    - bme280_is_ok()
 *
 *  Messwerte:
 *    - Temperatur: 0.01 °C  -> int32_t  temp_x100
 *    - Druck:       Pa       -> uint32_t press_pa
 *    - Feuchte:     0.001 %  -> uint32_t hum_x1000
 *
 *  DESIGN
 *  ---------------------------------------------------------------------------
 *  - Der I2C-Master-Bus wird NICHT lokal neu erzeugt, sondern über das
 *    Shared-Bus-Modul mr_i2c_bus_get(...) bezogen.
 *  - Dieses Modul hängt den BME280 genau einmal als I2C-Device an den
 *    gemeinsamen Bus.
 *  - Die Kompensation verwendet die Integer-Formeln aus dem Bosch-Datenblatt.
 *
 *  WICHTIG
 *  ---------------------------------------------------------------------------
 *  - Standardadresse ist 0x76. Manche Breakout-Boards verwenden 0x77.
 *  - Das Modul ist robust gegen fehlenden/ungültigen I2C-Handle.
 *  - BMP280 (ID 0x58) wird erkannt, aber die Feuchtemessung ist dort
 *    physikalisch nicht vorhanden. In diesem Fall wird hum_x1000 auf 0 gesetzt.
 *
 ******************************************************************************/

#define TAG "BME280"

/* ============================================================================
 * Konfiguration
 * ========================================================================== */

/* Standard-I2C-Adresse des BME280.
 * Je nach Board/Breakout kann alternativ auch 0x77 verwendet werden. */
#ifndef BME280_I2C_ADDR
#define BME280_I2C_ADDR      0x76
#endif

/* I2C-Takt */
#ifndef BME280_I2C_CLK_HZ
#define BME280_I2C_CLK_HZ    100000
#endif

/* BME280 Register */
#define BME280_REG_ID        0xD0
#define BME280_REG_RESET     0xE0
#define BME280_REG_CTRL_HUM  0xF2
#define BME280_REG_STATUS    0xF3
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG    0xF5
#define BME280_REG_PRESS_MSB 0xF7

/* Erwartete Chip-IDs:
 *   BME280 = 0x60
 *   BMP280 = 0x58 (ohne Feuchtesensor) */
#define BME280_CHIP_ID       0x60
#define BMP280_CHIP_ID       0x58

/* Timeout für I2C-Transfers in Millisekunden */
#define BME280_I2C_TIMEOUT_MS 100

/* ============================================================================
 * Modulinterner Zustand
 * ========================================================================== */

/* Einmalig geholter Shared-I2C-Bus und Device-Handle */
static i2c_master_bus_handle_t g_i2c_bus = NULL;
static i2c_master_dev_handle_t g_dev = NULL;

/* Interner Sensortyp */
typedef enum
{
    BME_KIND_UNKNOWN = 0,
    BME_KIND_BME280  = 1,
    BME_KIND_BMP280  = 2
} bme_kind_t;

/* Kalibrierdaten + Laufzeitstatus des Sensors */
typedef struct
{
    /* Temperatur-Kalibrierung */
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    /* Druck-Kalibrierung */
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    /* Feuchte-Kalibrierung (nur BME280) */
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;

    /* Zwischenwert aus Temperaturkompensation, nötig für Druck/Feuchte */
    int32_t  t_fine;

    /* Laufzeitstatus */
    bool       ok;
    bme_kind_t kind;
    uint8_t    chip_id;
} bme280_t;

static bme280_t g_bme = {0};

/* ============================================================================
 * Low-Level Hilfsfunktionen
 * ========================================================================== */

/**
 * @brief Sign-Extension für 12-Bit-Werte.
 *
 * Der BME280 speichert H4/H5 über mehrere Register verteilt als 12-Bit signed.
 */
static int16_t sign_extend_12(uint16_t v)
{
    if (v & 0x0800U) {
        v |= 0xF000U;
    }
    return (int16_t)v;
}

/**
 * @brief Liest n Bytes ab einem Register.
 *
 * @param reg  Startregister
 * @param out  Zielpuffer
 * @param n    Anzahl Bytes
 *
 * @return ESP_OK bei Erfolg, sonst Fehlercode
 */
static esp_err_t bme_read(uint8_t reg, uint8_t *out, size_t n)
{
    if (g_dev == NULL || out == NULL || n == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    return i2c_master_transmit_receive(g_dev,
                                       &reg,
                                       1,
                                       out,
                                       n,
                                       BME280_I2C_TIMEOUT_MS);
}

/**
 * @brief Schreibt ein Byte in ein Register.
 *
 * @param reg  Registeradresse
 * @param val  Wert
 *
 * @return ESP_OK bei Erfolg, sonst Fehlercode
 */
static esp_err_t bme_write(uint8_t reg, uint8_t val)
{
    if (g_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t tx[2] = { reg, val };
    return i2c_master_transmit(g_dev,
                               tx,
                               sizeof(tx),
                               BME280_I2C_TIMEOUT_MS);
}

/**
 * @brief Initialisiert den Shared-I2C-Bus und hängt den BME280 als Device an.
 *
 * Diese Funktion ist idempotent:
 * - Wenn das Device bereits existiert, passiert nichts.
 * - Der gemeinsame I2C-Bus wird über mr_i2c_bus_get(...) bezogen.
 *
 * @return ESP_OK bei Erfolg, sonst Fehlercode
 */
static esp_err_t i2c_init_once(void)
{
    if (g_dev != NULL) {
        return ESP_OK;
    }

    esp_err_t err = mr_i2c_bus_get(I2C_NUM_1,
                               PIN_I2C_SDA,
                               PIN_I2C_SCL,
                               &g_i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "shared I2C bus not available: %s", esp_err_to_name(err));
        g_i2c_bus = NULL;
        return err;
    }

    if (g_i2c_bus == NULL) {
        ESP_LOGE(TAG, "shared I2C bus handle is NULL");
        return ESP_ERR_INVALID_STATE;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = BME280_I2C_ADDR,
        .scl_speed_hz    = BME280_I2C_CLK_HZ,
    };

    err = i2c_master_bus_add_device(g_i2c_bus, &dev_cfg, &g_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "add BME280 device failed: %s", esp_err_to_name(err));
        g_dev = NULL;
        return err;
    }

    ESP_LOGI(TAG,
             "BME280 attached to shared I2C bus: addr=0x%02X, sda=%d scl=%d",
             BME280_I2C_ADDR, PIN_I2C_SDA, PIN_I2C_SCL);

    return ESP_OK;
}

/* ============================================================================
 * Bosch-Kompensationsfunktionen
 * ========================================================================== */

/**
 * @brief Kompensiert die Roh-Temperatur.
 *
 * @param adc_T  20-Bit Rohwert Temperatur
 * @param b      interner Zustand
 *
 * @return Temperatur in 0.01 °C
 */
static int32_t bme_comp_T_x100(int32_t adc_T, bme280_t *b)
{
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)b->dig_T1 << 1))) *
                    ((int32_t)b->dig_T2)) >> 11;

    int32_t var2 = (((((adc_T >> 4) - ((int32_t)b->dig_T1)) *
                      ((adc_T >> 4) - ((int32_t)b->dig_T1))) >> 12) *
                    ((int32_t)b->dig_T3)) >> 14;

    b->t_fine = var1 + var2;

    /* Ergebnis in 0.01 °C */
    return (b->t_fine * 5 + 128) >> 8;
}

/**
 * @brief Kompensiert den Roh-Druck.
 *
 * @param adc_P  20-Bit Rohwert Druck
 * @param b      interner Zustand
 *
 * @return Druck in Pascal
 */
static uint32_t bme_comp_P_Pa(int32_t adc_P, bme280_t *b)
{
    int64_t var1 = ((int64_t)b->t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)b->dig_P6;
    var2 = var2 + ((var1 * (int64_t)b->dig_P5) << 17);
    var2 = var2 + (((int64_t)b->dig_P4) << 35);

    var1 = ((var1 * var1 * (int64_t)b->dig_P3) >> 8) +
           ((var1 * (int64_t)b->dig_P2) << 12);

    var1 = (((((int64_t)1) << 47) + var1) * (int64_t)b->dig_P1) >> 33;

    if (var1 == 0) {
        return 0; /* Schutz gegen Division durch 0 */
    }

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;

    var1 = (((int64_t)b->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)b->dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)b->dig_P7) << 4);

    return (uint32_t)(p >> 8);
}

/**
 * @brief Kompensiert die Roh-Feuchte.
 *
 * Nur gültig für den echten BME280. Beim BMP280 existiert kein Feuchtesensor.
 *
 * @param adc_H  16-Bit Rohwert Feuchte
 * @param b      interner Zustand
 *
 * @return Relative Feuchte in 0.001 %
 */
static uint32_t bme_comp_H_x1000(int32_t adc_H, bme280_t *b)
{
    int32_t v_x1_u32r = (b->t_fine - ((int32_t)76800));

    v_x1_u32r =
        (((((adc_H << 14) - (((int32_t)b->dig_H4) << 20) -
            (((int32_t)b->dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
         (((((((v_x1_u32r * ((int32_t)b->dig_H6)) >> 10) *
              (((v_x1_u32r * ((int32_t)b->dig_H3)) >> 11) +
               ((int32_t)32768))) >> 10) +
            ((int32_t)2097152)) * ((int32_t)b->dig_H2) + 8192) >> 14));

    v_x1_u32r =
        (v_x1_u32r -
         (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
           ((int32_t)b->dig_H1)) >> 4));

    if (v_x1_u32r < 0) {
        v_x1_u32r = 0;
    }
    if (v_x1_u32r > 419430400) {
        v_x1_u32r = 419430400;
    }

    /* Bosch-Formel liefert %RH * 1024 */
    uint32_t h1024 = (uint32_t)(v_x1_u32r >> 12);

    /* Umrechnung auf %RH * 1000 */
    return (h1024 * 1000U) / 1024U;
}

/* ============================================================================
 * Öffentliche API
 * ========================================================================== */

/**
 * @brief Initialisiert den BME280/BMP280.
 *
 * Ablauf:
 *   1. Shared-I2C-Bus holen
 *   2. Sensor als Device an den Bus hängen
 *   3. Chip-ID lesen
 *   4. Soft-Reset
 *   5. Kalibrierdaten lesen
 *   6. Oversampling / Modus setzen
 *
 * @return true bei Erfolg, sonst false
 */

bool bme280_init(void)
{
    /* Bereits erfolgreich initialisiert? */
    if (g_bme.ok && g_dev != NULL) {
        return true;
    }

    /* Zustand sauber zurücksetzen */
    memset(&g_bme, 0, sizeof(g_bme));

    ESP_LOGI(TAG, "bme280_init: start");

    esp_err_t err = i2c_init_once();
    if (err != ESP_OK || g_dev == NULL) {
        ESP_LOGE(TAG, "BME280 init failed: no valid I2C device handle");
        g_bme.ok = false;
        return false;
    }

    /* Kurze Einschwingzeit nach Bus-/Device-Setup */
    vTaskDelay(pdMS_TO_TICKS(20));

    /* Chip-ID lesen */
    uint8_t id = 0;
    err = bme_read(BME280_REG_ID, &id, 1);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "read ID failed: %s", esp_err_to_name(err));
        g_bme.ok = false;
        return false;
    }

    g_bme.chip_id = id;

    if (id == BME280_CHIP_ID) {
        g_bme.kind = BME_KIND_BME280;
    } else if (id == BMP280_CHIP_ID) {
        g_bme.kind = BME_KIND_BMP280;
    } else {
        ESP_LOGW(TAG, "unexpected sensor ID 0x%02X", id);
        g_bme.kind = BME_KIND_UNKNOWN;
        g_bme.ok = false;
        return false;
    }

    ESP_LOGI(TAG, "Sensor ID=0x%02X (%s)",
             id,
             (g_bme.kind == BME_KIND_BME280) ? "BME280" :
             (g_bme.kind == BME_KIND_BMP280) ? "BMP280" : "UNKNOWN");

    /* Soft-Reset */
    err = bme_write(BME280_REG_RESET, 0xB6);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "soft reset failed: %s", esp_err_to_name(err));
        g_bme.ok = false;
        return false;
    }

    /* Nach Reset warten, bis interne NVM-Aktualisierung abgeschlossen ist.
       Bit 0 von STATUS (im_update) muss 0 werden. */
    {
        bool ready = false;

        for (int i = 0; i < 20; i++) {
            uint8_t st = 0;
            err = bme_read(BME280_REG_STATUS, &st, 1);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "read STATUS after reset failed: %s", esp_err_to_name(err));
                g_bme.ok = false;
                return false;
            }

            if ((st & 0x01U) == 0) {
                ready = true;
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(5));
        }

        if (!ready) {
            ESP_LOGW(TAG, "sensor not ready after reset (STATUS.im_update still set)");
            g_bme.ok = false;
            return false;
        }
    }

    /* Kleine Zusatzpause für stabile Registerzugriffe */
    vTaskDelay(pdMS_TO_TICKS(2));

    /* Kalibrierdaten lesen */
    uint8_t c1[26] = {0};
    uint8_t c2[7]  = {0};

    err = bme_read(0x88, c1, sizeof(c1));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "read calib block 1 failed: %s", esp_err_to_name(err));
        g_bme.ok = false;
        return false;
    }

    /* Temperatur-Kalibrierung */
    g_bme.dig_T1 = (uint16_t)(c1[1]  << 8 | c1[0]);
    g_bme.dig_T2 = (int16_t) (c1[3]  << 8 | c1[2]);
    g_bme.dig_T3 = (int16_t) (c1[5]  << 8 | c1[4]);

    /* Druck-Kalibrierung */
    g_bme.dig_P1 = (uint16_t)(c1[7]  << 8 | c1[6]);
    g_bme.dig_P2 = (int16_t) (c1[9]  << 8 | c1[8]);
    g_bme.dig_P3 = (int16_t) (c1[11] << 8 | c1[10]);
    g_bme.dig_P4 = (int16_t) (c1[13] << 8 | c1[12]);
    g_bme.dig_P5 = (int16_t) (c1[15] << 8 | c1[14]);
    g_bme.dig_P6 = (int16_t) (c1[17] << 8 | c1[16]);
    g_bme.dig_P7 = (int16_t) (c1[19] << 8 | c1[18]);
    g_bme.dig_P8 = (int16_t) (c1[21] << 8 | c1[20]);
    g_bme.dig_P9 = (int16_t) (c1[23] << 8 | c1[22]);

    if (g_bme.kind == BME_KIND_BME280) {
        err = bme_read(0xE1, c2, sizeof(c2));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "read calib block 2 failed: %s", esp_err_to_name(err));
            g_bme.ok = false;
            return false;
        }

        /* Feuchte-Kalibrierung */
        g_bme.dig_H1 = c1[25];
        g_bme.dig_H2 = (int16_t)(c2[1] << 8 | c2[0]);
        g_bme.dig_H3 = c2[2];

        {
            uint16_t h4u = ((uint16_t)c2[3] << 4) | (c2[4] & 0x0F);
            uint16_t h5u = ((uint16_t)c2[5] << 4) | (c2[4] >> 4);
            g_bme.dig_H4 = sign_extend_12(h4u);
            g_bme.dig_H5 = sign_extend_12(h5u);
        }

        g_bme.dig_H6 = (int8_t)c2[6];

        ESP_LOGI(TAG, "Cal H: H1=%u H2=%d H3=%u H4=%d H5=%d H6=%d",
                 g_bme.dig_H1, g_bme.dig_H2, g_bme.dig_H3,
                 g_bme.dig_H4, g_bme.dig_H5, g_bme.dig_H6);
    }

    /* Konfiguration */
    err = bme_write(BME280_REG_CONFIG, 0x00);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "write CONFIG failed: %s", esp_err_to_name(err));
        g_bme.ok = false;
        return false;
    }

    err = bme_write(BME280_REG_CTRL_HUM, 0x01);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "write CTRL_HUM failed: %s", esp_err_to_name(err));
        g_bme.ok = false;
        return false;
    }

    err = bme_write(BME280_REG_CTRL_MEAS, (0x01 << 5) | (0x01 << 2) | 0x03);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "write CTRL_MEAS failed: %s", esp_err_to_name(err));
        g_bme.ok = false;
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(20));

    g_bme.ok = true;

    ESP_LOGI(TAG, "%s init OK (ID=0x%02X, addr=0x%02X)",
             (g_bme.kind == BME_KIND_BME280) ? "BME280" : "BMP280",
             g_bme.chip_id,
             BME280_I2C_ADDR);

    return true;
}


/**
 * @brief Liest Temperatur, Druck und Feuchte aus.
 *
 * @param[out] out_t_x100    Temperatur in 0.01 °C
 * @param[out] out_p_pa      Druck in Pa
 * @param[out] out_rh_x1000  Relative Feuchte in 0.001 %
 *
 * @return true bei Erfolg, sonst false
 */
bool bme280_read_weather(int32_t *out_t_x100,
                         uint32_t *out_p_pa,
                         uint32_t *out_rh_x1000)
{
    if (!g_bme.ok || g_dev == NULL) {
        return false;
    }

    /* Optional: warten bis interne NVM-Aktualisierung fertig ist */
    uint8_t st = 0;
    for (int i = 0; i < 10; i++) {
        if (bme_read(BME280_REG_STATUS, &st, 1) != ESP_OK) {
            break;
        }
        if ((st & 0x01U) == 0) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    /* Rohdatenblock lesen:
     *   F7..F9  Druck
     *   FA..FC  Temperatur
     *   FD..FE  Feuchte
     */
    uint8_t d[8] = {0};
    if (bme_read(BME280_REG_PRESS_MSB, d, sizeof(d)) != ESP_OK) {
        return false;
    }

    int32_t adc_P = (int32_t)((d[0] << 12) | (d[1] << 4) | (d[2] >> 4));
    int32_t adc_T = (int32_t)((d[3] << 12) | (d[4] << 4) | (d[5] >> 4));
    int32_t adc_H = (int32_t)((d[6] << 8)  |  d[7]);

    int32_t  t_x100   = bme_comp_T_x100(adc_T, &g_bme);
    uint32_t p_pa     = bme_comp_P_Pa(adc_P, &g_bme);
    uint32_t rh_x1000 = 0;

    if (g_bme.kind == BME_KIND_BME280) {
        rh_x1000 = bme_comp_H_x1000(adc_H, &g_bme);
    }

    if (out_t_x100 != NULL) {
        *out_t_x100 = t_x100;
    }
    if (out_p_pa != NULL) {
        *out_p_pa = p_pa;
    }
    if (out_rh_x1000 != NULL) {
        *out_rh_x1000 = rh_x1000;
    }

    return true;
}

/**
 * @brief Gibt zurück, ob der Sensor erfolgreich initialisiert wurde.
 */
bool bme280_is_ok(void)
{
    return g_bme.ok;
}