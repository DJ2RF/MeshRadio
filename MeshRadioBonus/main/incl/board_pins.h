/******************************************************************************
 *  MeshRadio Project
 *
 *  FILE: board_pins.h
 *
 *  DESCRIPTION
 *  ---------------------------------------------------------------------------
 *  Board-specific hardware configuration for the MeshRadio firmware.
 *
 *  This header defines all GPIO assignments required for the radio, SPI bus,
 *  battery measurement, I²C interface, and optional peripherals depending on
 *  the selected board preset.
 *
 *  Supported boards currently include:
 *
 *      - Heltec LoRa 32 V3.x (SX1262)
 *      - LILYGO / TTGO SX1276 based boards
 *      - T-Beam V1.1 (SX1276 + GPS)
 *      - T-Beam V1.2 (AXP2101 + GPS)
 *
 *  The file provides a unified abstraction layer for different hardware
 *  layouts so that the main firmware can operate independently of the
 *  physical board wiring.
 *
 *  Each board preset defines:
 *
 *      - LoRa SPI interface pins (SCK, MOSI, MISO, NSS)
 *      - LoRa control lines (RESET, IRQ, BUSY)
 *      - battery measurement pins and divider ratios
 *      - I²C bus pins for sensors or displays
 *      - optional relay or control GPIOs
 *
 *  Additional compile-time guards are implemented to prevent hardware pin
 *  conflicts (for example between LoRa IRQ lines and relay outputs).
 *
 *  This file must be included AFTER "config_meshradio.h" so that the selected
 *  board preset (MR_BOARD_PRESET) is already defined.
 *
 *  AUTHOR
 *  ---------------------------------------------------------------------------
 *  Friedrich Riedhammer (Fritz)
 *  https://nerdverlag.com
 *  fritz@nerdverlag.com
 *
 *  COPYRIGHT
 *  ---------------------------------------------------------------------------
 *  (c) 2026 Friedrich Riedhammer / NerdVerlag
 *
 *  This software is provided "as is", without any express or implied warranty.
 *  In no event will the author be held liable for any damages arising from
 *  the use of this software.
 *
 *  Permission is granted to use, modify, and distribute this software for
 *  educational, experimental, and amateur radio purposes, provided that this
 *  copyright notice and this disclaimer remain intact in all copies.
 *
 *  This software is intended for experimentation and research in wireless
 *  mesh networking. The author does not guarantee correctness, reliability,
 *  or suitability for any specific purpose.
 *
 *  Use at your own risk.
 *
 ******************************************************************************/

#pragma once

#include "config_meshradio.h"

// ============================================================================
// ============================ BOARD PIN PRESETS =============================
// ============================================================================
//
// Einheitliche LoRa-Signale:
//   - SPI: SCK / MOSI / MISO + NSS (CS)
//   - RESET
//   - IRQ:
//        SX1276: DIO0  (RxDone / TxDone)
//        SX1262: DIO1  (IRQ line)
//   - SX1262 zusätzlich: BUSY
//
// Heltec V3.x Besonderheit:
//   - VEXT_CTRL (GPIO36) muss LOW (ON) gesetzt werden.
// ============================================================================

#if (MR_BOARD_PRESET == MR_BOARD_HELTEC_V3)

// ----------------------------------------------------------------------------
// Heltec LoRa 32 V3.x (SX1262)
// ----------------------------------------------------------------------------
#define MR_LORA_CHIP_SX1262 1

#define PIN_LORA_NSS         8
#define PIN_LORA_SCK         9
#define PIN_LORA_MOSI       10
#define PIN_LORA_MISO       11
#define PIN_LORA_RST        12
#define PIN_LORA_BUSY       13
#define PIN_LORA_DIO1       14    // IRQ line

// Battery divider enable (Heltec)
#define BATT_EN_GPIO        37
#define BATT_EN_ACTIVE_LOW   0

// VEXT (Heltec: LOW = ON)
#define VEXT_CTRL_PIN       36
#define PIN_VEXT_CTRL       36

// VBAT sense pin am Heltec V3
#define BATT_ADC_GPIO        1

// Typischer Teiler: 390k / 100k => Faktor 4.9
#define BATT_DIV_RTOP_OHMS   390000.0f
#define BATT_DIV_RBOT_OHMS   100000.0f

// I2C
#define PIN_I2C_SDA         41
#define PIN_I2C_SCL         42
#define PIN_OLED_SDA        17
#define PIN_OLED_SCL        18
#define PIN_SENSOR_I2C_SDA  41
#define PIN_SENSOR_I2C_SCL  42
#define OLED_I2C_PORT       I2C_NUM_0
#define SENSOR_I2C_PORT     I2C_NUM_1


// RELAY: darf NICHT auf IRQ-Pin liegen
#ifndef RELAY_GPIO
#define RELAY_GPIO          35
#endif

// optionale Default-Werte für Boards ohne GPS
#ifndef PIN_GPS_TX
#define PIN_GPS_TX          (-1)
#endif
#ifndef PIN_GPS_RX
#define PIN_GPS_RX          (-1)
#endif
#ifndef PIN_GPS_PPS
#define PIN_GPS_PPS         (-1)
#endif

#elif (MR_BOARD_PRESET == MR_BOARD_LILYGO_SX1276)

// ----------------------------------------------------------------------------
// LILYGO / TTGO SX1276/78
// ----------------------------------------------------------------------------
#define MR_LORA_CHIP_SX1276 1

#define PIN_LORA_NSS        18
#define PIN_LORA_SCK         5
#define PIN_LORA_MOSI       27
#define PIN_LORA_MISO       19
#define PIN_LORA_RST        23
#define PIN_LORA_DIO0       26

#ifndef RELAY_GPIO
#define RELAY_GPIO          25
#endif

// Battery sense, typisch: GPIO35, Teiler 100k / 100k
#define BATT_ADC_GPIO       35
#define BATT_DIV_RTOP_OHMS  100000.0f
#define BATT_DIV_RBOT_OHMS  100000.0f

#define PIN_I2C_SDA         21
#define PIN_I2C_SCL         22

#ifndef PIN_GPS_TX
#define PIN_GPS_TX          (-1)
#endif
#ifndef PIN_GPS_RX
#define PIN_GPS_RX          (-1)
#endif
#ifndef PIN_GPS_PPS
#define PIN_GPS_PPS         (-1)
#endif

#elif (MR_BOARD_PRESET == MR_BOARD_TBEAM_V11_SX1276)

// ----------------------------------------------------------------------------
// T-Beam / T22 V1.1 (SX1276 + GPS)
// ----------------------------------------------------------------------------
#define MR_LORA_CHIP_SX1276 1

#define PIN_LORA_NSS        18
#define PIN_LORA_SCK         5
#define PIN_LORA_MOSI       27
#define PIN_LORA_MISO       19
#define PIN_LORA_RST        23
#define PIN_LORA_DIO0       26

#ifndef RELAY_GPIO
#define RELAY_GPIO          25
#endif

// Battery ADC klassisch
#define BATT_ADC_GPIO       35
#define BATT_DIV_RTOP_OHMS  100000.0f
#define BATT_DIV_RBOT_OHMS  100000.0f

#define PIN_I2C_SDA         21
#define PIN_I2C_SCL         22

// GPS
#define PIN_GPS_TX          12
#define PIN_GPS_RX          34
#define PIN_GPS_PPS         37

#elif (MR_BOARD_PRESET == MR_BOARD_TBEAM_V12_AXP2101)

// ----------------------------------------------------------------------------
// T-Beam V1.2 (AXP2101 + GPS)
// ----------------------------------------------------------------------------
#define MR_LORA_CHIP_SX1276 1

#define PIN_LORA_NSS        18
#define PIN_LORA_SCK         5
#define PIN_LORA_MOSI       27
#define PIN_LORA_MISO       19
#define PIN_LORA_RST        23
#define PIN_LORA_DIO0       26

#ifndef RELAY_GPIO
#define RELAY_GPIO          25
#endif

#define PIN_I2C_SDA         21
#define PIN_I2C_SCL         22

// GPS
#define PIN_GPS_TX          12
#define PIN_GPS_RX          34
#define PIN_GPS_PPS         (-1)

// Kompatibilitäts-Makros für alten Battery-Code in main.c
// Akku später über AXP2101 / mr_pmu.c lesen
#define BATT_ADC_GPIO       (-1)
#define BATT_DIV_RTOP_OHMS  100000.0f
#define BATT_DIV_RBOT_OHMS  100000.0f

#else
#error "Unknown MR_BOARD_PRESET"
#endif

// ============================================================================
// ============================== COMPAT HELPERS ===============================
// ============================================================================

// Falls ein Board keinen BUSY-Pin hat
#ifndef PIN_LORA_BUSY
#define PIN_LORA_BUSY       (-1)
#endif

// Falls ein Board keinen DIO0/DIO1-Pin hat
#ifndef PIN_LORA_DIO0
#define PIN_LORA_DIO0       (-1)
#endif

#ifndef PIN_LORA_DIO1
#define PIN_LORA_DIO1       (-1)
#endif

#ifndef BATT_EN_ACTIVE_LOW
#define BATT_EN_ACTIVE_LOW   0
#endif


#ifndef VEXT_CTRL_PIN
#define VEXT_CTRL_PIN       (-1)
#endif

#ifndef PIN_VEXT_CTRL
#define PIN_VEXT_CTRL       VEXT_CTRL_PIN
#endif

// ============================================================================
// ============================= CONFLICT GUARDS ===============================
// ============================================================================

// Heltec / SX1262: DIO1 ist IRQ
#if (MR_RELAY_ENABLE) && (RELAY_GPIO == PIN_LORA_DIO1) && (PIN_LORA_DIO1 >= 0)
#error "RELAY_GPIO conflicts with LoRa DIO1 (IRQ). Choose another pin!"
#endif

// SX1276: DIO0 ist IRQ
#if (MR_RELAY_ENABLE) && (RELAY_GPIO == PIN_LORA_DIO0) && (PIN_LORA_DIO0 >= 0)
#error "RELAY_GPIO conflicts with LoRa DIO0 (IRQ). Choose another pin!"
#endif
