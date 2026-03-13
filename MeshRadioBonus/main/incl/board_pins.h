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
 *
 *  AUTHOR
 *  ---------------------------------------------------------------------------
 *  Friedrich Riedhammer (Fritz)
 *  https://nerdverlag.com
 *  fritz@nerdverlag.com
 *
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
//   - SPI: SCK/MOSI/MISO + NSS (CS)
//   - RESET
//   - IRQ:
//        SX1276: DIO0  (RxDone/TxDone)
//        SX1262: DIO1  (IRQ line)
//   - SX1262 zusätzlich: BUSY
//
// Heltec V3.x Besonderheit:
//   - VEXT_CTRL (GPIO36) muss LOW (ON) gesetzt werden.
// ============================================================================

#if (MR_BOARD_PRESET == MR_BOARD_HELTEC_V3)

// Heltec LoRa 32 V3.x (SX1262) – typische Pins 
#define MR_LORA_CHIP_SX1262 1

#define PIN_LORA_NSS     8
#define PIN_LORA_SCK     9
#define PIN_LORA_MOSI   10
#define PIN_LORA_MISO   11
#define PIN_LORA_RST    12
#define PIN_LORA_BUSY   13
#define PIN_LORA_DIO1   14     // IRQ line

// Battery divider enable (Heltec)
#define BATT_EN_GPIO          37
#define BATT_EN_ACTIVE_LOW     0

// VEXT (Heltec: LOW = ON)
#define VEXT_CTRL_PIN         36
#define PIN_VEXT_CTRL         36

// VBAT Sense Pin am Heltec V3 (aktuell)
#define BATT_ADC_GPIO          1

// Für V3.2 ist 390k (Top) / 100k (Bot) üblich, ergibt Faktor 4.9
#define BATT_DIV_RTOP_OHMS  390000.0f
#define BATT_DIV_RBOT_OHMS  100000.0f

// I2C am externen Header (aktuell)
#define PIN_I2C_SDA          41
#define PIN_I2C_SCL          42

// RELAY: darf NICHT 14 sein (IRQ)!
#ifndef RELAY_GPIO
#define RELAY_GPIO           35  // default: 
#endif

#elif (MR_BOARD_PRESET == MR_BOARD_LILYGO_SX1276)

// LILYGO / TTGO SX1276/78 – Pins
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

// Battery sense beim T3 V1.6.1: GPIO35, Teiler 100k/100k (Faktor 2.0)
#define BATT_ADC_GPIO       35
#define BATT_DIV_RTOP_OHMS  100000.0f
#define BATT_DIV_RBOT_OHMS  100000.0f

// I2C (aktuell)
#define PIN_I2C_SDA         21
#define PIN_I2C_SCL         22

#else
#error "Unknown MR_BOARD_PRESET"
#endif

// GPIO conflict guard (Heltec V3: DIO1=14!)
#if defined(PIN_LORA_DIO1)
#if (MR_RELAY_ENABLE) && (RELAY_GPIO == PIN_LORA_DIO1)
#error "RELAY_GPIO conflicts with LoRa DIO1 (IRQ). Choose another pin!"
#endif
#endif