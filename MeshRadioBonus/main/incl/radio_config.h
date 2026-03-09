/******************************************************************************
 *  MeshRadio Project
 *
 *  FILE: radio_config.h
 *
 *  DESCRIPTION
 *  ---------------------------------------------------------------------------
 *  Radio configuration helper functions for the MeshRadio firmware.
 *
 *  This header provides small utility functions used for configuring LoRa
 *  radio chips supported by the MeshRadio project. It converts human-readable
 *  frequency values (Hz) into the register formats required by the hardware.
 *
 *  Supported radio chips:
 *
 *      • SX1276 / SX1278
 *        Register-based LoRa transceivers commonly used in ESP32 boards
 *        such as LILYGO / TTGO devices.
 *
 *      • SX1262
 *        Command-based LoRa transceiver used in newer ESP32-S3 boards
 *        such as the Heltec LoRa 32 V3.x series.
 *
 *  The helper functions convert frequency values into:
 *
 *      - SX127x FRF register format
 *      - SX126x RF frequency step format
 *
 *  These conversions are required because both radio families internally use
 *  fixed-point frequency representations derived from the 32 MHz crystal
 *  reference clock.
 *
 *  This file must be included AFTER "config_meshradio.h".
 *
 *
 *  AUTHOR
 *  ---------------------------------------------------------------------------
 *  Friedrich Riedhammer (Fritz)
 *  NerdVerlag
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
 *  Commercial use or redistribution requires permission from the author.
 *
 *  This software is intended for experimentation and research in wireless
 *  mesh networking using LoRa technology. The author does not guarantee
 *  correctness, regulatory compliance, or suitability for any specific
 *  purpose.
 *
 *  Users are responsible for complying with local radio regulations.
 *
 *  Use at your own risk.
 *
 ******************************************************************************/

#pragma once

#include <stdint.h>
#include "config_meshradio.h"

// LoRa TX timeout (für beide Chips)
#define LORA_TX_TIMEOUT_MS 2000

// SX1276/78: FRF register value from Hz
static inline uint32_t mr_sx127x_hz_to_frf(uint32_t f_hz)
{
    // FRF = Freq * 2^19 / 32e6
    uint64_t frf = ((uint64_t)f_hz << 19) / 32000000ULL;
    return (uint32_t)frf;
}

// SX1262: RF frequency steps from Hz
static inline uint32_t mr_sx126x_hz_to_rf(uint32_t f_hz)
{
    // rf = Freq * 2^25 / 32e6
    uint64_t rf = ((uint64_t)f_hz << 25) / 32000000ULL;
    return (uint32_t)rf;
}