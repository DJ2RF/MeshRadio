/* ============================================================================
 * FILE: radio_config.h
 * ============================================================================
 * Radio helpers 
 * Include AFTER config_meshradio.h
 * ============================================================================
 */
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