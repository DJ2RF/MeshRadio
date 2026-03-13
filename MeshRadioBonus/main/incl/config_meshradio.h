/******************************************************************************
 *  MeshRadio Project
 *
 *  FILE: config_meshradio.h
 *
 *  DESCRIPTION
 *  ---------------------------------------------------------------------------
 *  Central configuration file for the MeshRadio firmware.
 *
 *  This header defines all user-adjustable configuration parameters used by
 *  the MeshRadio system. It acts as the main control panel for enabling or
 *  disabling features, selecting hardware platforms, and adjusting network
 *  behavior.
 *
 *  The configuration options include:
 *
 *      • Board selection and hardware presets
 *      • Node identity (callsign)
 *      • WiFi access point configuration
 *      • LoRa radio frequency and regional band settings
 *      • Mesh control plane parameters (beacons, routing, ACK retries)
 *      • Token bucket rate limiting
 *      • Optional channel activity detection (CAD)
 *      • AES-CCM encryption and network security parameters
 *      • Node operating roles (RELAY, EDGE, SENSOR)
 *      • Power management and deep sleep behavior
 *      • Relay control features
 *      • Battery monitoring configuration
 *      • Serial command line interface (CLI)
 *      • Optional sensors (e.g. BME280)
 *
 *  The options defined in this file are intentionally documented in detail
 *  to support both firmware development and educational use in the MeshRadio
 *  documentation and book series.
 *
 *  IMPORTANT
 *  ---------------------------------------------------------------------------
 *  - This file is the primary configuration entry point for the firmware.
 *  - Changes here directly affect network behavior and hardware operation.
 *  - Board presets determine pin mapping and radio driver selection.
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

// ---- Board Presets (einfach hier umschalten) ----
/*
 * MR_BOARD_LILYGO_SX1276:
 *   - ESP32 + SX1276/SX1278 (Register-Driver)
 *   - Beispiele: LILYGO/TTGO T3, T-Beam Varianten etc.
 *
 * MR_BOARD_HELTEC_V3:
 *   - ESP32-S3 + SX1262 (Command-Driver)
 *   - Beispiele: Heltec LoRa 32 V3.x / V3.2
 */
#define MR_BOARD_LILYGO_SX1276   1
#define MR_BOARD_HELTEC_V3       2

/*
 * MR_BOARD_PRESET:
 *   - Auswahl des aktiven Boards (1 von 2 Presets)
 *   - Umschalten hier bestimmt Pinout, LoRa-Chip-Treiber und Board-Fixes.
 *  
 * - Wichtig: Bei Änderung hier auch die entsprechende Board-Auswahl in der
 *  Build-Konfiguration (KConfig) anpassen, damit die richtigen Quellen kompiliert
 *  werden.
 */
#ifndef MR_BOARD_PRESET
#define MR_BOARD_PRESET MR_BOARD_LILYGO_SX1276  // <-- HIER UMSCHALTEN
#endif

/* -----------------------------------------------------------
   Board-Name als lesbarer String für Web / OTA / Debug
   ----------------------------------------------------------- */
#if MR_BOARD_PRESET == MR_BOARD_LILYGO_SX1276
#define MR_BOARD_NAME "LILYGO SX1276"

#elif MR_BOARD_PRESET == MR_BOARD_HELTEC_V3
#define MR_BOARD_NAME "Heltec V3"

#else
#define MR_BOARD_NAME "Unknown Board"
#endif

// ---- Callsign / WiFi ----
/*
 * g_callsign:
 *   - Node-ID/Callsign (max. 8 Zeichen werden im Protokoll genutzt)
 *   - Wird in Frames als Quelle (src) verwendet und im Web/CLI angezeigt.
 * 
 */

#define MR_CALLSIGN        "DJ2RF-20"
#define MR_RELAY_CALLSIGN  "DJ2RF-20"


/*
 * MR_WIFI_AP_SSID / MR_WIFI_AP_PASS:
 *   - SSID/Passwort des Access Points (Web-Dashboard)
 *   - PASS = ""  -> OPEN (ohne Passwort)
 *   - PASS >= 8  -> WPA2-PSK
 */
#define MR_WIFI_AP_SSID    "MeshRadio-Setup2"
#define MR_WIFI_AP_PASS    ""          // "" => OPEN; >=8 Zeichen => WPA2-PSK

// =========================== WIFI STA CONFIG ===============================

// Router WLAN (Station Mode)
#define MR_WIFI_STA_ENABLE   1

#define MR_WIFI_STA_SSID     "farswitch"
#define MR_WIFI_STA_PASS     "Kl79_?Sa13_04_1961Kl79_?Sa"
// DHCP Hostname automatisch generieren (MeshRadio-XX)
#define MR_WIFI_STA_DHCP_HOSTNAME  1
#define MR_WIFI_STA_RETRY_MAX  2

// ---- RF Frequency (EU 863–870) ----
/*
 * DEFAULT_RF_FREQ_HZ:
 *   - LoRa Center Frequency in Hz
 *   - Muss zu Region/Bandplan passen (Regulierung beachten!)
 *   - Beispiele:
 *       863000000UL (EU 863–870)
 *       433050000UL (433 MHz Band – experimentell/je nach Region)
 */
//#define DEFAULT_RF_FREQ_HZ 863000000UL
#define DEFAULT_RF_FREQ_HZ 433050000UL

// ---- Control Plane ----
/*
 * DEFAULT_BEACON_INTERVAL_MS:
 *   - Beacon-Sendeintervall (Nachbarschaft/Route-Discovery)
 *
 * BEACON_JITTER_MS:
 *   - Zufalls-Jitter, um Kollisionen zu reduzieren (Anti-Sync)
 */
#define DEFAULT_BEACON_INTERVAL_MS 30000
#define BEACON_JITTER_MS           2000

/*
 * ACK_TIMEOUT_MS:
 *   - Wartezeit auf ACK bei ACKREQ (pro Versuch)
 *
 * ACK_RETRY_MAX:
 *   - Maximale Anzahl Retries (zusätzlich zum ersten Send)
 *
 * ACK_BACKOFF_MS:
 *   - Zufälliger Backoff (0..ACK_BACKOFF_MS) vor einem Retry
 */
#define ACK_TIMEOUT_MS     2200
#define ACK_RETRY_MAX      2
#define ACK_BACKOFF_MS     500

/*
 * ETX_MAX_X100:
 *   - ETX Obergrenze (x100), z.B. 1000 = ETX 10.00
 *
 * ETX_DECAY_MS:
 *   - Halbwerts-Decay für Link-Statistik
 */
#define ETX_MAX_X100       1000
#define ETX_DECAY_MS       60000

/*
 * DEFAULT_ROUTEADV_ENABLE:
 *   - 1: Route-Advertisements aktiv
 *   - 0: Aus
 *
 * DEFAULT_ROUTEADV_TOPN:
 *   - Anzahl Top-Routen im Advertise
 *
 * DEFAULT_ROUTEADV_DELTA_ETX:
 *   - Hysterese/Schwellwert für Route-Update (ETX-Vergleich)
 *
 * DEFAULT_HOLDDOWN_MS:
 *   - Hold-Down Zeit nach Next-Hop Wechsel
 */
#define DEFAULT_ROUTEADV_ENABLE    1
#define DEFAULT_ROUTEADV_TOPN      8
#define DEFAULT_ROUTEADV_DELTA_ETX 30
#define DEFAULT_HOLDDOWN_MS        20000

// ---- Token bucket rate limits ----
/*
 * Token-Bucket:
 *   - tps   = Tokens pro Sekunde
 *   - burst = Max. Burst
 */
#define RL_BEACON_TPS      0.05f
#define RL_BEACON_BURST    1.0f
#define RL_ROUTEADV_TPS    0.20f
#define RL_ROUTEADV_BURST  2.0f
#define RL_ACK_TPS         1.50f
#define RL_ACK_BURST       5.0f
#define RL_DATA_TPS        0.50f
#define RL_DATA_BURST      3.0f

// ---- CAD optional ----
/*
 * DEFAULT_CAD_ENABLE:
 *   - 1: CAD/extra Backoff vor TX
 *   - 0: Aus
 */
#define DEFAULT_CAD_ENABLE 0
#define CAD_WAIT_MS        80
#define CAD_JITTER_MS      120

// ---- Kapitel 29 Security Defaults ----
/*
 * DEFAULT_CRYPTO_ENABLE:
 *   - 1: AES-CCM aktiv
 *   - 0: Klartext
 */
#define DEFAULT_CRYPTO_ENABLE  1
#define MR_NET_KEY_HEX "00112233445566778899AABBCCDDEEFF"
#define MR_NET_ID 0x42

#define SEC_KEY_LEN        16
#define SEC_NONCE_LEN      12
#define SEC_TAG_LEN        8

#if SEC_TAG_LEN != 8
#error "SEC_TAG_LEN must be 8 for MeshRadio protocol"
#endif

// ---- Kapitel 34 Node Roles ----
/*
 * DEFAULT_NODE_MODE:
 *   - 0 = RELAY, 1 = EDGE, 2 = SENSOR
 */
#define DEFAULT_NODE_MODE  0   // 0=RELAY, 1=EDGE, 2=SENSOR

// ---- Power save  ----
/*
 * MR_POWERSAVE_ENABLE:
 *   - 1: DeepSleep Support aktiv (typisch nur SENSOR)
 *   - 0: Dauerbetrieb
 */
#define MR_POWERSAVE_ENABLE       1        // not at Relay and Edge, optional at Sensor
#define SENSOR_WAKE_PERIOD_MS     60000
#define SENSOR_BOOT_RX_WINDOW_MS  30000

/*
 * EDGE_DUTY_RX_ENABLE:
 *   - 1: EDGE Duty-Cycle RX
 *   - 0: RX dauerhaft
 */
#define EDGE_DUTY_RX_ENABLE       0
#define EDGE_RX_ON_MS             600
#define EDGE_RX_OFF_MS            1400


// ---- WiFi runtime default ----
/*
 * MR_WIFI_RUNTIME_DEFAULT:
 *   - 1: WiFi+HTTP direkt beim Boot starten
 *   - 0: aus
 */
#define MR_WIFI_RUNTIME_DEFAULT   1  // 1=WiFi+HTTP direkt beim Boot, 0=aus

// ---- Station: Relay ----
/*
 * MR_RELAY_ENABLE:
 *   - 1: Relay GPIO Feature aktiv
 *   - 0: aus
 */
#define MR_RELAY_ENABLE           1

#define RELAY_ACTIVE_LEVEL        1   // 1: HIGH=ON, 0: LOW=ON

// RELAY_GPIO: siehe board_pins.h, wird hier nur als Default definiert, kann aber auch zur Laufzeit geändert werden
// #define RELAY_GPIO                35  //25  // default relay pin

// ---- Station: Battery ADC ----
/*
 * MR_BATT_ENABLE:
 *   - 1: Battery Monitoring aktiv
 *   - 0: aus
 */
#define MR_BATT_ENABLE            1
#define BATT_MEASURE_INTERVAL_MS  5000
#define BATT_EMPTY_MV             3300
#define BATT_FULL_MV              4200
#define BATT_CAL_FACTOR 1.13f  // Kalibrierungsfaktor für ADC-Wert (z.B. 1.13 = 13% höher als gemessen, um realen Wert zu treffen)
#define MR_ADC_CALI_MODE          0

// ---- Serial CLI ----
/*
 * MR_CLI_ENABLE:
 *   - 1: UART CLI aktiv
 *   - 0: aus
 */
#define MR_CLI_ENABLE             1
#define MR_CLI_LINE_MAX           200

/*
 * MR_BME280_ENABLE:
 *   - 1: BME280 aktiv
 *   - 0: aus
 */
#define MR_BME280_ENABLE          1
#define BME280_I2C_ADDR           0x76   // oft 0x76, manchmal 0x77
#define BME280_I2C_CLK_HZ         100000 // 100k = robust

#define BME_PRESS_OFFSET_HPA      50     // Korrektur für Höhenlage (z.B. 50 hPa für 500m über NN)