/* ============================================================================
 * FILE: config_meshradio.h
 * ============================================================================
 *
 *  MeshRadio – Dokumentations-Header (User Defines)
 * ============================================================================
 *
 *  Copyright (c) 2025–2026
 *  Friedrich Riedhammer  / DJ2RF
 *
 *  Projekt: MeshRadio (ESP-IDF v5.5.2)
 *  Zweck  : "profi-level stabil" LoRa-Mesh (SX1276/SX1278 und SX1262 umschaltbar)
 *
 *  Lizenz / Nutzung:
 *  - Nutzung, Anpassung und Weitergabe für Bildung / Amateurfunk-Projekte erlaubt.
 *  - Kommerzielle Nutzung/Weitergabe nur mit Zustimmung des Autors.
 *
 *  Haftungsausschluss:
 *  - Bereitgestellt "as-is" ohne Garantie.
 *  - Verantwortung für Funkregeln, Interferenzen, Schäden und Betrieb liegt beim Nutzer.
 *
 * ============================================================================
 *  WICHTIG:
 *  - Die folgenden Defines sind die zentralen Projekt-Schalter.
 *  - Reihenfolge entspricht exakt der Konfiguration im Code.
 *  - Jede Option ist kurz beschrieben, damit man sie im Projekt/Buch sofort versteht.
 * ============================================================================
 */
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
 */
#ifndef MR_BOARD_PRESET
#define MR_BOARD_PRESET MR_BOARD_LILYGO_SX1276 // MR_BOARD_HELTEC_V3  // <-- HIER UMSCHALTEN
#endif

// ---- Callsign / WiFi ----
/*
 * MR_CALLSIGN:
 *   - Node-ID/Callsign (max. 7 Zeichen werden im Protokoll genutzt)
 *   - Wird in Frames als Quelle (src) verwendet und im Web/CLI angezeigt.
 */
#define MR_CALLSIGN        "DL7ABCF"

/*
 * MR_WIFI_AP_SSID / MR_WIFI_AP_PASS:
 *   - SSID/Passwort des Access Points (Web-Dashboard)
 *   - PASS = ""  -> OPEN (ohne Passwort)
 *   - PASS >= 8  -> WPA2-PSK
 */
#define MR_WIFI_AP_SSID    "MeshRadio-Setup3"
#define MR_WIFI_AP_PASS    ""          // "" => OPEN; >=8 Zeichen => WPA2-PSK

// ---- RF Frequency (EU 863–870) ----
/*
 * DEFAULT_RF_FREQ_HZ:
 *   - LoRa Center Frequency in Hz
 *   - Muss zu Region/Bandplan passen (Regulierung beachten!)
 *   - Beispiele:
 *       863000000UL (EU 863–870)
 *       433050000UL (433 MHz Band – experimentell/je nach Region)
 */
// #define DEFAULT_RF_FREQ_HZ 863000000UL
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
#define ACK_TIMEOUT_MS     1200
#define ACK_RETRY_MAX      2
#define ACK_BACKOFF_MS     350

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
#define DEFAULT_CRYPTO_ENABLE  0
#define MR_NET_KEY_HEX "00112233445566778899AABBCCDDEEFF"
#define MR_NET_ID 0x42

#define SEC_KEY_LEN        16
#define SEC_NONCE_LEN      12
#define SEC_TAG_LEN        8

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
#define MR_POWERSAVE_ENABLE       0         // not at Relay and Edge, optional at Sensor
#define SENSOR_WAKE_PERIOD_MS     300000
#define SENSOR_BOOT_RX_WINDOW_MS  30000

/*
 * EDGE_DUTY_RX_ENABLE:
 *   - 1: EDGE Duty-Cycle RX
 *   - 0: RX dauerhaft
 */
#define EDGE_DUTY_RX_ENABLE       0
#define EDGE_RX_ON_MS             600
#define EDGE_RX_OFF_MS            1400
#define MR_RELAY_CALLSIGN "DJ1ABCF"

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

// ---- Station: Battery ADC ----
/*
 * MR_BATT_ENABLE:
 *   - 1: Battery Monitoring aktiv
 *   - 0: aus
 */
#define MR_BATT_ENABLE            1
#define BATT_MEASURE_INTERVAL_MS  5000
#define BATT_EMPTY_MV             3300
#define BATT_FULL_MV              3900
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
#define MR_BME280_ENABLE          0
#define BME280_I2C_ADDR           0x76   // oft 0x76, manchmal 0x77
#define BME280_I2C_CLK_HZ         100000 // 100k = robust

#define BME_PRESS_OFFSET_HPA      50     // Korrektur für Höhenlage (z.B. 50 hPa für 500m über NN)