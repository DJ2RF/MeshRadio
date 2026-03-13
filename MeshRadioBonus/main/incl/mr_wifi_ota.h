#ifndef MR_WIFI_OTA_H
#define MR_WIFI_OTA_H

#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
    OTA Modul für MeshRadio
    -----------------------

    Registriert:
      GET  /api/ota/status
      POST /api/ota/upload
      GET  /update

    Verwendung:
      - HTTP-Server normal starten
      - danach mr_wifi_ota_register(server) aufrufen

    Optional:
      - mr_wifi_ota_confirm_running_image() früh in app_main()
        aufrufen, wenn Rollback-Schutz aktiv ist.
*/

/* Registriert OTA-Endpunkte und OTA-Webseite am bestehenden HTTP-Server */
void mr_wifi_ota_register(httpd_handle_t server);

/* Bestätigt eine frisch gestartete OTA-Firmware als gültig */
void mr_wifi_ota_confirm_running_image(void);

#ifdef __cplusplus
}
#endif

#endif /* MR_WIFI_OTA_H */