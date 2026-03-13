#ifndef MR_WIFI_OTA_WEB_H
#define MR_WIFI_OTA_WEB_H

#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
    OTA Browser-Webseite
    --------------------

    Registriert:
      GET /update

    Die Seite zeigt:
      - Projektname
      - Version
      - Target
      - Build-Datum
      - Build-Zeit
      - ESP-IDF Version

    und erlaubt das direkte Hochladen einer .bin Datei.
*/

void mr_wifi_ota_web_register(httpd_handle_t server);

#ifdef __cplusplus
}
#endif

#endif /* MR_WIFI_OTA_WEB_H */