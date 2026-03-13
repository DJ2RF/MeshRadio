#include "incl/mr_wifi_ota.h"
#include "incl/mr_wifi_ota_web.h"

#include "incl/config_meshradio.h"

#include "esp_app_desc.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>

static const char *TAG = "mr_wifi_ota";

/* ---------------------------------------------------------
   OTA Status
   --------------------------------------------------------- */

static esp_err_t ota_status_handler(httpd_req_t *req)
{
    const esp_app_desc_t *app = esp_app_get_description();

    char resp[256];

    int len = snprintf(
        resp, sizeof(resp),
        "{ \"status\": \"ready\", \"version\": \"%s\", \"target\": \"%s\" }",
        app->version,
        MR_BOARD_NAME
    );

    if (len < 0 || len >= (int)sizeof(resp))
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Status buffer too small");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

/* ---------------------------------------------------------
   OTA Upload
   --------------------------------------------------------- */

static esp_err_t ota_upload_handler(httpd_req_t *req)
{
    esp_ota_handle_t ota_handle = 0;
    const esp_partition_t *ota_partition = esp_ota_get_next_update_partition(NULL);

    if (!ota_partition)
    {
        ESP_LOGE(TAG, "No OTA partition found");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Starting OTA update");
    ESP_LOGI(TAG, "Incoming image length: %d bytes", req->content_len);

    if (esp_ota_begin(ota_partition, OTA_SIZE_UNKNOWN, &ota_handle) != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_begin failed");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return ESP_FAIL;
    }

    int remaining = req->content_len;
    char buffer[1024];

    while (remaining > 0)
    {
        size_t chunk = (remaining < (int)sizeof(buffer)) ? (size_t)remaining : sizeof(buffer);

        int recv = httpd_req_recv(req, buffer, chunk);

        if (recv <= 0)
        {
            ESP_LOGE(TAG, "Receive failed");
            esp_ota_end(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive failed");
            return ESP_FAIL;
        }

        if (esp_ota_write(ota_handle, buffer, recv) != ESP_OK)
        {
            ESP_LOGE(TAG, "OTA write failed");
            esp_ota_end(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA write failed");
            return ESP_FAIL;
        }

        remaining -= recv;
    }

    if (esp_ota_end(ota_handle) != ESP_OK)
    {
        ESP_LOGE(TAG, "OTA end failed");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA end failed");
        return ESP_FAIL;
    }

    if (esp_ota_set_boot_partition(ota_partition) != ESP_OK)
    {
        ESP_LOGE(TAG, "Boot partition set failed");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Boot set failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA successful - rebooting");

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "OTA OK - rebooting");

    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();

    return ESP_OK;
}

/* ---------------------------------------------------------
   Rollback-Bestätigung
   --------------------------------------------------------- */

void mr_wifi_ota_confirm_running_image(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;

    if (!running)
        return;

    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK)
    {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
        {
            ESP_LOGI(TAG, "Running image pending verify -> mark valid");
            esp_ota_mark_app_valid_cancel_rollback();
        }
    }
}

/* ---------------------------------------------------------
   Registrierung
   --------------------------------------------------------- */

void mr_wifi_ota_register(httpd_handle_t server)
{
    httpd_uri_t status_uri = {
        .uri      = "/api/ota/status",
        .method   = HTTP_GET,
        .handler  = ota_status_handler,
        .user_ctx = NULL
    };

    httpd_uri_t upload_uri = {
        .uri      = "/api/ota/upload",
        .method   = HTTP_POST,
        .handler  = ota_upload_handler,
        .user_ctx = NULL
    };

    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &status_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &upload_uri));

    mr_wifi_ota_web_register(server);

    ESP_LOGI(TAG, "OTA endpoints registered");
}