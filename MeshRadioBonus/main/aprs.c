#include "incl/aprs.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/inet.h"

static const char *TAG = "APRS";

extern bool g_wifi_enabled;

bool aprs_build_packet_from_cfg(const aprs_cfg_t *cfg,
                                const char *mesh_msg,
                                char *out_packet,
                                size_t out_packet_sz)
{
    if(!cfg || !mesh_msg || !out_packet || out_packet_sz < 32){
        return false;
    }

    if(!cfg->enabled) return false;
    if(cfg->callsign[0] == 0) return false;

    snprintf(out_packet, out_packet_sz,
             "%s>APRS,TCPIP*:%s",
             cfg->callsign,
             mesh_msg);

    return true;
}

bool aprs_send_packet_with_cfg(const aprs_cfg_t *cfg, const char *packet)
{
    if(!cfg || !packet || !*packet){
        ESP_LOGW(TAG, "APRS: empty cfg/packet");
        return false;
    }

    if(!cfg->enabled){
        ESP_LOGW(TAG, "APRS: disabled");
        return false;
    }

    if(cfg->host[0] == 0){
        ESP_LOGW(TAG, "APRS: host missing");
        return false;
    }

    if(cfg->callsign[0] == 0){
        ESP_LOGW(TAG, "APRS: callsign missing");
        return false;
    }

    if(cfg->passcode[0] == 0){
        ESP_LOGW(TAG, "APRS: passcode missing");
        return false;
    }

    if(cfg->port == 0){
        ESP_LOGW(TAG, "APRS: invalid port");
        return false;
    }

    if(!g_wifi_enabled){
        ESP_LOGW(TAG, "APRS: WiFi not enabled");
        return false;
    }

    ESP_LOGI(TAG, "APRS runtime: host=%s port=%u call=%s",
             cfg->host, (unsigned)cfg->port, cfg->callsign);

    struct hostent *he = gethostbyname(cfg->host);
    if(!he || !he->h_addr_list || !he->h_addr_list[0]){
        ESP_LOGW(TAG, "APRS: DNS failed for %s", cfg->host);
        return false;
    }

    struct sockaddr_in dest = {0};
    dest.sin_family = AF_INET;
    dest.sin_port = htons(cfg->port);
    memcpy(&dest.sin_addr, he->h_addr_list[0], he->h_length);

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if(sock < 0){
        ESP_LOGW(TAG, "APRS: socket failed");
        return false;
    }

    bool ok = false;

    struct timeval tv = {
        .tv_sec = APRS_SEND_TIMEOUT_MS / 1000,
        .tv_usec = (APRS_SEND_TIMEOUT_MS % 1000) * 1000
    };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    ESP_LOGI(TAG, "APRS: connect %s:%u call=%s",
             cfg->host, (unsigned)cfg->port, cfg->callsign);

    if(connect(sock, (struct sockaddr*)&dest, sizeof(dest)) != 0){
        ESP_LOGW(TAG, "APRS: connect failed to %s:%u",
                 cfg->host, (unsigned)cfg->port);
        goto done;
    }

    char rxbuf[256];
    int n;

    memset(rxbuf, 0, sizeof(rxbuf));
    n = recv(sock, rxbuf, sizeof(rxbuf) - 1, 0);
    if(n > 0){
        rxbuf[n] = 0;
        ESP_LOGI(TAG, "APRS: server hello: %s", rxbuf);
    }

    char login[128];
    snprintf(login, sizeof(login),
             "user %s pass %s vers MeshRadio 1.0\r\n",
             cfg->callsign, cfg->passcode);

    if(send(sock, login, strlen(login), 0) < 0){
        ESP_LOGW(TAG, "APRS: login send failed");
        goto done;
    }

    ESP_LOGI(TAG, "APRS: login sent for %s", cfg->callsign);

    vTaskDelay(pdMS_TO_TICKS(500));

    memset(rxbuf, 0, sizeof(rxbuf));
    n = recv(sock, rxbuf, sizeof(rxbuf) - 1, 0);
    if(n > 0){
        rxbuf[n] = 0;
        ESP_LOGI(TAG, "APRS: login reply: %s", rxbuf);

        if(strstr(rxbuf, "unverified") || strstr(rxbuf, "invalid") || strstr(rxbuf, "reject")){
            ESP_LOGW(TAG, "APRS: login not accepted");
            goto done;
        }
    } else {
        ESP_LOGW(TAG, "APRS: no login reply from server");
    }

    if(send(sock, packet, strlen(packet), 0) < 0){
        ESP_LOGW(TAG, "APRS: packet send failed");
        goto done;
    }

    if(send(sock, "\r\n", 2, 0) < 0){
        ESP_LOGW(TAG, "APRS: CRLF send failed");
        goto done;
    }

    ESP_LOGI(TAG, "APRS-IS TX: %s", packet);

    vTaskDelay(pdMS_TO_TICKS(500));

    memset(rxbuf, 0, sizeof(rxbuf));
    n = recv(sock, rxbuf, sizeof(rxbuf) - 1, 0);
    if(n > 0){
        rxbuf[n] = 0;
        ESP_LOGI(TAG, "APRS: post-TX reply: %s", rxbuf);
    }

    ok = true;

done:
    shutdown(sock, 0);
    close(sock);
    return ok;
}