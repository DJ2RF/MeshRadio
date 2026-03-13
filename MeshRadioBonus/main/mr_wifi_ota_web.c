#include "incl/mr_wifi_ota_web.h"

#include "incl/config_meshradio.h"

#include "esp_app_desc.h"
#include "esp_http_server.h"
#include "esp_log.h"

#include <stdio.h>

static const char *TAG = "mr_wifi_ota_web";

static esp_err_t ota_web_page_handler(httpd_req_t *req)
{
    const esp_app_desc_t *app = esp_app_get_description();

    char html[8192];

    int len = snprintf(
        html, sizeof(html),
        "<!doctype html>"
        "<html lang='de'>"
        "<head>"
        "  <meta charset='utf-8'>"
        "  <meta name='viewport' content='width=device-width,initial-scale=1'>"
        "  <title>MeshRadio OTA Update (c) 2023 Nerd-Verlag</title>"
        "  <style>"
        "    body{font-family:Arial,Helvetica,sans-serif;background:#0f172a;color:#e5e7eb;"
        "         margin:0;padding:24px;}"
        "    .card{max-width:760px;margin:0 auto;background:#111827;border:1px solid #374151;"
        "          border-radius:16px;padding:24px;box-shadow:0 8px 30px rgba(0,0,0,.30);}"
        "    h1{margin-top:0;font-size:28px;}"
        "    p{color:#cbd5e1;line-height:1.5;}"
        "    .row{margin:16px 0;}"
        "    .box{background:#0b1220;border:1px solid #334155;border-radius:12px;padding:14px;}"
        "    .muted{font-size:14px;color:#94a3b8;}"
        "    .ok{color:#22c55e;}"
        "    .err{color:#ef4444;}"
        "    input[type=file]{display:block;margin:10px 0;padding:8px;background:#1f2937;"
        "                     color:#e5e7eb;border-radius:10px;border:1px solid #4b5563;width:100%%;}"
        "    button{background:#2563eb;color:white;border:none;border-radius:10px;"
        "           padding:12px 18px;font-size:16px;cursor:pointer;}"
        "    button:disabled{opacity:.6;cursor:not-allowed;}"
        "    progress{width:100%%;height:22px;}"
        "    code{background:#1e293b;padding:2px 6px;border-radius:6px;}"
        "    table{width:100%%;border-collapse:collapse;}"
        "    td{padding:6px 0;vertical-align:top;}"
        "    td:first-child{color:#94a3b8;width:180px;}"
        "  </style>"
        "</head>"
        "<body>"
        "  <div class='card'>"
        "    <h1>MeshRadio OTA Update (c) 2023 Nerd-Verlag</h1>"
        "    <p>Firmware direkt im Browser hochladen. Bitte nur eine passende <code>.bin</code>-Datei verwenden.</p>"

        "    <div class='row box'>"
        "      <strong>Laufende Firmware</strong>"
        "      <table>"
        "        <tr><td>Projekt</td><td>%s</td></tr>"
        "        <tr><td>Version</td><td>%s</td></tr>"
        "        <tr><td>Target</td><td>%s</td></tr>"
        "        <tr><td>Build-Datum</td><td>%s</td></tr>"
        "        <tr><td>Build-Zeit</td><td>%s</td></tr>"
        "        <tr><td>ESP-IDF</td><td>%s</td></tr>"
        "      </table>"
        "    </div>"

        "    <div class='row box'>"
        "      <div><strong>Status:</strong> <span id='status'>Bereit</span></div>"
        "      <div class='muted'>Endpoint: <code>/api/ota/upload</code></div>"
        "    </div>"

        "    <div class='row'>"
        "      <label for='fw'><strong>Firmware-Datei auswählen</strong></label>"
        "      <input id='fw' type='file' accept='.bin,application/octet-stream'>"
        "      <div class='muted'>Empfohlen: Datei aus <code>build/</code>, z.B. <code>MeshRadioBonus.bin</code></div>"
        "    </div>"

        "    <div class='row'>"
        "      <button id='btn' onclick='uploadFw()'>Firmware hochladen</button>"
        "    </div>"

        "    <div class='row'>"
        "      <progress id='bar' max='100' value='0'></progress>"
        "      <div class='muted'><span id='pct'>0</span>%%</div>"
        "    </div>"

        "    <div class='row box'>"
        "      <strong>Log</strong>"
        "      <pre id='log' style='white-space:pre-wrap;margin:8px 0 0 0;color:#cbd5e1;'></pre>"
        "    </div>"
        "  </div>"

        "<script>"
        "function setStatus(txt, cls=''){"
        "  const s=document.getElementById('status');"
        "  s.className=cls;"
        "  s.textContent=txt;"
        "}"
        "function logln(txt){"
        "  const l=document.getElementById('log');"
        "  l.textContent += txt + '\\n';"
        "}"
        "function setProgress(v){"
        "  document.getElementById('bar').value=v;"
        "  document.getElementById('pct').textContent=String(v);"
        "}"
        "async function uploadFw(){"
        "  const fileInput=document.getElementById('fw');"
        "  const btn=document.getElementById('btn');"
        "  const file=fileInput.files[0];"
        "  if(!file){"
        "    setStatus('Bitte zuerst eine .bin-Datei auswählen','err');"
        "    logln('Keine Datei gewählt.');"
        "    return;"
        "  }"
        "  if(!file.name.toLowerCase().endsWith('.bin')){"
        "    setStatus('Datei ist keine .bin','err');"
        "    logln('Falscher Dateityp: ' + file.name);"
        "    return;"
        "  }"
        "  btn.disabled=true;"
        "  setProgress(0);"
        "  setStatus('Lese Datei...');"
        "  logln('Datei: ' + file.name + ' (' + file.size + ' Bytes)');"
        "  try{"
        "    const data = await file.arrayBuffer();"
        "    const xhr = new XMLHttpRequest();"
        "    xhr.open('POST','/api/ota/upload',true);"
        "    xhr.setRequestHeader('Content-Type','application/octet-stream');"
        "    xhr.upload.onprogress = function(e){"
        "      if(e.lengthComputable){"
        "        const p = Math.round((e.loaded / e.total) * 100);"
        "        setProgress(p);"
        "        setStatus('Upload läuft... ' + p + '%%');"
        "      }"
        "    };"
        "    xhr.onload = function(){"
        "      if(xhr.status >= 200 && xhr.status < 300){"
        "        setProgress(100);"
        "        setStatus('Upload erfolgreich - Gerät startet neu','ok');"
        "        logln('Server: ' + xhr.responseText);"
        "        logln('Wenn OTA erfolgreich war, startet das Gerät gleich neu.');"
        "      }else{"
        "        setStatus('Upload fehlgeschlagen (HTTP ' + xhr.status + ')','err');"
        "        logln('Fehler: ' + xhr.responseText);"
        "      }"
        "      btn.disabled=false;"
        "    };"
        "    xhr.onerror = function(){"
        "      setStatus('Netzwerkfehler beim Upload','err');"
        "      logln('XHR Netzwerkfehler');"
        "      btn.disabled=false;"
        "    };"
        "    setStatus('Upload startet...');"
        "    xhr.send(data);"
        "  }catch(e){"
        "    setStatus('Fehler: ' + e,'err');"
        "    logln('Exception: ' + e);"
        "    btn.disabled=false;"
        "  }"
        "}"
        "async function checkOta(){"
        "  try{"
        "    const r = await fetch('/api/ota/status');"
        "    const t = await r.text();"
        "    logln('OTA Status: ' + t);"
        "  }catch(e){"
        "    logln('Statusabfrage fehlgeschlagen: ' + e);"
        "  }"
        "}"
        "checkOta();"
        "</script>"
        "</body>"
        "</html>",
        app->project_name,
        app->version,
        MR_BOARD_NAME,
        app->date,
        app->time,
        app->idf_ver
    );

    if (len < 0 || len >= (int)sizeof(html))
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "HTML buffer too small");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

void mr_wifi_ota_web_register(httpd_handle_t server)
{
    httpd_uri_t page_uri = {
        .uri      = "/update",
        .method   = HTTP_GET,
        .handler  = ota_web_page_handler,
        .user_ctx = NULL
    };

    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &page_uri));
    ESP_LOGI(TAG, "OTA web page registered: /update");
}