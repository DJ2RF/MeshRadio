#include "incl/aprs_web.h"
#include "incl/aprs.h"
#include "incl/mr_board.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "APRSWEB";
static aprs_web_cfg_t g_aprs;
static bool g_init = false;

#define APRS_NVS_NS "aprs"

static void cfg_defaults(aprs_web_cfg_t *c)
{
    memset(c, 0, sizeof(*c));
    c->enabled = true;
    snprintf(c->callsign, sizeof(c->callsign), "%s", APRS_CALLSIGN);
    snprintf(c->passcode, sizeof(c->passcode), "%s", APRS_PASSCODE);
    c->symbol_table = APRS_SYMBOL_TABLE;
    c->symbol_code = APRS_SYMBOL_CODE;
    snprintf(c->comment, sizeof(c->comment), "%s", APRS_COMMENT);
    snprintf(c->host, sizeof(c->host), "%s", APRS_IS_HOST);
    c->port = APRS_IS_PORT;
    c->interval_ms = 60000;
    c->use_static_pos = false;
    c->latitude[0] = 0;
    c->longitude[0] = 0;
}

static void trim_inplace(char *s)
{
    if(!s) return;
    size_t n = strlen(s);
    while(n && isspace((unsigned char)s[n - 1])) s[--n] = 0;
    size_t i = 0;
    while(s[i] && isspace((unsigned char)s[i])) i++;
    if(i) memmove(s, s + i, strlen(s + i) + 1);
}

static int hexval(char c)
{
    if(c >= '0' && c <= '9') return c - '0';
    if(c >= 'a' && c <= 'f') return c - 'a' + 10;
    if(c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static void url_decode_inplace(char *s)
{
    char *r = s, *w = s;
    while(*r){
        if(*r == '+'){
            *w++ = ' ';
            r++;
        }else if(*r == '%' && r[1] && r[2]){
            int a = hexval(r[1]);
            int b = hexval(r[2]);
            if(a >= 0 && b >= 0){
                *w++ = (char)((a << 4) | b);
                r += 3;
            }else{
                *w++ = *r++;
            }
        }else{
            *w++ = *r++;
        }
    }
    *w = 0;
}

static bool form_get(char *body, const char *key, char *out, size_t out_sz)
{
    if(!body || !key || !out || out_sz == 0) return false;

    size_t klen = strlen(key);
    char *p = body;

    out[0] = 0;

    while(p && *p){
        char *amp = strchr(p, '&');
        size_t part_len = amp ? (size_t)(amp - p) : strlen(p);

        char part[256];
        if(part_len >= sizeof(part)){
            if(amp) p = amp + 1;
            else break;
            continue;
        }

        memcpy(part, p, part_len);
        part[part_len] = 0;

        char *eq = strchr(part, '=');
        if(eq){
            *eq = 0;
            char *k = part;
            char *v = eq + 1;

            url_decode_inplace(k);
            url_decode_inplace(v);

            if(strlen(k) == klen && strcmp(k, key) == 0){
                strlcpy(out, v, out_sz);
                return true;
            }
        }

        if(amp) p = amp + 1;
        else break;
    }

    return false;
}

static bool read_body(httpd_req_t *req, char *buf, size_t buf_sz)
{
    if(!req || !buf || buf_sz < 2) return false;
    if(req->content_len <= 0 || req->content_len >= (int)buf_sz) return false;
    int got = 0;
    while(got < req->content_len){
        int r = httpd_req_recv(req, buf + got, req->content_len - got);
        if(r <= 0) return false;
        got += r;
    }
    buf[got] = 0;
    return true;
}

static esp_err_t send_text(httpd_req_t *req, const char *txt)
{
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_send(req, txt, HTTPD_RESP_USE_STRLEN);
}

static void load_str_or_default(nvs_handle_t nvs, const char *key, char *dst, size_t dst_sz, const char *def)
{
    size_t sz = dst_sz;
    if(nvs_get_str(nvs, key, dst, &sz) != ESP_OK){
        snprintf(dst, dst_sz, "%s", def ? def : "");
    }
}

static esp_err_t save_cfg(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(APRS_NVS_NS, NVS_READWRITE, &nvs);
    if(err != ESP_OK) return err;

    err  = nvs_set_u8(nvs, "enabled", g_aprs.enabled ? 1 : 0);
    err |= nvs_set_str(nvs, "call", g_aprs.callsign);
    err |= nvs_set_str(nvs, "pass", g_aprs.passcode);
    err |= nvs_set_u8(nvs, "stab", (uint8_t)g_aprs.symbol_table);
    err |= nvs_set_u8(nvs, "scode", (uint8_t)g_aprs.symbol_code);
    err |= nvs_set_str(nvs, "comment", g_aprs.comment);
    err |= nvs_set_str(nvs, "host", g_aprs.host);
    err |= nvs_set_u16(nvs, "port", g_aprs.port);
    err |= nvs_set_u32(nvs, "interval", g_aprs.interval_ms);
    err |= nvs_set_u8(nvs, "usepos", g_aprs.use_static_pos ? 1 : 0);
    err |= nvs_set_str(nvs, "lat", g_aprs.latitude);
    err |= nvs_set_str(nvs, "lon", g_aprs.longitude);
    if(err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);

    ESP_LOGI(TAG,
         "SAVE APRS: enabled=%u call='%s' pass='%s' host='%s' port=%u interval_ms=%lu sym=%c%c static=%u lat='%s' lon='%s' comment='%s'",
         g_aprs.enabled ? 1 : 0,
         g_aprs.callsign,
         g_aprs.passcode,
         g_aprs.host,
         (unsigned)g_aprs.port,
         (unsigned long)g_aprs.interval_ms,
         g_aprs.symbol_table,
         g_aprs.symbol_code,
         g_aprs.use_static_pos ? 1 : 0,
         g_aprs.latitude,
         g_aprs.longitude,
         g_aprs.comment);
    return err;
}

static void load_cfg(void)
{
    cfg_defaults(&g_aprs);

    nvs_handle_t nvs;
    if(nvs_open(APRS_NVS_NS, NVS_READONLY, &nvs) != ESP_OK){
        return;
    }

    uint8_t u8 = 0;
    uint16_t u16 = 0;
    uint32_t u32 = 0;

    if(nvs_get_u8(nvs, "enabled", &u8) == ESP_OK) g_aprs.enabled = (u8 != 0);
    load_str_or_default(nvs, "call", g_aprs.callsign, sizeof(g_aprs.callsign), APRS_CALLSIGN);
    load_str_or_default(nvs, "pass", g_aprs.passcode, sizeof(g_aprs.passcode), APRS_PASSCODE);
    if(nvs_get_u8(nvs, "stab", &u8) == ESP_OK && u8 >= 33 && u8 <= 126) g_aprs.symbol_table = (char)u8;
    if(nvs_get_u8(nvs, "scode", &u8) == ESP_OK && u8 >= 33 && u8 <= 126) g_aprs.symbol_code = (char)u8;
    load_str_or_default(nvs, "comment", g_aprs.comment, sizeof(g_aprs.comment), APRS_COMMENT);
    load_str_or_default(nvs, "host", g_aprs.host, sizeof(g_aprs.host), APRS_IS_HOST);
    if(nvs_get_u16(nvs, "port", &u16) == ESP_OK && u16 > 0) g_aprs.port = u16;
    if(nvs_get_u32(nvs, "interval", &u32) == ESP_OK && u32 >= 5000UL) g_aprs.interval_ms = u32;
    if(nvs_get_u8(nvs, "usepos", &u8) == ESP_OK) g_aprs.use_static_pos = (u8 != 0);
    load_str_or_default(nvs, "lat", g_aprs.latitude, sizeof(g_aprs.latitude), "");
    load_str_or_default(nvs, "lon", g_aprs.longitude, sizeof(g_aprs.longitude), "");

    nvs_close(nvs);
}

static const char *PAGE_HTML =
"<!doctype html><html><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>APRS Config</title>"
"<style>body{font-family:system-ui;margin:16px;max-width:760px}"
"button{padding:10px 14px;margin:6px 4px;font-size:16px}"
"input{padding:10px;margin:6px 0;font-size:16px;width:100%;box-sizing:border-box}"
".card{border:1px solid #ddd;border-radius:12px;padding:12px;box-shadow:0 1px 3px rgba(0,0,0,.06);margin:10px 0}"
".muted{color:#666}.row{display:grid;grid-template-columns:1fr 1fr;gap:12px}@media(max-width:760px){.row{grid-template-columns:1fr}}"
"</style></head><body>"
"<h2>APRS Konfiguration</h2>"
"<div class='card'><div class='muted'>Gateway-Position kann statisch ohne GPS gesetzt werden.</div>"
"<label><input id='enabled' type='checkbox' style='width:auto'> APRS aktiviert</label>"
"<label>Rufzeichen</label><input id='callsign'>"
"<label>Passwort / Passcode</label><input id='passcode'>"
"<div class='row'><div><label>Symbol Table</label><input id='symbol_table' maxlength='1'></div><div><label>Symbol Code</label><input id='symbol_code' maxlength='1'></div></div>"
"<label>Kommentar</label><input id='comment'>"
"<label>APRS-IS Host</label><input id='host'>"
"<label>Port</label><input id='port'>"
"<label>Sendeintervall (ms)</label><input id='interval'>"
"<label><input id='use_static_pos' type='checkbox' style='width:auto'> Statische Gateway-Position senden</label>"
"<div class='row'><div><label>Latitude (dezimal)</label><input id='latitude' placeholder='49.0583'></div><div><label>Longitude (dezimal)</label><input id='longitude' placeholder='11.0633'></div></div>"
"<div><button onclick='loadCfg()'>Laden</button><button onclick='saveCfg()'>Speichern</button><button onclick=\"location.href='/'\">Zurück</button></div>"
"<div id='msg' class='muted'></div></div>"
"<script>"
"async function post(u,b){let r=await fetch(u,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:b});return await r.text();}"
"async function loadCfg(){try{let j=JSON.parse(await (await fetch('/api/aprs/get')).text());document.getElementById('enabled').checked=!!j.enabled;document.getElementById('callsign').value=j.callsign||'';document.getElementById('passcode').value=j.passcode||'';document.getElementById('symbol_table').value=j.symbol_table||'/';document.getElementById('symbol_code').value=j.symbol_code||'>';document.getElementById('comment').value=j.comment||'';document.getElementById('host').value=j.host||'';document.getElementById('port').value=j.port||14580;document.getElementById('interval').value=j.interval_ms||60000;document.getElementById('use_static_pos').checked=!!j.use_static_pos;document.getElementById('latitude').value=j.latitude||'';document.getElementById('longitude').value=j.longitude||'';document.getElementById('msg').textContent='Konfiguration geladen';}catch(e){document.getElementById('msg').textContent='Ladefehler: '+e;}}"
"async function saveCfg(){let body='enabled='+(document.getElementById('enabled').checked?'1':'0')+'&callsign='+encodeURIComponent(document.getElementById('callsign').value)+'&passcode='+encodeURIComponent(document.getElementById('passcode').value)+'&symbol_table='+encodeURIComponent(document.getElementById('symbol_table').value)+'&symbol_code='+encodeURIComponent(document.getElementById('symbol_code').value)+'&comment='+encodeURIComponent(document.getElementById('comment').value)+'&host='+encodeURIComponent(document.getElementById('host').value)+'&port='+encodeURIComponent(document.getElementById('port').value)+'&interval='+encodeURIComponent(document.getElementById('interval').value)+'&use_static_pos='+(document.getElementById('use_static_pos').checked?'1':'0')+'&latitude='+encodeURIComponent(document.getElementById('latitude').value)+'&longitude='+encodeURIComponent(document.getElementById('longitude').value);document.getElementById('msg').textContent=await post('/api/aprs/set',body);}"
"loadCfg();"
"</script></body></html>";

static esp_err_t page_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, PAGE_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t api_get(httpd_req_t *req)
{
    char out[768];
    snprintf(out, sizeof(out),
             "{\"enabled\":%s,\"callsign\":\"%s\",\"passcode\":\"%s\",\"symbol_table\":\"%c\",\"symbol_code\":\"%c\",\"comment\":\"%s\",\"host\":\"%s\",\"port\":%u,\"interval_ms\":%lu,\"use_static_pos\":%s,\"latitude\":\"%s\",\"longitude\":\"%s\"}",
             g_aprs.enabled ? "true" : "false",
             g_aprs.callsign,
             g_aprs.passcode,
             g_aprs.symbol_table,
             g_aprs.symbol_code,
             g_aprs.comment,
             g_aprs.host,
             (unsigned)g_aprs.port,
             (unsigned long)g_aprs.interval_ms,
             g_aprs.use_static_pos ? "true" : "false",
             g_aprs.latitude,
             g_aprs.longitude);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, out, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t api_set(httpd_req_t *req)
{
    char body[768];
    char tmp[128];
    if(!read_body(req, body, sizeof(body))) return send_text(req, "ERR body");
    ESP_LOGI(TAG, "POST BODY: %s", body);

    if(form_get(body, "enabled", tmp, sizeof(tmp))) g_aprs.enabled = (tmp[0] == '1');

    if(form_get(body, "callsign", tmp, sizeof(tmp))){
        trim_inplace(tmp);
        strlcpy(g_aprs.callsign, tmp, sizeof(g_aprs.callsign));
    }

    if(form_get(body, "passcode", tmp, sizeof(tmp))){
        trim_inplace(tmp);
        strlcpy(g_aprs.passcode, tmp, sizeof(g_aprs.passcode));
    }

    if(form_get(body, "symbol_table", tmp, sizeof(tmp)) && tmp[0]) g_aprs.symbol_table = tmp[0];
    if(form_get(body, "symbol_code", tmp, sizeof(tmp)) && tmp[0]) g_aprs.symbol_code = tmp[0];

    if(form_get(body, "comment", tmp, sizeof(tmp))){
        trim_inplace(tmp);
        strlcpy(g_aprs.comment, tmp, sizeof(g_aprs.comment));
    }

    if(form_get(body, "host", tmp, sizeof(tmp))){
        trim_inplace(tmp);
        strlcpy(g_aprs.host, tmp, sizeof(g_aprs.host));
    }

    if(form_get(body, "port", tmp, sizeof(tmp))){
        int p = atoi(tmp);
        if(p > 0 && p <= 65535) g_aprs.port = (uint16_t)p;
    }

    if(form_get(body, "interval", tmp, sizeof(tmp))){
        unsigned long v = strtoul(tmp, NULL, 10);
        if(v >= 5000UL && v <= 86400000UL){
            g_aprs.interval_ms = (uint32_t)v;
        }
    }

    if(form_get(body, "use_static_pos", tmp, sizeof(tmp))) g_aprs.use_static_pos = (tmp[0] == '1');

    if(form_get(body, "latitude", tmp, sizeof(tmp))){
        trim_inplace(tmp);
        strlcpy(g_aprs.latitude, tmp, sizeof(g_aprs.latitude));
    }

    if(form_get(body, "longitude", tmp, sizeof(tmp))){
        trim_inplace(tmp);
        strlcpy(g_aprs.longitude, tmp, sizeof(g_aprs.longitude));
    }

    if(g_aprs.callsign[0] == 0) snprintf(g_aprs.callsign, sizeof(g_aprs.callsign), "%s", APRS_CALLSIGN);
    if(g_aprs.passcode[0] == 0) snprintf(g_aprs.passcode, sizeof(g_aprs.passcode), "%s", APRS_PASSCODE);
    if(g_aprs.host[0] == 0) snprintf(g_aprs.host, sizeof(g_aprs.host), "%s", APRS_IS_HOST);
    if(g_aprs.port == 0) g_aprs.port = APRS_IS_PORT;
    if(g_aprs.interval_ms < 5000UL) g_aprs.interval_ms = 60000UL;
    if(g_aprs.symbol_table == 0) g_aprs.symbol_table = APRS_SYMBOL_TABLE;
    if(g_aprs.symbol_code == 0) g_aprs.symbol_code = APRS_SYMBOL_CODE;

    esp_err_t err = save_cfg();
    if(err != ESP_OK){
        ESP_LOGW(TAG, "save failed: %s", esp_err_to_name(err));
        return send_text(req, "ERR save");
    }
    return send_text(req, "OK saved");
}

void aprs_web_init(void)
{
    load_cfg();
    g_init = true;
    ESP_LOGI(TAG, "APRS cfg init: enabled=%u call=%s host=%s port=%u interval_ms=%lu static=%u lat=%s lon=%s",
             g_aprs.enabled ? 1 : 0,
             g_aprs.callsign,
             g_aprs.host,
             (unsigned)g_aprs.port,
             (unsigned long)g_aprs.interval_ms,
             g_aprs.use_static_pos ? 1 : 0,
             g_aprs.latitude,
             g_aprs.longitude);
}

void aprs_web_register_http(httpd_handle_t server)
{
    if(!server) return;
    httpd_uri_t u0 = { .uri = "/aprs", .method = HTTP_GET, .handler = page_get };
    httpd_uri_t u1 = { .uri = "/api/aprs/get", .method = HTTP_GET, .handler = api_get };
    httpd_uri_t u2 = { .uri = "/api/aprs/set", .method = HTTP_POST, .handler = api_set };
    httpd_register_uri_handler(server, &u0);
    httpd_register_uri_handler(server, &u1);
    httpd_register_uri_handler(server, &u2);
}

bool aprs_web_has_gps_board(void)
{
    const mr_board_info_t *b = mr_board_get();
    return (b && b->has_gps);
}

bool aprs_web_enabled(void)
{
    if(!g_init) aprs_web_init();
    return g_aprs.enabled;
}

const char *aprs_web_callsign(void)
{
    if(!g_init) aprs_web_init();
    return g_aprs.callsign;
}

const char *aprs_web_passcode(void)
{
    if(!g_init) aprs_web_init();
    return g_aprs.passcode;
}

char aprs_web_symbol_table(void)
{
    if(!g_init) aprs_web_init();
    return g_aprs.symbol_table;
}

char aprs_web_symbol_code(void)
{
    if(!g_init) aprs_web_init();
    return g_aprs.symbol_code;
}

const char *aprs_web_comment(void)
{
    if(!g_init) aprs_web_init();
    return g_aprs.comment;
}

const char *aprs_web_host(void)
{
    if(!g_init) aprs_web_init();
    return g_aprs.host;
}

uint16_t aprs_web_port(void)
{
    if(!g_init) aprs_web_init();
    return g_aprs.port;
}

uint32_t aprs_web_interval_ms(void)
{
    if(!g_init) aprs_web_init();
    return g_aprs.interval_ms;
}

bool aprs_web_use_static_pos(void)
{
    if(!g_init) aprs_web_init();
    return g_aprs.use_static_pos;
}

const char *aprs_web_latitude(void)
{
    if(!g_init) aprs_web_init();
    return g_aprs.latitude;
}

const char *aprs_web_longitude(void)
{
    if(!g_init) aprs_web_init();
    return g_aprs.longitude;
}

void aprs_web_get_cfg(aprs_web_cfg_t *out)
{
    if(!out) return;
    if(!g_init) aprs_web_init();
    *out = g_aprs;
}