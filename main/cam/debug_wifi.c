#include "debug_wifi.h"
#include "cam_config.h"
#include "cam_state.h"
#include "util.h"
#include "vision_line.h"

#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "AVOID_CATCH";

static const char *debug_hint(const Sight *p)
{
    if (g_phase == PHASE_STOP) {
        return "终止线：横着全黑后再全白，停车。";
    }
    if (g_phase == PHASE_ALIGN) {
        return "超声波已近，先原地摆正朝向再横移。";
    }
    if (g_phase == PHASE_STRAFE_FORCE || g_phase == PHASE_STRAFE_WAIT) {
        return "横移绕障。";
    }
    if (g_phase == PHASE_FWD) {
        return "绕过障碍后直行一段。";
    }
    if (g_phase == PHASE_STRAFE_BACK) {
        return "反向横移，直到再次看到黑线。";
    }
    if (p->t_bar) {
        return "T 路口：左或右有一大片横带（或两边都有）。绕障后见此则直行，全白停车。";
    }
    switch (p->type) {
    case VIEW_STEM:
        if (p->turn_left) {
            return "有竖线，左侧有横带或折线：记下左转，现在仍直行微调。";
        }
        if (p->turn_right) {
            return "有竖线，右侧有横带或折线：记下右转，现在仍直行微调。";
        }
        return "有竖线：慢速直行，按最低端偏角及时小调或大调。";
    case VIEW_BAR:
        return "窗口里是横带、还看不见竖线：记下方向，继续往前，等丢线再自转。";
    default:
        return "看不见黑线：按记忆方向原地转。";
    }
}

static void debug_copy_images(void)
{
    int w = s_img_w;
    int h = s_img_h;
    int x0 = roi_x0();
    int x1 = roi_x1();
    int y0 = g_scan_y[SCAN_ROWS - 1];
    int y1 = g_scan_y[0];
    if (y0 > y1) {
        int t = y0;
        y0 = y1;
        y1 = t;
    }
    for (int y = 0; y < h; y++) {
        uint8_t *dg = s_dbg_gray + y * w;
        const uint8_t *sg = s_gray + map_y(y, h) * w;
        if (CAM_FLIP_LR) {
            for (int x = 0; x < w; x++) {
                dg[x] = sg[w - 1 - x];
            }
        } else {
            memcpy(dg, sg, (size_t)w);
        }
        uint8_t *db = s_dbg_bin + y * w;
        memset(db, 40, (size_t)w);
        if (s_bin && y >= y0 && y <= y1) {
            for (int x = x0; x <= x1; x++) {
                db[x] = s_bin[y * w + x] ? 0 : 255;
            }
        }
    }
}

void debug_update(const Sight *p)
{
    static int dbg_n;
    if ((++dbg_n & 1) != 0) {
        return;
    }
    if (!s_dbg_mutex || !s_dbg_gray || !s_dbg_bin) {
        return;
    }
    if (xSemaphoreTake(s_dbg_mutex, 0) != pdTRUE) {
        return;
    }
    g_dbg_path = *p;
    g_dbg_w = s_img_w;
    g_dbg_h = s_img_h;
    debug_copy_images();

    int n = snprintf(s_dbg_json, sizeof(s_dbg_json),
                     "{\"type\":\"%s\",\"md\":%d,\"near\":%d,\"far\":%d,\"heading\":%d,"
                     "\"offset\":%d,\"L\":%d,\"R\":%d,\"vy\":%.0f,\"om\":%.0f,\"ms\":%d,"
                     "\"dist\":%.1f,\"tbar\":%d,\"roi0\":%d,\"roi1\":%d,\"ny\":%d,\"fy\":%d,"
                     "\"kx\":%d,\"ky\":%d,\"stem\":%d,\"kink\":%d,\"rows\":%d,"
                     "\"ldir\":\"%s\",\"hint\":\"%s\",\"scan_y\":[",
                     view_name(p->type), (int)g_phase, p->near_cx, p->far_cx, p->angle,
                     p->offset, p->left_mass, p->right_mass, g_last_vy, g_last_om, g_decode_ms,
                     g_last_dist, p->t_bar ? 1 : 0,
                     roi_x0(), roi_x1(), p->near_y, p->far_y, p->corner_x, p->corner_y,
                     p->stem_n, p->kink, SCAN_ROWS,
                     (g_last_dir == LAST_DIR_LEFT) ? "L" : "R",
                     debug_hint(p));
    for (int i = 0; i < SCAN_ROWS && n > 0 && n < (int)sizeof(s_dbg_json) - 120; i++) {
        n += snprintf(s_dbg_json + n, sizeof(s_dbg_json) - (size_t)n, "%s%d",
                      (i ? "," : ""), g_scan_y[i]);
    }
    if (n > 0 && n < (int)sizeof(s_dbg_json) - 20) {
        n += snprintf(s_dbg_json + n, sizeof(s_dbg_json) - (size_t)n, "],\"blobs\":[");
    }
    int first = 1;
    for (int i = 0; i < SCAN_ROWS && n > 0 && n < (int)sizeof(s_dbg_json) - 80; i++) {
        for (int k = 0; k < g_scan_rows[i].n && n < (int)sizeof(s_dbg_json) - 80; k++) {
            n += snprintf(s_dbg_json + n, sizeof(s_dbg_json) - (size_t)n,
                          "%s{\"x\":%d,\"y\":%d,\"w\":%d}",
                          first ? "" : ",",
                          g_scan_rows[i].b[k].cx, g_scan_y[i], g_scan_rows[i].b[k].width);
            first = 0;
        }
    }
    if (n > 0 && n < (int)sizeof(s_dbg_json) - 20) {
        n += snprintf(s_dbg_json + n, sizeof(s_dbg_json) - (size_t)n, "],\"poly\":[");
    }
    first = 1;
    for (int i = 0; i < g_poly_n && n > 0 && n < (int)sizeof(s_dbg_json) - 40; i++) {
        n += snprintf(s_dbg_json + n, sizeof(s_dbg_json) - (size_t)n,
                      "%s{\"x\":%d,\"y\":%d}",
                      first ? "" : ",", g_poly_x[i], g_poly_y[i]);
        first = 0;
    }
    if (n > 0 && n < (int)sizeof(s_dbg_json) - 3) {
        n += snprintf(s_dbg_json + n, sizeof(s_dbg_json) - (size_t)n, "]}");
    }
    if (n < 0) {
        n = 0;
    }
    if (n >= (int)sizeof(s_dbg_json)) {
        n = (int)sizeof(s_dbg_json) - 1;
        s_dbg_json[n] = '\0';
    }
    s_dbg_json_len = n;
    s_dbg_ready = true;
    xSemaphoreGive(s_dbg_mutex);
}

static const char DBG_HTML[] =
    "<!DOCTYPE html><html><head><meta charset=utf-8><title>摄像头画面</title>"
    "<style>body{font-family:sans-serif;background:#111;color:#ddd;margin:16px;max-width:1100px}"
    ".tip{background:#1e2a1e;border:1px solid #3a5;color:#cfc;padding:10px 12px;margin:0 0 14px}"
    ".row{display:flex;flex-wrap:wrap;gap:16px}"
    "figure{margin:0}"
    "canvas{background:#000;image-rendering:pixelated;width:min(90vw,640px);border:1px solid #444}"
    "figcaption{color:#aaa;margin:6px 0 0}"
    "pre{background:#1a1a1a;padding:10px;line-height:1.45;white-space:pre-wrap}"
    "ol{line-height:1.55;color:#ccc} li{margin:6px 0}</style></head><body>"
    "<h2>摄像头实时画面</h2>"
    "<p class=tip>手机/电脑连 WiFi <b>CAM_LINE</b> 密码 <b>12345678</b>，浏览器打开 "
    "<b>http://192.168.4.1/</b>（必须 http，不要 https）。左图就是摄像头看到的内容。</p>"
    "<div class=row>"
    "<figure><canvas id=a width=60 height=40></canvas>"
    "<figcaption>左：摄像头原图（已按车头方向翻转）。青框=循迹检测窗口</figcaption></figure>"
    "<figure><canvas id=b width=60 height=40></canvas>"
    "<figcaption>右：识别图。白=地面　黑=线　灰=不看。品红=竖线　绿=最低端</figcaption></figure>"
    "</div>"
    "<pre id=t>连接中...</pre>"
    "<h3>判断逻辑</h3>"
    "<ol>"
    "<li>慢速循迹。有竖线就直行，用绿点偏角及时小调/大调，把车身摆正，方便后面横移。</li>"
    "<li>超声波 ≤10cm：先原地摆正，再强制横移一小段；仍 &lt;13cm 继续横移；大于则直行一段，再反向横移直到看到黑线。</li>"
    "<li>终止 T：左或右任意一侧有一大片横带，或两边都有，都算 T。绕障回来后见 T 就直行，变成全白停车（不再当直角转）。</li>"
    "<li>直角/锐角：只记方向，丢线后再原地转。</li>"
    "</ol>"
    "<script>"
    "function paint(cv,pix,w,h){cv.width=w;cv.height=h;const ctx=cv.getContext('2d');"
    "const im=ctx.createImageData(w,h);"
    "for(let i=0;i<w*h;i++){const v=pix[i]||0;im.data[i*4]=v;im.data[i*4+1]=v;im.data[i*4+2]=v;im.data[i*4+3]=255;}"
    "ctx.putImageData(im,0,0);return ctx;}"
    "let busy=false;"
    "async function tick(){"
    "if(busy)return;busy=true;"
    "try{"
    "const r=await fetch('/snap');"
    "if(!r.ok)throw new Error('HTTP '+r.status+' '+r.statusText);"
    "const buf=new Uint8Array(await r.arrayBuffer());"
    "const jl=buf[0]|buf[1]<<8;"
    "const inf=JSON.parse(new TextDecoder().decode(buf.subarray(2,2+jl)));"
    "const o=2+jl,w=buf[o]|buf[o+1]<<8,h=buf[o+2]|buf[o+3]<<8,n=w*h;"
    "const orig=buf.subarray(o+4,o+4+n),bin=buf.subarray(o+4+n,o+4+2*n);"
    "const c1=paint(document.getElementById('a'),orig,w,h);"
    "const c2=paint(document.getElementById('b'),bin,w,h);"
    "const r0=(inf.roi0|0),r1=(inf.roi1||w);"
    "const yTop=inf.scan_y[inf.scan_y.length-1]||0,yBot=inf.scan_y[0]||h;"
    "c1.strokeStyle='#0cf';c1.strokeRect(r0+0.5,yTop+0.5,Math.max(2,r1-r0),yBot-yTop);"
    "c2.strokeStyle='#0cf';c2.strokeRect(r0+0.5,yTop+0.5,Math.max(2,r1-r0),yBot-yTop);"
    "c2.strokeStyle='#cc0';"
    "for(const y of inf.scan_y){c2.beginPath();c2.moveTo(r0,y+0.5);c2.lineTo(r1,y+0.5);c2.stroke();}"
    "c2.strokeStyle='#f0f';c2.lineWidth=2;c2.beginPath();"
    "if(inf.poly&&inf.poly.length){inf.poly.forEach((p,i)=>{i?c2.lineTo(p.x,p.y):c2.moveTo(p.x,p.y);});c2.stroke();}"
    "c2.lineWidth=1;c2.strokeStyle='#f44';"
    "for(const b of inf.blobs){c2.strokeRect(b.x-b.w/2,b.y-1,Math.max(2,b.w),3);}"
    "function dot(ctx,x,y,c){if(x<0||y<0)return;ctx.fillStyle=c;ctx.beginPath();ctx.arc(x,y,3.5,0,6.28);ctx.fill();}"
    "dot(c2,inf.near,inf.ny,'#0f0');dot(c2,inf.far,inf.fy,'#0ff');dot(c2,inf.kx,inf.ky,'#fa0');"
    "const md=['循迹 FOLLOW','摆正 ALIGN','强制横移','继续横移','绕障直行','回线横移','停车 STOP'][inf.md]||inf.md;"
    "document.getElementById('t').textContent="
    "'判定：'+inf.type+'    模式：'+md+'    记忆方向：'+inf.ldir+"
    "'\\n超声波 dist='+inf.dist+'cm    终止T='+inf.tbar+"
    "'\\n最低端(绿) n='+inf.near+' 偏角 heading='+inf.heading+' 偏移 offset='+inf.offset+"
    "'\\nstem='+inf.stem+'  kink='+inf.kink+'  L/R='+inf.L+'/'+inf.R+"
    "'\\nvy='+inf.vy+' om='+inf.om+'  解码 '+inf.ms+'ms'"
    "+'\\n\\n'+inf.hint;"
    "}catch(e){document.getElementById('t').textContent='等待画面 '+e;}"
    "busy=false;}"
    "setInterval(tick,300);tick();"
    "</script></body></html>";

static void dbg_http_hdr(httpd_req_t *req, const char *type)
{
    httpd_resp_set_type(req, type);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
}

static esp_err_t dbg_index_handler(httpd_req_t *req)
{
    dbg_http_hdr(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, DBG_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t dbg_ping_handler(httpd_req_t *req)
{
    dbg_http_hdr(req, "text/plain");
    return httpd_resp_send(req, "ok", 2);
}

static bool dbg_copy_locked(char *json, int *json_len, uint8_t *gray, uint8_t *bin, int *w, int *h)
{
    if (!s_dbg_mutex || !s_dbg_gray || !s_dbg_bin || !s_dbg_ready) {
        return false;
    }
    if (xSemaphoreTake(s_dbg_mutex, pdMS_TO_TICKS(40)) != pdTRUE) {
        return false;
    }
    *json_len = s_dbg_json_len;
    if (*json_len < 0) {
        *json_len = 0;
    }
    if (*json_len > (int)sizeof(s_dbg_json)) {
        *json_len = (int)sizeof(s_dbg_json);
    }
    memcpy(json, s_dbg_json, (size_t)*json_len);
    *w = g_dbg_w;
    *h = g_dbg_h;
    if (*w > 0 && *h > 0) {
        size_t n = (size_t)(*w) * (size_t)(*h);
        if (gray) {
            memcpy(gray, s_dbg_gray, n);
        }
        if (bin) {
            memcpy(bin, s_dbg_bin, n);
        }
    }
    xSemaphoreGive(s_dbg_mutex);
    return *w > 0 && *h > 0 && *json_len > 2;
}

static esp_err_t dbg_snap_handler(httpd_req_t *req)
{
    static char json[4096];
    static uint8_t *gray;
    static uint8_t *bin;
    if (!gray) {
        gray = (uint8_t *)psram_alloc(CAM_WIDTH * CAM_HEIGHT);
    }
    if (!bin) {
        bin = (uint8_t *)psram_alloc(CAM_WIDTH * CAM_HEIGHT);
    }
    int jl = 0, w = 0, h = 0;
    if (!gray || !bin || !dbg_copy_locked(json, &jl, gray, bin, &w, &h)) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        dbg_http_hdr(req, "text/plain");
        return httpd_resp_send(req, "wait", 4);
    }

    size_t pix = (size_t)w * (size_t)h;
    size_t total = 6 + (size_t)jl + pix * 2;
    uint8_t *pkt = (uint8_t *)psram_alloc(total);
    if (!pkt) {
        return httpd_resp_send_500(req);
    }
    pkt[0] = (uint8_t)(jl & 0xff);
    pkt[1] = (uint8_t)((jl >> 8) & 0xff);
    memcpy(pkt + 2, json, (size_t)jl);
    pkt[2 + jl] = (uint8_t)(w & 0xff);
    pkt[3 + jl] = (uint8_t)((w >> 8) & 0xff);
    pkt[4 + jl] = (uint8_t)(h & 0xff);
    pkt[5 + jl] = (uint8_t)((h >> 8) & 0xff);
    memcpy(pkt + 6 + jl, gray, pix);
    memcpy(pkt + 6 + jl + pix, bin, pix);

    dbg_http_hdr(req, "application/octet-stream");
    esp_err_t r = httpd_resp_send(req, (const char *)pkt, (ssize_t)total);
    free(pkt);
    return r;
}

void wifi_debug_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    esp_err_t el = esp_event_loop_create_default();
    if (el != ESP_OK && el != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(el);
    }
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = { 0 };
    memcpy(wifi_config.ap.ssid, "CAM_LINE", 8);
    wifi_config.ap.ssid_len = 8;
    wifi_config.ap.channel = 1;
    memcpy(wifi_config.ap.password, "12345678", 8);
    wifi_config.ap.max_connection = 4;
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.stack_size = 8192;
    config.core_id = tskNO_AFFINITY;
    config.max_open_sockets = 4;
    config.recv_wait_timeout = 10;
    config.send_wait_timeout = 10;
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP 启动失败");
        return;
    }
    httpd_uri_t u0 = { .uri = "/", .method = HTTP_GET, .handler = dbg_index_handler };
    httpd_uri_t u1 = { .uri = "/ping", .method = HTTP_GET, .handler = dbg_ping_handler };
    httpd_uri_t u2 = { .uri = "/snap", .method = HTTP_GET, .handler = dbg_snap_handler };
    httpd_register_uri_handler(server, &u0);
    httpd_register_uri_handler(server, &u1);
    httpd_register_uri_handler(server, &u2);
    ESP_LOGI(TAG, "调试页: 连 WiFi CAM_LINE 密码 12345678，浏览器打开 http://192.168.4.1/");
}
