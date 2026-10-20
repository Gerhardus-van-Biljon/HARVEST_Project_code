/**
 * @file ota_update.c
 * @brief Over-The-Air (OTA) Wireless Firmware Update Implementation for ESP32-S3.
 */

#include "ota_update.h"
#include "tmc2209.h"
#include "as5600.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_mac.h"

static const char *TAG = "OTA_SYSTEM";

static ota_status_t s_ota_status = {0};
static httpd_handle_t s_http_server = NULL;
static esp_netif_t *s_netif_sta = NULL;
static esp_netif_t *s_netif_ap = NULL;

// -----------------------------------------------------------------------------
// HTML5 & CSS3 Web Portal UI (Embedded Single-Page Application)
// -----------------------------------------------------------------------------
static const char INDEX_HTML[] = 
"<!DOCTYPE html>"
"<html lang='en'>"
"<head>"
"<meta charset='UTF-8'>"
"<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
"<title>ESP32-S3 Robot OTA Portal</title>"
"<style>"
":root{--bg:#0f172a;--card:#1e293b;--primary:#3b82f6;--accent:#10b981;--warn:#f59e0b;--text:#f8fafc;--sub:#94a3b8;--border:#334155;}"
"*{box-sizing:border-box;margin:0;padding:0;font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,Helvetica,Arial,sans-serif;}"
"body{background:var(--bg);color:var(--text);display:flex;justify-content:center;padding:24px 16px;min-height:100vh;}"
".container{max-width:680px;width:100%;display:flex;flex-direction:column;gap:20px;}"
".header{text-align:center;padding:12px 0;}"
".header h1{font-size:1.6rem;font-weight:700;color:#fff;margin-bottom:6px;}"
".header p{color:var(--sub);font-size:0.95rem;}"
".card{background:var(--card);border:1px solid var(--border);border-radius:12px;padding:20px;box-shadow:0 4px 6px -1px rgba(0,0,0,0.3);}"
".card-title{font-size:1.1rem;font-weight:600;margin-bottom:14px;display:flex;align-items:center;gap:8px;color:#fff;}"
".grid-2{display:grid;grid-template-columns:1fr 1fr;gap:12px;}"
"@media(max-width:540px){.grid-2{grid-template-columns:1fr;}}"
".info-box{background:rgba(15,23,42,0.6);border:1px solid var(--border);border-radius:8px;padding:10px 14px;}"
".info-box span{display:block;font-size:0.75rem;color:var(--sub);text-transform:uppercase;letter-spacing:0.05em;margin-bottom:4px;}"
".info-box strong{font-size:1rem;color:#fff;}"
".badge{display:inline-block;padding:2px 8px;border-radius:999px;font-size:0.75rem;font-weight:600;}"
".badge-green{background:rgba(16,185,129,0.2);color:#34d399;border:1px solid rgba(16,185,129,0.3);}"
".badge-blue{background:rgba(59,130,246,0.2);color:#60a5fa;border:1px solid rgba(59,130,246,0.3);}"
".alert{background:rgba(245,158,11,0.1);border:1px solid rgba(245,158,11,0.3);border-radius:8px;padding:12px;color:#fbbf24;font-size:0.88rem;line-height:1.4;margin-bottom:16px;display:flex;gap:10px;align-items:center;}"
".drop-zone{border:2px dashed var(--border);border-radius:10px;padding:32px 20px;text-align:center;cursor:pointer;transition:all 0.2s ease;background:rgba(15,23,42,0.4);}"
".drop-zone:hover,.drop-zone.dragover{border-color:var(--primary);background:rgba(59,130,246,0.05);}"
".btn{width:100%;padding:12px;border:none;border-radius:8px;background:var(--primary);color:#fff;font-size:1rem;font-weight:600;cursor:pointer;transition:opacity 0.2s;margin-top:16px;}"
".btn:disabled{opacity:0.4;cursor:not-allowed;}"
".progress-wrap{margin-top:16px;display:none;}"
".progress-bar{height:12px;background:rgba(15,23,42,0.8);border-radius:6px;overflow:hidden;position:relative;border:1px solid var(--border);}"
".progress-fill{height:100%;width:0%;background:linear-gradient(90deg,var(--primary),var(--accent));transition:width 0.2s ease;border-radius:6px;}"
".progress-text{font-size:0.85rem;color:var(--sub);margin-top:8px;display:flex;justify-content:space-between;}"
"#status-box{margin-top:14px;font-size:0.9rem;padding:10px;border-radius:6px;display:none;text-align:center;}"
"</style>"
"</head>"
"<body>"
"<div class='container'>"
"<div class='header'>"
"<h1>ESP32-S3 TMC2209 Motion Controller</h1>"
"<p>Wireless Over-The-Air (OTA) Firmware Management</p>"
"</div>"
"<div class='card'>"
"<div class='card-title'><span>System Information</span></div>"
"<div class='grid-2'>"
"<div class='info-box'><span>Chip & Hardware</span><strong>ESP32-S3 N16R8 (16MB)</strong></div>"
"<div class='info-box'><span>Active Boot Partition</span><strong id='part-name'>Loading...</strong></div>"
"<div class='info-box'><span>Firmware Build Date</span><strong id='build-time'>" __DATE__ " " __TIME__ "</strong></div>"
"<div class='info-box'><span>Wi-Fi Network Status</span><strong id='wifi-status'>Loading...</strong></div>"
"</div>"
"</div>"
"<div class='card'>"
"<div class='card-title'><span>Upload New Firmware</span></div>"
"<div class='alert'>⚠️ <strong>Safety Interlock Active:</strong> All 4 motor drivers will be automatically decelerated, halted, and freewheeled during flashing to prevent unexpected mechanical movement.</div>"
"<form id='upload-form'>"
"<div class='drop-zone' id='drop-zone'>"
"<p style='font-size:1rem;font-weight:500;margin-bottom:6px;'>Drag & Drop <code>2209_control.bin</code> here</p>"
"<p style='font-size:0.85rem;color:var(--sub);'>or click to browse your local computer</p>"
"<input type='file' id='file-input' accept='.bin' style='display:none;'>"
"<div id='file-name' style='margin-top:10px;font-size:0.9rem;color:var(--primary);font-weight:600;'></div>"
"</div>"
"<button type='button' class='btn' id='upload-btn' disabled onclick='startUpload()'>Flash Firmware Over-The-Air</button>"
"<div class='progress-wrap' id='progress-wrap'>"
"<div class='progress-bar'><div class='progress-fill' id='progress-fill'></div></div>"
"<div class='progress-text'><span id='pct'>0%</span><span id='bytes'>0 / 0 KB</span></div>"
"</div>"
"<div id='status-box'></div>"
"</form>"
"</div>"
"</div>"
"<script>"
"const dz=document.getElementById('drop-zone'),fi=document.getElementById('file-input'),ub=document.getElementById('upload-btn'),fn=document.getElementById('file-name'),sb=document.getElementById('status-box'),pw=document.getElementById('progress-wrap'),pf=document.getElementById('progress-fill'),pct=document.getElementById('pct'),by=document.getElementById('bytes');"
"let selFile=null;"
"dz.onclick=()=>fi.click();"
"dz.ondragover=(e)=>{e.preventDefault();dz.classList.add('dragover');};"
"dz.ondragleave=()=>dz.classList.remove('dragover');"
"dz.ondrop=(e)=>{e.preventDefault();dz.classList.remove('dragover');if(e.dataTransfer.files.length){handleFile(e.dataTransfer.files[0]);}};"
"fi.onchange=()=>{if(fi.files.length)handleFile(fi.files[0]);};"
"function handleFile(f){if(!f.name.endsWith('.bin')){alert('Please select a valid .bin firmware file');return;}selFile=f;fn.innerText=f.name+' ('+Math.round(f.size/1024)+' KB)';ub.disabled=false;}"
"function fetchStatus(){fetch('/api/status').then(r=>r.json()).then(d=>{"
"document.getElementById('part-name').innerText=d.running_partition||'ota_0';"
"document.getElementById('wifi-status').innerText=d.sta_connected?'STA ('+d.sta_ip+')':'AP ('+d.ap_ip+')';"
"}).catch(()=>{});}"
"setInterval(fetchStatus,3000);fetchStatus();"
"function startUpload(){if(!selFile)return;ub.disabled=true;pw.style.display='block';sb.style.display='block';sb.style.background='rgba(59,130,246,0.1)';sb.style.color='#60a5fa';sb.innerText='Stopping motors & writing flash... Do not disconnect!';"
"const xhr=new XMLHttpRequest();"
"xhr.open('POST','/update',true);"
"xhr.setRequestHeader('Content-Type','application/octet-stream');"
"xhr.upload.onprogress=(e)=>{if(e.lengthComputable){const p=Math.round((e.loaded/e.total)*100);pf.style.width=p+'%';pct.innerText=p+'%';by.innerText=Math.round(e.loaded/1024)+' / '+Math.round(e.total/1024)+' KB';}};"
"xhr.onload=()=>{if(xhr.status===200){pf.style.width='100%';pct.innerText='100%';sb.style.background='rgba(16,185,129,0.15)';sb.style.color='#34d399';sb.innerText='✅ Flashing Complete! Rebooting ESP32 into new partition...';"
"let c=6;const it=setInterval(()=>{c--;if(c<=0){clearInterval(it);location.reload();}else{sb.innerText='✅ Flashing Complete! Rebooting in '+c+'s...';}},1000);"
"}else{sb.style.background='rgba(239,68,68,0.15)';sb.style.color='#f87171';sb.innerText='❌ Upload Failed: '+xhr.responseText;ub.disabled=false;}};"
"xhr.onerror=()=>{sb.style.background='rgba(239,68,68,0.15)';sb.style.color='#f87171';sb.innerText='❌ Network error during update.';ub.disabled=false;};"
"xhr.send(selFile);}"
"</script>"
"</body>"
"</html>";

// -----------------------------------------------------------------------------
// Reboot Helper Task
// -----------------------------------------------------------------------------
static void reboot_task(void *param)
{
    vTaskDelay(pdMS_TO_TICKS(1800));
    ESP_LOGI(TAG, "Rebooting into new OTA firmware now...");
    esp_restart();
}

// -----------------------------------------------------------------------------
// HTTP Handlers
// -----------------------------------------------------------------------------
static esp_err_t ota_index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t ota_status_handler(httpd_req_t *req)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);

    char json[256];
    snprintf(json, sizeof(json),
             "{\"running_partition\":\"%s\",\"next_partition\":\"%s\","
             "\"sta_connected\":%s,\"sta_ip\":\"%s\","
             "\"ap_active\":%s,\"ap_ip\":\"%s\","
             "\"free_heap\":%lu}",
             running ? running->label : "unknown",
             next ? next->label : "unknown",
             s_ota_status.sta_connected ? "true" : "false",
             s_ota_status.sta_ip,
             s_ota_status.ap_active ? "true" : "false",
             s_ota_status.ap_ip,
             (unsigned long)esp_get_free_heap_size());

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t ota_post_update_handler(httpd_req_t *req)
{
    ESP_LOGW(TAG, "==================================================");
    ESP_LOGW(TAG, "  WIRELESS OTA UPDATE INITIATED OVER HTTP         ");
    ESP_LOGW(TAG, "==================================================");

    // 1. SAFETY INTERLOCK: Halt pulse engine and disable motor coils
    ESP_LOGI(TAG, "[SAFETY] Stopping all motors & disabling power stages...");
    tmc_stop_all_motors();
    for (int i = 0; i < TMC_NUM_MOTORS; i++) {
        motors[i].target_speed_hz = 0;
        gpio_set_level(motors[i].en_pin, 1); // Active HIGH = coils unpowered
    }

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        ESP_LOGE(TAG, "Cannot locate candidate OTA partition!");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition available");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Writing firmware to partition: '%s' (0x%08lx, size %lu KB)",
             update_partition->label,
             (unsigned long)update_partition->address,
             (unsigned long)(update_partition->size / 1024));

    esp_ota_handle_t ota_handle = 0;
    esp_err_t err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to start OTA session");
        return ESP_FAIL;
    }

    s_ota_status.is_updating = true;
    s_ota_status.bytes_written = 0;
    s_ota_status.total_size = req->content_len;

    char *buf = malloc(OTA_MAX_CHUNK_SIZE);
    if (buf == NULL) {
        ESP_LOGE(TAG, "Out of memory allocating OTA buffer");
        esp_ota_abort(ota_handle);
        s_ota_status.is_updating = false;
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_FAIL;
    }

    int remaining = req->content_len;
    while (remaining > 0) {
        int to_read = (remaining < OTA_MAX_CHUNK_SIZE) ? remaining : OTA_MAX_CHUNK_SIZE;
        int received = httpd_req_recv(req, buf, to_read);
        if (received <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            ESP_LOGE(TAG, "HTTP receive error (%d)", received);
            free(buf);
            esp_ota_abort(ota_handle);
            s_ota_status.is_updating = false;
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Socket connection dropped");
            return ESP_FAIL;
        }

        err = esp_ota_write(ota_handle, buf, received);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write error (%s)", esp_err_to_name(err));
            free(buf);
            esp_ota_abort(ota_handle);
            s_ota_status.is_updating = false;
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Flash write error");
            return ESP_FAIL;
        }

        s_ota_status.bytes_written += received;
        remaining -= received;
    }

    free(buf);

    // Finalize and validate SHA256 image checksum
    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed (%s): Corrupted image or bad checksum!", esp_err_to_name(err));
        s_ota_status.is_updating = false;
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Image verification checksum error");
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)", esp_err_to_name(err));
        s_ota_status.is_updating = false;
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to switch boot partition");
        return ESP_FAIL;
    }

    s_ota_status.is_updating = false;
    ESP_LOGI(TAG, ">>> OTA UPDATE SUCCESSFUL! %u bytes written to %s <<<",
             (unsigned int)s_ota_status.bytes_written, update_partition->label);

    httpd_resp_sendstr(req, "{\"status\":\"ok\",\"message\":\"OTA update completed successfully\"}");

    // Spawn non-blocking task to trigger reboot in ~1.8 seconds
    xTaskCreate(&reboot_task, "ota_reboot", 2048, NULL, 5, NULL);
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Web Server Startup
// -----------------------------------------------------------------------------
static esp_err_t start_ota_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = OTA_HTTP_PORT;
    config.stack_size = 8192;
    config.max_uri_handlers = 8;
    config.lru_purge_enable = true;

    esp_err_t err = httpd_start(&s_http_server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(err));
        return err;
    }

    httpd_uri_t uri_index = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = ota_index_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(s_http_server, &uri_index);

    httpd_uri_t uri_update_get = {
        .uri = "/update",
        .method = HTTP_GET,
        .handler = ota_index_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(s_http_server, &uri_update_get);

    httpd_uri_t uri_status = {
        .uri = "/api/status",
        .method = HTTP_GET,
        .handler = ota_status_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(s_http_server, &uri_status);

    httpd_uri_t uri_update_post = {
        .uri = "/update",
        .method = HTTP_POST,
        .handler = ota_post_update_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(s_http_server, &uri_update_post);

    ESP_LOGI(TAG, "OTA HTTP Server running on port %d", config.server_port);
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Wi-Fi Event Handler
// -----------------------------------------------------------------------------
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Wi-Fi Station started, attempting connection...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_ota_status.sta_connected = false;
        memset(s_ota_status.sta_ip, 0, sizeof(s_ota_status.sta_ip));
        ESP_LOGW(TAG, "Wi-Fi Station disconnected. Retrying connection...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        s_ota_status.sta_connected = true;
        snprintf(s_ota_status.sta_ip, sizeof(s_ota_status.sta_ip),
                 IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, ">>> STATION CONNECTED! IP: %s (Access at http://%s) <<<",
                 s_ota_status.sta_ip, s_ota_status.sta_ip);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "Client joined SoftAP: MAC=" MACSTR ", AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

// -----------------------------------------------------------------------------
// OTA System Initialization (Station + SoftAP Dual-Mode)
// -----------------------------------------------------------------------------
esp_err_t ota_system_init(const char *sta_ssid, const char *sta_pass)
{
    ESP_LOGI(TAG, "Initializing Dual-Mode Wi-Fi & OTA Subsystem...");

    // Record running and next partition names
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    if (running) strncpy(s_ota_status.running_partition, running->label, sizeof(s_ota_status.running_partition) - 1);
    if (next) strncpy(s_ota_status.next_partition, next->label, sizeof(s_ota_status.next_partition) - 1);

    ESP_LOGI(TAG, "Active Running Partition: '%s' | Next Target: '%s'",
             s_ota_status.running_partition, s_ota_status.next_partition);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 1. Create SoftAP network interface
    s_netif_ap = esp_netif_create_default_wifi_ap();
    strncpy(s_ota_status.ap_ip, OTA_AP_STATIC_IP, sizeof(s_ota_status.ap_ip));
    s_ota_status.ap_active = true;

    // 2. Create Station network interface
    s_netif_sta = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler, NULL, NULL));

    // Configure SoftAP settings
    wifi_config_t ap_config = {
        .ap = {
            .ssid = OTA_DEFAULT_AP_SSID,
            .ssid_len = strlen(OTA_DEFAULT_AP_SSID),
            .channel = OTA_AP_CHANNEL,
            .password = OTA_DEFAULT_AP_PASS,
            .max_connection = OTA_AP_MAX_CONN,
            .authmode = (strlen(OTA_DEFAULT_AP_PASS) > 0) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN,
        },
    };

    // Configure Station settings if credentials provided
    wifi_config_t sta_config = {0};
    bool has_sta = false;
    if (sta_ssid != NULL && strlen(sta_ssid) > 0) {
        has_sta = true;
        strncpy((char *)sta_config.sta.ssid, sta_ssid, sizeof(sta_config.sta.ssid));
        if (sta_pass != NULL) {
            strncpy((char *)sta_config.sta.password, sta_pass, sizeof(sta_config.sta.password));
        }
        sta_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    if (has_sta) {
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    }

    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "SoftAP active! SSID: '%s', Password: '%s', IP: http://%s",
             OTA_DEFAULT_AP_SSID, OTA_DEFAULT_AP_PASS, s_ota_status.ap_ip);

    // 3. Start embedded HTTP server
    start_ota_webserver();

    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Rollback Protection & App Validation
// -----------------------------------------------------------------------------
esp_err_t ota_confirm_app_valid(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            ESP_LOGI(TAG, "Boot diagnostics passed. Confirming new firmware as valid...");
            return esp_ota_mark_app_valid_cancel_rollback();
        }
    }
    return ESP_OK;
}

void ota_get_status(ota_status_t *out_status)
{
    if (out_status != NULL) {
        memcpy(out_status, &s_ota_status, sizeof(ota_status_t));
    }
}

void ota_print_network_info(void)
{
    printf("\n+--------------------------------------------------------------+\n");
    printf("|             ESP32-S3 WIRELESS OTA NETWORK STATUS             |\n");
    printf("+--------------------------------------------------------------+\n");
    printf("| Running Partition : %-40s |\n", s_ota_status.running_partition);
    printf("| Next OTA Target   : %-40s |\n", s_ota_status.next_partition);
    printf("| SoftAP Hotspot    : %-40s |\n", OTA_DEFAULT_AP_SSID);
    printf("| SoftAP Password   : %-40s |\n", OTA_DEFAULT_AP_PASS);
    printf("| SoftAP Web URL    : http://%-33s |\n", s_ota_status.ap_ip);
    if (s_ota_status.sta_connected) {
        printf("| Station Web URL   : http://%-33s |\n", s_ota_status.sta_ip);
    } else {
        printf("| Station Status    : DISCONNECTED (SoftAP Available)          |\n");
    }
    printf("+--------------------------------------------------------------+\n\n");
}

