#include "web_rc.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "battery.h"
#include "flight_control.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

extern const uint8_t web_rc_html_start[] asm("_binary_web_rc_html_start");
extern const uint8_t web_rc_html_end[]   asm("_binary_web_rc_html_end");

static const char *TAG = "WEB_RC";

#define WEB_RC_TIMEOUT_US 2000000 // 2 seconds

static rc_input_t s_web_rc_data = {
    .roll = 0.0f,
    .pitch = 0.0f,
    .yaw = 0.0f,
    .throttle = 0.0f,
    .mode = 0.5f, // Default STAB
    .buttons = 0,
    .valid = false,
    .last_update_s = 0.0f
};

static int64_t s_last_update_us = 0;

static const char* find_json_value(const char* json, const char* key) {
    const char* p = strstr(json, key);
    if (!p) return NULL;
    p = strchr(p, ':');
    if (!p) return NULL;
    p++;
    while (*p == ' ' || *p == '"') p++;
    return p;
}

static esp_err_t index_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)web_rc_html_start, web_rc_html_end - web_rc_html_start);
    return ESP_OK;
}

static esp_err_t web_rc_post_handler(httpd_req_t *req) {
    char buf[256];
    int ret, remaining = req->content_len;

    if (remaining >= sizeof(buf)) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    if ((ret = httpd_req_recv(req, buf, remaining)) <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    const char* v;
    const char* typePos = strstr(buf, "\"t\":");
    if (!typePos) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    int type = atoi(typePos + 4);
    
    if (type == 1) { // Joystick data
        float th = 0, r = 0, p = 0, y = 0;
        if ((v = find_json_value(buf, "\"th\""))) th = atof(v);
        if ((v = find_json_value(buf, "\"r\"")))  r  = atof(v);
        if ((v = find_json_value(buf, "\"p\"")))  p  = atof(v);
        if ((v = find_json_value(buf, "\"y\"")))  y  = atof(v);
        
        s_web_rc_data.throttle = th / 100.0f; // [0, 1]
        s_web_rc_data.roll = r / 30.0f;       // [-1, 1] (CF-drone uses STICK_MAX = 30)
        s_web_rc_data.pitch = p / 30.0f;
        s_web_rc_data.yaw = y / 30.0f;
        
        s_web_rc_data.valid = true;
        s_web_rc_data.last_update_s = (float)esp_timer_get_time() / 1000000.0f;
        s_last_update_us = esp_timer_get_time();
        
    } else if (type == 2) { // Button data
        int idx = 0, state = 0;
        if ((v = find_json_value(buf, "\"b\"")))  idx   = atoi(v);
        if ((v = find_json_value(buf, "\"s\"")))  state = atoi(v);
        
        if (idx >= 0 && idx < 16) {
            if (state) s_web_rc_data.buttons |= (1 << idx);
            else       s_web_rc_data.buttons &= ~(1 << idx);
        }
        
        if (idx == 6 && state == 1) s_web_rc_data.mode = 0.5f; // STAB
        if (idx == 7 && state == 1) s_web_rc_data.mode = 0.0f; // ACRO
        if (idx == 8 && state == 1) s_web_rc_data.mode = 1.0f; // ALTHOLD

        s_web_rc_data.valid = true;
        s_web_rc_data.last_update_s = (float)esp_timer_get_time() / 1000000.0f;
        s_last_update_us = esp_timer_get_time();
    } else if (type == 4) { // Heartbeat
        s_web_rc_data.valid = true;
        s_web_rc_data.last_update_s = (float)esp_timer_get_time() / 1000000.0f;
        s_last_update_us = esp_timer_get_time();
    }

    // Send response
    char resp[128];
    int current_mode = 2; // Default to STAB for frontend
    if (s_web_rc_data.mode < 0.25f) current_mode = 1; // ACRO
    else if (s_web_rc_data.mode > 0.75f) current_mode = 3; // ALTHOLD
    
    bool armed = fc_is_armed();

    snprintf(resp, sizeof(resp), "{\"s\":\"ok\",\"m\":%d,\"arm\":%d}", current_mode, armed);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    
    return ESP_OK;
}

static esp_err_t status_get_handler(httpd_req_t *req) {
    char json[256];
    float vbat = battery_get_voltage();
    snprintf(json, sizeof(json),
        "{\"enabled\":%s,\"active\":%s,\"voltage\":%.2f,"
        "\"throttle\":%.1f,\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f}",
        web_rc_is_connected() ? "true" : "false",
        web_rc_is_connected() ? "true" : "false",
        vbat,
        s_web_rc_data.throttle * 100.0f,
        s_web_rc_data.roll * 30.0f,
        s_web_rc_data.pitch * 30.0f,
        s_web_rc_data.yaw * 30.0f);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

void web_rc_init(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_uri_handlers = 10;
    
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t uri_index = {
            .uri      = "/",
            .method   = HTTP_GET,
            .handler  = index_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri_index);

        httpd_uri_t uri_post = {
            .uri      = "/web_rc",
            .method   = HTTP_POST,
            .handler  = web_rc_post_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri_post);
        
        httpd_uri_t uri_hb = {
            .uri      = "/web_rc/heartbeat",
            .method   = HTTP_POST,
            .handler  = web_rc_post_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri_hb);

        httpd_uri_t uri_status = {
            .uri      = "/web_rc/status",
            .method   = HTTP_GET,
            .handler  = status_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri_status);
        
        ESP_LOGI(TAG, "Web RC Server started on port %d", config.server_port);
    } else {
        ESP_LOGE(TAG, "Failed to start Web RC Server");
    }
}

bool web_rc_is_connected(void) {
    if (s_last_update_us == 0) return false;
    return (esp_timer_get_time() - s_last_update_us) < WEB_RC_TIMEOUT_US;
}

bool web_rc_read(rc_input_t *out) {
    if (web_rc_is_connected()) {
        memcpy(out, &s_web_rc_data, sizeof(rc_input_t));
        return true;
    }
    return false;
}
