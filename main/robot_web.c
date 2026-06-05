#include "robot_web.h"

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

#include "robot_motor.h"

static const char *TAG = "robot_web";

static volatile bool s_manual_active;
static volatile robot_random_mode_t s_random_mode = ROBOT_RANDOM_NORMAL;

static const char INDEX_HTML[] =
"<!DOCTYPE html>\n"
"<html lang=\"zh-CN\">\n"
"<head>\n"
"<meta charset=\"UTF-8\">\n"
"<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
"<title>Chax Table Robots</title>\n"
"<style>\n"
"*{box-sizing:border-box;-webkit-tap-highlight-color:transparent;}\n"
"body{margin:0;min-height:100svh;display:flex;align-items:center;justify-content:center;padding:22px;background:radial-gradient(circle at 50% 15%,#456984 0,#223f58 42%,#153044 100%);color:#f6fbff;font-family:Arial,\"Microsoft YaHei\",sans-serif;}\n"
"body:before{content:\"\";position:fixed;inset:0;background:repeating-linear-gradient(55deg,rgba(255,255,255,.035) 0 2px,transparent 2px 9px);opacity:.45;pointer-events:none;}\n"
".panel{position:relative;width:min(360px,100%);min-height:548px;padding:31px 28px 26px;border-radius:31px;background:linear-gradient(180deg,#17385a 0,#103650 52%,#11354a 100%);border:1px solid rgba(136,188,230,.22);box-shadow:0 22px 38px rgba(0,0,0,.34),inset 0 1px 0 rgba(255,255,255,.12),inset 0 -1px 0 rgba(255,255,255,.08);overflow:hidden;}\n"
".panel:before{content:\"\";position:absolute;left:50%;top:88px;width:58px;height:2px;transform:translateX(-50%);background:linear-gradient(90deg,transparent,#7ecfff,transparent);box-shadow:0 0 14px #75caff;}\n"
"h1{position:relative;margin:0;text-align:center;font-size:24px;line-height:1.23;letter-spacing:0;font-weight:800;text-transform:uppercase;text-shadow:0 4px 14px rgba(98,170,235,.45);}\n"
".dpad{position:relative;width:286px;height:292px;margin:29px auto 16px;}\n"
"button{font:inherit;border:0;color:#fff;cursor:pointer;}\n"
".cmd{position:absolute;width:76px;height:76px;border-radius:50%;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:4px;background:radial-gradient(circle at 32% 25%,rgba(145,205,255,.7),rgba(42,104,153,.95) 58%,rgba(24,69,111,.95));border:1px solid rgba(168,218,255,.48);box-shadow:inset 0 2px 7px rgba(255,255,255,.2),inset 0 -8px 12px rgba(7,35,63,.45),0 0 18px rgba(75,154,225,.33),0 10px 18px rgba(0,0,0,.28);}\n"
".cmd:active,.modebtn:active{transform:scale(.96);}\n"
".cmd .ico{font-size:23px;line-height:20px;text-shadow:0 2px 7px rgba(255,255,255,.55);}\n"
".cmd .txt{font-size:15px;font-weight:800;letter-spacing:.2px;}\n"
".up{left:105px;top:0;}.left{left:7px;top:96px;}.right{right:7px;top:96px;}.down{left:105px;bottom:0;}\n"
".stop{left:96px;top:86px;width:94px;height:94px;background:radial-gradient(circle at 35% 25%,#ffb2a8 0,#d95248 46%,#8d2420 100%);border-color:rgba(255,193,183,.74);box-shadow:inset 0 2px 8px rgba(255,255,255,.25),inset 0 -10px 18px rgba(90,0,0,.38),0 0 24px rgba(255,78,67,.85),0 12px 22px rgba(0,0,0,.32);}\n"
".stop .ico{font-size:26px;}.stop .txt{font-size:21px;letter-spacing:1px;}\n"
".modes{display:flex;gap:9px;margin:0 auto;width:100%;}\n"
".modebtn{height:60px;flex:1;border-radius:25px;background:linear-gradient(180deg,#1a3046,#122537);border:1px solid rgba(170,210,240,.25);box-shadow:inset 0 1px 5px rgba(255,255,255,.08),0 8px 14px rgba(0,0,0,.2);font-size:14px;font-weight:800;color:#c9dbea;}\n"
".modebtn small{display:block;margin-top:3px;font-size:10px;color:#7bb6e7;}\n"
".modebtn.active{background:linear-gradient(180deg,#45ba77,#1c794f);border-color:rgba(167,255,204,.75);box-shadow:0 0 24px rgba(57,255,144,.65),inset 0 1px 6px rgba(255,255,255,.18);color:#fff;}\n"
".modebtn.active small{color:#ffe3a5;}\n"
".footer{margin-top:18px;text-align:center;font-size:11px;letter-spacing:0;color:#80a5bf;opacity:.68;}\n"
"@media(max-width:360px){.panel{padding-left:19px;padding-right:19px}.dpad{transform:scale(.94);transform-origin:center top;margin-bottom:-2px}h1{font-size:21px}}\n"
"</style>\n"
"</head>\n"
"<body>\n"
"<div class=\"panel\">\n"
"<h1>CHAX'S TABLE<br>ROBOTS</h1>\n"
"<div class=\"dpad\">\n"
"<button class=\"cmd up\" onclick=\"sendCommand('/f')\"><span class=\"ico\">&#9650;</span><span class=\"txt\">UP</span></button>\n"
"<button class=\"cmd left\" onclick=\"sendCommand('/l')\"><span class=\"ico\">&#9664;</span><span class=\"txt\">Left</span></button>\n"
"<button class=\"cmd stop\" onclick=\"sendCommand('/s')\"><span class=\"ico\">&#9632;</span><span class=\"txt\">STOP</span></button>\n"
"<button class=\"cmd right\" onclick=\"sendCommand('/r')\"><span class=\"ico\">&#9654;</span><span class=\"txt\">Right</span></button>\n"
"<button class=\"cmd down\" onclick=\"sendCommand('/b')\"><span class=\"ico\">&#9660;</span><span class=\"txt\">Down</span></button>\n"
"</div>\n"
"<div class=\"modes\">\n"
"<button class=\"modebtn\" id=\"btn_sleep\" onclick=\"setMode('off')\">&#30561;&#30496;&#27169;&#24335;<small>-z</small></button>\n"
"<button class=\"modebtn\" id=\"btn_wiggle\" onclick=\"setMode('soft')\">&#25670;&#21160;&#27169;&#24335;<small>&#9679;</small></button>\n"
"<button class=\"modebtn active\" id=\"btn_curious\" onclick=\"setMode('normal')\">&#22909;&#22855;&#27169;&#24335;<small>&#9679;</small></button>\n"
"</div>\n"
"<div class=\"footer\">Powered by ChaX !</div>\n"
"</div>\n"
"<script>\n"
"function sendCommand(cmd){fetch(cmd).catch(function(e){console.error(e);});}\n"
"function clearActive(){document.getElementById('btn_sleep').classList.remove('active');document.getElementById('btn_wiggle').classList.remove('active');document.getElementById('btn_curious').classList.remove('active');}\n"
"function setMode(mode){sendCommand('/mode_'+mode);clearActive();if(mode==='off')document.getElementById('btn_sleep').classList.add('active');if(mode==='soft')document.getElementById('btn_wiggle').classList.add('active');if(mode==='normal')document.getElementById('btn_curious').classList.add('active');}\n"
"</script>\n"
"</body>\n"
"</html>\n";

bool robot_web_manual_active(void)
{
    return s_manual_active;
}

robot_random_mode_t robot_web_get_random_mode(void)
{
    return s_random_mode;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_base;

    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        const wifi_event_ap_staconnected_t *event = event_data;
        ESP_LOGI(TAG, "station " MACSTR " joined, AID=%d", MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        const wifi_event_ap_stadisconnected_t *event = event_data;
        ESP_LOGI(TAG, "station " MACSTR " left, AID=%d", MAC2STR(event->mac), event->aid);
    }
}

static void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    ESP_ERROR_CHECK(ap_netif == NULL ? ESP_FAIL : ESP_OK);

    const wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {0};
    const size_t ssid_len = strlen(ROBOT_WIFI_AP_SSID);
    memcpy(wifi_config.ap.ssid, ROBOT_WIFI_AP_SSID, ssid_len);
    wifi_config.ap.ssid_len = ssid_len;
    wifi_config.ap.channel = ROBOT_WIFI_AP_CHANNEL;
    wifi_config.ap.max_connection = ROBOT_WIFI_AP_MAX_CONNECTIONS;
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_netif_ip_info_t ip_info = {0};
    ESP_ERROR_CHECK(esp_netif_get_ip_info(ap_netif, &ip_info));
    ESP_LOGI(TAG, "WiFi AP started. SSID:%s IP:" IPSTR, ROBOT_WIFI_AP_SSID, IP2STR(&ip_info.ip));
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static const char *command_text(uint8_t command)
{
    switch (command) {
    case 0:
        return "stop command sent";
    case 1:
        return "forward command sent";
    case 2:
        return "backward command sent";
    case 3:
        return "left command sent";
    case 4:
        return "right command sent";
    default:
        return "unknown command";
    }
}

static esp_err_t command_get_handler(httpd_req_t *req)
{
    const uint8_t command = (uint8_t)(uintptr_t)req->user_ctx;

    s_manual_active = true;
    robot_motor_wifi_command(command);

    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    const esp_err_t err = httpd_resp_sendstr(req, command_text(command));

    s_manual_active = false;
    return err;
}

static const char *mode_text(robot_random_mode_t mode)
{
    switch (mode) {
    case ROBOT_RANDOM_OFF:
        return "sleep mode";
    case ROBOT_RANDOM_SOFT:
        return "wiggle mode";
    case ROBOT_RANDOM_NORMAL:
        return "curious mode";
    default:
        return "unknown mode";
    }
}

static esp_err_t mode_get_handler(httpd_req_t *req)
{
    const robot_random_mode_t mode = (robot_random_mode_t)(uintptr_t)req->user_ctx;
    s_random_mode = mode;

    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_sendstr(req, mode_text(mode));
}

static void register_http_routes(httpd_handle_t server)
{
    const httpd_uri_t uri_forward = {
        .uri = "/f",
        .method = HTTP_GET,
        .handler = command_get_handler,
        .user_ctx = (void *)1,
    };
    const httpd_uri_t uri_backward = {
        .uri = "/b",
        .method = HTTP_GET,
        .handler = command_get_handler,
        .user_ctx = (void *)2,
    };
    const httpd_uri_t uri_left = {
        .uri = "/l",
        .method = HTTP_GET,
        .handler = command_get_handler,
        .user_ctx = (void *)3,
    };
    const httpd_uri_t uri_right = {
        .uri = "/r",
        .method = HTTP_GET,
        .handler = command_get_handler,
        .user_ctx = (void *)4,
    };
    const httpd_uri_t uri_stop = {
        .uri = "/s",
        .method = HTTP_GET,
        .handler = command_get_handler,
        .user_ctx = (void *)0,
    };
    const httpd_uri_t uri_mode_off = {
        .uri = "/mode_off",
        .method = HTTP_GET,
        .handler = mode_get_handler,
        .user_ctx = (void *)ROBOT_RANDOM_OFF,
    };
    const httpd_uri_t uri_mode_soft = {
        .uri = "/mode_soft",
        .method = HTTP_GET,
        .handler = mode_get_handler,
        .user_ctx = (void *)ROBOT_RANDOM_SOFT,
    };
    const httpd_uri_t uri_mode_normal = {
        .uri = "/mode_normal",
        .method = HTTP_GET,
        .handler = mode_get_handler,
        .user_ctx = (void *)ROBOT_RANDOM_NORMAL,
    };
    const httpd_uri_t uri_root_exact = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
    };
    const httpd_uri_t uri_root = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = root_get_handler,
    };

    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_forward));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_backward));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_left));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_right));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_stop));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_mode_off));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_mode_soft));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_mode_normal));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_root_exact));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_root));
}

static void start_web_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = ROBOT_HTTP_PORT;
    config.max_uri_handlers = 12;
    config.uri_match_fn = httpd_uri_match_wildcard;

    httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(httpd_start(&server, &config));
    register_http_routes(server);
    ESP_LOGI(TAG, "HTTP server started on port %d", ROBOT_HTTP_PORT);
}

static void dns_set_u16(uint8_t *buf, uint16_t value)
{
    buf[0] = (uint8_t)(value >> 8);
    buf[1] = (uint8_t)(value & 0xFF);
}

static int dns_build_reply(const uint8_t *request, int request_len, uint8_t *reply, size_t reply_max_len)
{
    if (request_len < 12 || (size_t)request_len > reply_max_len) {
        return 0;
    }

    const uint16_t qd_count = ((uint16_t)request[4] << 8) | request[5];
    if (qd_count == 0 || qd_count > 4) {
        return 0;
    }

    size_t question_end = 12;
    for (uint16_t q = 0; q < qd_count; q++) {
        while (question_end < (size_t)request_len && request[question_end] != 0) {
            if ((request[question_end] & 0xC0) == 0xC0) {
                question_end += 2;
                break;
            }
            question_end += (size_t)request[question_end] + 1;
        }
        if (question_end >= (size_t)request_len) {
            return 0;
        }
        if (request[question_end] == 0) {
            question_end++;
        }
        if (question_end + 4 > (size_t)request_len) {
            return 0;
        }
        question_end += 4;
    }

    memcpy(reply, request, (size_t)request_len);
    reply[2] = 0x81;
    reply[3] = 0x80;
    reply[6] = request[4];
    reply[7] = request[5];
    reply[8] = 0;
    reply[9] = 0;
    reply[10] = 0;
    reply[11] = 0;

    size_t out = (size_t)request_len;
    for (uint16_t q = 0; q < qd_count; q++) {
        if (out + 16 > reply_max_len) {
            return 0;
        }

        reply[out++] = 0xC0;
        reply[out++] = 0x0C;
        dns_set_u16(&reply[out], 1);
        out += 2;
        dns_set_u16(&reply[out], 1);
        out += 2;
        reply[out++] = 0x00;
        reply[out++] = 0x00;
        reply[out++] = 0x00;
        reply[out++] = 0x3C;
        dns_set_u16(&reply[out], 4);
        out += 2;
        reply[out++] = 192;
        reply[out++] = 168;
        reply[out++] = 4;
        reply[out++] = 1;
    }

    return (int)out;
}

static void dns_server_task(void *pv_parameters)
{
    (void)pv_parameters;

    const int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "DNS socket create failed: errno=%d", errno);
        vTaskDelete(NULL);
        return;
    }

    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    const struct sockaddr_in listen_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(ROBOT_DNS_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(sock, (const struct sockaddr *)&listen_addr, sizeof(listen_addr)) < 0) {
        ESP_LOGE(TAG, "DNS socket bind failed: errno=%d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "DNS redirect server started on port %d", ROBOT_DNS_PORT);

    while (1) {
        uint8_t request[512];
        uint8_t reply[512];
        struct sockaddr_in source_addr;
        socklen_t source_addr_len = sizeof(source_addr);

        const int len = recvfrom(sock, request, sizeof(request), 0,
                                 (struct sockaddr *)&source_addr, &source_addr_len);
        if (len < 0) {
            ESP_LOGW(TAG, "DNS recv failed: errno=%d", errno);
            continue;
        }

        const int reply_len = dns_build_reply(request, len, reply, sizeof(reply));
        if (reply_len > 0) {
            sendto(sock, reply, (size_t)reply_len, 0,
                   (const struct sockaddr *)&source_addr, source_addr_len);
        }
    }
}

static void start_dns_server(void)
{
    xTaskCreate(dns_server_task, "dns_server", 4096, NULL, 5, NULL);
}

void robot_web_start(void)
{
    wifi_init_softap();
    start_dns_server();
    start_web_server();
}
