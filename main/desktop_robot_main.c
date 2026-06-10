#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "robot_config.h"
#include "robot_display.h"
#include "robot_motor.h"
#include "robot_web.h"

static const char *TAG = "desktop_robot";

static uint32_t random_upto(uint32_t max)
{
    return max == 0 ? 0 : esp_random() % max;
}

static int random_range(int min, int max)
{
    if (max <= min) {
        return min;
    }
    return min + (int)random_upto((uint32_t)(max - min));
}

static void nvs_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

static void robot_loop_task(void *pv_parameters)
{
    (void)pv_parameters;

    int64_t last_random_tick_ms = 0;
    int64_t last_status_ms = 0;
    robot_random_mode_t last_mode = (robot_random_mode_t)-1;

    while (1) {
        const int64_t now_ms = esp_timer_get_time() / 1000;
        const robot_random_mode_t mode = robot_web_get_random_mode();

        if (mode != last_mode) {
            robot_display_set_random_mode(mode);
            last_mode = mode;
        }

        robot_display_update_eyes();

        if (!robot_web_manual_active() && now_ms - last_random_tick_ms > 40) {
            last_random_tick_ms = now_ms;

            if (mode == ROBOT_RANDOM_SOFT) {
                if (random_upto(120) == 1) {
                    robot_motor_random_action((uint8_t)random_upto(9), random_range(6, 18), random_range(40, 90), 1);
                }
            } else if (mode == ROBOT_RANDOM_NORMAL) {
                if (random_upto(100) == 1) {
                    robot_motor_random_action((uint8_t)random_upto(9), random_range(5, 50),
                                              random_range(10, 100), (int)random_upto(20));
                }
            }
        }

        if (now_ms - last_status_ms > 5000) {
            last_status_ms = now_ms;
            ESP_LOGI(TAG, "web control ready at http://192.168.4.1");
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(2000));

    nvs_init();
    robot_motor_init();

    const esp_err_t display_err = robot_display_init();
    if (display_err == ESP_OK) {
        robot_display_show_startup();
    } else {
        ESP_LOGW(TAG, "OLED init failed: %s", esp_err_to_name(display_err));
    }

    robot_web_start();
    robot_display_show_wifi_info("AP ONLY");
    ESP_LOGI(TAG, "system ready. Connect WiFi SSID:%s, then open http://192.168.4.1", ROBOT_WIFI_AP_SSID);
    vTaskDelay(pdMS_TO_TICKS(2000));

    xTaskCreate(robot_loop_task, "robot_loop", 4096, NULL, 4, NULL);
}
