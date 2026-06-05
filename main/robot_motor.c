#include "robot_motor.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "robot_config.h"

static SemaphoreHandle_t s_motor_mutex;

static void motor_lock(void)
{
    if (s_motor_mutex != NULL) {
        xSemaphoreTake(s_motor_mutex, portMAX_DELAY);
    }
}

static void motor_unlock(void)
{
    if (s_motor_mutex != NULL) {
        xSemaphoreGive(s_motor_mutex);
    }
}

static void motor_write_raw(int lf, int lb, int rf, int rb)
{
    ESP_ERROR_CHECK(gpio_set_level(ROBOT_LF_GPIO, lf));
    ESP_ERROR_CHECK(gpio_set_level(ROBOT_LB_GPIO, lb));
    ESP_ERROR_CHECK(gpio_set_level(ROBOT_RF_GPIO, rf));
    ESP_ERROR_CHECK(gpio_set_level(ROBOT_RB_GPIO, rb));
}

static void motor_stop_raw(void)
{
    motor_write_raw(0, 0, 0, 0);
}

void robot_motor_init(void)
{
    s_motor_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(s_motor_mutex == NULL ? ESP_ERR_NO_MEM : ESP_OK);

    const uint64_t pin_mask =
        (1ULL << ROBOT_LF_GPIO) |
        (1ULL << ROBOT_LB_GPIO) |
        (1ULL << ROBOT_RF_GPIO) |
        (1ULL << ROBOT_RB_GPIO) |
        (1ULL << ROBOT_STBY_GPIO);

    const gpio_config_t io_conf = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(ROBOT_STBY_GPIO, 0));
    motor_stop_raw();
}

void robot_motor_wifi_command(uint8_t command)
{
    motor_lock();
    ESP_ERROR_CHECK(gpio_set_level(ROBOT_STBY_GPIO, 1));

    switch (command) {
    case 0:
        motor_write_raw(0, 0, 0, 0);
        break;
    case 1:
        motor_write_raw(1, 0, 0, 1);
        break;
    case 2:
        motor_write_raw(0, 1, 1, 0);
        break;
    case 3:
        motor_write_raw(0, 1, 0, 1);
        break;
    case 4:
        motor_write_raw(1, 0, 1, 0);
        break;
    default:
        motor_write_raw(0, 0, 0, 0);
        break;
    }

    motor_unlock();
}

void robot_motor_random_action(uint8_t command, int active_ms, int stop_ms, int times)
{
    if (times <= 0) {
        return;
    }

    motor_lock();
    ESP_ERROR_CHECK(gpio_set_level(ROBOT_STBY_GPIO, 1));

    for (int i = 0; i < times; i++) {
        switch (command) {
        case 0:
            motor_write_raw(0, 0, 0, 0);
            break;
        case 1:
            motor_write_raw(0, 1, 0, 1);
            break;
        case 2:
            motor_write_raw(1, 0, 1, 0);
            break;
        case 3:
            motor_write_raw(0, 1, 1, 0);
            break;
        case 4:
            motor_write_raw(1, 0, 0, 1);
            break;
        case 5:
            motor_write_raw(0, 1, 0, 0);
            break;
        case 6:
            motor_write_raw(0, 0, 0, 1);
            break;
        case 7:
            motor_write_raw(1, 0, 0, 0);
            break;
        case 8:
            motor_write_raw(0, 0, 1, 0);
            break;
        default:
            motor_write_raw(0, 0, 0, 0);
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(active_ms));
        motor_stop_raw();
        vTaskDelay(pdMS_TO_TICKS(stop_ms));
    }

    motor_unlock();
}
