#pragma once

#include "esp_err.h"

#include "robot_config.h"

esp_err_t robot_display_init(void);
void robot_display_show_startup(void);
void robot_display_show_wifi_info(const char *status_line);
void robot_display_set_random_mode(robot_random_mode_t mode);
void robot_display_update_eyes(void);
