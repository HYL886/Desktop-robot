#pragma once

#include "esp_err.h"

esp_err_t robot_display_init(void);
void robot_display_show_startup(void);
void robot_display_show_wifi_info(const char *status_line);
void robot_display_update_eyes(void);
