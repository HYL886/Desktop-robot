#pragma once

#include <stdbool.h>

#include "robot_config.h"

void robot_web_start(void);
bool robot_web_manual_active(void);
robot_random_mode_t robot_web_get_random_mode(void);
