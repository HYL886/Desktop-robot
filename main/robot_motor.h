#pragma once

#include <stdint.h>

void robot_motor_init(void);
void robot_motor_wifi_command(uint8_t command);
void robot_motor_random_action(uint8_t command, int active_ms, int stop_ms, int times);
