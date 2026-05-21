#pragma once
#include <stdbool.h>
#include "drone_types.h"
#include "battery.h"

void led_ctrl_init(void);
void led_ctrl_set(bool on);
void led_ctrl_update(const flight_state_t *fs);
