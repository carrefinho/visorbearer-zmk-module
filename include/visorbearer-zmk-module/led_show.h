#pragma once

#include <stdint.h>
#include <stdbool.h>

void led_show_ble_status(void);
void led_show_battery_status(void);
void led_set_soft_off_mode(bool enabled);
void led_show_soft_off_anim(void);