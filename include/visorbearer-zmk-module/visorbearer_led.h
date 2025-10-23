#pragma once

#include <stdint.h>
#include <stdbool.h>

void visorbearer_led_show_ble_status(void);
void visorbearer_led_show_battery_status(void);
void visorbearer_led_set_soft_off_mode(bool enabled);
void visorbearer_led_show_soft_off_anim(void);