#define DT_DRV_COMPAT zmk_behavior_visorbearer_led

#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>

#include <zmk/behavior.h>
#include "visorbearer-zmk-module/visorbearer_led.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct behavior_visorbearer_led_config {
    bool indicate_battery;
    bool indicate_connectivity;
};

static int behavior_visorbearer_led_init(const struct device *dev) {
    return 0;
}

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_visorbearer_led_config *cfg = dev->config;

    if (cfg->indicate_battery) {
        visorbearer_led_show_battery_status();
    }
    if (cfg->indicate_connectivity) {
        visorbearer_led_show_ble_status();
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_visorbearer_led_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
    .locality = BEHAVIOR_LOCALITY_GLOBAL,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = zmk_behavior_get_empty_param_metadata,
#endif
};

#define LEDVISORBEARER_INST(n)                                                                               \
    static struct behavior_visorbearer_led_config behavior_visorbearer_led_config_##n = {              \
        .indicate_battery = DT_INST_PROP(n, indicate_battery),                                       \
        .indicate_connectivity = DT_INST_PROP(n, indicate_connectivity),                             \
    };                                                                                               \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_visorbearer_led_init, NULL, NULL, &behavior_visorbearer_led_config_##n, \
                            POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                        \
                            &behavior_visorbearer_led_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LEDVISORBEARER_INST)