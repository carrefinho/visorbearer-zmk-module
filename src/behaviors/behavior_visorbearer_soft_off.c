#define DT_DRV_COMPAT zmk_behavior_visorbearer_soft_off

#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>

#include <zmk/pm.h>
#include <zmk/behavior.h>

#include "visorbearer-zmk-module/visorbearer_led.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

static atomic_t soft_off_running = ATOMIC_INIT(0);

struct behavior_visorbearer_soft_off_config {
    uint32_t hold_time_ms;
};

struct behavior_visorbearer_soft_off_data {
    uint32_t press_start;
};

static void visorbearer_reset_soft_off_state(void) {
    visorbearer_led_resume_controllers();
    visorbearer_led_set_soft_off_mode(false);
    atomic_set(&soft_off_running, 0);
}

static int visorbearer_run_soft_off(void) {
    if (atomic_cas(&soft_off_running, 0, 1) == false) {
        LOG_DBG("Soft-off already in progress, ignoring trigger");
        return 0;
    }

    LOG_INF("Visorbearer soft-off: stopping LED thread");
    visorbearer_led_set_soft_off_mode(true);

    LOG_INF("Visorbearer soft-off: running LED animation");
    visorbearer_led_show_soft_off_anim();

    int ret = visorbearer_led_suspend_controllers();
    if (ret < 0) {
        LOG_ERR("Visorbearer soft-off: LED suspend failed (%d)", ret);
        visorbearer_reset_soft_off_state();
        return ret;
    }

    ret = zmk_pm_soft_off();
    if (ret < 0) {
        LOG_ERR("Visorbearer soft-off: PM soft-off failed (%d)", ret);
        visorbearer_reset_soft_off_state();
    }

    return ret;
}

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_visorbearer_soft_off_data *data = dev->data;

    data->press_start = k_uptime_get();

    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_visorbearer_soft_off_data *data = dev->data;
    const struct behavior_visorbearer_soft_off_config *config = dev->config;

    if (config->hold_time_ms == 0) {
        visorbearer_run_soft_off();
    } else {
        uint32_t hold_time = k_uptime_get() - data->press_start;

        if (hold_time > config->hold_time_ms) {
            visorbearer_run_soft_off();
        } else {
            LOG_INF("Not triggering soft off: held for %d and hold time is %d", hold_time,
                    config->hold_time_ms);
        }
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_visorbearer_soft_off_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
    .locality = BEHAVIOR_LOCALITY_GLOBAL,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = zmk_behavior_get_empty_param_metadata,
#endif // IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
};

#define BVSO_INST(n)                                                                               \
    static const struct behavior_visorbearer_soft_off_config bvso_config_##n = {                   \
        .hold_time_ms = DT_INST_PROP_OR(n, hold_time_ms, 0),                                       \
    };                                                                                             \
    static struct behavior_visorbearer_soft_off_data bvso_data_##n = {};                           \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, &bvso_data_##n, &bvso_config_##n, POST_KERNEL,          \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                   \
                            &behavior_visorbearer_soft_off_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BVSO_INST)
