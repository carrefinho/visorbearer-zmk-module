#include <stdlib.h>
#include <string.h>
#include <errno.h>
#ifdef CONFIG_VISORBEARER_LED_SHOW_MODIFIERS
#include <ctype.h>
#endif
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led/lp50xx.h>
#include <zephyr/dt-bindings/led/led.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/util.h>

#include <zmk/ble.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/event_manager.h>
#include <zmk/battery.h>
#include <zmk/activity.h>
#include <zmk/usb.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/hid.h>
#include <zmk/endpoints.h>

#include "visorbearer-zmk-module/visorbearer_led.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(visorbearer_led, 4);

#define LPA_NODE DT_NODELABEL(lp5012a)
#define LPB_NODE DT_NODELABEL(lp5012b)
#define NUM_SEGMENTS 4
#define MAX_BRIGHTNESS 100

#define LED_FADE_STEP_SIZE CONFIG_VISORBEARER_LED_FADE_STEP_SIZE
#define LED_INIT_FADE_STEP_SIZE CONFIG_VISORBEARER_LED_INIT_FADE_STEP_SIZE
#define LED_BREATH_STEP_SIZE CONFIG_VISORBEARER_LED_BREATH_STEP_SIZE
#define LED_BREATH_MIN CONFIG_VISORBEARER_LED_BREATH_MIN
#define LED_BREATH_MAX CONFIG_VISORBEARER_LED_BREATH_MAX
#define MODIFIER_FADE_STEP_SIZE CONFIG_VISORBEARER_LED_MODIFIER_FADE_STEP_SIZE

#define LED_STARTUP_DISPLAY_TIME_MS CONFIG_VISORBEARER_LED_STARTUP_DISPLAY_TIME_MS
#define LED_EVENT_DISPLAY_TIME_MS CONFIG_VISORBEARER_LED_EVENT_DISPLAY_TIME_MS
#define LED_INIT_PAUSE_TIME_MS CONFIG_VISORBEARER_LED_INIT_PAUSE_TIME_MS
#define LED_SOFT_OFF_ACK_TIMEOUT_MS 250
#define LED_PM_SUSPEND_RETRIES 5
#define LED_PM_RETRY_DELAY_MS 20

#define BATTERY_CRITICAL_THRESHOLD CONFIG_VISORBEARER_LED_BATTERY_CRITICAL_THRESHOLD
#define BATTERY_LOW_THRESHOLD CONFIG_VISORBEARER_LED_BATTERY_LOW_THRESHOLD
#define BATTERY_PER_SEGMENT 25
#define CHARGING_CHECK_THROTTLE_MS 1000

#ifdef CONFIG_VISORBEARER_LED_SHOW_MODIFIERS
enum modifier_type {
    MOD_SEGMENT_SHIFT,
    MOD_SEGMENT_CTRL,
    MOD_SEGMENT_ALT,
    MOD_SEGMENT_GUI,
    MOD_SEGMENT_COUNT
};
#endif

enum color_index {
    COLOR_OFF,
    COLOR_PROFILE_CONNECTED,
    COLOR_PROFILE_OPEN,
    COLOR_PROFILE_PAIRED,
    COLOR_BACKGROUND,
    COLOR_BACKGROUND_RED,
    COLOR_BATTERY_WHITE,
    COLOR_BATTERY_YELLOW,
    COLOR_BATTERY_RED,
    COLOR_CHARGING_GREEN,
    COLOR_MODIFIER_ACTIVE,
    COLOR_SOFTOFF_WHITE
#ifdef CONFIG_VISORBEARER_LED_BATTERY_GRANULAR
    ,
    COLOR_BATTERY_WHITE_MID,
    COLOR_BATTERY_YELLOW_MID
#endif
};

static const uint8_t colors[][3] = {
    [COLOR_OFF] = {0x00, 0x00, 0x00},
    [COLOR_PROFILE_CONNECTED] = {0xFF, 0xFF, 0xFF},
    [COLOR_PROFILE_OPEN] = {0xCC, 0xCC, 0x00},
    [COLOR_PROFILE_PAIRED] = {0x00, 0xFF, 0xFF},
    [COLOR_BACKGROUND] = {0x08, 0x08, 0x08},
    [COLOR_BACKGROUND_RED] = {0x10, 0x00, 0x00},
    [COLOR_BATTERY_WHITE] = {0xFF, 0xFF, 0xFF},
    [COLOR_BATTERY_YELLOW] = {0xB3, 0xB3, 0x00},
    [COLOR_BATTERY_RED] = {0xB3, 0x00, 0x00},
    [COLOR_CHARGING_GREEN] = {0x00, 0xB3, 0x00},
    [COLOR_MODIFIER_ACTIVE] = {0x1A, 0x1A, 0x1A},
    [COLOR_SOFTOFF_WHITE] = {0x2A, 0x2A, 0x2A}
#ifdef CONFIG_VISORBEARER_LED_BATTERY_GRANULAR
    ,
    [COLOR_BATTERY_WHITE_MID] = {0x4D, 0x4D, 0x4D},
    [COLOR_BATTERY_YELLOW_MID] = {0x4D, 0x4D, 0x00}
#endif
};

#ifdef CONFIG_VISORBEARER_LED_SHOW_MODIFIERS

#define MODIFIER_ORDER_DEFAULT "SCAG"

#ifndef CONFIG_VISORBEARER_LED_MODIFIER_ORDER
#define CONFIG_VISORBEARER_LED_MODIFIER_ORDER MODIFIER_ORDER_DEFAULT
#endif

BUILD_ASSERT(sizeof(CONFIG_VISORBEARER_LED_MODIFIER_ORDER) - 1 == NUM_SEGMENTS,
             "Modifier order configuration must be exactly 4 characters");

static const enum modifier_type modifier_order_fallback[NUM_SEGMENTS] = {
    MOD_SEGMENT_SHIFT,
    MOD_SEGMENT_CTRL,
    MOD_SEGMENT_ALT,
    MOD_SEGMENT_GUI
};

static enum modifier_type modifier_order[NUM_SEGMENTS];

static enum modifier_type char_to_modifier(char ch) {
    switch (toupper(ch)) {
        case 'S': return MOD_SEGMENT_SHIFT;
        case 'C': return MOD_SEGMENT_CTRL;
        case 'A': return MOD_SEGMENT_ALT;
        case 'G': return MOD_SEGMENT_GUI;
        default:  return MOD_SEGMENT_COUNT;  // Invalid
    }
}

static bool parse_modifier_order(const char *order_str, enum modifier_type out_order[NUM_SEGMENTS]) {
    if (!order_str) {
        return false;
    }

    bool seen[MOD_SEGMENT_COUNT] = {false};

    for (int i = 0; i < NUM_SEGMENTS; i++) {
        enum modifier_type modifier = char_to_modifier(order_str[i]);
        if (modifier >= MOD_SEGMENT_COUNT || seen[modifier]) {
            return false;
        }
        seen[modifier] = true;
        out_order[i] = modifier;
    }

    return true;
}

static void initialize_modifier_order(void) {
    if (!parse_modifier_order(CONFIG_VISORBEARER_LED_MODIFIER_ORDER, modifier_order)) {
        if (strcmp(CONFIG_VISORBEARER_LED_MODIFIER_ORDER, MODIFIER_ORDER_DEFAULT) != 0) {
            LOG_WRN("Invalid modifier order \"%s\"; falling back to default \"%s\"",
                    CONFIG_VISORBEARER_LED_MODIFIER_ORDER, MODIFIER_ORDER_DEFAULT);
        }
        memcpy(modifier_order, modifier_order_fallback, sizeof(modifier_order));
    }
}

#endif // CONFIG_VISORBEARER_LED_SHOW_MODIFIERS

enum animation_type {
    ANIM_NONE,
    ANIM_FADE,
    ANIM_BREATH
};

struct led_segment {
    uint8_t color[3];
    uint8_t brightness;
    uint8_t target_brightness;
    enum animation_type animation;
    int8_t fade_step;
    bool breath_ascending;
    bool dirty;
};

struct led_bar {
    struct led_segment segments[NUM_SEGMENTS];
    int64_t expire_time;
#ifdef CONFIG_VISORBEARER_LED_SHOW_MODIFIERS
    bool showing_modifiers;
#endif
};

struct battery_segment_config {
    enum color_index color;
    enum animation_type animation;
};

static struct led_bar conn_bar;
static struct led_bar batt_bar;
static const struct device *led_conn_dev;
static const struct device *led_batt_dev;
static const struct device *gpio0_dev;
#if IS_ENABLED(CONFIG_PM_DEVICE)
static const struct device *const led_devices[] = {
    DEVICE_DT_GET(LPA_NODE),
    DEVICE_DT_GET(LPB_NODE)
};
#endif

static atomic_t soft_off_in_progress = ATOMIC_INIT(0);

static struct {
    uint8_t active_profile;
    bool connected;
    bool advertising;
    uint8_t battery_percentage;
    bool charging;
    bool actively_charging;
    int64_t last_charging_check;
#ifdef CONFIG_VISORBEARER_LED_SHOW_MODIFIERS
    bool modifiers[MOD_SEGMENT_COUNT];
#endif
} system_state;

K_SEM_DEFINE(led_wake_sem, 0, 1);
K_SEM_DEFINE(soft_off_ack_sem, 0, 1);

static void sem_clear(struct k_sem *sem) {
    while (k_sem_take(sem, K_NO_WAIT) == 0) {
    }
}

static void signal_soft_off_ready(void) {
    k_sem_give(&soft_off_ack_sem);
}

static void reset_soft_off_ack(void) {
    sem_clear(&soft_off_ack_sem);
}

static bool bars_animating(void);

#if IS_ENABLED(CONFIG_PM_DEVICE)
static bool drivers_suspended = false;

static int suspend_led_device(const struct device *dev) {
    int ret = pm_device_action_run(dev, PM_DEVICE_ACTION_SUSPEND);

    if (ret == -EBUSY) {
        k_msleep(50);
        ret = pm_device_action_run(dev, PM_DEVICE_ACTION_SUSPEND);
    }

    if (ret == 0 || ret == -ENOSYS || ret == -ENOTSUP || ret == -EALREADY) {
        return 0;
    }

    return ret;
}

static void resume_led_device(const struct device *dev) {
    int ret = pm_device_action_run(dev, PM_DEVICE_ACTION_RESUME);
    if (ret < 0 && ret != -ENOSYS && ret != -ENOTSUP && ret != -EALREADY) {
        LOG_WRN("PM: resume of %s returned %d", dev->name, ret);
    }
}

static bool all_leds_off(void) {
    for (int i = 0; i < NUM_SEGMENTS; i++) {
        if (conn_bar.segments[i].brightness != 0 ||
            batt_bar.segments[i].brightness != 0) {
            return false;
        }
    }
    return true;
}

static void suspend_drivers_if_idle(void) {
    if (!drivers_suspended && all_leds_off() && !bars_animating()) {
        visorbearer_led_suspend_controllers();
        drivers_suspended = true;
        LOG_DBG("LED drivers suspended for power saving");
    }
}

static void resume_drivers_if_needed(void) {
    if (drivers_suspended) {
        visorbearer_led_resume_controllers();
        drivers_suspended = false;
        LOG_DBG("LED drivers resumed");
    }
}
#else
static inline void suspend_drivers_if_idle(void) {}
static inline void resume_drivers_if_needed(void) {}
#endif

static void segment_set(struct led_segment *seg, enum color_index color,
                       uint8_t target, enum animation_type anim, int8_t fade_step) {
    if (memcmp(seg->color, colors[color], 3) == 0 &&
        seg->target_brightness == target &&
        seg->animation == anim) {
        return;
    }

    memcpy(seg->color, colors[color], 3);
    seg->target_brightness = target;
    seg->animation = anim;
    seg->fade_step = fade_step;
    seg->dirty = true;

    if (anim == ANIM_BREATH) {
        if (seg->brightness > LED_BREATH_MAX) {
            seg->brightness = LED_BREATH_MAX;
            seg->breath_ascending = false;
        } else if (seg->brightness < LED_BREATH_MIN) {
            seg->brightness = LED_BREATH_MIN;
            seg->breath_ascending = true;
        }
    }
}

static void segment_update(struct led_segment *seg) {
    switch (seg->animation) {
        case ANIM_NONE:
            if (seg->brightness != seg->target_brightness) {
                seg->brightness = seg->target_brightness;
                seg->dirty = true;
            }
            break;

        case ANIM_FADE:
            if (seg->brightness != seg->target_brightness) {
                int16_t diff = seg->target_brightness - seg->brightness;
                int16_t step = abs(seg->fade_step);
                if (abs(diff) <= step) {
                    seg->brightness = seg->target_brightness;
                    seg->animation = ANIM_NONE;
                } else {
                    seg->brightness += (diff > 0) ? step : -step;
                }
                seg->dirty = true;
            } else {
                seg->animation = ANIM_NONE;
            }
            break;

        case ANIM_BREATH:
            seg->brightness += seg->breath_ascending ? LED_BREATH_STEP_SIZE : -LED_BREATH_STEP_SIZE;
            if (seg->brightness >= LED_BREATH_MAX) {
                seg->brightness = LED_BREATH_MAX;
                seg->breath_ascending = false;
            } else if (seg->brightness <= LED_BREATH_MIN) {
                seg->brightness = LED_BREATH_MIN;
                seg->breath_ascending = true;
            }
            seg->dirty = true;
            break;
    }
}

static void segment_write_hardware(const struct device *dev, int index, struct led_segment *seg) {
    if (!seg->dirty) return;

    if (seg->brightness == 0) {
        led_off(dev, index);
    } else {
        led_set_color(dev, index, 3, seg->color);
        led_on(dev, index);
        led_set_brightness(dev, index, seg->brightness);
    }
    seg->dirty = false;
}

#ifdef CONFIG_VISORBEARER_LED_SHOW_MODIFIERS
static bool any_modifier_active(void) {
    for (int i = 0; i < MOD_SEGMENT_COUNT; i++) {
        if (system_state.modifiers[i]) return true;
    }
    return false;
}
#endif

static bool is_actively_charging(void) {
    if (!gpio0_dev) return false;

    // Configure as input only when reading to minimize leakage current
    gpio_pin_configure(gpio0_dev, 17, GPIO_INPUT);

    k_usleep(10);

    int pin_value = gpio_pin_get(gpio0_dev, 17);
    bool charging = (pin_value == 0);

    // If gpio_pin_get fails (returns negative), treat as not charging
    if (pin_value < 0) {
        charging = false;
    }

    gpio_pin_configure(gpio0_dev, 17, GPIO_DISCONNECTED);

    return charging;
}

static struct battery_segment_config get_battery_segment_config(int segment,
                                                                uint8_t battery_pct,
                                                                bool charging,
                                                                bool actively_charging) {
    bool is_critical = battery_pct < BATTERY_CRITICAL_THRESHOLD && !charging;

    struct battery_segment_config config = {
        .color = is_critical ? COLOR_BACKGROUND_RED : COLOR_BACKGROUND,
        .animation = ANIM_FADE
    };

    uint8_t filled_segments = battery_pct / BATTERY_PER_SEGMENT;

    if (segment < filled_segments) {
        // full segment
        if (charging) {
            config.color = COLOR_CHARGING_GREEN;
        } else if (is_critical) {
            config.color = COLOR_BATTERY_RED;
            config.animation = ANIM_BREATH;
        } else if (segment == 0 && battery_pct < BATTERY_LOW_THRESHOLD) {
            config.color = COLOR_BATTERY_YELLOW;
        } else {
            config.color = COLOR_BATTERY_WHITE;
        }
    } else if (segment == filled_segments && filled_segments < NUM_SEGMENTS) {
        // partial segment
        if (charging) {
            config.color = COLOR_CHARGING_GREEN;
            if (actively_charging) {
                config.animation = ANIM_BREATH;
            }
        } else if (is_critical) {
            config.color = COLOR_BATTERY_RED;
            config.animation = ANIM_BREATH;
        } else {
#ifdef CONFIG_VISORBEARER_LED_BATTERY_GRANULAR
            uint8_t segment_start = filled_segments * BATTERY_PER_SEGMENT;
            uint8_t pct_in_segment = battery_pct - segment_start;

            if (pct_in_segment < 8) {
                config.color = is_critical ? COLOR_BACKGROUND_RED : COLOR_BACKGROUND;
            } else {
                bool is_yellow = (segment == 0 && battery_pct < BATTERY_LOW_THRESHOLD);
                if (pct_in_segment < 17) {
                    config.color = is_yellow ? COLOR_BATTERY_YELLOW_MID : COLOR_BATTERY_WHITE_MID;
                } else {
                    config.color = is_yellow ? COLOR_BATTERY_YELLOW : COLOR_BATTERY_WHITE;
                }
            }
#else
            config.color = (segment == 0 && battery_pct < BATTERY_LOW_THRESHOLD) ?
                          COLOR_BATTERY_YELLOW : COLOR_BATTERY_WHITE;
#endif
        }
    }

    return config;
}

static void display_connection_status(void) {
    for (int i = 0; i < NUM_SEGMENTS; i++) {
        if (i == system_state.active_profile) {
            enum color_index color = system_state.connected ? COLOR_PROFILE_CONNECTED :
                                    system_state.advertising ? COLOR_PROFILE_OPEN :
                                    COLOR_PROFILE_PAIRED;

            enum animation_type anim = (system_state.advertising ||
                                       (!system_state.connected && !system_state.advertising))
                                       ? ANIM_BREATH : ANIM_FADE;

            segment_set(&conn_bar.segments[i], color, MAX_BRIGHTNESS, anim, LED_FADE_STEP_SIZE);
        } else {
            segment_set(&conn_bar.segments[i], COLOR_BACKGROUND, MAX_BRIGHTNESS,
                       ANIM_FADE, LED_FADE_STEP_SIZE);
        }
    }
}

#ifdef CONFIG_VISORBEARER_LED_SHOW_MODIFIERS
static void display_modifiers(void) {
    for (int i = 0; i < NUM_SEGMENTS; i++) {
        enum modifier_type modifier = modifier_order[i];

        if (system_state.modifiers[modifier]) {
            segment_set(&conn_bar.segments[i], COLOR_MODIFIER_ACTIVE, MAX_BRIGHTNESS,
                       ANIM_FADE, MODIFIER_FADE_STEP_SIZE);
        } else {
            segment_set(&conn_bar.segments[i], conn_bar.segments[i].color[0] == 0 ?
                       COLOR_OFF : COLOR_MODIFIER_ACTIVE, 0, ANIM_FADE, MODIFIER_FADE_STEP_SIZE);
        }
    }
}
#endif

static void display_battery_status(void) {
    for (int i = 0; i < NUM_SEGMENTS; i++) {
        struct battery_segment_config config = get_battery_segment_config(
            i, system_state.battery_percentage, system_state.charging, system_state.actively_charging);

        segment_set(&batt_bar.segments[i], config.color, MAX_BRIGHTNESS,
                   config.animation, LED_FADE_STEP_SIZE);
    }
}

static void fade_out_bar(struct led_bar *bar) {
    for (int i = 0; i < NUM_SEGMENTS; i++) {
        bar->segments[i].target_brightness = 0;
        bar->segments[i].animation = ANIM_FADE;
        bar->segments[i].fade_step = LED_FADE_STEP_SIZE;
    }
}

static bool bars_animating(void) {
    for (int i = 0; i < NUM_SEGMENTS; i++) {
        if (conn_bar.segments[i].animation != ANIM_NONE ||
            batt_bar.segments[i].animation != ANIM_NONE) {
            return true;
        }
    }
    return false;
}


static void update_ble_state(void) {
    system_state.connected = zmk_ble_active_profile_is_connected();
    system_state.advertising = zmk_ble_active_profile_is_open() && !system_state.connected;
}

static void update_charging_state(void) {
    int64_t now = k_uptime_get();
    if (now - system_state.last_charging_check >= CHARGING_CHECK_THROTTLE_MS) {
        system_state.actively_charging = is_actively_charging();
        system_state.last_charging_check = now;
    }
}

static bool bars_active(void) {
#ifdef CONFIG_VISORBEARER_LED_SHOW_MODIFIERS
    if (any_modifier_active()) {
        return true;
    }
#endif
    return conn_bar.expire_time > 0 || batt_bar.expire_time > 0 || bars_animating();
}

static void update_bars(void) {
    int64_t current_time = k_uptime_get();

    if (bars_active()) {
        resume_drivers_if_needed();
    }

    if (batt_bar.expire_time > 0 && system_state.charging) {
        update_charging_state();
    }

    if (conn_bar.expire_time > 0 && current_time >= conn_bar.expire_time) {
        conn_bar.expire_time = 0;
#ifdef CONFIG_VISORBEARER_LED_SHOW_MODIFIERS
        if (!conn_bar.showing_modifiers) {
            fade_out_bar(&conn_bar);
        }
#else
        fade_out_bar(&conn_bar);
#endif
    }

#ifdef CONFIG_VISORBEARER_LED_SHOW_MODIFIERS
    if (conn_bar.expire_time > 0 && !conn_bar.showing_modifiers) {
        display_connection_status();
    } else if (any_modifier_active()) {
        conn_bar.showing_modifiers = true;
        display_modifiers();
    } else if (conn_bar.showing_modifiers) {
        conn_bar.showing_modifiers = false;
        fade_out_bar(&conn_bar);
    }
#else
    if (conn_bar.expire_time > 0) {
        display_connection_status();
    }
#endif

    if (batt_bar.expire_time > 0 && current_time >= batt_bar.expire_time) {
        batt_bar.expire_time = 0;
        fade_out_bar(&batt_bar);
    }

    if (batt_bar.expire_time > 0) {
        display_battery_status();
    }

    for (int i = 0; i < NUM_SEGMENTS; i++) {
        if (atomic_get(&soft_off_in_progress)) {
            signal_soft_off_ready();
            return;
        }

        segment_update(&conn_bar.segments[i]);
        segment_update(&batt_bar.segments[i]);
        segment_write_hardware(led_conn_dev, i, &conn_bar.segments[i]);
        segment_write_hardware(led_batt_dev, i, &batt_bar.segments[i]);
    }
}

static void show_connection_status(void) {
    int64_t new_expire = k_uptime_get() + LED_EVENT_DISPLAY_TIME_MS;
    if (conn_bar.expire_time < new_expire) {
        conn_bar.expire_time = new_expire;
    }
#ifdef CONFIG_VISORBEARER_LED_SHOW_MODIFIERS
    conn_bar.showing_modifiers = false;
#endif
    k_sem_give(&led_wake_sem);
}

static void show_battery_status(void) {
    int64_t new_expire = k_uptime_get() + LED_EVENT_DISPLAY_TIME_MS;
    if (batt_bar.expire_time < new_expire) {
        batt_bar.expire_time = new_expire;
    }
    k_sem_give(&led_wake_sem);
}

#ifdef CONFIG_VISORBEARER_LED_SHOW_MODIFIERS
static void update_modifier_state(uint8_t keycode, bool pressed) {
    enum modifier_type modifier = MOD_SEGMENT_COUNT;

    switch (keycode) {
        case HID_USAGE_KEY_KEYBOARD_LEFTSHIFT:
        case HID_USAGE_KEY_KEYBOARD_RIGHTSHIFT:
            modifier = MOD_SEGMENT_SHIFT;
            break;
        case HID_USAGE_KEY_KEYBOARD_LEFTCONTROL:
        case HID_USAGE_KEY_KEYBOARD_RIGHTCONTROL:
            modifier = MOD_SEGMENT_CTRL;
            break;
        case HID_USAGE_KEY_KEYBOARD_LEFTALT:
        case HID_USAGE_KEY_KEYBOARD_RIGHTALT:
            modifier = MOD_SEGMENT_ALT;
            break;
        case HID_USAGE_KEY_KEYBOARD_LEFT_GUI:
        case HID_USAGE_KEY_KEYBOARD_RIGHT_GUI:
            modifier = MOD_SEGMENT_GUI;
            break;
    }

    if (modifier < MOD_SEGMENT_COUNT && system_state.modifiers[modifier] != pressed) {
        system_state.modifiers[modifier] = pressed;
        k_sem_give(&led_wake_sem);
    }
}
#endif

static int led_init(void) {
    led_conn_dev = DEVICE_DT_GET(LPA_NODE);
    led_batt_dev = DEVICE_DT_GET(LPB_NODE);

    if (!device_is_ready(led_conn_dev) || !device_is_ready(led_batt_dev)) {
        LOG_ERR("LED devices not ready");
        return -ENODEV;
    }

    gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio0_dev)) {
        LOG_ERR("GPIO0 device not ready");
        return -ENODEV;
    }

    memset(&conn_bar, 0, sizeof(conn_bar));
    memset(&batt_bar, 0, sizeof(batt_bar));
    for (int i = 0; i < NUM_SEGMENTS; i++) {
        conn_bar.segments[i].dirty = true;
        batt_bar.segments[i].dirty = true;
    }

#ifdef CONFIG_VISORBEARER_LED_SHOW_MODIFIERS
    initialize_modifier_order();
#endif

    system_state.active_profile = zmk_ble_active_profile_index();
    update_ble_state();
    system_state.battery_percentage = zmk_battery_state_of_charge();
    system_state.charging = zmk_usb_is_powered();
    system_state.actively_charging = is_actively_charging();
    system_state.last_charging_check = k_uptime_get();

#ifdef CONFIG_VISORBEARER_LED_SHOW_MODIFIERS
    zmk_mod_flags_t mods = zmk_hid_get_explicit_mods();
    system_state.modifiers[MOD_SEGMENT_SHIFT] = (mods & (MOD_LSFT | MOD_RSFT)) != 0;
    system_state.modifiers[MOD_SEGMENT_CTRL] = (mods & (MOD_LCTL | MOD_RCTL)) != 0;
    system_state.modifiers[MOD_SEGMENT_ALT] = (mods & (MOD_LALT | MOD_RALT)) != 0;
    system_state.modifiers[MOD_SEGMENT_GUI] = (mods & (MOD_LGUI | MOD_RGUI)) != 0;
#endif

    for (int retry = 0; retry < 10 && system_state.battery_percentage == 0; retry++) {
        k_sleep(K_MSEC(10));
        system_state.battery_percentage = zmk_battery_state_of_charge();
    }

    for (int stage = 0; stage < NUM_SEGMENTS; stage++) {
        int conn_idx = NUM_SEGMENTS - 1 - stage;
        int batt_idx = stage;

        segment_set(&conn_bar.segments[conn_idx], COLOR_BACKGROUND, MAX_BRIGHTNESS,
                   ANIM_FADE, LED_INIT_FADE_STEP_SIZE);
        segment_set(&batt_bar.segments[batt_idx], COLOR_BACKGROUND, MAX_BRIGHTNESS,
                   ANIM_FADE, LED_INIT_FADE_STEP_SIZE);

        while (conn_bar.segments[conn_idx].animation != ANIM_NONE ||
               batt_bar.segments[batt_idx].animation != ANIM_NONE) {
            segment_update(&conn_bar.segments[conn_idx]);
            segment_update(&batt_bar.segments[batt_idx]);
            segment_write_hardware(led_conn_dev, conn_idx, &conn_bar.segments[conn_idx]);
            segment_write_hardware(led_batt_dev, batt_idx, &batt_bar.segments[batt_idx]);
            k_sleep(K_MSEC(10));
        }
    }

    k_sleep(K_MSEC(LED_INIT_PAUSE_TIME_MS));

    int64_t startup_expire = k_uptime_get() + LED_STARTUP_DISPLAY_TIME_MS;
    conn_bar.expire_time = startup_expire;
    batt_bar.expire_time = startup_expire;

    LOG_INF("LED initialized - Profile:%d Connected:%d Battery:%d%% Charging:%d",
            system_state.active_profile, system_state.connected,
            system_state.battery_percentage, system_state.charging);

    return 0;
}

static void led_thread(void *arg1, void *arg2, void *arg3) {
    led_init();

    while (1) {
        if (atomic_get(&soft_off_in_progress)) {
            signal_soft_off_ready();
            k_sleep(K_MSEC(100));
            continue;
        }

        update_bars();

        if (bars_animating()) {
            k_sleep(K_MSEC(10));
        } else {
            suspend_drivers_if_idle();
            k_sem_take(&led_wake_sem, K_MSEC(100));
        }
    }
}

static int ble_profile_changed_listener(const zmk_event_t *eh) {
    system_state.active_profile = zmk_ble_active_profile_index();
    update_ble_state();
    LOG_INF("Profile changed to %d", system_state.active_profile);
    show_connection_status();
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(led_bar, ble_profile_changed_listener);
ZMK_SUBSCRIPTION(led_bar, zmk_ble_active_profile_changed);

static int activity_state_changed_listener(const zmk_event_t *eh) {
    const struct zmk_activity_state_changed *event = as_zmk_activity_state_changed(eh);
    if (event && event->state == ZMK_ACTIVITY_ACTIVE) {
        update_ble_state();

        // show status for disconnected profile (when endpoint is BLE) or critical battery
        if (!system_state.connected && zmk_endpoint_get_preferred_transport() == ZMK_TRANSPORT_BLE) {
            show_connection_status();
        }
        if (system_state.battery_percentage < BATTERY_CRITICAL_THRESHOLD) {
            show_battery_status();
        }
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(led_activity, activity_state_changed_listener);
ZMK_SUBSCRIPTION(led_activity, zmk_activity_state_changed);

static int usb_conn_state_changed_listener(const zmk_event_t *eh) {
    system_state.charging = zmk_usb_is_powered();
    system_state.actively_charging = is_actively_charging();
    system_state.last_charging_check = k_uptime_get();
    LOG_INF("USB state changed - charging: %d", system_state.charging);
    show_battery_status();
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(led_usb, usb_conn_state_changed_listener);
ZMK_SUBSCRIPTION(led_usb, zmk_usb_conn_state_changed);

static int battery_state_changed_listener(const zmk_event_t *eh) {
    const struct zmk_battery_state_changed *event = as_zmk_battery_state_changed(eh);
    if (event) {
        system_state.battery_percentage = event->state_of_charge;
        if (system_state.battery_percentage < BATTERY_CRITICAL_THRESHOLD) {
            show_battery_status();
        }
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(led_battery, battery_state_changed_listener);
ZMK_SUBSCRIPTION(led_battery, zmk_battery_state_changed);

#ifdef CONFIG_VISORBEARER_LED_SHOW_MODIFIERS
static int keycode_state_changed_listener(const zmk_event_t *eh) {
    const struct zmk_keycode_state_changed *event = as_zmk_keycode_state_changed(eh);
    if (event && is_mod(event->usage_page, event->keycode)) {
        update_modifier_state(event->keycode, event->state);
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(led_keycode, keycode_state_changed_listener);
ZMK_SUBSCRIPTION(led_keycode, zmk_keycode_state_changed);
#endif

K_THREAD_DEFINE(led_thread_id, 1024, led_thread, NULL, NULL, NULL,
                K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);

void visorbearer_led_show_ble_status(void) {
    show_connection_status();
}

void visorbearer_led_show_battery_status(void) {
    show_battery_status();
}

void visorbearer_led_set_soft_off_mode(bool enabled) {
    if (enabled) {
        reset_soft_off_ack();
        atomic_set(&soft_off_in_progress, 1);
        k_sem_give(&led_wake_sem);

        int ret = k_sem_take(&soft_off_ack_sem, K_MSEC(LED_SOFT_OFF_ACK_TIMEOUT_MS));
        if (ret != 0) {
            LOG_DBG("Soft-off: LED thread did not idle within %d ms", LED_SOFT_OFF_ACK_TIMEOUT_MS);
        }
    } else {
        atomic_set(&soft_off_in_progress, 0);
        reset_soft_off_ack();
        k_sem_give(&led_wake_sem);
    }
}

int visorbearer_led_suspend_controllers(void) {
#if !IS_ENABLED(CONFIG_PM_DEVICE)
    return 0;
#else
    for (size_t i = 0; i < ARRAY_SIZE(led_devices); i++) {
        const struct device *dev = led_devices[i];

        if (!device_is_ready(dev)) {
            LOG_WRN("Soft-off: device %s not ready for suspend", dev->name);
            continue;
        }

        int ret = suspend_led_device(dev);
        if (ret < 0) {
            LOG_ERR("Soft-off: suspend of %s failed (%d)", dev->name, ret);
            visorbearer_led_resume_controllers();
            return ret;
        }
    }

    return 0;
#endif
}

void visorbearer_led_resume_controllers(void) {
#if IS_ENABLED(CONFIG_PM_DEVICE)
    for (int i = ARRAY_SIZE(led_devices) - 1; i >= 0; i--) {
        const struct device *dev = led_devices[i];

        if (device_is_ready(dev)) {
            resume_led_device(dev);
        }
    }
#endif
}

void visorbearer_led_show_soft_off_anim(void) {
    resume_drivers_if_needed();
    for (uint32_t led_idx = 0; led_idx < NUM_SEGMENTS; led_idx++) {
        segment_set(&conn_bar.segments[led_idx], COLOR_SOFTOFF_WHITE, MAX_BRIGHTNESS,
                   ANIM_NONE, 0);
        segment_set(&batt_bar.segments[led_idx], COLOR_SOFTOFF_WHITE, MAX_BRIGHTNESS,
                   ANIM_NONE, 0);
        segment_update(&conn_bar.segments[led_idx]);
        segment_update(&batt_bar.segments[led_idx]);
        segment_write_hardware(led_conn_dev, led_idx, &conn_bar.segments[led_idx]);
        segment_write_hardware(led_batt_dev, led_idx, &batt_bar.segments[led_idx]);
    }

    k_msleep(300);

    for (int stage = 0; stage < NUM_SEGMENTS; stage++) {
        int conn_idx = stage;                       // A0, A1, A2, A3
        int batt_idx = NUM_SEGMENTS - 1 - stage;    // B3, B2, B1, B0

        segment_set(&conn_bar.segments[conn_idx], COLOR_SOFTOFF_WHITE, 0,
                   ANIM_FADE, 10);
        segment_set(&batt_bar.segments[batt_idx], COLOR_SOFTOFF_WHITE, 0,
                   ANIM_FADE, 10);

        while (conn_bar.segments[conn_idx].animation != ANIM_NONE ||
               batt_bar.segments[batt_idx].animation != ANIM_NONE) {
            segment_update(&conn_bar.segments[conn_idx]);
            segment_update(&batt_bar.segments[batt_idx]);
            segment_write_hardware(led_conn_dev, conn_idx, &conn_bar.segments[conn_idx]);
            segment_write_hardware(led_batt_dev, batt_idx, &batt_bar.segments[batt_idx]);
            k_sleep(K_MSEC(10));
        }
    }
}
