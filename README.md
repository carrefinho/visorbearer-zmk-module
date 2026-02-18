# Visorbearer ZMK Module

This is the ZMK module for the [Visorbearer](https://github.com/carrefinho/visorbearer) keyboard, a 32-key unibody split wireless keyboard featuring 8 RGB LED indicators.

> [!IMPORTANT]
>
> This module has been upgraded to work with the Zephyr 4.1 version of ZMK. There are some naming changes and new features. If you're upgrading from an earlier version, you'll need to update your configuration:
>
> **Behavior Devicetree File:**
> - At the top of your keymap: `#include <behaviors/led_bars.dtsi>` → `#include <behaviors/visorbearer_led.dtsi>`
>
> **Behaviors and Macros:**
> - **NEW**: `&vb_soft_off` (soft-off with LED animation)
> - `&ind_bat` → `&vb_ind_bat` (show battery status)
> - `&ind_con` → `&vb_ind_con` (show connection status)
> - `&bt_sel_led` → `&vb_bt_sel` (BT select with LED indication)
> - `&bt_clr_led` → `&vb_bt_clr` (BT clear with LED indication)
>
> **Kconfig Options:**
> - `_BAR_` has been removed from all Kconfig option names
> - Example: `CONFIG_VISORBEARER_LED_BAR_FADE_STEP_SIZE` → `CONFIG_VISORBEARER_LED_FADE_STEP_SIZE`
>
> If you'd like to stick with ZMK v0.3, you can use the [zmk-v0.3 branch](https://github.com/carrefinho/visorbearer-zmk-module/tree/zmk-v0.3).

## Usage

> [!IMPORTANT]
> If you're upgrading from a previous ZMK version (v0.3 and earlier), `board` has changed from `seeeduino_xiao_ble` to `xiao_ble//zmk`.

Add these lines to `config/west.yml` in your `zmk-config` repository:

```yaml
manifest:
  remotes:
    - name: zmkfirmware
      url-base: https://github.com/zmkfirmware
    - name: carrefinho                            # <---
      url-base: https://github.com/carrefinho     # <---
  projects:
    - name: zmk
      remote: zmkfirmware
      revision: main
      import: app/west.yml
    - name: zmk-keyboards-visorbearer             # <---
      repo-path: visorbearer-zmk-module           # <---
      remote: carrefinho                          # <---
      revision: main                              # <---
  self:
    path: config
```

Then add the `visorbearer` shield to your `build.yaml`:

```yaml
---
include:
  - board: xiao_ble//zmk
    shield: visorbearer
    snippet: studio-rpc-usb-uart
  - board: xiao_ble//zmk
    shield: settings_reset
```

For more information on ZMK Modules and building locally, see [the ZMK docs page on modules.](https://zmk.dev/docs/features/modules)

## LED Features

Visorbearer features two RGB LED bars with four segments each:

**Connection Bar (left):**
- **Upon Bluetooth events or behavior**:
  - **White**: Connected Bluetooth profile
  - **Cyan (Breathing)**: Paired but not connected
  - **Yellow (Breathing)**: Open profile, advertising
- **When idle**:
  - **Dim White**: Active modifier keys (order configurable, default: Shift/Ctrl/Alt/GUI)

**Battery Bar (right):**
- **White**: Normal battery
- **Yellow**: Low battery
- **Red**: Critical battery
- **Green**: Charging

## LED Indication Behaviors

Add `#include <behaviors/visorbearer_led.dtsi>` to the top of your keymap file to use the following behaviors and macros.

Two behaviors are available to trigger LED indications on demand:

```dts
&vb_ind_bat    // Show battery status when pressed
&vb_ind_con    // Show connection status when pressed
```

### Bluetooth Behavior Wrapper Macros

Use these macros to trigger LED indications when selecting/clearing BT profiles:

```dts
&vb_bt_sel 0   // Select BT profile 0 and show connection status
&vb_bt_sel 1   // Select BT profile 1 and show connection status
&vb_bt_sel 2   // Select BT profile 2 and show connection status
&vb_bt_sel 3   // Select BT profile 3 and show connection status
&vb_bt_clr     // Clear current BT profile and show connection status
```

### Soft-Off Behavior

A custom soft-off behavior with LED animation:

```dts
&vb_soft_off   // Enter soft-off mode with LED sweep animation
```

## Configuration

Customize LED behavior by adding options to your `config/visorbearer.conf` file:

```ini
# Animation speeds (higher = faster)
CONFIG_VISORBEARER_LED_FADE_STEP_SIZE=8
CONFIG_VISORBEARER_LED_BREATH_STEP_SIZE=3

# Display timing (milliseconds)
CONFIG_VISORBEARER_LED_EVENT_DISPLAY_TIME_MS=3000
CONFIG_VISORBEARER_LED_STARTUP_DISPLAY_TIME_MS=6000

# Battery thresholds (percentage)
CONFIG_VISORBEARER_LED_BATTERY_CRITICAL_THRESHOLD=8
CONFIG_VISORBEARER_LED_BATTERY_LOW_THRESHOLD=15

# Modifier display
CONFIG_VISORBEARER_LED_SHOW_MODIFIERS=y                 # Enable/disable modifier display
CONFIG_VISORBEARER_LED_MODIFIER_ORDER="SCAG"            # Customize modifier order: S=Shift, C=Ctrl, A=Alt, G=GUI
```

See `Kconfig` for all available configuration options.
