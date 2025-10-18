# Visorbearer ZMK Module

This is the ZMK module for the [Visorbearer](https://github.com/carrefinho/visorbearer) keyboard, a 32-key unibody split wireless keyboard featuring 8 RGB LED indicators.

## Usage

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
  - board: seeeduino_xiao_ble
    shield: visorbearer
    snippet: studio-rpc-usb-uart
  - board: seeeduino_xiao_ble
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

Two behaviors are available to trigger LED indications on demand:

```dts
&ind_bat    // Show battery status when pressed
&ind_con    // Show connection status when pressed
```

### Bluetooth Behavior Wrapper Macros

Use these macros to also trigger LED indications when selecting/clearing BT profiles:

```dts
&bt_sel_led 0   // Select BT profile 0 and show connection status
&bt_sel_led 1   // Select BT profile 1 and show connection status
&bt_sel_led 2   // Select BT profile 2 and show connection status
&bt_sel_led 3   // Select BT profile 3 and show connection status
&bt_clr_led     // Clear current BT profile and show connection status
```

## Configuration

Customize LED behavior by adding options to your `config/visorbearer.conf` file:

```ini
# Animation speeds (higher = faster)
CONFIG_VISORBEARER_LED_BAR_FADE_STEP_SIZE=8
CONFIG_VISORBEARER_LED_BAR_BREATH_STEP_SIZE=3

# Display timing (milliseconds)
CONFIG_VISORBEARER_LED_BAR_EVENT_DISPLAY_TIME_MS=3000
CONFIG_VISORBEARER_LED_BAR_STARTUP_DISPLAY_TIME_MS=6000

# Battery thresholds (percentage)
CONFIG_VISORBEARER_LED_BAR_BATTERY_CRITICAL_THRESHOLD=8
CONFIG_VISORBEARER_LED_BAR_BATTERY_LOW_THRESHOLD=15

# Modifier display
CONFIG_VISORBEARER_LED_BAR_SHOW_MODIFIERS=y             # Enable/disable modifier display
CONFIG_VISORBEARER_LED_BAR_MODIFIER_ORDER="SCAG"        # Order: S=Shift, C=Ctrl, A=Alt, G=GUI
```

See `Kconfig` for all available configuration options.
