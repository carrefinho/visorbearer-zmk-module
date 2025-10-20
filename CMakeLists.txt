if(CONFIG_VISORBEARER_LED_BAR)
    target_sources(app PRIVATE src/led.c src/behaviors/behavior_visorbearer_led_bars.c src/behaviors/behavior_visorbearer_soft_off.c)
    zephyr_include_directories(include)
endif()
