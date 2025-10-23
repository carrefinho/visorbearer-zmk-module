if(CONFIG_VISORBEARER_LED)
    target_sources(app PRIVATE src/visorbearer_led.c src/behaviors/behavior_visorbearer_led.c src/behaviors/behavior_visorbearer_soft_off.c)
    zephyr_include_directories(include)
endif()
