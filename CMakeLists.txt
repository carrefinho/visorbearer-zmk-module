add_subdirectory(drivers)

if(CONFIG_VISORBEARER_LED)
    target_sources(app PRIVATE src/visorbearer_led.c src/behaviors/behavior_visorbearer_led.c)
    target_sources_ifdef(CONFIG_VISORBEARER_SOFT_OFF app PRIVATE src/behaviors/behavior_visorbearer_soft_off.c)
    zephyr_include_directories(include)
endif()
