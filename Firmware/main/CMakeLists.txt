idf_component_register(
    SRCS
        "main.c"
        "power_cal.c"
        "energy_storage.c"
        "lcd2004.c"
        "measurement.c"

    INCLUDE_DIRS
        "."

    REQUIRES
        esp_adc
        esp_timer
        nvs_flash
        esp_driver_i2c
)