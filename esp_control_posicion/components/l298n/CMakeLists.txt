idf_component_register(
    SRCS "l298n.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_ledc esp_driver_gpio
)
