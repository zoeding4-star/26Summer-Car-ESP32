idf_component_register(SRCS "main_line_tracking.c" "main_obstacle_avoid.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio led_strip esp_driver_ledc esp_timer)