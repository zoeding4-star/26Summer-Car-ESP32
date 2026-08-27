#通过注释来确定编译哪个文件
# 编译循迹
# idf_component_register(SRCS "main_line_tracking.c"
#                        INCLUDE_DIRS "."
#                        REQUIRES esp_driver_gpio led_strip esp_driver_ledc esp_timer)

# 编译避障
# idf_component_register(SRCS "main_obstacle_avoid.c"
#                        INCLUDE_DIRS "."
#                        REQUIRES esp_driver_gpio led_strip esp_driver_ledc esp_timer)

#编译全部功能
idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio led_strip esp_driver_ledc esp_timer)                       