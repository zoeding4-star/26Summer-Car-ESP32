#通过注释来确定编译哪个文件
# 编译循迹
# idf_component_register(SRCS  "main_line_tracking.c"
#                        INCLUDE_DIRS "."
#                        REQUIRES esp_driver_gpio led_strip esp_driver_ledc esp_timer)

# 编译避障
# idf_component_register(SRCS "cam_final.c" "cam_line_avoid_ui.c" "mpu_test.c" "catch.c" "main_obstacle_avoid.c"
#                        INCLUDE_DIRS "."
#                        REQUIRES esp_driver_gpio led_strip esp_driver_ledc esp_timer)

#编译液晶屏
# idf_component_register(SRCS "main_led.c"
#                        INCLUDE_DIRS "."
#                        REQUIRES esp_driver_gpio led_strip esp_driver_spi esp_driver_ledc esp_timer esp_lcd lvgl)  

# #编译全部功能
# idf_component_register(SRCS "main_final.c"
#                        INCLUDE_DIRS "."
#                        REQUIRES esp_driver_gpio led_strip esp_driver_spi esp_driver_ledc esp_timer esp_lcd lvgl)   

#编译测速文件
# idf_component_register(SRCS "main_speed.c"
#                        INCLUDE_DIRS "."
#                        REQUIRES esp_driver_gpio led_strip esp_driver_spi esp_driver_ledc esp_timer esp_lcd lvgl)   

#编译全部功能
# idf_component_register(SRCS "main.c"
#                        INCLUDE_DIRS "."
#                        REQUIRES esp_driver_gpio led_strip esp_driver_ledc esp_timer)                       

#编译全部功能   
# idf_component_register(SRCS "main_final.c"
#                        INCLUDE_DIRS "."
#                        REQUIRES esp_driver_pcnt esp_driver_gpio led_strip esp_driver_spi esp_driver_ledc esp_timer esp_lcd lvgl)   

# 推球入网（catch.c，彩色 JPEG 解码用 esp_jpeg）
idf_component_register(
    SRCS "catch.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        freertos
        usb
        usb_stream
        nvs_flash
        esp_psram
        esp_timer
        esp_wifi
        esp_event
        esp_netif
        esp_http_server
        esp_jpeg
)

# 编译 MPU6500 测试
# idf_component_register(
#     SRCS "mpu_test.c"
#     INCLUDE_DIRS "."
#     REQUIRES
#         driver
#         esp_driver_i2c
#         freertos
# )

# 推球颜色调参（车静止，网页调 HSV）
# idf_component_register(
#     SRCS "catch_tune.c"
#     INCLUDE_DIRS "."
#     REQUIRES
#         driver
#         freertos
#         usb
#         usb_stream
#         nvs_flash
#         esp_psram
#         esp_timer
#         esp_wifi
#         esp_event
#         esp_netif
#         esp_http_server
#         esp_jpeg
# )

# 绕前方点 / 贴球微调单独测试
# idf_component_register(
#     SRCS "orbit_test.c"
#     INCLUDE_DIRS "."
#     REQUIRES
#         driver
#         freertos
#         esp_timer
# )

# 全程：循迹避障 + 停 3 秒 + 推球（屏幕）
# idf_component_register(
#     SRCS "cam_final.c"
#     INCLUDE_DIRS "."
#     REQUIRES
#         driver
#         freertos
#         usb
#         usb_stream
#         nvs_flash
#         esp_psram
#         esp_timer
#         esp_wifi
#         esp_event
#         esp_netif
#         esp_http_server
#         esp_driver_pcnt
#         esp_driver_spi
#         esp_lcd
#         lvgl
#         esp_jpeg
# )

# 摄像头循迹 + 超声波避障 + 屏幕转速/距离
# idf_component_register(
#     SRCS "cam_line_avoid_ui.c" "tjpgd/tjpgd.c"
#     INCLUDE_DIRS "." "tjpgd"
#     REQUIRES
#         driver
#         freertos
#         usb
#         usb_stream
#         nvs_flash
#         esp_psram
#         esp_timer
#         esp_wifi
#         esp_event
#         esp_netif
#         esp_http_server
#         esp_driver_pcnt
#         esp_driver_spi
#         esp_lcd
#         lvgl
# )

# 摄像头循迹 + 超声波避障（无屏幕）
# idf_component_register(
#     SRCS "cam_line_avoid.c" "tjpgd/tjpgd.c"
#     INCLUDE_DIRS "." "tjpgd"
#     REQUIRES
#         driver
#         freertos
#         usb
#         usb_stream
#         nvs_flash
#         esp_psram
#         esp_timer
#         esp_wifi
#         esp_event
#         esp_netif
#         esp_http_server
# )

# 仅摄像头循迹
# idf_component_register(
#     SRCS "cam_line_follow.c" "tjpgd/tjpgd.c"
#     INCLUDE_DIRS "." "tjpgd"
#     REQUIRES
#         driver
#         freertos
#         usb
#         usb_stream
#         nvs_flash
#         esp_psram
#         esp_timer
#         esp_wifi
#         esp_event
#         esp_netif
#         esp_http_server
# )

# 声明源码文件与依赖组件
# idf_component_register(
#     SRCS "cam_motion.c"                       # 你的 C 代码文件名，如果不是 main.c 请修改为实际文件名
#     INCLUDE_DIRS "."                    # 头文件包含路径
#     REQUIRES 
#         driver                          # 包含 GPIO、LEDC 驱动
#         freertos                        # 包含 FreeRTOS 任务与信号量
#         usb                             # 包含 ESP32-S3 底层 USB Host 框架
#         espressif__usb_host_uvc         # 必须显式链接 USB UVC 组件
#         esp_wifi
#         esp_event
#         nvs_flash
#         esp_http_server
#         usb_stream
#         esp_psram
#         esp_log
#         esp_timer
#         esp_err
#         esp_lcd_panel_io
#         esp_lcd_panel_vendor
#         esp_lcd_panel_ops
# )