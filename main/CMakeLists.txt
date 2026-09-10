# ============================================================
# 应用模式切换：只改下面这一行
#   "ir"  = 红外循迹 + 超声波避障        （原 main_final.c）
#   "cam" = 摄像头循迹 + 避障 + 找球推球 （原 cam_avoid_catch.c）
#
# 两套工程完全并列，引脚/超参数各自独立（见 ir/ir_config.h 与 cam/cam_config.h）
# ============================================================
set(ROBOT_APP_MODE "cam")

if(ROBOT_APP_MODE STREQUAL "ir")
    message(STATUS "ROBOT_APP_MODE=ir  (红外循迹+避障)")
    idf_component_register(
        SRCS
            "ir/main.c"
            "ir/motor.c"
            "ir/sensors.c"
            "ir/encoder.c"
            "ir/kinematics.c"
            "ir/tracking.c"
            "ir/lcd_ui.c"
        INCLUDE_DIRS
            "."
            "ir"
        REQUIRES
            driver
            freertos
            esp_timer
            esp_driver_pcnt
            esp_driver_gpio
            esp_driver_spi
            esp_driver_ledc
            esp_lcd
            lvgl
    )

elseif(ROBOT_APP_MODE STREQUAL "cam")
    message(STATUS "ROBOT_APP_MODE=cam (摄像头循迹+避障+找球)")
    idf_component_register(
        SRCS
            "cam/main.c"
            "cam/cam_state.c"
            "cam/util.c"
            "cam/motor.c"
            "cam/sensing.c"
            "cam/lcd_ui.c"
            "cam/camera.c"
            "cam/decode.c"
            "cam/vision_line.c"
            "cam/vision_catch.c"
            "cam/catch_fsm.c"
            "cam/debug_wifi.c"
            "tjpgd/tjpgd.c"
        INCLUDE_DIRS
            "."
            "cam"
            "tjpgd"
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
            esp_driver_pcnt
            esp_driver_spi
            esp_lcd
            lvgl
            esp_jpeg
    )

else()
    message(FATAL_ERROR "Unknown ROBOT_APP_MODE='${ROBOT_APP_MODE}'. Use \"ir\" or \"cam\".")
endif()
