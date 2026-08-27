#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"

static const char *TAG = "main";

// 1. 引脚配置
#define PIN_NUM_CS     GPIO_NUM_2
#define PIN_NUM_SCK    GPIO_NUM_1
#define PIN_NUM_SDI    GPIO_NUM_38
#define PIN_NUM_DC     GPIO_NUM_39
#define PIN_NUM_RST    GPIO_NUM_40

// 2. 屏幕分辨率配置 (ST7735 常见的 1.8 寸屏分辨率为 128x160)
#define LCD_H_RES      128
#define LCD_V_RES      160

// LVGL 绘制缓冲区
static lv_disp_draw_buf_t disp_buf;
static lv_color_t buf1[LCD_H_RES * 20]; // 20 行像素大小的动态缓存区
static lv_color_t buf2[LCD_H_RES * 20];

// LVGL 心跳定时器回调
static void increase_lvgl_tick(void *arg) {
    lv_tick_inc(2); // 每 2ms 增加一次 tick
}

// LVGL 刷屏回调函数
static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    
    // 将 LVGL 渲染的数据更新到屏幕
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    
    // 通知 LVGL 刷新完成
    lv_disp_flush_ready(drv);
}

void app_main(void) {
    ESP_LOGI(TAG, "初始化 SPI 总线...");
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_SCK,
        .mosi_io_num = PIN_NUM_SDI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "配置 Panel IO (SPI)...");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_DC,
        .cs_gpio_num = PIN_NUM_CS,
        .pclk_hz = 20 * 1000 * 1000, // 20MHz 传输速率
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &io_handle));

    ESP_LOGI(TAG, "安装面板驱动...");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_RST,
        .rgb_endian = LCD_RGB_ENDIAN_BGR,
        .bits_per_pixel = 16, // RGB565
    };
    // 此处使用 esp_lcd_new_panel_st7789
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    // 重置并初始化屏
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    
    // 反色与偏置调整（解决 ST7735 兼容问题）
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 0, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "初始化 LVGL 库...");
    lv_init();

    // 初始化 LVGL 双缓冲区
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_H_RES * 20);

    // 配置 LVGL 驱动结构体
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_drv_register(&disp_drv);

    // 创建高精度硬件定时器给 LVGL 提供 Tick
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, 2000)); // 2ms

    ESP_LOGI(TAG, "UI 界面构建中...");

    // ------------------ 创建 LVGL UI 界面 ------------------ //
    
    // 1. 设置屏幕背景为白色（确保黑色字体能够清晰显示）
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_white(), LV_PART_MAIN);

    // 2. 模拟传感器变量
    float speed_a = 15.5f;   // 车轮 A 转速
    float speed_b = 16.0f;   // 车轮 B 转速
    float speed_d = 15.2f;   // 车轮 D 转速
    float distance = 42.8f;  // 超声波测距 (cm)

    // 3. 创建垂直布局容器
    lv_obj_t *cont = lv_obj_create(lv_scr_act());
    lv_obj_set_size(cont, 120, 150);
    lv_obj_center(cont);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_pad_all(cont, 2, 0);

    // 4. 创建文本并设置为纯黑色 (0x000000 或 lv_color_black())
    lv_obj_t *lbl_wheel_a = lv_label_create(cont);
    lv_obj_set_style_text_color(lbl_wheel_a, lv_color_black(), LV_PART_MAIN);
    lv_label_set_text_fmt(lbl_wheel_a, "WheelA: %.1f", speed_a);

    lv_obj_t *lbl_wheel_b = lv_label_create(cont);
    lv_obj_set_style_text_color(lbl_wheel_b, lv_color_black(), LV_PART_MAIN);
    lv_label_set_text_fmt(lbl_wheel_b, "WheelB: %.1f", speed_b);

    lv_obj_t *lbl_wheel_d = lv_label_create(cont);
    lv_obj_set_style_text_color(lbl_wheel_d, lv_color_black(), LV_PART_MAIN);
    lv_label_set_text_fmt(lbl_wheel_d, "WheelD: %.1f", speed_d);

    lv_obj_t *lbl_dist = lv_label_create(cont);
    lv_obj_set_style_text_color(lbl_dist, lv_color_black(), LV_PART_MAIN);
    lv_label_set_text_fmt(lbl_dist, "US Dist: %.1fcm", distance);

// ------------------ LVGL 任务循环 ------------------ //

    ESP_LOGI(TAG, "启动 LVGL 任务循环");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        lv_timer_handler();
    }
}