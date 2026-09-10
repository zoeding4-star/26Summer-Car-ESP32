#include "lcd_ui.h"
#include "encoder.h"
#include "sensors.h"
#include "ir_config.h"
#include "ir_types.h"

#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "lvgl.h"

static const char *TAG = "IR_LCD";

static lv_disp_draw_buf_t disp_buf;
static lv_color_t buf1[IR_LCD_H_RES * 20];
static lv_color_t buf2[IR_LCD_H_RES * 20];

static lv_obj_t *lbl_wheel_a = NULL;
static lv_obj_t *lbl_wheel_b = NULL;
static lv_obj_t *lbl_wheel_d = NULL;
static lv_obj_t *lbl_dist    = NULL;

static void increase_lvgl_tick(void *arg)
{
    (void)arg;
    lv_tick_inc(2);
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_map);
    lv_disp_flush_ready(drv);
}

void ir_lcd_init_and_ui(void)
{
    ESP_LOGI(TAG, "初始化 LCD SPI 总线...");
    spi_bus_config_t buscfg = {
        .sclk_io_num = IR_PIN_NUM_SCK,
        .mosi_io_num = IR_PIN_NUM_SDI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = IR_LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = IR_PIN_NUM_DC,
        .cs_gpio_num = IR_PIN_NUM_CS,
        .pclk_hz = 20 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = IR_PIN_NUM_RST,
        .rgb_endian = LCD_RGB_ENDIAN_BGR,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 0, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    lv_init();
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, IR_LCD_H_RES * 20);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = IR_LCD_H_RES;
    disp_drv.ver_res = IR_LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_drv_register(&disp_drv);

    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, 2000));

    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_white(), LV_PART_MAIN);

    lv_obj_t *cont = lv_obj_create(lv_scr_act());
    lv_obj_set_size(cont, 120, 150);
    lv_obj_center(cont);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_pad_all(cont, 2, 0);

    lbl_wheel_a = lv_label_create(cont);
    lv_obj_set_style_text_color(lbl_wheel_a, lv_color_black(), LV_PART_MAIN);
    lv_label_set_text(lbl_wheel_a, "WheelA: 0.0");

    lbl_wheel_b = lv_label_create(cont);
    lv_obj_set_style_text_color(lbl_wheel_b, lv_color_black(), LV_PART_MAIN);
    lv_label_set_text(lbl_wheel_b, "WheelB: 0.0");

    lbl_wheel_d = lv_label_create(cont);
    lv_obj_set_style_text_color(lbl_wheel_d, lv_color_black(), LV_PART_MAIN);
    lv_label_set_text(lbl_wheel_d, "WheelD: 0.0");

    lbl_dist = lv_label_create(cont);
    lv_obj_set_style_text_color(lbl_dist, lv_color_black(), LV_PART_MAIN);
    lv_label_set_text(lbl_dist, "US Dist: 0.0cm");
}

void ir_gui_task(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    char buf[32];

    while (1) {
        lv_timer_handler();

        if (xTaskGetTickCount() - xLastWakeTime >= pdMS_TO_TICKS(1000)) {
            xLastWakeTime = xTaskGetTickCount();

            ir_get_all_wheel_rpm();
            IrAllWheelRPM rpm = ir_telemetry_rpm();
            float dist = ir_telemetry_distance();

            if (lbl_wheel_a && lbl_wheel_b && lbl_wheel_d && lbl_dist) {
                snprintf(buf, sizeof(buf), "WheelA: %d.%d", (int)rpm.rpm_A, (int)fabs(rpm.rpm_A * 10) % 10);
                lv_label_set_text(lbl_wheel_a, buf);

                snprintf(buf, sizeof(buf), "WheelB: %d.%d", (int)rpm.rpm_B, (int)fabs(rpm.rpm_B * 10) % 10);
                lv_label_set_text(lbl_wheel_b, buf);

                snprintf(buf, sizeof(buf), "WheelD: %d.%d", (int)rpm.rpm_D, (int)fabs(rpm.rpm_D * 10) % 10);
                lv_label_set_text(lbl_wheel_d, buf);

                snprintf(buf, sizeof(buf), "US Dist: %d.%dcm", (int)dist, (int)fabs(dist * 10) % 10);
                lv_label_set_text(lbl_dist, buf);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
