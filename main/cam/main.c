#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_psram.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "cam_config.h"
#include "cam_state.h"
#include "camera.h"
#include "catch_fsm.h"
#include "decode.h"
#include "debug_wifi.h"
#include "lcd_ui.h"
#include "motor.h"
#include "sensing.h"
#include "util.h"
#include "vision_line.h"

static const char *TAG = "AVOID_CATCH";

void line_task(void *arg)
{
    (void)arg;
    uint8_t *local_jpg = (uint8_t *)psram_alloc(JPEG_XFER_SIZE);
    if (!local_jpg) {
        ESP_LOGE(TAG, "任务 JPEG 缓冲分配失败");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "全程：循迹避障 -> 等 3 秒 -> 找球推球");
    if (esp_task_wdt_add(NULL) != ESP_OK) {
        ESP_LOGW(TAG, "line_follow 未能加入任务看门狗");
    }
    while (1) {
        (void)esp_task_wdt_reset();
        if (xSemaphoreTake(s_frame_ready, pdMS_TO_TICKS(150)) != pdTRUE) {
            if (g_mission == MISSION_CATCH) {
                if (g_state == ST_SEARCH_BALL || g_state == ST_IDLE) {
                    catch_spin_search_ball(true);
                }
            } else if (g_mission != MISSION_DONE) {
                g_lost_frames++;
                if (g_lost_frames > LOST_STOP_FRAMES) {
                    stop_motors();
                }
            } else {
                stop_motors();
            }
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        while (xSemaphoreTake(s_frame_ready, 0) == pdTRUE) {
        }

        uint32_t len = 0;
        if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            len = s_jpeg_len;
            if (len > 0 && len <= JPEG_XFER_SIZE) {
                memcpy(local_jpg, s_jpeg, len);
            }
            xSemaphoreGive(s_frame_mutex);
        }
        if (len == 0) {
            vTaskDelay(1);
            continue;
        }

        if (g_mission == MISSION_WAIT) {
            stop_motors();
            if ((esp_timer_get_time() - g_wait_t0) >= (int64_t)CATCH_WAIT_MS * 1000) {
                ESP_LOGI(TAG, "等待结束，开始找球推球");
                g_mission = MISSION_CATCH;
                enter_state(ST_SEARCH_BALL);
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        int64_t t0 = esp_timer_get_time();
        if (g_mission == MISSION_CATCH || g_mission == MISSION_DONE) {
            if (!decode_mjpeg_rgb(local_jpg, (int)len)) {
                ESP_LOGW(TAG, "JPEG 彩色解码失败 len=%u", (unsigned)len);
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            g_decode_ms = (int)((esp_timer_get_time() - t0) / 1000);
            (void)esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(15));
            if (g_mission == MISSION_CATCH) {
                catch_control_once();
            } else {
                stop_motors();
            }
            (void)esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (!decode_mjpeg(local_jpg, (int)len)) {
            ESP_LOGW(TAG, "JPEG 解码失败 len=%u", (unsigned)len);
            vTaskDelay(1);
            continue;
        }
        g_decode_ms = (int)((esp_timer_get_time() - t0) / 1000);
        control_once();
        vTaskDelay(1);
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    if (esp_psram_is_initialized()) {
        ESP_LOGI(TAG, "PSRAM %d KB", (int)(esp_psram_get_size() / 1024));
    } else {
        ESP_LOGW(TAG, "PSRAM 未启用，解码缓冲可能不足");
    }

    s_frame_mutex = xSemaphoreCreateMutex();
    s_frame_ready = xSemaphoreCreateBinary();
    s_dbg_mutex = xSemaphoreCreateMutex();
    s_dbg_gray = (uint8_t *)psram_alloc(CAM_WIDTH * CAM_HEIGHT);
    s_dbg_bin = (uint8_t *)psram_alloc(CAM_WIDTH * CAM_HEIGHT);
    if (!s_dbg_gray || !s_dbg_bin) {
        ESP_LOGW(TAG, "调试画面缓冲分配失败，网页将没有图像");
    }

    motor_init();
    ultrasonic_init();
    encoder_init();
    stop_motors();
    lcd_init_and_ui();
    wifi_debug_start();

    if (!camera_start()) {
        ESP_LOGE(TAG, "摄像头初始化失败");
        return;
    }

    xTaskCreatePinnedToCore(gui_task, "gui_task", 6144, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(line_task, "line_follow", 16384, NULL, 6, NULL, 0);
}
