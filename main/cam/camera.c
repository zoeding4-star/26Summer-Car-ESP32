#include "camera.h"
#include "cam_config.h"
#include "cam_state.h"
#include "util.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "usb_stream.h"
#include <string.h>

static const char *TAG = "AVOID_CATCH";

static void camera_frame_cb(uvc_frame_t *frame, void *ptr)
{
    (void)ptr;
    if (!frame || !frame->data || frame->data_bytes == 0 || !s_frame_mutex) {
        return;
    }
    if (frame->data_bytes > JPEG_XFER_SIZE) {
        return;
    }
    if (xSemaphoreTake(s_frame_mutex, 0) == pdTRUE) {
        memcpy(s_jpeg, frame->data, frame->data_bytes);
        s_jpeg_len = frame->data_bytes;
        xSemaphoreGive(s_frame_mutex);
        xSemaphoreGive(s_frame_ready);
    }
}

bool camera_start(void)
{
    uint8_t *xfer_a = (uint8_t *)psram_alloc(JPEG_XFER_SIZE);
    uint8_t *xfer_b = (uint8_t *)psram_alloc(JPEG_XFER_SIZE);
    uint8_t *frame_buf = (uint8_t *)psram_alloc(JPEG_XFER_SIZE);
    s_jpeg = (uint8_t *)psram_alloc(JPEG_XFER_SIZE);
    s_gray = (uint8_t *)psram_alloc(CAM_WIDTH * CAM_HEIGHT);
    s_bin = (uint8_t *)psram_alloc(CAM_WIDTH * CAM_HEIGHT);
    s_rgb = (uint8_t *)psram_alloc(CAM_WIDTH * CAM_HEIGHT * 3);
    s_mask = (uint8_t *)psram_alloc(CAM_WIDTH * CAM_HEIGHT);
    s_visited = (uint8_t *)psram_alloc(CAM_WIDTH * CAM_HEIGHT);
    s_jpeg_work = (uint8_t *)psram_alloc(JPEG_RGB_WORK_SZ);

    if (!xfer_a || !xfer_b || !frame_buf || !s_jpeg || !s_gray || !s_bin ||
        !s_rgb || !s_mask || !s_visited || !s_jpeg_work) {
        ESP_LOGE(TAG, "PSRAM 分配失败");
        return false;
    }

    uvc_config_t cfg = {
        .frame_width = CAM_WIDTH,
        .frame_height = CAM_HEIGHT,
        .frame_index = 4,
        .frame_interval = FPS2INTERVAL(CAM_FPS),
        .xfer_buffer_size = JPEG_XFER_SIZE,
        .xfer_buffer_a = xfer_a,
        .xfer_buffer_b = xfer_b,
        .frame_buffer_size = JPEG_XFER_SIZE,
        .frame_buffer = frame_buf,
        .frame_cb = camera_frame_cb,
        .frame_cb_arg = NULL,
        .format = UVC_FORMAT_MJPEG,
    };

    ESP_LOGI(TAG, "UVC MJPEG %dx%d @ %d fps", CAM_WIDTH, CAM_HEIGHT, CAM_FPS);
    if (uvc_streaming_config(&cfg) != ESP_OK) {
        ESP_LOGE(TAG, "UVC 配置失败");
        return false;
    }
    if (usb_streaming_start() != ESP_OK) {
        ESP_LOGE(TAG, "USB 启动失败");
        return false;
    }
    return true;
}
