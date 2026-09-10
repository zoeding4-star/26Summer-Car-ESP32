#include "decode.h"
#include "cam_config.h"
#include "cam_state.h"
#include "util.h"

#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "jpeg_decoder.h"
#include "tjpgd.h"
#include <string.h>

static const char *TAG = "AVOID_CATCH";

uint8_t luma_at(int x, int y)
{
    return s_gray[map_y(y, s_img_h) * s_img_w + map_x(x, s_img_w)];
}

typedef struct {
    const uint8_t *data;
    size_t len;
    size_t pos;
    uint8_t *gray;
    int stride;
} tjd_io_t;

static size_t tjd_in(JDEC *jd, uint8_t *buff, size_t nbyte)
{
    tjd_io_t *io = (tjd_io_t *)jd->device;
    if (io->pos >= io->len) {
        return 0;
    }
    if (io->pos + nbyte > io->len) {
        nbyte = io->len - io->pos;
    }
    if (buff) {
        memcpy(buff, io->data + io->pos, nbyte);
    }
    io->pos += nbyte;
    return nbyte;
}

static int tjd_out(JDEC *jd, void *bitmap, JRECT *rect)
{
    tjd_io_t *io = (tjd_io_t *)jd->device;
    const uint8_t *src = (const uint8_t *)bitmap;
    int bw = rect->right - rect->left + 1;
    for (int y = rect->top; y <= rect->bottom; y++) {
        memcpy(io->gray + (size_t)y * io->stride + rect->left, src, (size_t)bw);
        src += bw;
    }
    return 1;
}

bool decode_mjpeg(const uint8_t *jpg, int len)
{
    static uint8_t pool[4096];
    JDEC jd;
    tjd_io_t io = {
        .data = jpg,
        .len = (size_t)len,
        .pos = 0,
        .gray = s_gray,
        .stride = CAM_WIDTH,
    };

    JRESULT r = jd_prepare(&jd, tjd_in, pool, sizeof(pool), &io);
    if (r != JDR_OK) {
        ESP_LOGW(TAG, "jd_prepare=%d", (int)r);
        return false;
    }
    if (jd.width == 0 || jd.height == 0 || jd.width > CAM_WIDTH || jd.height > CAM_HEIGHT) {
        return false;
    }

    int out_w = jd.width >> JPEG_DSCALE;
    int out_h = jd.height >> JPEG_DSCALE;
    if (out_w < 16 || out_h < 16) {
        return false;
    }
    s_img_w = out_w;
    s_img_h = out_h;
    io.stride = s_img_w;

    r = jd_decomp(&jd, tjd_out, JPEG_DSCALE);
    if (r != JDR_OK) {
        ESP_LOGW(TAG, "jd_decomp=%d", (int)r);
        return false;
    }
    return true;
}
void rgb_at(int x, int y, uint8_t *r, uint8_t *g, uint8_t *b)
{
    int mx = map_x(x, s_img_w);
    int my = map_y(y, s_img_h);
    const uint8_t *p = s_rgb + ((size_t)my * s_img_w + mx) * 3;
    *r = p[0];
    *g = p[1];
    *b = p[2];
}

/* UVC MJPEG 常省略 DHT；必须用软件 esp_jpeg + 默认 Huffman，不能用 ROM tjpgd */
static bool jpeg_clip_soi_eoi(const uint8_t *jpg, int len, int *off, int *out_len)
{
    int start = -1;
    int end = -1;
    for (int i = 0; i < len - 1; i++) {
        if (jpg[i] == 0xFF && jpg[i + 1] == 0xD8) {
            start = i;
            break;
        }
    }
    if (start < 0) {
        return false;
    }
    for (int i = len - 2; i > start; i--) {
        if (jpg[i] == 0xFF && jpg[i + 1] == 0xD9) {
            end = i + 2;
            break;
        }
    }
    *off = start;
    /* 个别帧缺 EOI：仍尝试解码剩余数据 */
    *out_len = (end > start) ? (end - start) : (len - start);
    return *out_len >= 128;
}

bool decode_mjpeg_rgb(const uint8_t *jpg, int len)
{
    int off = 0;
    int jlen = 0;
    if (!jpeg_clip_soi_eoi(jpg, len, &off, &jlen)) {
        static int n;
        if ((n++ % 30) == 0) {
            ESP_LOGW(TAG, "JPEG 无 SOI len=%d head=%02X %02X %02X %02X",
                     len,
                     len > 0 ? jpg[0] : 0, len > 1 ? jpg[1] : 0,
                     len > 2 ? jpg[2] : 0, len > 3 ? jpg[3] : 0);
        }
        return false;
    }

    if (!s_rgb || !s_jpeg_work) {
        return false;
    }

    esp_jpeg_image_cfg_t cfg = {
        .indata = (uint8_t *)(jpg + off),
        .indata_size = (uint32_t)jlen,
        .outbuf = s_rgb,
        .outbuf_size = (uint32_t)(CAM_WIDTH * CAM_HEIGHT * 3),
        .out_format = JPEG_IMAGE_FORMAT_RGB888,
        .out_scale = JPEG_IMAGE_SCALE_1_4,  /* 对应原 JPEG_DSCALE=2 */
        .flags = {
            .swap_color_bytes = 0,
        },
        .advanced = {
            .working_buffer = s_jpeg_work,
            .working_buffer_size = JPEG_RGB_WORK_SZ,
        },
    };
    esp_jpeg_image_output_t out = {0};

    (void)esp_task_wdt_reset();
    vTaskDelay(1);
    esp_err_t err = esp_jpeg_decode(&cfg, &out);
    (void)esp_task_wdt_reset();
    vTaskDelay(1);
    if (err != ESP_OK) {
        static int n;
        if ((n++ % 20) == 0) {
            ESP_LOGW(TAG, "esp_jpeg_decode=%s len=%d soi_off=%d",
                     esp_err_to_name(err), jlen, off);
        }
        return false;
    }
    if (out.width < 16 || out.height < 16 ||
        out.width > CAM_WIDTH || out.height > CAM_HEIGHT) {
        ESP_LOGW(TAG, "解码尺寸异常 %ux%u", out.width, out.height);
        return false;
    }
    s_img_w = (int)out.width;
    s_img_h = (int)out.height;
    return true;
}
