/**
 * @file main.c
 * @brief 红外循迹 + 超声波避障：任务编排入口
 */
#include "motor.h"
#include "sensors.h"
#include "encoder.h"
#include "kinematics.h"
#include "tracking.h"
#include "lcd_ui.h"
#include "ir_config.h"
#include "ir_types.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "IR_MAIN";

static void main_control_task(void *pvParameters)
{
    (void)pvParameters;

    ir_motor_init();
    ir_sensor_init();
    ir_ultrasonic_init();
    ir_encoder_init();

    ESP_LOGI(TAG, "阶段 1：循迹控制，监控障碍物...");

    while (1) {
        float distance = ir_get_ultrasonic_distance();
        if (distance > 0.0f && distance <= IR_AVOID_TRIGGER_CM) {
            ESP_LOGW(TAG, "障碍触发 (%.2f cm)！避障中...", distance);
            break;
        }
        ir_control_loop(IR_BASE_SPEED);
    }

    /* 阶段 2：平移避障 */
    IrVelocity vel_strafe_away = { .vx = (float)IR_STRAFE_SPEED, .vy = 0.0f, .omega = 0.0f };
    ir_execute_pulse(vel_strafe_away, IR_STRAFE_FORCE_MS);

    while (1) {
        ir_execute_pulse(vel_strafe_away, IR_STRAFE_POLL_MS);
        float distance = ir_get_ultrasonic_distance();
        if (distance > IR_AVOID_CLEAR_CM) {
            ir_execute_pulse(vel_strafe_away, IR_STRAFE_EXTRA_MS);
            ir_stop_motors();
            vTaskDelay(pdMS_TO_TICKS(IR_STRAFE_STOP_DELAY_MS));
            break;
        }
    }

    /* 阶段 3：直行绕过 */
    IrVelocity vel_fwd = { .vx = 0.0f, .vy = (float)IR_BASE_SPEED, .omega = 0.0f };
    ir_execute_pulse(vel_fwd, IR_AVOID_FWD_MS);
    ir_stop_motors();
    vTaskDelay(pdMS_TO_TICKS(IR_PHASE_STOP_DELAY_MS));

    /* 阶段 4：反向平移找黑线 */
    IrVelocity vel_strafe_back = {
        .vx = -(float)IR_STRAFE_SPEED * IR_STRAFE_BACK_FACTOR,
        .vy = 0.0f,
        .omega = 0.0f
    };
    while (1) {
        ir_execute_pulse(vel_strafe_back, IR_STRAFE_BACK_PULSE_MS);
        IrSensorData data;
        ir_read_sensors(&data);
        if (ir_get_sensor_code(&data) != 0b0000) {
            ir_stop_motors();
            vTaskDelay(pdMS_TO_TICKS(IR_PHASE_STOP_DELAY_MS));
            break;
        }
    }

    /* 阶段 5：循迹至终点 */
    ir_tracking_set_avoided(true);
    while (1) {
        if (ir_control_loop(IR_BASE_SPEED)) {
            ESP_LOGI(TAG, "终点到达，停机！");
            break;
        }
    }

    ir_stop_motors();
    vTaskDelete(NULL);
}

void app_main(void)
{
    ir_lcd_init_and_ui();
    xTaskCreatePinnedToCore(ir_gui_task, "gui_task", 4096, NULL, 2, NULL, 1);
    xTaskCreate(main_control_task, "main_control_task", 4096, NULL, 5, NULL);
}
