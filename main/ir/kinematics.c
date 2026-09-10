#include "kinematics.h"
#include "motor.h"
#include "ir_config.h"

#include <math.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

IrMotorSpeed ir_inverse_kinematics(const IrVelocity *vel)
{
    IrMotorSpeed wheels;
    float L = IR_WHEEL_DISTANCE;

    float raw_D = -IR_SIN_60 * vel->vx + IR_COS_60 * vel->vy + L * vel->omega;
    float raw_A =  IR_SIN_60 * vel->vx + IR_COS_60 * vel->vy - L * vel->omega;
    float raw_B =  vel->vx + L * vel->omega;

    bool is_strafe  = (fabsf(vel->vx) > 0.001f) && (fabsf(vel->vy) < 0.001f) && (fabsf(vel->omega) < 0.001f);
    bool is_forward = (fabsf(vel->vx) < 0.001f) && (fabsf(vel->vy) > 0.001f) && (fabsf(vel->omega) < 0.001f);
    bool is_rotate  = (fabsf(vel->vx) < 0.001f) && (fabsf(vel->vy) < 0.001f) && (fabsf(vel->omega) > 0.001f);

    if (is_strafe) {
        wheels.D = raw_D * IR_STRAFE_SCALE_D;
        wheels.A = raw_A * IR_STRAFE_SCALE_A;
        wheels.B = raw_B * IR_STRAFE_SCALE_B;
    } else if (is_forward) {
        wheels.D = raw_D * IR_FWD_SCALE_D;
        wheels.A = raw_A * IR_FWD_SCALE_A;
        wheels.B = raw_B * IR_FWD_SCALE_B;
    } else if (is_rotate) {
        wheels.D = raw_D * IR_ROT_SCALE_D;
        wheels.A = raw_A * IR_ROT_SCALE_A;
        wheels.B = raw_B * IR_ROT_SCALE_B;
    } else {
        wheels.D = raw_D; wheels.A = raw_A; wheels.B = raw_B;
    }
    return wheels;
}

void ir_execute_pulse(IrVelocity vel, int duration_ms)
{
    IrMotorSpeed speed = ir_inverse_kinematics(&vel);
    ir_set_all_motors(&speed);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
}
