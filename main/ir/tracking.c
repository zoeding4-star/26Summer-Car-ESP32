#include "tracking.h"
#include "sensors.h"
#include "kinematics.h"
#include "motor.h"
#include "ir_config.h"
#include "ir_types.h"

static IrLastDirection g_last_dir = IR_LAST_DIR_LEFT;
static bool g_has_avoided = false;

void ir_tracking_set_avoided(bool avoided)
{
    g_has_avoided = avoided;
}

bool ir_tracking_has_avoided(void)
{
    return g_has_avoided;
}

bool ir_control_loop(int base_speed)
{
    IrSensorData data;
    ir_read_sensors(&data);
    int code = ir_get_sensor_code(&data);

    IrVelocity targetVel = { .vx = 0.0f, .vy = 0.0f, .omega = 0.0f };

    switch (code) {
        case 0b0110:
            targetVel.vy = (float)base_speed;
            targetVel.omega = 0;
            ir_execute_pulse(targetVel, IR_CONTROL_PERIOD);
            break;
        case 0b0100:
            g_last_dir = IR_LAST_DIR_LEFT;
            targetVel.vy = (float)base_speed;
            targetVel.omega = -IR_OMEGA_MICRO;
            ir_execute_pulse(targetVel, IR_MICRO_PULSE_MS);
            break;
        case 0b1000: case 0b1100: case 0b1110:
            g_last_dir = IR_LAST_DIR_LEFT;
            targetVel.vy = (float)base_speed;
            targetVel.omega = -IR_OMEGA_MACRO;
            ir_execute_pulse(targetVel, IR_MACRO_PULSE_MS);
            break;
        case 0b0010:
            g_last_dir = IR_LAST_DIR_RIGHT;
            targetVel.vy = (float)base_speed;
            targetVel.omega = IR_OMEGA_MICRO;
            ir_execute_pulse(targetVel, IR_MICRO_PULSE_MS);
            break;
        case 0b0001: case 0b0011: case 0b0111:
            g_last_dir = IR_LAST_DIR_RIGHT;
            targetVel.vy = (float)base_speed;
            targetVel.omega = IR_OMEGA_MACRO;
            ir_execute_pulse(targetVel, IR_MACRO_PULSE_MS);
            break;
        case 0b0000:
            targetVel.omega = (g_last_dir == IR_LAST_DIR_LEFT) ? -IR_OMEGA_SEARCH : IR_OMEGA_SEARCH;
            ir_execute_pulse(targetVel, IR_SEARCH_PERIOD);
            break;
        case 0b1111:
            if (!g_has_avoided) {
                targetVel.vy = (float)base_speed;
                ir_execute_pulse(targetVel, IR_CONTROL_PERIOD);
            } else {
                ir_stop_motors();
                return true;
            }
            break;
        default:
            targetVel.omega = (g_last_dir == IR_LAST_DIR_LEFT) ? -IR_OMEGA_SEARCH : IR_OMEGA_SEARCH;
            ir_execute_pulse(targetVel, IR_CONTROL_PERIOD);
            break;
    }
    return false;
}
