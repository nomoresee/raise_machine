#include "headfile.h"

#define VOFA_DEBUG_PRINT_MS     100U

#define VOFA_DEBUG_MOTOR1_DIR   1.0f
#define VOFA_DEBUG_MOTOR2_DIR  -1.0f
#define VOFA_DEBUG_MOTOR3_DIR   1.0f
#define VOFA_DEBUG_MOTOR4_DIR   1.0f

typedef struct
{
    motor_num motor1_index;
    motor_num motor2_index;
    motor_num motor3_index;
    motor_num motor4_index;
    uint32_t print_tick;
    uint8_t initialized;
    vofa_debug_snapshot_t snapshot;
} vofa_debug_t;

static vofa_debug_t vofa_debug;

static float vofa_debug_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

void vofa_debug_init(motor_num motor1_index,
                     motor_num motor2_index,
                     motor_num motor3_index,
                     motor_num motor4_index)
{
    memset(&vofa_debug, 0, sizeof(vofa_debug));
    vofa_debug.motor1_index = motor1_index;
    vofa_debug.motor2_index = motor2_index;
    vofa_debug.motor3_index = motor3_index;
    vofa_debug.motor4_index = motor4_index;
    vofa_debug.print_tick = HAL_GetTick();
    vofa_debug.initialized = 1U;
}

void vofa_debug_process(void)
{
    uint32_t now_ms;
    float motor1_raw;
    float motor2_raw;
    float motor3_pos;
    float motor4_pos;
    float motor1_show;
    float motor2_show;
    float target_x;
    float target_y;
    float target_z;
    float target_error;
    float motor1_para_vel;
    float motor2_para_vel;
    float motor3_para_vel;
    float motor4_para_vel;

    if (vofa_debug.initialized == 0U)
    {
        return;
    }

    now_ms = HAL_GetTick();
    if ((now_ms - vofa_debug.print_tick) < VOFA_DEBUG_PRINT_MS)
    {
        return;
    }
    vofa_debug.print_tick = now_ms;

    motor1_raw = motor_angle_get(vofa_debug.motor1_index);
    motor2_raw = motor_angle_get(vofa_debug.motor2_index);
    motor3_pos = beam_ctrl_get_current_pos();
    motor4_pos = lift_ctrl_get_current_pos();
    motor1_show = VOFA_DEBUG_MOTOR1_DIR * motor1_raw;
    motor2_show = VOFA_DEBUG_MOTOR2_DIR * motor2_raw;
    target_error = vofa_debug_absf(vofa_debug_absf(motor1_show) - vofa_debug_absf(motor2_show));
    motor1_para_vel = VOFA_DEBUG_MOTOR1_DIR * motor[vofa_debug.motor1_index].para.vel * 3.0f;
    motor2_para_vel = VOFA_DEBUG_MOTOR2_DIR * motor[vofa_debug.motor2_index].para.vel * 3.0f;
    motor3_para_vel = VOFA_DEBUG_MOTOR3_DIR * motor[vofa_debug.motor3_index].para.vel * 3.0f;
    motor4_para_vel = VOFA_DEBUG_MOTOR4_DIR * motor[vofa_debug.motor4_index].para.vel * 3.0f;
    crane_route_get_current_pose_target(&target_x, &target_y, &target_z);

    vofa_debug.snapshot.target_x = target_x;
    vofa_debug.snapshot.target_y = target_y;
    vofa_debug.snapshot.motor1_pos = motor1_show;
    vofa_debug.snapshot.motor2_pos = motor2_show;
    vofa_debug.snapshot.pos_error = target_error;
    vofa_debug.snapshot.motor3_pos = motor3_pos;
    vofa_debug.snapshot.motor1_vel = motor1_para_vel;
    vofa_debug.snapshot.motor2_vel = motor2_para_vel;
    vofa_debug.snapshot.motor3_vel = motor3_para_vel;
    vofa_debug.snapshot.motor4_pos = motor4_pos;
    vofa_debug.snapshot.motor4_vel = motor4_para_vel;
    vofa_debug.snapshot.valid = 1U;

    printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
           target_x,
           motor1_show,
           motor2_show,
           target_error,
           motor1_para_vel,
           motor2_para_vel,
           target_y,
           motor3_pos,
           motor3_para_vel,
           target_z,
           motor4_pos,
           motor4_para_vel);
}

uint8_t vofa_debug_get_snapshot(vofa_debug_snapshot_t *out)
{
    if ((out == NULL) || (vofa_debug.snapshot.valid == 0U))
    {
        return 0U;
    }

    *out = vofa_debug.snapshot;
    return 1U;
}
