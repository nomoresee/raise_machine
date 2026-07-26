#include "headfile.h"

#define VOFA_DEBUG_PRINT_MS       200U
/* 单步调试需要观察 10 ms 控制环内的速度变化，独立提高到 20 ms。 */
#define VOFA_DEBUG_STEP_PRINT_MS   20U

#define VOFA_DEBUG_MOTOR1_DIR   1.0f
#define VOFA_DEBUG_MOTOR2_DIR  -1.0f
#define VOFA_DEBUG_MOTOR3_DIR  -1.0f
#define VOFA_DEBUG_MOTOR4_DIR   1.0f
#define VOFA_DEBUG_MOTOR5_DIR  -1.0f
#define VOFA_DEBUG_MOTOR6_DIR  -1.0f

typedef struct
{
    motor_num motor1_index;
    motor_num motor2_index;
    motor_num motor3_index;
    motor_num motor4_index;
    motor_num motor5_index;
    motor_num motor6_index;
    uint32_t print_tick;
    uint8_t initialized;
    vofa_debug_snapshot_t snapshot;
} vofa_debug_t;

static vofa_debug_t vofa_debug;
static uint32_t vofa_debug_step_print_tick;

static float vofa_debug_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

void vofa_debug_init(motor_num motor1_index,
                     motor_num motor2_index,
                     motor_num motor3_index,
                     motor_num motor4_index,
                     motor_num motor5_index,
                     motor_num motor6_index)
{
    memset(&vofa_debug, 0, sizeof(vofa_debug));
    vofa_debug.motor1_index = motor1_index;
    vofa_debug.motor2_index = motor2_index;
    vofa_debug.motor3_index = motor3_index;
    vofa_debug.motor4_index = motor4_index;
    vofa_debug.motor5_index = motor5_index;
    vofa_debug.motor6_index = motor6_index;
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
    float motor5_pos;
    float motor6_pos;
    float motor1_show;
    float motor2_show;
    float target_x;
    float target_y;
    float target_z;
    float target_upper_y;
    float target_lower_y;
    float target_error;
    float motor1_para_vel;
    float motor2_para_vel;
    float motor3_para_vel;
    float motor4_para_vel;
    float motor5_para_vel;
    float motor6_para_vel;

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
    motor5_pos = upper_hopper_y_ctrl_get_current_pos();
    motor6_pos = lower_hopper_y_ctrl_get_current_pos();
    motor1_show = VOFA_DEBUG_MOTOR1_DIR * motor1_raw;
    motor2_show = VOFA_DEBUG_MOTOR2_DIR * motor2_raw;
    target_error = vofa_debug_absf(vofa_debug_absf(motor1_show) - vofa_debug_absf(motor2_show));
    motor1_para_vel = VOFA_DEBUG_MOTOR1_DIR * motor[vofa_debug.motor1_index].para.vel * 3.0f;
    motor2_para_vel = VOFA_DEBUG_MOTOR2_DIR * motor[vofa_debug.motor2_index].para.vel * 3.0f;
    motor3_para_vel = VOFA_DEBUG_MOTOR3_DIR * motor[vofa_debug.motor3_index].para.vel * 3.0f;
    motor4_para_vel = VOFA_DEBUG_MOTOR4_DIR * motor[vofa_debug.motor4_index].para.vel * 3.0f;
    motor5_para_vel = VOFA_DEBUG_MOTOR5_DIR * motor[vofa_debug.motor5_index].para.vel * 3.0f;
    motor6_para_vel = VOFA_DEBUG_MOTOR6_DIR * motor[vofa_debug.motor6_index].para.vel * 3.0f;
    crane_route_get_current_pose_target(&target_x, &target_y, &target_z);
    crane_route_get_all_y_targets(NULL, &target_upper_y, &target_lower_y);

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
    vofa_debug.snapshot.target_upper_y = target_upper_y;
    vofa_debug.snapshot.target_lower_y = target_lower_y;
    vofa_debug.snapshot.motor5_pos = motor5_pos;
    vofa_debug.snapshot.motor6_pos = motor6_pos;
    vofa_debug.snapshot.motor5_vel = motor5_para_vel;
    vofa_debug.snapshot.motor6_vel = motor6_para_vel;
    vofa_debug.snapshot.motor5_feedback_age_ms = dm_motor_feedback_age_ms(vofa_debug.motor5_index);
    vofa_debug.snapshot.motor6_feedback_age_ms = dm_motor_feedback_age_ms(vofa_debug.motor6_index);
    vofa_debug.snapshot.route_state = (uint16_t)crane_route_get_state();
    vofa_debug.snapshot.route_fault = (uint16_t)crane_route_get_fault();
    vofa_debug.snapshot.valid = 1U;

    /*
     * VOFA FireWater 纯数字 CSV，共 12 个通道：
     * M1实际,M1目标,M2实际,M2目标,...,M6实际,M6目标。
     * Motor1/Motor2 共用底盘 X 目标；其余电机使用各自机构坐标系目标。
     */
    printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
           "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
           motor1_show,
           target_x,
           motor2_show,
           target_x,
           motor3_pos,
           target_y,
           motor4_pos,
           target_z,
           motor5_pos,
           target_upper_y,
           motor6_pos,
           target_lower_y);
}

void vofa_debug_step_process(uint8_t motor_select,
                             float actual_pos,
                             float target_pos,
                             float feedback_motor_vel,
                             float command_motor_vel)
{
    uint32_t now_ms = HAL_GetTick();

    /* motor_select 已由调试模块校验；保留该参数便于调用处明确当前输出对象。 */
    (void)motor_select;

    if ((now_ms - vofa_debug_step_print_tick) < VOFA_DEBUG_STEP_PRINT_MS)
    {
        return;
    }
    vofa_debug_step_print_tick = now_ms;

    /* VOFA ASCII：实际位置、目标位置、反馈速度、下发速度上限。 */
    printf("%.3f,%.3f,%.3f,%.3f\r\n",
           (double)actual_pos,
           (double)target_pos,
           (double)feedback_motor_vel,
           (double)command_motor_vel);
}

void vofa_debug_upper_hopper_process(float target_pos, float actual_pos)
{
    uint32_t now_ms = HAL_GetTick();

    if ((now_ms - vofa_debug_step_print_tick) < VOFA_DEBUG_STEP_PRINT_MS)
    {
        return;
    }
    vofa_debug_step_print_tick = now_ms;

    printf("%.3f,%.3f\r\n", (double)target_pos, (double)actual_pos);
}

void vofa_debug_lower_hopper_process(float target_pos, float actual_pos)
{
    uint32_t now_ms = HAL_GetTick();

    if ((now_ms - vofa_debug_step_print_tick) < VOFA_DEBUG_STEP_PRINT_MS)
    {
        return;
    }
    vofa_debug_step_print_tick = now_ms;

    printf("%.3f,%.3f\r\n", (double)target_pos, (double)actual_pos);
}

void vofa_debug_beam_process(float target_pos, float actual_pos)
{
    uint32_t now_ms = HAL_GetTick();

    if ((now_ms - vofa_debug_step_print_tick) < VOFA_DEBUG_STEP_PRINT_MS)
    {
        return;
    }
    vofa_debug_step_print_tick = now_ms;

    printf("%.3f,%.3f\r\n", (double)target_pos, (double)actual_pos);
}

void vofa_debug_lift_process(float target_pos, float actual_pos)
{
    uint32_t now_ms = HAL_GetTick();

    if ((now_ms - vofa_debug_step_print_tick) < VOFA_DEBUG_STEP_PRINT_MS)
    {
        return;
    }
    vofa_debug_step_print_tick = now_ms;

    printf("%.3f,%.3f\r\n", (double)target_pos, (double)actual_pos);
}

void vofa_debug_chassis_process(float motor1_actual_pos,
                                float motor1_target_pos,
                                float motor1_feedback_vel,
                                float motor1_command_vel,
                                float motor2_actual_pos,
                                float motor2_target_pos,
                                float motor2_feedback_vel,
                                float motor2_command_vel)
{
    uint32_t now_ms = HAL_GetTick();

    if ((now_ms - vofa_debug_step_print_tick) < VOFA_DEBUG_STEP_PRINT_MS)
    {
        return;
    }
    vofa_debug_step_print_tick = now_ms;

    (void)motor1_feedback_vel;
    (void)motor1_command_vel;
    (void)motor2_feedback_vel;
    (void)motor2_command_vel;

    printf("%.3f,%.3f,%.3f,%.3f\r\n",
           (double)motor1_actual_pos,
           (double)motor1_target_pos,
           (double)motor2_actual_pos,
           (double)motor2_target_pos);
}

void vofa_debug_chassis_feedback_process(float motor1_actual_pos,
                                         float motor1_feedback_vel,
                                         float motor1_target_pos,
                                         float motor1_command_vel,
                                         float motor2_actual_pos,
                                         float motor2_feedback_vel,
                                         float motor2_target_pos,
                                         float motor2_command_vel)
{
    uint32_t now_ms = HAL_GetTick();

    if ((now_ms - vofa_debug_step_print_tick) < VOFA_DEBUG_STEP_PRINT_MS)
    {
        return;
    }
    vofa_debug_step_print_tick = now_ms;

    printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
           (double)motor1_actual_pos,
           (double)motor1_feedback_vel,
           (double)motor1_target_pos,
           (double)motor1_command_vel,
           (double)motor2_actual_pos,
           (double)motor2_feedback_vel,
           (double)motor2_target_pos,
           (double)motor2_command_vel);
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
