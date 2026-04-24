#include "headfile.h"

#define MOTOR_ANGLE_DEFAULT_PMAX      12.5f

typedef struct
{
    motor_num motor_index;
    uint8_t initialized;
    float last_raw_pos;
    float actual_pos;
} motor_angle_state_t;

static motor_angle_state_t motor_angle_state[2];

/**
***********************************************************************
* @brief:      motor_angle_absf(float value)
* @param:      value：输入浮点数
* @retval:     float
* @details:    返回输入值的绝对值。
***********************************************************************
**/
static float motor_angle_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

/**
***********************************************************************
* @brief:      motor_angle_get_half_range(const motor_t *motor_ptr)
* @param:      motor_ptr：指向 motor_t 结构体的指针
* @retval:     float
* @details:    获取电机位置映射半量程。若电机参数未配置，则使用默认 12.5。
***********************************************************************
**/
static float motor_angle_get_half_range(const motor_t *motor_ptr)
{
    if ((motor_ptr != NULL) && (motor_angle_absf(motor_ptr->tmp.PMAX) > 0.001f))
    {
        return motor_angle_absf(motor_ptr->tmp.PMAX);
    }

    return MOTOR_ANGLE_DEFAULT_PMAX;
}

/**
***********************************************************************
* @brief:      motor_angle_update_one(motor_angle_state_t *state)
* @param:      state：单台电机的多圈角度状态结构体指针
* @retval:     void
* @details:    将 -PMAX 到 +PMAX 的映射位置展开成连续位置。跨过边界时自动补偿一个映射周期。
***********************************************************************
**/
static void motor_angle_update_one(motor_angle_state_t *state)
{
    motor_t *motor_ptr;
    float half_range;
    float full_range;
    float half_full_range;
    float raw_pos;
    float delta;

    if (state == NULL)
    {
        return;
    }

    motor_ptr = &motor[state->motor_index];
    half_range = motor_angle_get_half_range(motor_ptr);
    full_range = 2.0f * half_range;
    half_full_range = 0.5f * full_range;
    raw_pos = motor_ptr->para.pos;

    if (state->initialized == 0U)
    {
        state->initialized = 1U;
        state->last_raw_pos = raw_pos;
        state->actual_pos = raw_pos;
        return;
    }

    delta = raw_pos - state->last_raw_pos;

    if (delta > half_full_range)
    {
        delta -= full_range;
    }
    else if (delta < -half_full_range)
    {
        delta += full_range;
    }

    state->actual_pos += delta;
    state->last_raw_pos = raw_pos;
}

/**
***********************************************************************
* @brief:      motor_angle_init(motor_num motor1_index, motor_num motor2_index)
* @param:      motor1_index：第一台电机在 motor 数组中的索引
* @param:      motor2_index：第二台电机在 motor 数组中的索引
* @retval:     void
* @details:    初始化两台电机的多圈实际输出轴角度计算模块。
***********************************************************************
**/
void motor_angle_init(motor_num motor1_index, motor_num motor2_index)
{
    memset(motor_angle_state, 0, sizeof(motor_angle_state));

    motor_angle_state[0].motor_index = motor1_index;
    motor_angle_state[1].motor_index = motor2_index;
}

/**
***********************************************************************
* @brief:      motor_angle_reset(void)
* @retval:     void
* @details:    清零多圈角度累计状态，下次更新时会重新以当前反馈位置作为起点。
***********************************************************************
**/
void motor_angle_reset(void)
{
    motor_angle_state[0].initialized = 0U;
    motor_angle_state[0].last_raw_pos = 0.0f;
    motor_angle_state[0].actual_pos = 0.0f;

    motor_angle_state[1].initialized = 0U;
    motor_angle_state[1].last_raw_pos = 0.0f;
    motor_angle_state[1].actual_pos = 0.0f;
}

/**
***********************************************************************
* @brief:      motor_angle_update(void)
* @retval:     void
* @details:    根据电机当前映射位置反馈更新连续实际转动角度。
***********************************************************************
**/
void motor_angle_update(void)
{
    motor_angle_update_one(&motor_angle_state[0]);
    motor_angle_update_one(&motor_angle_state[1]);
}

/**
***********************************************************************
* @brief:      motor_angle_get(motor_num motor_index)
* @param:      motor_index：电机在 motor 数组中的索引
* @retval:     float
* @details:    获取指定电机展开后的连续实际输出轴角度，单位与电机位置反馈单位一致。
***********************************************************************
**/
float motor_angle_get(motor_num motor_index)
{
    if (motor_angle_state[0].motor_index == motor_index)
    {
        return motor_angle_state[0].actual_pos;
    }

    if (motor_angle_state[1].motor_index == motor_index)
    {
        return motor_angle_state[1].actual_pos;
    }

    return motor[motor_index].para.pos;
}
