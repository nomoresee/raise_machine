#include "headfile.h"

#define MOTOR_ANGLE_DEFAULT_PMAX      12.5f
#define MOTOR_ANGLE_RAW_EPS           0.0005f

typedef struct
{
    motor_num motor_index;
    uint8_t used;
    uint8_t initialized;
    float pos_ratio;
    float last_raw_pos;
    float actual_pos;
    uint32_t last_update_ms;
    uint32_t last_feedback_sequence;
} motor_angle_state_t;

static motor_angle_state_t motor_angle_state[MOTOR_ANGLE_MAX_TRACKED];

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

static motor_angle_state_t *motor_angle_find_state(motor_num motor_index)
{
    uint8_t i;

    for (i = 0U; i < MOTOR_ANGLE_MAX_TRACKED; i++)
    {
        if ((motor_angle_state[i].used != 0U) &&
            (motor_angle_state[i].motor_index == motor_index))
        {
            return &motor_angle_state[i];
        }
    }

    return NULL;
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
    uint32_t now_ms;
    float half_range;
    float full_range;
    float half_full_range;
    float raw_pos;
    float delta;
    uint32_t feedback_sequence;

    if ((state == NULL) || (state->used == 0U))
    {
        return;
    }

    motor_ptr = &motor[state->motor_index];
    feedback_sequence = dm_motor_feedback_sequence(state->motor_index);
    if ((feedback_sequence == 0U) ||
        (feedback_sequence == state->last_feedback_sequence))
    {
        return;
    }

    half_range = motor_angle_get_half_range(motor_ptr);
    full_range = 2.0f * half_range;
    half_full_range = 0.5f * full_range;
    raw_pos = motor_ptr->para.pos;
    now_ms = HAL_GetTick();

    if (state->initialized == 0U)
    {
        state->initialized = 1U;
        state->last_raw_pos = raw_pos;
        state->actual_pos = raw_pos * state->pos_ratio;
        state->last_update_ms = now_ms;
        state->last_feedback_sequence = feedback_sequence;
        return;
    }

    delta = raw_pos - state->last_raw_pos;
    if (motor_angle_absf(delta) <= MOTOR_ANGLE_RAW_EPS)
    {
        state->last_raw_pos = raw_pos;
        state->last_update_ms = now_ms;
        state->last_feedback_sequence = feedback_sequence;
        return;
    }

    /*
     * 本函数现在由每一帧有效 CAN 反馈触发。相邻反馈之间只需按位置映射
     * 周期进行一次边界补偿，不再利用主循环间隔和速度估算跨圈数。
     * 这样不会因蜂鸣等待、LCD 阻塞或错误 VMAX 把静止时间算进首帧运动。
     */
    if (delta > half_full_range)
    {
        delta -= full_range;
    }
    else if (delta < -half_full_range)
    {
        delta += full_range;
    }

    state->actual_pos += delta * state->pos_ratio;
    state->last_raw_pos = raw_pos;
    state->last_update_ms = now_ms;
    state->last_feedback_sequence = feedback_sequence;
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
void motor_angle_module_init(void)
{
    memset(motor_angle_state, 0, sizeof(motor_angle_state));
}

uint8_t motor_angle_register(motor_num motor_index)
{
    motor_angle_state_t *state;
    uint8_t i;

    state = motor_angle_find_state(motor_index);
    if (state != NULL)
    {
        return 1U;
    }

    for (i = 0U; i < MOTOR_ANGLE_MAX_TRACKED; i++)
    {
        if (motor_angle_state[i].used == 0U)
        {
            memset(&motor_angle_state[i], 0, sizeof(motor_angle_state[i]));
            motor_angle_state[i].motor_index = motor_index;
            motor_angle_state[i].pos_ratio = 1.0f;
            /* used 最后置位，避免 CAN 中断看到尚未初始化完整的状态。 */
            motor_angle_state[i].used = 1U;
            return 1U;
        }
    }

    return 0U;
}

uint8_t motor_angle_set_pos_ratio(motor_num motor_index, float pos_ratio)
{
    motor_angle_state_t *state;

    if (motor_angle_absf(pos_ratio) <= 0.001f)
    {
        return 0U;
    }

    if (motor_angle_register(motor_index) == 0U)
    {
        return 0U;
    }

    state = motor_angle_find_state(motor_index);
    if (state == NULL)
    {
        return 0U;
    }

    /*
     * 调整比例期间暂时从查找表隐藏该状态。若此时恰好到达一帧反馈，
     * 该帧留给下一帧继续展开，避免中断使用到一半更新的状态。
     */
    state->used = 0U;
    state->initialized = 0U;
    state->pos_ratio = pos_ratio;
    state->last_raw_pos = 0.0f;
    state->actual_pos = 0.0f;
    state->last_update_ms = 0U;
    state->last_feedback_sequence = 0U;
    state->used = 1U;
    return 1U;
}

void motor_angle_init(motor_num motor1_index, motor_num motor2_index)
{
    motor_angle_module_init();

    (void)motor_angle_register(motor1_index);
    (void)motor_angle_register(motor2_index);
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
    uint8_t i;

    for (i = 0U; i < MOTOR_ANGLE_MAX_TRACKED; i++)
    {
        uint8_t was_used = motor_angle_state[i].used;

        motor_angle_state[i].used = 0U;
        motor_angle_state[i].initialized = 0U;
        motor_angle_state[i].last_raw_pos = 0.0f;
        motor_angle_state[i].actual_pos = 0.0f;
        motor_angle_state[i].last_update_ms = 0U;
        motor_angle_state[i].last_feedback_sequence = 0U;
        motor_angle_state[i].used = was_used;
    }
}

void motor_angle_reset_one(motor_num motor_index)
{
    uint8_t i;

    for (i = 0U; i < MOTOR_ANGLE_MAX_TRACKED; i++)
    {
        if ((motor_angle_state[i].used != 0U) &&
            (motor_angle_state[i].motor_index == motor_index))
        {
            motor_angle_state[i].used = 0U;
            motor_angle_state[i].initialized = 0U;
            motor_angle_state[i].last_raw_pos = 0.0f;
            motor_angle_state[i].actual_pos = 0.0f;
            motor_angle_state[i].last_update_ms = 0U;
            motor_angle_state[i].last_feedback_sequence = 0U;
            motor_angle_state[i].used = 1U;
            return;
        }
    }
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
    motor_angle_update_all();
}

void motor_angle_update_all(void)
{
    uint8_t i;

    for (i = 0U; i < MOTOR_ANGLE_MAX_TRACKED; i++)
    {
        motor_angle_update_one(&motor_angle_state[i]);
    }
}

void motor_angle_feedback_update(motor_num motor_index)
{
    motor_angle_state_t *state = motor_angle_find_state(motor_index);

    if (state != NULL)
    {
        motor_angle_update_one(state);
    }
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
    motor_angle_state_t *state;

    state = motor_angle_find_state(motor_index);
    if (state != NULL)
    {
        return state->actual_pos;
    }

    return motor[motor_index].para.pos;
}

float motor_angle_to_raw_pos(motor_num motor_index, float actual_pos)
{
    motor_angle_state_t *state;

    state = motor_angle_find_state(motor_index);
    if ((state != NULL) && (motor_angle_absf(state->pos_ratio) > 0.001f))
    {
        return actual_pos / state->pos_ratio;
    }

    return actual_pos;
}
