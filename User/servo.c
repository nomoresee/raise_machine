#include "headfile.h"

static float servo1_current_angle_deg = 0.0f;
static float servo2_current_angle_deg = 0.0f;

/**
************************************************************************
* @brief:      	servo_clamp_u32(uint32_t value, uint32_t min_value, uint32_t max_value)
* @param:      	value：输入的无符号整型数值
* @param:      	min_value：最小限制值
* @param:      	max_value：最大限制值
* @retval:     	uint32_t
* @details:    	将输入值限制在给定范围内
************************************************************************
**/
static uint32_t servo_clamp_u32(uint32_t value, uint32_t min_value, uint32_t max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

/**
************************************************************************
* @brief:      	servo_absf(float value)
* @param:      	value：输入浮点数
* @retval:     	float
* @details:    	返回输入值的绝对值
************************************************************************
**/
static float servo_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

/**
************************************************************************
* @brief:      	servo_clamp_angle_range(float angle_deg, float max_angle_deg)
* @param:      	angle_deg：输入的舵机角度，单位为度
* @param:      	max_angle_deg：舵机允许的最大角度，单位为度
* @retval:     	float
* @details:    	将输入角度限制在 0~max_angle_deg 范围内
************************************************************************
**/
static float servo_clamp_angle_range(float angle_deg, float max_angle_deg)
{
    if (angle_deg < SERVO_MIN_ANGLE_DEG)
    {
        return SERVO_MIN_ANGLE_DEG;
    }

    if (angle_deg > max_angle_deg)
    {
        return max_angle_deg;
    }

    return angle_deg;
}

/**
************************************************************************
* @brief:      	servo_angle_to_compare_range(float angle_deg, float max_angle_deg)
* @param:      	angle_deg：输入的舵机角度，单位为度
* @param:      	max_angle_deg：舵机允许的最大角度，单位为度
* @retval:     	uint32_t
* @details:    	将输入角度映射为 PWM 比较值
************************************************************************
**/
static uint32_t servo_angle_to_compare_range(float angle_deg, float max_angle_deg)
{
    float angle = servo_clamp_angle_range(angle_deg, max_angle_deg);
    float ratio = (angle - SERVO_MIN_ANGLE_DEG) / (max_angle_deg - SERVO_MIN_ANGLE_DEG);
    float pulse = (float)SERVO_MIN_PULSE_US + ratio * (float)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);

    return servo_pulse_us_to_compare((uint32_t)(pulse + 0.5f));
}

/**
************************************************************************
* @brief:      	servo_step_towards(float current_angle, float target_angle, float step_deg)
* @param:      	current_angle：当前角度
* @param:      	target_angle：目标角度
* @param:      	step_deg：单次步进角度
* @retval:     	float
* @details:    	按指定步进角度逐步逼近目标角度，不会越过目标点
************************************************************************
**/
static float servo_step_towards(float current_angle, float target_angle, float step_deg)
{
    float delta = target_angle - current_angle;

    if (step_deg <= 0.0f)
    {
        return target_angle;
    }

    if (servo_absf(delta) <= step_deg)
    {
        return target_angle;
    }

    return (delta > 0.0f) ? (current_angle + step_deg) : (current_angle - step_deg);
}

/**
************************************************************************
* @brief:      	servo_clamp_angle(float angle_deg)
* @param:      	angle_deg：输入的舵机1角度，单位为度
* @retval:     	float
* @details:    	将舵机1角度限制在 0~270 度范围内
************************************************************************
**/
float servo_clamp_angle(float angle_deg)
{
    return servo_clamp_angle_range(angle_deg, SERVO1_MAX_ANGLE_DEG);
}

/**
************************************************************************
* @brief:      	servo_pulse_us_to_compare(uint32_t pulse_us)
* @param:      	pulse_us：输入的 PWM 脉宽，单位为 us
* @retval:     	uint32_t
* @details:    	将脉宽限制到舵机允许的范围内
************************************************************************
**/
uint32_t servo_pulse_us_to_compare(uint32_t pulse_us)
{
    return servo_clamp_u32(pulse_us, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
}

/**
************************************************************************
* @brief:      	servo_compare_to_pulse_us(uint32_t compare)
* @param:      	compare：定时器比较值
* @retval:     	uint32_t
* @details:    	将比较值转换为限制后的脉宽值
************************************************************************
**/
uint32_t servo_compare_to_pulse_us(uint32_t compare)
{
    return servo_clamp_u32(compare, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
}

/**
************************************************************************
* @brief:      	servo_angle_to_compare(float angle_deg)
* @param:      	angle_deg：输入的舵机1角度，单位为度
* @retval:     	uint32_t
* @details:    	将舵机1的 0~270 度角度映射为比较值
************************************************************************
**/
uint32_t servo_angle_to_compare(float angle_deg)
{
    return servo_angle_to_compare_range(angle_deg, SERVO1_MAX_ANGLE_DEG);
}

/**
************************************************************************
* @brief:      	servo_compare_to_angle(uint32_t compare)
* @param:      	compare：定时器比较值
* @retval:     	float
* @details:    	将比较值反算为舵机1的 0~270 度角度
************************************************************************
**/
float servo_compare_to_angle(uint32_t compare)
{
    float pulse = (float)servo_compare_to_pulse_us(compare);
    float ratio = (pulse - (float)SERVO_MIN_PULSE_US) / (float)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);

    return SERVO_MIN_ANGLE_DEG + ratio * (SERVO1_MAX_ANGLE_DEG - SERVO_MIN_ANGLE_DEG);
}

/**
************************************************************************
* @brief:      	servo_set_pulse_us(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t pulse_us)
* @param:      	htim：定时器句柄
* @param:      	channel：PWM 通道
* @param:      	pulse_us：PWM 脉宽，单位为 us
* @retval:     	void
* @details:    	向指定定时器通道写入脉宽值
************************************************************************
**/
void servo_set_pulse_us(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t pulse_us)
{
    if (htim == NULL)
    {
        return;
    }

    __HAL_TIM_SET_COMPARE(htim, channel, servo_pulse_us_to_compare(pulse_us));
}

/**
************************************************************************
* @brief:      	servo_write_angle(TIM_HandleTypeDef *htim, uint32_t channel, float angle_deg)
* @param:      	htim：定时器句柄
* @param:      	channel：PWM 通道
* @param:      	angle_deg：输入的舵机1角度，单位为度
* @retval:     	float
* @details:    	将舵机1角度映射为比较值并写入定时器
************************************************************************
**/
float servo_write_angle(TIM_HandleTypeDef *htim, uint32_t channel, float angle_deg)
{
    float angle = servo_clamp_angle(angle_deg);

    if (htim != NULL)
    {
        __HAL_TIM_SET_COMPARE(htim, channel, servo_angle_to_compare(angle));
    }

    return angle;
}

/**
************************************************************************
* @brief:      	servo_set_angle(TIM_HandleTypeDef *htim, uint32_t channel, float angle_deg)
* @param:      	htim：定时器句柄
* @param:      	channel：PWM 通道
* @param:      	angle_deg：输入的舵机1角度，单位为度
* @retval:     	void
* @details:    	servo_write_angle 的 void 封装
************************************************************************
**/
void servo_set_angle(TIM_HandleTypeDef *htim, uint32_t channel, float angle_deg)
{
    (void)servo_write_angle(htim, channel, angle_deg);
}

/**
************************************************************************
* @brief:      	servo1_set_angle(float angle_deg)
* @param:      	angle_deg：输入的舵机1角度，单位为度
* @retval:     	void
* @details:    	控制舵机1转到目标角度，内部固定使用 TIM2 的 CH1 通道，范围为 0~270 度
************************************************************************
**/
void servo1_set_angle(float angle_deg)
{
    servo1_current_angle_deg = servo_clamp_angle(angle_deg);
    (void)servo_write_angle(&htim2, TIM_CHANNEL_1, servo1_current_angle_deg);
}

/**
************************************************************************
* @brief:      	servo1_set_pulse_us(uint32_t pulse_us)
* @param:      	pulse_us：PWM 脉宽，单位为 us
* @retval:     	void
* @details:    	控制舵机1输出指定脉宽，内部固定使用 TIM2 的 CH1 通道
************************************************************************
**/
void servo1_set_pulse_us(uint32_t pulse_us)
{
    servo_set_pulse_us(&htim2, TIM_CHANNEL_1, pulse_us);
}

/**
************************************************************************
* @brief:       servo1_move_gradual(float start_angle, float target_angle, float step_deg, uint32_t step_delay_ms)
* @param:       start_angle：舵机1起始角度，单位为度
* @param:       target_angle：舵机1目标角度，单位为度
* @param:       step_deg：每次更新的步进角度，单位为度
* @param:       step_delay_ms：每次步进后的延时，单位为 ms
* @retval:      void
* @details:     控制舵机1按指定步进逐渐转到目标角度，避免直接跳变造成机械冲击。
************************************************************************
**/
void servo1_move_gradual(float start_angle, float target_angle, float step_deg, uint32_t step_delay_ms)
{
    float angle = start_angle;

    if (target_angle >= start_angle)
    {
        while (angle < target_angle)
        {
            servo1_set_angle(angle);
            HAL_Delay(step_delay_ms);
            angle += step_deg;
        }
    }
    else
    {
        while (angle > target_angle)
        {
            servo1_set_angle(angle);
            HAL_Delay(step_delay_ms);
            angle -= step_deg;
        }
    }

    servo1_set_angle(target_angle);
}

/**
************************************************************************
* @brief:       servo1_grip_cycle(void)
* @retval:      void
* @details:     舵机1执行一次闭合再打开动作，开合角度、速度和保持时间由 servo.h 中的宏控制。
************************************************************************
**/
void servo1_grip_cycle(void)
{
    servo1_move_gradual(SERVO1_GRIP_OPEN_ANGLE,
                        SERVO1_GRIP_CLOSE_ANGLE,
                        SERVO1_GRIP_STEP_DEG,
                        SERVO1_GRIP_STEP_DELAY_MS);
    HAL_Delay(SERVO1_GRIP_HOLD_MS);

    servo1_move_gradual(SERVO1_GRIP_CLOSE_ANGLE,
                        SERVO1_GRIP_OPEN_ANGLE,
                        SERVO1_GRIP_STEP_DEG,
                        SERVO1_GRIP_STEP_DELAY_MS);
}

/**
************************************************************************
* @brief:      	servo2_set_angle(float angle_deg)
* @param:      	angle_deg：输入的舵机2角度，单位为度
* @retval:     	void
* @details:    	控制舵机2转到目标角度，内部固定使用 TIM1 的 CH1 通道，范围为 0~180 度
************************************************************************
**/
void servo2_set_angle(float angle_deg)
{
    servo2_current_angle_deg = servo_clamp_angle_range(angle_deg, SERVO2_MAX_ANGLE_DEG);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, servo_angle_to_compare_range(servo2_current_angle_deg, SERVO2_MAX_ANGLE_DEG));
}

/**
************************************************************************
* @brief:      	servo2_set_pulse_us(uint32_t pulse_us)
* @param:      	pulse_us：PWM 脉宽，单位为 us
* @retval:     	void
* @details:    	控制舵机2输出指定脉宽，内部固定使用 TIM1 的 CH1 通道
************************************************************************
**/
void servo2_set_pulse_us(uint32_t pulse_us)
{
    servo_set_pulse_us(&htim1, TIM_CHANNEL_1, pulse_us);
}

/**
************************************************************************
* @brief:      	servo_sync_move(float servo1_target_deg, float servo2_target_deg)
* @param:      	servo1_target_deg：舵机1目标角度，单位为度
* @param:      	servo2_target_deg：舵机2目标角度，单位为度
* @retval:     	void
* @details:    	按默认步进角度和默认步进延时，同步驱动两个舵机逐步转到目标角度
************************************************************************
**/
void servo_sync_move(float servo1_target_deg, float servo2_target_deg)
{
    servo_sync_move_custom(servo1_target_deg, servo2_target_deg,
                           SERVO1_SYNC_STEP_DEG, SERVO2_SYNC_STEP_DEG,
                           SERVO_SYNC_STEP_DELAY_MS);
}

/**
************************************************************************
* @brief:      	servo_sync_move_custom(float servo1_target_deg, float servo2_target_deg, float servo1_step_deg, float servo2_step_deg, uint32_t step_delay_ms)
* @param:      	servo1_target_deg：舵机1目标角度，单位为度
* @param:      	servo2_target_deg：舵机2目标角度，单位为度
* @param:      	servo1_step_deg：舵机1每次更新的步进角度
* @param:      	servo2_step_deg：舵机2每次更新的步进角度
* @param:      	step_delay_ms：每次步进后的延时，单位为 ms
* @retval:     	void
* @details:    	两个舵机按步进方式同步逼近目标角度，可通过两个步进角度补偿不同品牌舵机的速度差
************************************************************************
**/
void servo_sync_move_custom(float servo1_target_deg, float servo2_target_deg,
                            float servo1_step_deg, float servo2_step_deg,
                            uint32_t step_delay_ms)
{
    float servo1_target = servo_clamp_angle(servo1_target_deg);
    float servo2_target = servo_clamp_angle_range(servo2_target_deg, SERVO2_MAX_ANGLE_DEG);
    uint8_t moving = 1U;

    while (moving != 0U)
    {
        float next_servo1 = servo_step_towards(servo1_current_angle_deg, servo1_target, servo1_step_deg);
        float next_servo2 = servo_step_towards(servo2_current_angle_deg, servo2_target, servo2_step_deg);

        moving = 0U;

        if (next_servo1 != servo1_current_angle_deg)
        {
            servo1_set_angle(next_servo1);
            moving = 1U;
        }
        else
        {
            servo1_set_angle(servo1_target);
        }

        if (next_servo2 != servo2_current_angle_deg)
        {
            servo2_set_angle(next_servo2);
            moving = 1U;
        }
        else
        {
            servo2_set_angle(servo2_target);
        }

        if (moving != 0U)
        {
            HAL_Delay(step_delay_ms);
        }
    }
}
