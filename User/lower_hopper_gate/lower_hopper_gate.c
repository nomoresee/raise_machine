#include "lower_hopper_gate.h"

#include "main.h"
#include "servo.h"
#include <string.h>

typedef struct
{
    float current_angle_deg;
    float target_angle_deg;
    uint32_t command_tick;
    uint32_t settle_ms;
    uint8_t initialized;
    uint8_t busy;
} lower_hopper_gate_t;

static lower_hopper_gate_t lower_hopper_gate;

static float lower_hopper_gate_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static uint8_t lower_hopper_gate_command_available(void)
{
    return ((LOWER_HOPPER_GATE_HW_READY != 0U) ||
            (LOWER_HOPPER_GATE_SIMULATION_ENABLE != 0U)) ? 1U : 0U;
}

static uint32_t lower_hopper_gate_estimate_settle_ms(float from_angle_deg,
                                                      float to_angle_deg)
{
    float delta = lower_hopper_gate_absf(to_angle_deg - from_angle_deg);
    float speed = LOWER_HOPPER_GATE_SPEED_DEG_PER_SEC;
    float settle_ms;

    if (speed <= 1.0f)
    {
        speed = 1.0f;
    }

    settle_ms = (delta * 1000.0f) / speed;
    return (uint32_t)(settle_ms + 0.5f) + LOWER_HOPPER_GATE_SETTLE_MARGIN_MS;
}

/*
 * 下斗门唯一的硬件输出入口。
 * 函数形式与现有 servo3_set_angle(float) 保持一致，但第一版严禁复用
 * TIM1_CH3 等已占用通道，因此默认只消费参数、不写 PWM。
 *
 * 确定独立 PWM 后，只需在此处写入下斗门专用定时器与通道。
 */
static void lower_hopper_gate_hw_write_angle(float angle_deg)
{
    /* PA2 -> TIM2_CH3 */
    servo_set_angle(&htim2, TIM_CHANNEL_3, angle_deg);
}

static void lower_hopper_gate_set_target(float target_angle_deg)
{
    if (lower_hopper_gate.initialized == 0U)
    {
        lower_hopper_gate_init();
    }

    lower_hopper_gate_process();

    if (lower_hopper_gate_command_available() == 0U)
    {
        return;
    }

    if (lower_hopper_gate_absf(target_angle_deg -
                               lower_hopper_gate.target_angle_deg) <=
        LOWER_HOPPER_GATE_ANGLE_TOL_DEG)
    {
        return;
    }

    lower_hopper_gate.settle_ms =
        lower_hopper_gate_estimate_settle_ms(lower_hopper_gate.current_angle_deg,
                                              target_angle_deg);
    lower_hopper_gate.target_angle_deg = target_angle_deg;
    lower_hopper_gate.command_tick = HAL_GetTick();
    lower_hopper_gate.busy = 1U;

    lower_hopper_gate_hw_write_angle(target_angle_deg);
}

void lower_hopper_gate_init(void)
{
    memset(&lower_hopper_gate, 0, sizeof(lower_hopper_gate));
    lower_hopper_gate.current_angle_deg = LOWER_HOPPER_GATE_CLOSE_ANGLE_DEG;
    lower_hopper_gate.target_angle_deg = LOWER_HOPPER_GATE_CLOSE_ANGLE_DEG;
    lower_hopper_gate.command_tick = HAL_GetTick();
    lower_hopper_gate.initialized = 1U;
    lower_hopper_gate.busy = 0U;

    lower_hopper_gate_hw_write_angle(LOWER_HOPPER_GATE_CLOSE_ANGLE_DEG);
}

void lower_hopper_gate_open(void)
{
    lower_hopper_gate_set_target(LOWER_HOPPER_GATE_OPEN_ANGLE_DEG);
}

void lower_hopper_gate_close(void)
{
    lower_hopper_gate_set_target(LOWER_HOPPER_GATE_CLOSE_ANGLE_DEG);
}

void lower_hopper_gate_process(void)
{
    if ((lower_hopper_gate.initialized == 0U) ||
        (lower_hopper_gate.busy == 0U))
    {
        return;
    }

    if ((HAL_GetTick() - lower_hopper_gate.command_tick) >=
        lower_hopper_gate.settle_ms)
    {
        lower_hopper_gate.current_angle_deg = lower_hopper_gate.target_angle_deg;
        lower_hopper_gate.busy = 0U;
    }
}

uint8_t lower_hopper_gate_is_busy(void)
{
    lower_hopper_gate_process();
    return lower_hopper_gate.busy;
}

uint8_t lower_hopper_gate_is_open(void)
{
    lower_hopper_gate_process();
    return ((lower_hopper_gate.initialized != 0U) &&
            (lower_hopper_gate_command_available() != 0U) &&
            (lower_hopper_gate.busy == 0U) &&
            (lower_hopper_gate_absf(lower_hopper_gate.current_angle_deg -
                                    LOWER_HOPPER_GATE_OPEN_ANGLE_DEG) <=
             LOWER_HOPPER_GATE_ANGLE_TOL_DEG)) ? 1U : 0U;
}

uint8_t lower_hopper_gate_is_closed(void)
{
    lower_hopper_gate_process();
    return ((lower_hopper_gate.initialized != 0U) &&
            (lower_hopper_gate_command_available() != 0U) &&
            (lower_hopper_gate.busy == 0U) &&
            (lower_hopper_gate_absf(lower_hopper_gate.current_angle_deg -
                                    LOWER_HOPPER_GATE_CLOSE_ANGLE_DEG) <=
             LOWER_HOPPER_GATE_ANGLE_TOL_DEG)) ? 1U : 0U;
}

uint8_t lower_hopper_gate_is_hw_ready(void)
{
    return (LOWER_HOPPER_GATE_HW_READY != 0U) ? 1U : 0U;
}

uint8_t lower_hopper_gate_is_simulation_enabled(void)
{
    return (LOWER_HOPPER_GATE_SIMULATION_ENABLE != 0U) ? 1U : 0U;
}
