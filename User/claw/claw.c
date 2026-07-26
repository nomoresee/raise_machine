#include "headfile.h"

typedef struct
{
    float left_current_angle_deg;
    float right_current_angle_deg;
    float left_target_angle_deg;
    float right_target_angle_deg;
    uint32_t command_tick;
    uint32_t settle_ms;
    uint8_t initialized;
    uint8_t busy;
} claw_t;

static claw_t claw;

static float claw_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float claw_clamp_angle(float angle_deg)
{
    if (angle_deg < CLAW_OPEN_ANGLE_DEG)
    {
        return CLAW_OPEN_ANGLE_DEG;
    }

    if (angle_deg > CLAW_CLOSE_ANGLE_DEG)
    {
        return CLAW_CLOSE_ANGLE_DEG;
    }

    return angle_deg;
}

static uint32_t claw_estimate_settle_ms(float left_from_deg,
                                        float left_to_deg,
                                        float right_from_deg,
                                        float right_to_deg,
                                        float speed_deg_per_sec)
{
    float left_delta = claw_absf(left_to_deg - left_from_deg);
    float right_delta = claw_absf(right_to_deg - right_from_deg);
    float max_delta = (left_delta >= right_delta) ? left_delta : right_delta;
    float settle_ms;

    if (speed_deg_per_sec <= 1.0f)
    {
        speed_deg_per_sec = 1.0f;
    }

    settle_ms = (max_delta * 1000.0f) / speed_deg_per_sec;
    return (uint32_t)(settle_ms + 0.5f) + CLAW_SETTLE_MARGIN_MS;
}

void claw_init(void)
{
    memset(&claw, 0, sizeof(claw));
    claw.left_current_angle_deg = CLAW_LEFT_CLOSE_ANGLE_DEG;
    claw.right_current_angle_deg = CLAW_RIGHT_CLOSE_ANGLE_DEG;
    claw.left_target_angle_deg = CLAW_LEFT_CLOSE_ANGLE_DEG;
    claw.right_target_angle_deg = CLAW_RIGHT_CLOSE_ANGLE_DEG;
    claw.command_tick = HAL_GetTick();
    claw.settle_ms = 0U;
    claw.initialized = 1U;
    claw.busy = 0U;
    servo1_set_angle(CLAW_LEFT_CLOSE_ANGLE_DEG);
    servo2_set_angle(CLAW_RIGHT_CLOSE_ANGLE_DEG);
}

void claw_set_angle(float left_angle_deg, float right_angle_deg, float speed_deg_per_sec)
{
    float left_target = claw_clamp_angle(left_angle_deg);
    float right_target = claw_clamp_angle(right_angle_deg);

    if (claw.initialized == 0U)
    {
        claw_init();
    }

    claw_process();

    if ((claw_absf(left_target - claw.left_target_angle_deg) <= CLAW_ANGLE_TOL_DEG) &&
        (claw_absf(right_target - claw.right_target_angle_deg) <= CLAW_ANGLE_TOL_DEG))
    {
        return;
    }

    claw.settle_ms = claw_estimate_settle_ms(claw.left_current_angle_deg,
                                             left_target,
                                             claw.right_current_angle_deg,
                                             right_target,
                                             speed_deg_per_sec);
    claw.left_target_angle_deg = left_target;
    claw.right_target_angle_deg = right_target;
    claw.command_tick = HAL_GetTick();
    claw.busy = 1U;

    servo1_set_angle(left_target);
    servo2_set_angle(right_target);
}

void claw_open(void)
{
    claw_set_angle(CLAW_LEFT_OPEN_ANGLE_DEG,
                   CLAW_RIGHT_OPEN_ANGLE_DEG,
                   CLAW_NORMAL_SPEED_DEG_PER_SEC);
}

/* 放入上/下斗时仅张到 60°，避免全张开与斗子机构发生干涉。 */
void claw_open_hopper(void)
{
    claw_set_angle(CLAW_LEFT_HOPPER_OPEN_ANGLE_DEG,
                   CLAW_RIGHT_HOPPER_OPEN_ANGLE_DEG,
                   CLAW_NORMAL_SPEED_DEG_PER_SEC);
}

void claw_close(void)
{
    claw_set_angle(CLAW_LEFT_CLOSE_ANGLE_DEG,
                   CLAW_RIGHT_CLOSE_ANGLE_DEG,
                   CLAW_NORMAL_SPEED_DEG_PER_SEC);
}

void claw_close_pick(void)
{
    claw_set_angle(CLAW_LEFT_CLOSE_ANGLE_DEG,
                   CLAW_RIGHT_CLOSE_ANGLE_DEG,
                   CLAW_PICK_CLOSE_SPEED_DEG_PER_SEC);
}

void claw_process(void)
{
    if ((claw.initialized == 0U) || (claw.busy == 0U))
    {
        return;
    }

    if ((HAL_GetTick() - claw.command_tick) >= claw.settle_ms)
    {
        claw.left_current_angle_deg = claw.left_target_angle_deg;
        claw.right_current_angle_deg = claw.right_target_angle_deg;
        claw.busy = 0U;
    }
}

uint8_t claw_is_busy(void)
{
    claw_process();
    return claw.busy;
}

uint8_t claw_is_open(void)
{
    claw_process();
    return ((claw.busy == 0U) &&
            (claw_absf(claw.left_current_angle_deg - CLAW_LEFT_OPEN_ANGLE_DEG) <= CLAW_ANGLE_TOL_DEG) &&
            (claw_absf(claw.right_current_angle_deg - CLAW_RIGHT_OPEN_ANGLE_DEG) <= CLAW_ANGLE_TOL_DEG)) ? 1U : 0U;
}

uint8_t claw_is_hopper_open(void)
{
    claw_process();
    return ((claw.busy == 0U) &&
            (claw_absf(claw.left_current_angle_deg - CLAW_LEFT_HOPPER_OPEN_ANGLE_DEG) <= CLAW_ANGLE_TOL_DEG) &&
            (claw_absf(claw.right_current_angle_deg - CLAW_RIGHT_HOPPER_OPEN_ANGLE_DEG) <= CLAW_ANGLE_TOL_DEG)) ? 1U : 0U;
}

uint8_t claw_is_closed(void)
{
    claw_process();
    return ((claw.busy == 0U) &&
            (claw_absf(claw.left_current_angle_deg - CLAW_LEFT_CLOSE_ANGLE_DEG) <= CLAW_ANGLE_TOL_DEG) &&
            (claw_absf(claw.right_current_angle_deg - CLAW_RIGHT_CLOSE_ANGLE_DEG) <= CLAW_ANGLE_TOL_DEG)) ? 1U : 0U;
}
