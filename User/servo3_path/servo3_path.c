#include "headfile.h"

typedef struct
{
    float current_angle_deg;
    float target_angle_deg;
    uint32_t command_tick;
    uint32_t settle_ms;
    uint8_t initialized;
} servo3_path_t;

static servo3_path_t servo3_path;

static float servo3_path_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float servo3_path_clamp_angle(float angle_deg)
{
    if (angle_deg < SERVO_MIN_ANGLE_DEG)
    {
        return SERVO_MIN_ANGLE_DEG;
    }

    if (angle_deg > SERVO3_MAX_ANGLE_DEG)
    {
        return SERVO3_MAX_ANGLE_DEG;
    }

    return angle_deg;
}

static uint32_t servo3_path_estimate_settle_ms(float from_angle_deg, float to_angle_deg)
{
    float delta = servo3_path_absf(to_angle_deg - from_angle_deg);
    float speed = SERVO3_ROTATE_SPEED_DEG_PER_SEC;
    float settle_ms;

    if (speed <= 1.0f)
    {
        speed = 1.0f;
    }

    settle_ms = (delta * 1000.0f) / speed;
    return (uint32_t)(settle_ms + 0.5f);
}

void servo3_path_init(void)
{
    memset(&servo3_path, 0, sizeof(servo3_path));
    servo3_path.current_angle_deg = SERVO3_PICK_AREA_ANGLE_DEG;
    servo3_path.target_angle_deg = SERVO3_PICK_AREA_ANGLE_DEG;
    servo3_path.command_tick = HAL_GetTick();
    servo3_path.settle_ms = 0U;
    servo3_path.initialized = 1U;
    servo3_set_angle(SERVO3_PICK_AREA_ANGLE_DEG);
}

void servo3_path_release_angle(float angle_deg)
{
    float target = servo3_path_clamp_angle(angle_deg);
    float current;

    if (servo3_path.initialized == 0U)
    {
        servo3_path_init();
    }

    if (servo3_path_is_arrived() != 0U)
    {
        servo3_path.current_angle_deg = servo3_path.target_angle_deg;
    }

    if (servo3_path_absf(target - servo3_path.target_angle_deg) <= SERVO3_ANGLE_TOL_DEG)
    {
        return;
    }

    current = servo3_path.current_angle_deg;
    servo3_path.target_angle_deg = target;
    servo3_path.command_tick = HAL_GetTick();
    servo3_path.settle_ms = servo3_path_estimate_settle_ms(current, target);
    servo3_set_angle(target);
}

void servo3_path_release_pick_area(void)
{
    servo3_path_release_angle(SERVO3_PICK_AREA_ANGLE_DEG);
}

void servo3_path_release_for_slot(uint8_t slot)
{
    servo3_path_release_angle(servo3_path_angle_for_slot(slot));
}

uint8_t servo3_path_is_arrived(void)
{
    uint32_t now_tick;

    if (servo3_path.initialized == 0U)
    {
        return 1U;
    }

    now_tick = HAL_GetTick();
    if ((now_tick - servo3_path.command_tick) >= servo3_path.settle_ms)
    {
        servo3_path.current_angle_deg = servo3_path.target_angle_deg;
        return 1U;
    }

    return 0U;
}

uint8_t servo3_path_is_extreme_slot(uint8_t slot)
{
    return ((slot == 4U) || (slot == 8U)) ? 1U : 0U;
}

float servo3_path_angle_for_slot(uint8_t slot)
{
    if ((slot >= 1U) && (slot <= 3U))
    {
        return SERVO3_PICK_AREA_ANGLE_DEG;
    }

    if (slot == 4U)
    {
        return SERVO3_SLOT4_ANGLE_DEG;
    }

    if (slot == 8U)
    {
        return SERVO3_SLOT8_ANGLE_DEG;
    }

    if ((slot >= 5U) && (slot <= 7U))
    {
        return SERVO3_PLACE_MID_ANGLE_DEG;
    }

    return SERVO3_PICK_AREA_ANGLE_DEG;
}

float servo3_path_safe_y_for_slot(uint8_t slot)
{
    if (slot == 4U)
    {
        return SERVO3_SLOT4_SAFE_Y;
    }

    if (slot == 8U)
    {
        return SERVO3_SLOT8_SAFE_Y;
    }

    return 0.0f;
}
