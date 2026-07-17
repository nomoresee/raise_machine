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

static uint32_t servo3_path_estimate_settle_ms(float from_angle_deg,
                                                float to_angle_deg)
{
    float delta = servo3_path_absf(to_angle_deg - from_angle_deg);
    float speed = SERVO3_ROTATE_SPEED_DEG_PER_SEC;
    float settle_ms;

    if (speed <= 1.0f)
    {
        speed = 1.0f;
    }

    settle_ms = (delta * 1000.0f) / speed;
    return (uint32_t)(settle_ms + 0.5f) + SERVO3_SETTLE_MARGIN_MS;
}

void servo3_path_init(void)
{
    memset(&servo3_path, 0, sizeof(servo3_path));

    /* 上电默认朝向置物区。 */
    servo3_path.current_angle_deg = SERVO3_PLACE_DEFAULT_ANGLE_DEG;
    servo3_path.target_angle_deg = SERVO3_PLACE_DEFAULT_ANGLE_DEG;
    servo3_path.command_tick = HAL_GetTick();
    servo3_path.initialized = 1U;
    servo3_set_angle(SERVO3_PLACE_DEFAULT_ANGLE_DEG);
}

void servo3_path_release_angle(float angle_deg)
{
    float target = servo3_path_clamp_angle(angle_deg);

    if (servo3_path.initialized == 0U)
    {
        servo3_path_init();
    }

    if (servo3_path_is_arrived() != 0U)
    {
        servo3_path.current_angle_deg = servo3_path.target_angle_deg;
    }

    if (servo3_path_absf(target - servo3_path.target_angle_deg) <=
        SERVO3_ANGLE_TOL_DEG)
    {
        return;
    }

    servo3_path.settle_ms =
        servo3_path_estimate_settle_ms(servo3_path.current_angle_deg, target);
    servo3_path.target_angle_deg = target;
    servo3_path.command_tick = HAL_GetTick();
    servo3_set_angle(target);
}

void servo3_path_release_pick_area(void)
{
    servo3_path_release_angle(SERVO3_PICK_AREA_ANGLE_DEG);
}

float servo3_path_place_angle_for_pick(uint8_t pick_slot)
{
    switch (pick_slot)
    {
        case 1U:
            return SERVO3_PICK1_PLACE_ANGLE_DEG;

        case 2U:
            return SERVO3_PICK2_PLACE_ANGLE_DEG;

        case 3U:
            return SERVO3_PICK3_PLACE_ANGLE_DEG;

        default:
            return SERVO3_PLACE_DEFAULT_ANGLE_DEG;
    }
}

void servo3_path_release_place_for_pick(uint8_t pick_slot)
{
    servo3_path_release_angle(servo3_path_place_angle_for_pick(pick_slot));
}

void servo3_path_release_pick_from_place(uint8_t pick_slot)
{
    /*
     * 绝对角度命令会自然沿原路径返回：270 -> 135 或 0 -> 135。
     * pick_slot 保留在接口中，用于明确这次返回属于哪条旋转路径。
     */
    (void)pick_slot;
    servo3_path_release_pick_area();
}

uint8_t servo3_path_is_arrived(void)
{
    if (servo3_path.initialized == 0U)
    {
        return 0U;
    }

    if ((HAL_GetTick() - servo3_path.command_tick) >= servo3_path.settle_ms)
    {
        servo3_path.current_angle_deg = servo3_path.target_angle_deg;
        return 1U;
    }

    return 0U;
}

float servo3_path_get_current_angle(void)
{
    (void)servo3_path_is_arrived();
    return servo3_path.current_angle_deg;
}

float servo3_path_get_target_angle(void)
{
    return servo3_path.target_angle_deg;
}
