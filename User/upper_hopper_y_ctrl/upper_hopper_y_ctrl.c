#include "upper_hopper_y_ctrl.h"

#include "dm_motor_ctrl.h"
#include "motor_angle/motor_angle.h"
#include <string.h>

#define UPPER_HOPPER_Y_CTRL_PERIOD_MS        10U
#define UPPER_HOPPER_Y_CTRL_REACH_TOL        1.2f
#define UPPER_HOPPER_Y_CTRL_REACH_HOLD_MS    80U
#define UPPER_HOPPER_Y_CTRL_DEFAULT_MAX_VEL  1.0f
#define UPPER_HOPPER_Y_CTRL_DIR              -1.0f
#define UPPER_HOPPER_Y_CTRL_POS_RATIO        (1.0f / 30.0f)
#define UPPER_HOPPER_Y_CTRL_DECEL_RANGE      12.0f
#define UPPER_HOPPER_Y_CTRL_SETTLE_TOL       0.05f
#define UPPER_HOPPER_Y_CTRL_VEL_OUT_MIN      0.18f
#define UPPER_HOPPER_Y_CTRL_VEL_STEP_UP      0.05f
#define UPPER_HOPPER_Y_CTRL_VEL_STEP_DOWN    0.05f

typedef struct
{
    hcan_t *hcan;
    motor_num motor_index;
    uint32_t ctrl_tick;
    uint32_t reach_tick;
    float target_pos;
    float current_pos;
    float max_vel;
    float cmd_output_vel;
    float reach_tol;
    uint8_t enabled;
    uint8_t busy;
    uint8_t arrived;
} upper_hopper_y_ctrl_t;

static upper_hopper_y_ctrl_t upper_hopper_y_ctrl;

static float upper_hopper_y_ctrl_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float upper_hopper_y_ctrl_clampf(float value, float min_value, float max_value)
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

static float upper_hopper_y_ctrl_move_towards(float current, float target, float step)
{
    float delta = target - current;

    if (delta > step)
    {
        return current + step;
    }

    if (delta < -step)
    {
        return current - step;
    }

    return target;
}

void upper_hopper_y_ctrl_init(hcan_t *hcan, motor_num motor_index)
{
    memset(&upper_hopper_y_ctrl, 0, sizeof(upper_hopper_y_ctrl));

    upper_hopper_y_ctrl.hcan = hcan;
    upper_hopper_y_ctrl.motor_index = motor_index;
    upper_hopper_y_ctrl.max_vel = UPPER_HOPPER_Y_CTRL_DEFAULT_MAX_VEL;
    upper_hopper_y_ctrl.cmd_output_vel = 0.0f;
    upper_hopper_y_ctrl.reach_tol = UPPER_HOPPER_Y_CTRL_REACH_TOL;
    upper_hopper_y_ctrl.arrived = 1U;
    upper_hopper_y_ctrl.ctrl_tick = HAL_GetTick();
    upper_hopper_y_ctrl.reach_tick = HAL_GetTick();

    motor[motor_index].ctrl.mode = pos_mode;
    (void)motor_angle_register(motor_index);
    (void)motor_angle_set_pos_ratio(motor_index, UPPER_HOPPER_Y_CTRL_POS_RATIO);
}

void upper_hopper_y_ctrl_set_target(float target_pos)
{
    upper_hopper_y_ctrl.target_pos = target_pos;
    upper_hopper_y_ctrl.busy = 1U;
    upper_hopper_y_ctrl.arrived = 0U;
    upper_hopper_y_ctrl.reach_tick = HAL_GetTick();
}

void upper_hopper_y_ctrl_set_max_vel(float max_vel)
{
    if (max_vel < 0.0f)
    {
        max_vel = 0.0f;
    }

    upper_hopper_y_ctrl.max_vel = max_vel;
}

void upper_hopper_y_ctrl_start(void)
{
    upper_hopper_y_ctrl.enabled = 1U;
    upper_hopper_y_ctrl.busy = 1U;
    upper_hopper_y_ctrl.arrived = 0U;
    upper_hopper_y_ctrl.reach_tick = HAL_GetTick();
}

void upper_hopper_y_ctrl_stop(void)
{
    upper_hopper_y_ctrl.enabled = 0U;
    upper_hopper_y_ctrl.busy = 0U;
    upper_hopper_y_ctrl.arrived = 1U;
    upper_hopper_y_ctrl.cmd_output_vel = 0.0f;
}

float upper_hopper_y_ctrl_get_current_pos(void)
{
    upper_hopper_y_ctrl.current_pos =
        UPPER_HOPPER_Y_CTRL_DIR * motor_angle_get(upper_hopper_y_ctrl.motor_index);
    return upper_hopper_y_ctrl.current_pos;
}

float upper_hopper_y_ctrl_get_target_pos(void)
{
    return upper_hopper_y_ctrl.target_pos;
}

uint8_t upper_hopper_y_ctrl_is_busy(void)
{
    uint32_t now_tick;
    float pos_error;

    if ((upper_hopper_y_ctrl.enabled == 0U) || (upper_hopper_y_ctrl.hcan == NULL))
    {
        return 0U;
    }

    now_tick = HAL_GetTick();
    pos_error = upper_hopper_y_ctrl_absf(upper_hopper_y_ctrl.target_pos -
                                         upper_hopper_y_ctrl_get_current_pos());

    if (pos_error <= upper_hopper_y_ctrl.reach_tol)
    {
        if ((now_tick - upper_hopper_y_ctrl.reach_tick) >= UPPER_HOPPER_Y_CTRL_REACH_HOLD_MS)
        {
            upper_hopper_y_ctrl.busy = 0U;
            upper_hopper_y_ctrl.arrived = 1U;
        }
    }
    else
    {
        upper_hopper_y_ctrl.busy = 1U;
        upper_hopper_y_ctrl.arrived = 0U;
        upper_hopper_y_ctrl.reach_tick = now_tick;
    }

    return upper_hopper_y_ctrl.busy;
}

uint8_t upper_hopper_y_ctrl_is_arrived(void)
{
    (void)upper_hopper_y_ctrl_is_busy();
    return upper_hopper_y_ctrl.arrived;
}

void upper_hopper_y_ctrl_process(void)
{
    motor_t *motor_ptr;
    uint32_t now_tick;
    float pos_error;
    float target_output_vel;
    float cmd_motor_vel;

    if ((upper_hopper_y_ctrl.hcan == NULL) || (upper_hopper_y_ctrl.enabled == 0U))
    {
        return;
    }

    now_tick = HAL_GetTick();
    if ((now_tick - upper_hopper_y_ctrl.ctrl_tick) < UPPER_HOPPER_Y_CTRL_PERIOD_MS)
    {
        return;
    }
    upper_hopper_y_ctrl.ctrl_tick = now_tick;

    motor_ptr = &motor[upper_hopper_y_ctrl.motor_index];
    upper_hopper_y_ctrl.current_pos = upper_hopper_y_ctrl_get_current_pos();
    pos_error = upper_hopper_y_ctrl_absf(upper_hopper_y_ctrl.target_pos -
                                         upper_hopper_y_ctrl.current_pos);

    if (pos_error <= UPPER_HOPPER_Y_CTRL_SETTLE_TOL)
    {
        target_output_vel = 0.0f;
    }
    else if (pos_error >= UPPER_HOPPER_Y_CTRL_DECEL_RANGE)
    {
        target_output_vel = upper_hopper_y_ctrl.max_vel;
    }
    else
    {
        target_output_vel = upper_hopper_y_ctrl.max_vel *
                            (pos_error / UPPER_HOPPER_Y_CTRL_DECEL_RANGE);
        target_output_vel = upper_hopper_y_ctrl_clampf(target_output_vel,
                                                       UPPER_HOPPER_Y_CTRL_VEL_OUT_MIN,
                                                       upper_hopper_y_ctrl.max_vel);
    }

    if (upper_hopper_y_ctrl.cmd_output_vel < target_output_vel)
    {
        upper_hopper_y_ctrl.cmd_output_vel =
            upper_hopper_y_ctrl_move_towards(upper_hopper_y_ctrl.cmd_output_vel,
                                             target_output_vel,
                                             UPPER_HOPPER_Y_CTRL_VEL_STEP_UP);
    }
    else
    {
        upper_hopper_y_ctrl.cmd_output_vel =
            upper_hopper_y_ctrl_move_towards(upper_hopper_y_ctrl.cmd_output_vel,
                                             target_output_vel,
                                             UPPER_HOPPER_Y_CTRL_VEL_STEP_DOWN);
    }

    cmd_motor_vel = upper_hopper_y_ctrl.cmd_output_vel / UPPER_HOPPER_Y_CTRL_POS_RATIO;
    motor_ptr->ctrl.mode = pos_mode;
    motor_ptr->ctrl.pos_set =
        motor_angle_to_raw_pos(upper_hopper_y_ctrl.motor_index,
                               UPPER_HOPPER_Y_CTRL_DIR * upper_hopper_y_ctrl.target_pos);
    motor_ptr->ctrl.vel_set = cmd_motor_vel;
    pos_ctrl(upper_hopper_y_ctrl.hcan,
             motor_ptr->id,
             motor_ptr->ctrl.pos_set,
             motor_ptr->ctrl.vel_set);

    (void)upper_hopper_y_ctrl_is_busy();
}
