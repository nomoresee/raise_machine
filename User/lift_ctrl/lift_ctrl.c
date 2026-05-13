#include "headfile.h"

#define LIFT_CTRL_PERIOD_MS        20U
#define LIFT_CTRL_REACH_TOL        1.2f
#define LIFT_CTRL_REACH_HOLD_MS    80U
#define LIFT_CTRL_DEFAULT_MAX_VEL  0.7f
#define LIFT_CTRL_DIR              1.0f
#define LIFT_CTRL_POS_RATIO        (1.0f / 30.0f)
#define LIFT_CTRL_DECEL_RANGE      12.0f
#define LIFT_CTRL_SETTLE_TOL       0.05f
#define LIFT_CTRL_VEL_OUT_MIN      0.18f
#define LIFT_CTRL_VEL_STEP_UP      0.05f
#define LIFT_CTRL_VEL_STEP_DOWN    0.05f

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
} lift_ctrl_t;

static lift_ctrl_t lift_ctrl;

static float lift_ctrl_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float lift_ctrl_clampf(float value, float min_value, float max_value)
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

static float lift_ctrl_move_towards(float current, float target, float step)
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

void lift_ctrl_init(hcan_t *hcan, motor_num motor_index)
{
    memset(&lift_ctrl, 0, sizeof(lift_ctrl));

    lift_ctrl.hcan = hcan;
    lift_ctrl.motor_index = motor_index;
    lift_ctrl.max_vel = LIFT_CTRL_DEFAULT_MAX_VEL;
    lift_ctrl.cmd_output_vel = 0.0f;
    lift_ctrl.reach_tol = LIFT_CTRL_REACH_TOL;
    lift_ctrl.arrived = 1U;
    lift_ctrl.ctrl_tick = HAL_GetTick();
    lift_ctrl.reach_tick = HAL_GetTick();

    motor[motor_index].ctrl.mode = pos_mode;
    (void)motor_angle_register(motor_index);
    (void)motor_angle_set_pos_ratio(motor_index, LIFT_CTRL_POS_RATIO);
}

void lift_ctrl_set_target(float target_pos)
{
    lift_ctrl.target_pos = target_pos;
    lift_ctrl.busy = 1U;
    lift_ctrl.arrived = 0U;
    lift_ctrl.reach_tick = HAL_GetTick();
}

void lift_ctrl_set_max_vel(float max_vel)
{
    if (max_vel < 0.0f)
    {
        max_vel = 0.0f;
    }

    lift_ctrl.max_vel = max_vel;
}

void lift_ctrl_start(void)
{
    lift_ctrl.enabled = 1U;
    lift_ctrl.busy = 1U;
    lift_ctrl.arrived = 0U;
    lift_ctrl.reach_tick = HAL_GetTick();
}

void lift_ctrl_stop(void)
{
    lift_ctrl.enabled = 0U;
    lift_ctrl.busy = 0U;
    lift_ctrl.arrived = 1U;
    lift_ctrl.cmd_output_vel = 0.0f;
}

float lift_ctrl_get_current_pos(void)
{
    lift_ctrl.current_pos = LIFT_CTRL_DIR * motor_angle_get(lift_ctrl.motor_index);
    return lift_ctrl.current_pos;
}

uint8_t lift_ctrl_is_busy(void)
{
    uint32_t now_tick;
    float pos_error;

    if ((lift_ctrl.enabled == 0U) || (lift_ctrl.hcan == NULL))
    {
        return 0U;
    }

    now_tick = HAL_GetTick();
    pos_error = lift_ctrl_absf(lift_ctrl.target_pos - lift_ctrl_get_current_pos());

    if (pos_error <= lift_ctrl.reach_tol)
    {
        if ((now_tick - lift_ctrl.reach_tick) >= LIFT_CTRL_REACH_HOLD_MS)
        {
            lift_ctrl.busy = 0U;
            lift_ctrl.arrived = 1U;
        }
    }
    else
    {
        lift_ctrl.busy = 1U;
        lift_ctrl.arrived = 0U;
        lift_ctrl.reach_tick = now_tick;
    }

    return lift_ctrl.busy;
}

uint8_t lift_ctrl_is_arrived(void)
{
    (void)lift_ctrl_is_busy();
    return lift_ctrl.arrived;
}

void lift_ctrl_process(void)
{
    motor_t *motor_ptr;
    uint32_t now_tick;
    float pos_error;
    float target_output_vel;
    float cmd_motor_vel;

    if ((lift_ctrl.hcan == NULL) || (lift_ctrl.enabled == 0U))
    {
        return;
    }

    now_tick = HAL_GetTick();
    if ((now_tick - lift_ctrl.ctrl_tick) < LIFT_CTRL_PERIOD_MS)
    {
        return;
    }
    lift_ctrl.ctrl_tick = now_tick;

    motor_ptr = &motor[lift_ctrl.motor_index];
    lift_ctrl.current_pos = lift_ctrl_get_current_pos();
    pos_error = lift_ctrl_absf(lift_ctrl.target_pos - lift_ctrl.current_pos);

    if (pos_error <= LIFT_CTRL_SETTLE_TOL)
    {
        target_output_vel = 0.0f;
    }
    else
    {
        if (pos_error >= LIFT_CTRL_DECEL_RANGE)
        {
            target_output_vel = lift_ctrl.max_vel;
        }
        else
        {
            target_output_vel = lift_ctrl.max_vel * (pos_error / LIFT_CTRL_DECEL_RANGE);
            target_output_vel = lift_ctrl_clampf(target_output_vel,
                                                 LIFT_CTRL_VEL_OUT_MIN,
                                                 lift_ctrl.max_vel);
        }
    }

    if (lift_ctrl.cmd_output_vel < target_output_vel)
    {
        lift_ctrl.cmd_output_vel = lift_ctrl_move_towards(lift_ctrl.cmd_output_vel,
                                                          target_output_vel,
                                                          LIFT_CTRL_VEL_STEP_UP);
    }
    else
    {
        lift_ctrl.cmd_output_vel = lift_ctrl_move_towards(lift_ctrl.cmd_output_vel,
                                                          target_output_vel,
                                                          LIFT_CTRL_VEL_STEP_DOWN);
    }

    cmd_motor_vel = lift_ctrl.cmd_output_vel / LIFT_CTRL_POS_RATIO;
    motor_ptr->ctrl.mode = pos_mode;
    motor_ptr->ctrl.pos_set = motor_angle_to_raw_pos(lift_ctrl.motor_index,
                                                     LIFT_CTRL_DIR * lift_ctrl.target_pos);
    motor_ptr->ctrl.vel_set = cmd_motor_vel;
    pos_ctrl(lift_ctrl.hcan, motor_ptr->id, motor_ptr->ctrl.pos_set, motor_ptr->ctrl.vel_set);

    (void)lift_ctrl_is_busy();
}
