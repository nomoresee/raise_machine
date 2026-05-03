#include "headfile.h"

#define BEAM_CTRL_PERIOD_MS        20U
#define BEAM_CTRL_REACH_TOL        1.2f
#define BEAM_CTRL_REACH_HOLD_MS    80U
#define BEAM_CTRL_DEFAULT_MAX_VEL  0.7f
#define BEAM_CTRL_DIR              1.0f
#define BEAM_CTRL_POS_RATIO        (1.0f / 30.0f)
#define BEAM_CTRL_DECEL_RANGE      12.0f
#define BEAM_CTRL_SETTLE_TOL       0.05f
#define BEAM_CTRL_VEL_OUT_MIN      0.18f
#define BEAM_CTRL_VEL_STEP_UP      0.05f
#define BEAM_CTRL_VEL_STEP_DOWN    0.05f

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
} beam_ctrl_t;

static beam_ctrl_t beam_ctrl;

static float beam_ctrl_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float beam_ctrl_clampf(float value, float min_value, float max_value)
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

static float beam_ctrl_move_towards(float current, float target, float step)
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

void beam_ctrl_init(hcan_t *hcan, motor_num motor_index)
{
    memset(&beam_ctrl, 0, sizeof(beam_ctrl));

    beam_ctrl.hcan = hcan;
    beam_ctrl.motor_index = motor_index;
    beam_ctrl.max_vel = BEAM_CTRL_DEFAULT_MAX_VEL;
    beam_ctrl.cmd_output_vel = 0.0f;
    beam_ctrl.reach_tol = BEAM_CTRL_REACH_TOL;
    beam_ctrl.arrived = 1U;
    beam_ctrl.ctrl_tick = HAL_GetTick();
    beam_ctrl.reach_tick = HAL_GetTick();

    motor[motor_index].ctrl.mode = pos_mode;
    (void)motor_angle_register(motor_index);
    (void)motor_angle_set_pos_ratio(motor_index, BEAM_CTRL_POS_RATIO);
}

void beam_ctrl_set_target(float target_pos)
{
    beam_ctrl.target_pos = target_pos;
    beam_ctrl.busy = 1U;
    beam_ctrl.arrived = 0U;
    beam_ctrl.reach_tick = HAL_GetTick();
}

void beam_ctrl_set_max_vel(float max_vel)
{
    if (max_vel < 0.0f)
    {
        max_vel = 0.0f;
    }

    beam_ctrl.max_vel = max_vel;
}

void beam_ctrl_start(void)
{
    beam_ctrl.enabled = 1U;
    beam_ctrl.busy = 1U;
    beam_ctrl.arrived = 0U;
    beam_ctrl.reach_tick = HAL_GetTick();
}

void beam_ctrl_stop(void)
{
    beam_ctrl.enabled = 0U;
    beam_ctrl.busy = 0U;
    beam_ctrl.arrived = 1U;
    beam_ctrl.cmd_output_vel = 0.0f;
}

float beam_ctrl_get_current_pos(void)
{
    beam_ctrl.current_pos = BEAM_CTRL_DIR * motor_angle_get(beam_ctrl.motor_index);
    return beam_ctrl.current_pos;
}

uint8_t beam_ctrl_is_busy(void)
{
    uint32_t now_tick;
    float pos_error;

    if ((beam_ctrl.enabled == 0U) || (beam_ctrl.hcan == NULL))
    {
        return 0U;
    }

    now_tick = HAL_GetTick();
    pos_error = beam_ctrl_absf(beam_ctrl.target_pos - beam_ctrl_get_current_pos());

    if (pos_error <= beam_ctrl.reach_tol)
    {
        if ((now_tick - beam_ctrl.reach_tick) >= BEAM_CTRL_REACH_HOLD_MS)
        {
            beam_ctrl.busy = 0U;
            beam_ctrl.arrived = 1U;
        }
    }
    else
    {
        beam_ctrl.busy = 1U;
        beam_ctrl.arrived = 0U;
        beam_ctrl.reach_tick = now_tick;
    }

    return beam_ctrl.busy;
}

uint8_t beam_ctrl_is_arrived(void)
{
    (void)beam_ctrl_is_busy();
    return beam_ctrl.arrived;
}

void beam_ctrl_process(void)
{
    motor_t *motor_ptr;
    uint32_t now_tick;
    float pos_error;
    float target_output_vel;
    float cmd_motor_vel;

    if ((beam_ctrl.hcan == NULL) || (beam_ctrl.enabled == 0U))
    {
        return;
    }

    now_tick = HAL_GetTick();
    if ((now_tick - beam_ctrl.ctrl_tick) < BEAM_CTRL_PERIOD_MS)
    {
        return;
    }
    beam_ctrl.ctrl_tick = now_tick;

    motor_ptr = &motor[beam_ctrl.motor_index];
    beam_ctrl.current_pos = beam_ctrl_get_current_pos();
    pos_error = beam_ctrl_absf(beam_ctrl.target_pos - beam_ctrl.current_pos);

    if (pos_error <= BEAM_CTRL_SETTLE_TOL)
    {
        target_output_vel = 0.0f;
    }
    else
    {
        if (pos_error >= BEAM_CTRL_DECEL_RANGE)
        {
            target_output_vel = beam_ctrl.max_vel;
        }
        else
        {
            target_output_vel = beam_ctrl.max_vel * (pos_error / BEAM_CTRL_DECEL_RANGE);
            target_output_vel = beam_ctrl_clampf(target_output_vel,
                                                 BEAM_CTRL_VEL_OUT_MIN,
                                                 beam_ctrl.max_vel);
        }
    }

    if (beam_ctrl.cmd_output_vel < target_output_vel)
    {
        beam_ctrl.cmd_output_vel = beam_ctrl_move_towards(beam_ctrl.cmd_output_vel,
                                                          target_output_vel,
                                                          BEAM_CTRL_VEL_STEP_UP);
    }
    else
    {
        beam_ctrl.cmd_output_vel = beam_ctrl_move_towards(beam_ctrl.cmd_output_vel,
                                                          target_output_vel,
                                                          BEAM_CTRL_VEL_STEP_DOWN);
    }

    cmd_motor_vel = beam_ctrl.cmd_output_vel / BEAM_CTRL_POS_RATIO;
    motor_ptr->ctrl.mode = pos_mode;
    motor_ptr->ctrl.pos_set = motor_angle_to_raw_pos(beam_ctrl.motor_index,
                                                     BEAM_CTRL_DIR * beam_ctrl.target_pos);
    motor_ptr->ctrl.vel_set = cmd_motor_vel;
    pos_ctrl(beam_ctrl.hcan, motor_ptr->id, motor_ptr->ctrl.pos_set, motor_ptr->ctrl.vel_set);

    (void)beam_ctrl_is_busy();
}
