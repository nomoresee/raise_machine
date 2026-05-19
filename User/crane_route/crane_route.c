#include "headfile.h"

#define CRANE_ROUTE_STEP_COUNT         7U
#define CRANE_ROUTE_LIFT_PICK_1_POS    2.0f
#define CRANE_ROUTE_LIFT_PICK_2_POS    5.0f
#define CRANE_ROUTE_LIFT_PICK_3_POS    6.0f
#define CRANE_ROUTE_LIFT_PLACE_POS     7.0f
#define CRANE_ROUTE_LIFT_SAFE_POS      12.0f
#define CRANE_ROUTE_STEP_DWELL_MS      3000U

typedef struct
{
    uint8_t slot;
    crane_route_action_e action;
} crane_route_step_t;

static crane_slot_pose_t crane_route_slot_pose[CRANE_ROUTE_SLOT_COUNT + 1U] =
{
    {0.0f, 0.0f, 0.0f, 0.0f}, /* unused slot 0 */
    {1500.0f, 20.0f, CRANE_ROUTE_LIFT_PICK_1_POS, CRANE_ROUTE_LIFT_SAFE_POS},
    {1500.0f, 0.0f, CRANE_ROUTE_LIFT_PICK_2_POS, CRANE_ROUTE_LIFT_SAFE_POS},
    {1500.0f, -20.0f, CRANE_ROUTE_LIFT_PICK_3_POS, CRANE_ROUTE_LIFT_SAFE_POS},
    {-1700.0f, 15.0f, CRANE_ROUTE_LIFT_PLACE_POS, CRANE_ROUTE_LIFT_SAFE_POS},
    {-1700.0f, 20.0f, CRANE_ROUTE_LIFT_PLACE_POS, CRANE_ROUTE_LIFT_SAFE_POS},
    {-1700.0f, 25.0f, CRANE_ROUTE_LIFT_PLACE_POS, CRANE_ROUTE_LIFT_SAFE_POS},
    {-1700.0f, -20.0f, CRANE_ROUTE_LIFT_PLACE_POS, CRANE_ROUTE_LIFT_SAFE_POS},
    {-1700.0f, -25.0f, CRANE_ROUTE_LIFT_PLACE_POS, CRANE_ROUTE_LIFT_SAFE_POS},
};

static const crane_route_step_t crane_route_steps[CRANE_ROUTE_STEP_COUNT] =
{
    {1U, CRANE_ROUTE_ACTION_PICK},
    {4U, CRANE_ROUTE_ACTION_PLACE},
    {3U, CRANE_ROUTE_ACTION_PICK},
    {5U, CRANE_ROUTE_ACTION_PLACE},
    {2U, CRANE_ROUTE_ACTION_PICK},
    {7U, CRANE_ROUTE_ACTION_PLACE},
    {0U, CRANE_ROUTE_ACTION_PLACE},
};

typedef struct
{
    crane_route_state_e state;
    uint8_t step_index;
    uint8_t current_slot;
    uint8_t first_move_done;
    crane_route_action_e current_action;
    uint32_t dwell_tick;
} crane_route_t;

static crane_route_t crane_route;

static void crane_route_move_xy_to_slot(uint8_t slot)
{
    pos_pid_sync_set_target(crane_route_slot_pose[slot].chassis_pos);
    beam_ctrl_set_target(crane_route_slot_pose[slot].beam_pos);
    pos_pid_sync_start();
    beam_ctrl_start();
}

static void crane_route_move_lift_to(float target_pos)
{
    lift_ctrl_set_target(target_pos);
    lift_ctrl_start();
}

static uint8_t crane_route_should_raise_with_xy(void)
{
    if ((crane_route.first_move_done == 0U) &&
        (crane_route.current_slot > 0U) &&
        (crane_route.current_slot <= CRANE_ROUTE_SLOT_COUNT))
    {
        return 1U;
    }

    return 0U;
}

static void crane_route_set_gripper(crane_route_action_e action)
{
#if (CRANE_ROUTE_USE_SERVO != 0U)
    if (action == CRANE_ROUTE_ACTION_PICK)
    {
        servo1_set_angle(SERVO1_GRIP_CLOSE_ANGLE);
    }
    else
    {
        servo1_set_angle(SERVO1_GRIP_OPEN_ANGLE);
    }
#else
    (void)action; /* 无舵机：不动作，由 CRANE_ROUTE_GRIPPER_HOLD 延时代替夹持 */
#endif
}

void crane_route_init(void)
{
    memset(&crane_route, 0, sizeof(crane_route));
    crane_route.state = CRANE_ROUTE_IDLE;
    crane_route.current_action = CRANE_ROUTE_ACTION_PICK;
}

void crane_route_start(void)
{
    crane_route.step_index = 0U;
    crane_route.current_slot = 0U;
    crane_route.first_move_done = 0U;
    crane_route.current_action = CRANE_ROUTE_ACTION_PICK;
    crane_route.dwell_tick = HAL_GetTick();
    pos_pid_sync_stop();
    beam_ctrl_stop();
    lift_ctrl_stop();
    crane_route.state = CRANE_ROUTE_LOAD_STEP;
}

void crane_route_stop(void)
{
    crane_route.state = CRANE_ROUTE_IDLE;
    pos_pid_sync_stop();
    beam_ctrl_stop();
    lift_ctrl_stop();
}

void crane_route_set_slot_pose(uint8_t slot, float chassis_pos, float beam_pos)
{
    if ((slot == 0U) || (slot > CRANE_ROUTE_SLOT_COUNT))
    {
        return;
    }

    crane_route_slot_pose[slot].chassis_pos = chassis_pos;
    crane_route_slot_pose[slot].beam_pos = beam_pos;
}

void crane_route_set_slot_lift_pos(uint8_t slot, float lift_work_pos, float lift_safe_pos)
{
    if ((slot == 0U) || (slot > CRANE_ROUTE_SLOT_COUNT))
    {
        return;
    }

    crane_route_slot_pose[slot].lift_work_pos = lift_work_pos;
    crane_route_slot_pose[slot].lift_safe_pos = lift_safe_pos;
}

crane_route_state_e crane_route_get_state(void)
{
    return crane_route.state;
}

uint8_t crane_route_is_finished(void)
{
    return (crane_route.state == CRANE_ROUTE_FINISHED) ? 1U : 0U;
}

uint8_t crane_route_get_current_slot(void)
{
    return crane_route.current_slot;
}

void crane_route_get_current_target(float *x, float *y)
{
    uint8_t slot = crane_route.current_slot;
    float target_x = 0.0f;
    float target_y = 0.0f;

    if ((slot > 0U) && (slot <= CRANE_ROUTE_SLOT_COUNT))
    {
        target_x = crane_route_slot_pose[slot].chassis_pos;
        target_y = crane_route_slot_pose[slot].beam_pos;
    }

    if (x != NULL)
    {
        *x = target_x;
    }
    if (y != NULL)
    {
        *y = target_y;
    }
}

void crane_route_get_current_pose_target(float *x, float *y, float *z)
{
    uint8_t slot = crane_route.current_slot;
    float target_x = 0.0f;
    float target_y = 0.0f;
    float target_z = 0.0f;

    if ((slot > 0U) && (slot <= CRANE_ROUTE_SLOT_COUNT))
    {
        target_x = crane_route_slot_pose[slot].chassis_pos;
        target_y = crane_route_slot_pose[slot].beam_pos;

        if ((crane_route.state == CRANE_ROUTE_LIFT_UP_SAFE) ||
            (crane_route.state == CRANE_ROUTE_WAIT_LIFT_UP_ARRIVE) ||
            (crane_route.state == CRANE_ROUTE_STEP_DWELL) ||
            (crane_route.state == CRANE_ROUTE_FINISHED))
        {
            target_z = crane_route_slot_pose[slot].lift_safe_pos;
        }
        else
        {
            target_z = crane_route_slot_pose[slot].lift_work_pos;
        }
    }

    if (x != NULL)
    {
        *x = target_x;
    }
    if (y != NULL)
    {
        *y = target_y;
    }
    if (z != NULL)
    {
        *z = target_z;
    }
}

void crane_route_process(void)
{
    uint8_t slot;

    switch (crane_route.state)
    {
        case CRANE_ROUTE_IDLE:
        case CRANE_ROUTE_FINISHED:
            break;

        case CRANE_ROUTE_LOAD_STEP:
            if (crane_route.step_index >= CRANE_ROUTE_STEP_COUNT)
            {
                crane_route.state = CRANE_ROUTE_FINISHED;
                break;
            }

            slot = crane_route_steps[crane_route.step_index].slot;
            if (slot > CRANE_ROUTE_SLOT_COUNT)
            {
                crane_route.state = CRANE_ROUTE_FINISHED;
                break;
            }

            crane_route.current_slot = slot;
            crane_route.current_action = crane_route_steps[crane_route.step_index].action;
            crane_route_move_xy_to_slot(slot);
            if (slot == 0U)
            {
                lift_ctrl_stop();
                crane_route.state = CRANE_ROUTE_WAIT_XY_ARRIVE;
            }
            else
            {
                if (crane_route_should_raise_with_xy() != 0U)
                {
                    crane_route_move_lift_to(crane_route_slot_pose[slot].lift_safe_pos);
                }
                crane_route.state = CRANE_ROUTE_WAIT_XY_ARRIVE;
            }
            break;

        case CRANE_ROUTE_WAIT_XY_ARRIVE:
            if ((pos_pid_sync_is_busy() == 0U) &&
                (beam_ctrl_is_busy() == 0U))
            {
                if (crane_route.current_slot == 0U)
                {
                    crane_route.dwell_tick = HAL_GetTick();
                    crane_route.state = CRANE_ROUTE_STEP_DWELL;
                }
                else if (crane_route_should_raise_with_xy() != 0U)
                {
                    if (lift_ctrl_is_busy() == 0U)
                    {
                        crane_route.first_move_done = 1U;
                        crane_route.state = CRANE_ROUTE_LIFT_DOWN;
                    }
                }
                else
                {
                    crane_route.state = CRANE_ROUTE_LIFT_DOWN;
                }
            }
            break;

        case CRANE_ROUTE_LIFT_DOWN:
            crane_route_move_lift_to(crane_route_slot_pose[crane_route.current_slot].lift_work_pos);
            crane_route.state = CRANE_ROUTE_WAIT_LIFT_DOWN_ARRIVE;
            break;

        case CRANE_ROUTE_WAIT_LIFT_DOWN_ARRIVE:
            if (lift_ctrl_is_busy() == 0U)
            {
                crane_route.state = CRANE_ROUTE_GRIPPER_ACT;
            }
            break;

        case CRANE_ROUTE_GRIPPER_ACT:
            crane_route_set_gripper(crane_route.current_action);
            crane_route.dwell_tick = HAL_GetTick();
            crane_route.state = CRANE_ROUTE_GRIPPER_HOLD;
            break;

        case CRANE_ROUTE_GRIPPER_HOLD:
            if ((HAL_GetTick() - crane_route.dwell_tick) >= CRANE_ROUTE_PICK_DWELL_MS)
            {
                crane_route.state = CRANE_ROUTE_LIFT_UP_SAFE;
            }
            break;

        case CRANE_ROUTE_LIFT_UP_SAFE:
            crane_route_move_lift_to(crane_route_slot_pose[crane_route.current_slot].lift_safe_pos);
            crane_route.state = CRANE_ROUTE_WAIT_LIFT_UP_ARRIVE;
            break;

        case CRANE_ROUTE_WAIT_LIFT_UP_ARRIVE:
            if (lift_ctrl_is_busy() == 0U)
            {
                crane_route.first_move_done = 1U;
                crane_route.dwell_tick = HAL_GetTick();
                crane_route.state = CRANE_ROUTE_STEP_DWELL;
            }
            break;

        case CRANE_ROUTE_STEP_DWELL:
            if ((HAL_GetTick() - crane_route.dwell_tick) >= CRANE_ROUTE_STEP_DWELL_MS)
            {
                crane_route.step_index++;
                crane_route.state = CRANE_ROUTE_LOAD_STEP;
            }
            break;

        default:
            crane_route.state = CRANE_ROUTE_IDLE;
            break;
    }
}
