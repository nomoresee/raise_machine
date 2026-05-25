#include "headfile.h"

#define CRANE_ROUTE_LEG_COUNT          3U

/*
 * 三趟放置目标：
 * - 第 1 趟固定取 3 号货，放到 CRANE_ROUTE_PLACE_FOR_PICK3。
 * - 第 1 趟如果放 4/5，后续取货顺序为 1 -> 2。
 * - 第 1 趟如果放 6/7/8，后续取货顺序为 2 -> 1。
 *
 * 例：
 * 4,5,6 => 3-4-直线-1-上绕-5-上绕-2-下绕-6-回中
 * 7,4,5 => 3-7-直线-2-下绕-4-直线-1-上绕-5-回中
 */
#define CRANE_ROUTE_PLACE_FOR_PICK3     4U
#define CRANE_ROUTE_PLACE_FOR_SECOND    5U
#define CRANE_ROUTE_PLACE_FOR_THIRD     6U

#define CRANE_ROUTE_LIFT_PICK_1_POS     2.0f
#define CRANE_ROUTE_LIFT_PICK_2_POS     5.0f
#define CRANE_ROUTE_LIFT_PICK_3_POS     6.0f
#define CRANE_ROUTE_LIFT_PLACE_POS      7.0f
#define CRANE_ROUTE_LIFT_SAFE_POS       12.0f
#define CRANE_ROUTE_LEG_DWELL_MS        300U

/* Z 从工作低位抬高 2 后，Y 可以提前去绕行入口；X 仍等 Z 到安全位后再跑。 */
#define CRANE_ROUTE_Y_PREMOVE_DELTA     2.5f
#define CRANE_ROUTE_Z_GATE_TOL          1.0f

typedef struct
{
    uint8_t pick_slot;
    uint8_t place_slot;
    uint8_t next_pick_slot;           /* 0 表示最后回中 */
    xy_route_type_e go_route;         /* 载货去放置点：上绕/下绕/直线 */
    xy_route_type_e return_route;     /* 空载回取货区：上绕/下绕/直线 */
    xy_release_mode_e go_release;     /* 载货段：入口释放或出口释放 */
    xy_release_mode_e return_release; /* 回程段：当前绕行回程都按出口释放 */
} crane_route_leg_t;

static crane_slot_pose_t crane_route_slot_pose[CRANE_ROUTE_SLOT_COUNT + 1U] =
{
    {0.0f, 0.0f, 0.0f, 0.0f}, /* 0 号：中心/回零点 */
    {1100.0f, 15.0f, CRANE_ROUTE_LIFT_PICK_1_POS, CRANE_ROUTE_LIFT_SAFE_POS},//1
    {1100.0f, -15.0f, CRANE_ROUTE_LIFT_PICK_2_POS, CRANE_ROUTE_LIFT_SAFE_POS},//2
    {1100.0f, 0.0f, CRANE_ROUTE_LIFT_PICK_3_POS, CRANE_ROUTE_LIFT_SAFE_POS},//3

    {-850.0f, 25.0f, CRANE_ROUTE_LIFT_PLACE_POS, CRANE_ROUTE_LIFT_SAFE_POS},//4
    {-850.0f, 10.0f, CRANE_ROUTE_LIFT_PLACE_POS, CRANE_ROUTE_LIFT_SAFE_POS},//5
    {-850.0f, 2.0f, CRANE_ROUTE_LIFT_PLACE_POS, CRANE_ROUTE_LIFT_SAFE_POS},//6
    {-850.0f, -10.0f, CRANE_ROUTE_LIFT_PLACE_POS, CRANE_ROUTE_LIFT_SAFE_POS},//7
    {-850.0f, -23.0f, CRANE_ROUTE_LIFT_PLACE_POS, CRANE_ROUTE_LIFT_SAFE_POS},//8
};

typedef struct
{
    crane_route_state_e state;
    crane_route_leg_t leg[CRANE_ROUTE_LEG_COUNT];
    uint8_t leg_index;
    uint8_t current_slot;
    uint8_t y_prepare_done;
    uint8_t beam_path_only;
    crane_route_action_e current_action;
    uint32_t dwell_tick;
} crane_route_t;

static crane_route_t crane_route;

static uint8_t crane_route_is_beam_path_only(void)
{
    return (crane_route.beam_path_only != 0U) ? 1U : 0U;
}

static uint8_t crane_route_is_upper_place(uint8_t slot)
{
    return ((slot == 4U) || (slot == 5U)) ? 1U : 0U;
}

static uint8_t crane_route_is_lower_place(uint8_t slot)
{
    return ((slot >= 6U) && (slot <= 8U)) ? 1U : 0U;
}

static uint8_t crane_route_is_upper_extreme_place(uint8_t slot)
{
    return ((slot == 4U) || (slot == 5U)) ? 1U : 0U;
}

static uint8_t crane_route_is_lower_extreme_place(uint8_t slot)
{
    return ((slot == 7U) || (slot == 8U)) ? 1U : 0U;
}

static uint8_t crane_route_is_valid_place(uint8_t slot)
{
    return ((crane_route_is_upper_place(slot) != 0U) ||
            (crane_route_is_lower_place(slot) != 0U)) ? 1U : 0U;
}

static xy_route_type_e crane_route_go_route_for(uint8_t pick_slot, uint8_t place_slot)
{
    if (pick_slot == 3U)
    {
        if (crane_route_is_upper_extreme_place(place_slot) != 0U)
        {
            return XY_ROUTE_DOWN; /* 3 -> 4/5：下绕，入口释放 */
        }

        return XY_ROUTE_UP;       /* 3 -> 6/7/8：上绕，其中 6 出口释放 */
    }

    if (pick_slot == 1U)
    {
        return XY_ROUTE_UP;
    }

    if (pick_slot == 2U)
    {
        return XY_ROUTE_DOWN;
    }

    return XY_ROUTE_DIRECT;
}

static xy_release_mode_e crane_route_go_release_for(uint8_t pick_slot, uint8_t place_slot)
{
    if (pick_slot == 1U)
    {
        return (crane_route_is_lower_extreme_place(place_slot) != 0U) ?
               XY_RELEASE_AFTER_ENTRY : XY_RELEASE_AFTER_EXIT;
    }

    if (pick_slot == 2U)
    {
        return (crane_route_is_upper_extreme_place(place_slot) != 0U) ?
               XY_RELEASE_AFTER_ENTRY : XY_RELEASE_AFTER_EXIT;
    }

    if (pick_slot == 3U)
    {
        if ((crane_route_is_upper_extreme_place(place_slot) != 0U) ||
            (crane_route_is_lower_extreme_place(place_slot) != 0U))
        {
            return XY_RELEASE_AFTER_ENTRY;
        }

        return XY_RELEASE_AFTER_EXIT; /* 3 -> 6：上绕，到出口后释放 */
    }

    return XY_RELEASE_AFTER_EXIT;
}

static xy_route_type_e crane_route_return_route_for(uint8_t pick_slot,
                                                    uint8_t place_slot,
                                                    uint8_t next_pick_slot)
{
    if (next_pick_slot == 0U)
    {
        return (place_slot <= 6U) ? XY_ROUTE_CENTER_DOWN_EXIT : XY_ROUTE_CENTER_UP_EXIT;
    }

    if (pick_slot == 3U)
    {
        return XY_ROUTE_DIRECT; /* 第一趟放完按首个目标区直线回 1 或 2 */
    }

    if (crane_route_is_upper_extreme_place(place_slot) != 0U)
    {
        return (next_pick_slot == 1U) ? XY_ROUTE_DIRECT : XY_ROUTE_DOWN;
    }

    if (crane_route_is_lower_extreme_place(place_slot) != 0U)
    {
        return (next_pick_slot == 2U) ? XY_ROUTE_DIRECT : XY_ROUTE_UP;
    }

    if (place_slot == 6U)
    {
        return (next_pick_slot == 1U) ? XY_ROUTE_UP : XY_ROUTE_DOWN;
    }

    return XY_ROUTE_DIRECT;
}

static xy_release_mode_e crane_route_return_release_for(xy_route_type_e return_route)
{
    if ((return_route == XY_ROUTE_UP) || (return_route == XY_ROUTE_DOWN))
    {
        return XY_RELEASE_AFTER_ENTRY;
    }

    return (return_route == XY_ROUTE_DIRECT) ? XY_RELEASE_AFTER_ENTRY : XY_RELEASE_AFTER_EXIT;
}

static void crane_route_build_plan(void)
{
    uint8_t second_pick;
    uint8_t third_pick;
    uint8_t first_place = CRANE_ROUTE_PLACE_FOR_PICK3;
    uint8_t second_place = CRANE_ROUTE_PLACE_FOR_SECOND;
    uint8_t third_place = CRANE_ROUTE_PLACE_FOR_THIRD;

    if ((crane_route_is_valid_place(first_place) == 0U) ||
        (crane_route_is_valid_place(second_place) == 0U) ||
        (crane_route_is_valid_place(third_place) == 0U))
    {
        first_place = 4U;
        second_place = 5U;
        third_place = 6U;
    }

    if (crane_route_is_upper_place(first_place) != 0U)
    {
        second_pick = 1U;
        third_pick = 2U;
    }
    else
    {
        second_pick = 2U;
        third_pick = 1U;
    }

    crane_route.leg[0].pick_slot = 3U;
    crane_route.leg[0].place_slot = first_place;
    crane_route.leg[0].next_pick_slot = second_pick;

    crane_route.leg[1].pick_slot = second_pick;
    crane_route.leg[1].place_slot = second_place;
    crane_route.leg[1].next_pick_slot = third_pick;

    crane_route.leg[2].pick_slot = third_pick;
    crane_route.leg[2].place_slot = third_place;
    crane_route.leg[2].next_pick_slot = 0U;

    for (uint8_t i = 0U; i < CRANE_ROUTE_LEG_COUNT; i++)
    {
        crane_route.leg[i].go_route =
            crane_route_go_route_for(crane_route.leg[i].pick_slot,
                                     crane_route.leg[i].place_slot);
        crane_route.leg[i].return_route =
            crane_route_return_route_for(crane_route.leg[i].pick_slot,
                                         crane_route.leg[i].place_slot,
                                         crane_route.leg[i].next_pick_slot);
        crane_route.leg[i].go_release =
            crane_route_go_release_for(crane_route.leg[i].pick_slot,
                                       crane_route.leg[i].place_slot);
        crane_route.leg[i].return_release =
            crane_route_return_release_for(crane_route.leg[i].return_route);
    }
}

static crane_route_leg_t *crane_route_current_leg(void)
{
    if (crane_route.leg_index >= CRANE_ROUTE_LEG_COUNT)
    {
        return NULL;
    }

    return &crane_route.leg[crane_route.leg_index];
}

static void crane_route_move_lift_to(float target_pos)
{
    if (crane_route_is_beam_path_only() != 0U)
    {
        (void)target_pos;
        return;
    }

    lift_ctrl_set_target(target_pos);
    lift_ctrl_start();
}

static uint8_t crane_route_lift_is_busy(void)
{
    if (crane_route_is_beam_path_only() != 0U)
    {
        return 0U;
    }

    return lift_ctrl_is_busy();
}

static void crane_route_move_xy_to_slot(uint8_t slot,
                                        xy_route_type_e route_type,
                                        xy_release_mode_e release_mode)
{
    if (slot > CRANE_ROUTE_SLOT_COUNT)
    {
        return;
    }

#if (CRANE_ROUTE_CHASSIS_ONLY != 0U)
    (void)route_type;
    (void)release_mode;
    pos_pid_sync_set_target(crane_route_slot_pose[slot].chassis_pos);
    pos_pid_sync_start();
#elif (CRANE_ROUTE_BEAM_ONLY != 0U)
    (void)route_type;
    (void)release_mode;
    beam_ctrl_set_target(crane_route_slot_pose[slot].beam_pos);
    beam_ctrl_start();
#else
    if (crane_route_is_beam_path_only() != 0U)
    {
        xy_route_start_y_only(crane_route_slot_pose[slot].beam_pos,
                              route_type,
                              release_mode);
    }
    else
    {
        xy_route_start(crane_route_slot_pose[slot].chassis_pos,
                       crane_route_slot_pose[slot].beam_pos,
                       route_type,
                       release_mode);
    }
#endif
}

static uint8_t crane_route_xy_is_busy(void)
{
#if (CRANE_ROUTE_CHASSIS_ONLY != 0U)
    return pos_pid_sync_is_busy();
#elif (CRANE_ROUTE_BEAM_ONLY != 0U)
    return beam_ctrl_is_busy();
#else
    return xy_route_is_busy();
#endif
}

static void crane_route_prepare_y_for_slot(uint8_t slot, xy_route_type_e route_type)
{
#if ((CRANE_ROUTE_CHASSIS_ONLY == 0U) && (CRANE_ROUTE_BEAM_ONLY == 0U))
    if ((crane_route_is_beam_path_only() == 0U) && (slot <= CRANE_ROUTE_SLOT_COUNT))
    {
        xy_route_prepare_y(crane_route_slot_pose[slot].chassis_pos,
                           route_type,
                           crane_route_slot_pose[slot].beam_pos);
    }
#else
    (void)slot;
    (void)route_type;
#endif
}

static void crane_route_try_prepare_y(uint8_t target_slot, xy_route_type_e route_type)
{
    float lift_now;
    float y_premove_pos;

    if (crane_route.y_prepare_done != 0U)
    {
        return;
    }

    if ((crane_route.current_slot == 0U) || (crane_route.current_slot > CRANE_ROUTE_SLOT_COUNT))
    {
        return;
    }

    lift_now = lift_ctrl_get_current_pos();
    y_premove_pos = crane_route_slot_pose[crane_route.current_slot].lift_work_pos +
                    CRANE_ROUTE_Y_PREMOVE_DELTA;

    if (lift_now >= (y_premove_pos - CRANE_ROUTE_Z_GATE_TOL))
    {
        crane_route_prepare_y_for_slot(target_slot, route_type);
        crane_route.y_prepare_done = 1U;
    }
}

static void crane_route_stop_xy(void)
{
#if (CRANE_ROUTE_CHASSIS_ONLY != 0U)
    pos_pid_sync_stop();
#elif (CRANE_ROUTE_BEAM_ONLY != 0U)
    beam_ctrl_stop();
#else
    xy_route_stop();
#endif
}

static void crane_route_set_gripper(crane_route_action_e action)
{
    if (crane_route_is_beam_path_only() != 0U)
    {
        (void)action;
        return;
    }

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
    (void)action;
#endif
}

void crane_route_init(void)
{
    memset(&crane_route, 0, sizeof(crane_route));
    crane_route.state = CRANE_ROUTE_IDLE;
    crane_route.beam_path_only = CRANE_ROUTE_BEAM_PATH_ONLY_DEFAULT;
    crane_route.current_action = CRANE_ROUTE_ACTION_PICK;
}

void crane_route_start(void)
{
    memset(crane_route.leg, 0, sizeof(crane_route.leg));
    crane_route.leg_index = 0U;
    crane_route.current_slot = 0U;
    crane_route.y_prepare_done = 0U;
    crane_route.current_action = CRANE_ROUTE_ACTION_PICK;
    crane_route.dwell_tick = HAL_GetTick();
    crane_route_stop_xy();
    lift_ctrl_stop();
    crane_route.state = CRANE_ROUTE_BUILD_PLAN;
}

void crane_route_stop(void)
{
    crane_route.state = CRANE_ROUTE_IDLE;
    crane_route_stop_xy();
    lift_ctrl_stop();
}

void crane_route_set_beam_path_only(uint8_t enable)
{
    crane_route.beam_path_only = (enable != 0U) ? 1U : 0U;
}

uint8_t crane_route_get_beam_path_only(void)
{
    return crane_route_is_beam_path_only();
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
    float target_x = xy_route_get_target_x();
    float target_y = xy_route_get_target_y();

    if ((target_x == 0.0f) && (target_y == 0.0f) &&
        (slot > 0U) && (slot <= CRANE_ROUTE_SLOT_COUNT))
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
    float target_x = xy_route_get_target_x();
    float target_y = xy_route_get_target_y();
    float target_z = 0.0f;

    if ((target_x == 0.0f) && (target_y == 0.0f) &&
        (slot > 0U) && (slot <= CRANE_ROUTE_SLOT_COUNT))
    {
        target_x = crane_route_slot_pose[slot].chassis_pos;
        target_y = crane_route_slot_pose[slot].beam_pos;
    }

    if ((slot > 0U) && (slot <= CRANE_ROUTE_SLOT_COUNT))
    {
        if ((crane_route.state == CRANE_ROUTE_LIFT_DOWN_PICK) ||
            (crane_route.state == CRANE_ROUTE_WAIT_LIFT_DOWN_PICK) ||
            (crane_route.state == CRANE_ROUTE_GRIPPER_PICK) ||
            (crane_route.state == CRANE_ROUTE_GRIPPER_PICK_HOLD) ||
            (crane_route.state == CRANE_ROUTE_LIFT_DOWN_PLACE) ||
            (crane_route.state == CRANE_ROUTE_WAIT_LIFT_DOWN_PLACE) ||
            (crane_route.state == CRANE_ROUTE_GRIPPER_PLACE) ||
            (crane_route.state == CRANE_ROUTE_GRIPPER_PLACE_HOLD))
        {
            target_z = crane_route_slot_pose[slot].lift_work_pos;
        }
        else
        {
            target_z = crane_route_slot_pose[slot].lift_safe_pos;
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
    crane_route_leg_t *leg = crane_route_current_leg();
    uint8_t return_slot;

    switch (crane_route.state)
    {
        case CRANE_ROUTE_IDLE:
        case CRANE_ROUTE_FINISHED:
            break;

        case CRANE_ROUTE_BUILD_PLAN:
            crane_route_build_plan();
            crane_route.leg_index = 0U;
            crane_route.state = CRANE_ROUTE_MOVE_TO_PICK;
            break;

        case CRANE_ROUTE_MOVE_TO_PICK:
            leg = crane_route_current_leg();
            if (leg == NULL)
            {
                crane_route.state = CRANE_ROUTE_FINISHED;
                break;
            }
            crane_route.current_slot = leg->pick_slot;
            crane_route.current_action = CRANE_ROUTE_ACTION_PICK;
            /* 去取货点时，Z 同步抬到该取货点安全高度；XY 不等待 Z，三轴一起抢时间。 */
            crane_route_move_lift_to(crane_route_slot_pose[leg->pick_slot].lift_safe_pos);
            if (leg->pick_slot == 3U)
            {
                crane_route_move_xy_to_slot(leg->pick_slot,
                                            XY_ROUTE_DOWN,
                                            XY_RELEASE_AFTER_ENTRY);
            }
            else
            {
                crane_route_move_xy_to_slot(leg->pick_slot,
                                            XY_ROUTE_DIRECT,
                                            XY_RELEASE_AFTER_EXIT);
            }
            crane_route.state = CRANE_ROUTE_WAIT_PICK_XY;
            break;

        case CRANE_ROUTE_WAIT_PICK_XY:
            if (crane_route_xy_is_busy() == 0U)
            {
                crane_route.state = CRANE_ROUTE_LIFT_DOWN_PICK;
            }
            break;

        case CRANE_ROUTE_LIFT_DOWN_PICK:
            crane_route_move_lift_to(crane_route_slot_pose[crane_route.current_slot].lift_work_pos);
            crane_route.state = CRANE_ROUTE_WAIT_LIFT_DOWN_PICK;
            break;

        case CRANE_ROUTE_WAIT_LIFT_DOWN_PICK:
            if (crane_route_lift_is_busy() == 0U)
            {
                crane_route.state = CRANE_ROUTE_GRIPPER_PICK;
            }
            break;

        case CRANE_ROUTE_GRIPPER_PICK:
            crane_route.current_action = CRANE_ROUTE_ACTION_PICK;
            crane_route_set_gripper(CRANE_ROUTE_ACTION_PICK);
            crane_route.dwell_tick = HAL_GetTick();
            crane_route.state = CRANE_ROUTE_GRIPPER_PICK_HOLD;
            break;

        case CRANE_ROUTE_GRIPPER_PICK_HOLD:
            if ((crane_route_is_beam_path_only() != 0U) ||
                ((HAL_GetTick() - crane_route.dwell_tick) >= CRANE_ROUTE_PICK_DWELL_MS))
            {
                crane_route.state = CRANE_ROUTE_LIFT_UP_AFTER_PICK;
            }
            break;

        case CRANE_ROUTE_LIFT_UP_AFTER_PICK:
            leg = crane_route_current_leg();
            if (leg == NULL)
            {
                crane_route.state = CRANE_ROUTE_FINISHED;
                break;
            }
            crane_route.y_prepare_done = 0U;
            crane_route_move_lift_to(crane_route_slot_pose[crane_route.current_slot].lift_safe_pos);
            crane_route.state = CRANE_ROUTE_WAIT_LIFT_UP_AFTER_PICK;
            break;

        case CRANE_ROUTE_WAIT_LIFT_UP_AFTER_PICK:
            leg = crane_route_current_leg();
            if (leg == NULL)
            {
                crane_route.state = CRANE_ROUTE_FINISHED;
                break;
            }
            crane_route_try_prepare_y(leg->place_slot, leg->go_route);
            if (crane_route_lift_is_busy() == 0U)
            {
                if (crane_route.y_prepare_done == 0U)
                {
                    crane_route_prepare_y_for_slot(leg->place_slot, leg->go_route);
                    crane_route.y_prepare_done = 1U;
                }
                crane_route.state = CRANE_ROUTE_MOVE_TO_PLACE;
            }
            break;

        case CRANE_ROUTE_MOVE_TO_PLACE:
            leg = crane_route_current_leg();
            if (leg == NULL)
            {
                crane_route.state = CRANE_ROUTE_FINISHED;
                break;
            }
            crane_route.current_slot = leg->place_slot;
            crane_route.current_action = CRANE_ROUTE_ACTION_PLACE;
            crane_route_move_xy_to_slot(leg->place_slot,
                                        leg->go_route,
                                        leg->go_release);
            crane_route.state = CRANE_ROUTE_WAIT_PLACE_XY;
            break;

        case CRANE_ROUTE_WAIT_PLACE_XY:
            if (crane_route_xy_is_busy() == 0U)
            {
                crane_route.state = CRANE_ROUTE_LIFT_DOWN_PLACE;
            }
            break;

        case CRANE_ROUTE_LIFT_DOWN_PLACE:
            crane_route_move_lift_to(crane_route_slot_pose[crane_route.current_slot].lift_work_pos);
            crane_route.state = CRANE_ROUTE_WAIT_LIFT_DOWN_PLACE;
            break;

        case CRANE_ROUTE_WAIT_LIFT_DOWN_PLACE:
            if (crane_route_lift_is_busy() == 0U)
            {
                crane_route.state = CRANE_ROUTE_GRIPPER_PLACE;
            }
            break;

        case CRANE_ROUTE_GRIPPER_PLACE:
            crane_route.current_action = CRANE_ROUTE_ACTION_PLACE;
            crane_route_set_gripper(CRANE_ROUTE_ACTION_PLACE);
            crane_route.dwell_tick = HAL_GetTick();
            crane_route.state = CRANE_ROUTE_GRIPPER_PLACE_HOLD;
            break;

        case CRANE_ROUTE_GRIPPER_PLACE_HOLD:
            if ((crane_route_is_beam_path_only() != 0U) ||
                ((HAL_GetTick() - crane_route.dwell_tick) >= CRANE_ROUTE_PICK_DWELL_MS))
            {
                crane_route.state = CRANE_ROUTE_LIFT_UP_AFTER_PLACE;
            }
            break;

        case CRANE_ROUTE_LIFT_UP_AFTER_PLACE:
            leg = crane_route_current_leg();
            if (leg == NULL)
            {
                crane_route.state = CRANE_ROUTE_FINISHED;
                break;
            }
            crane_route.y_prepare_done = 0U;
            crane_route_move_lift_to(crane_route_slot_pose[crane_route.current_slot].lift_safe_pos);
            crane_route.state = CRANE_ROUTE_WAIT_LIFT_UP_AFTER_PLACE;
            break;

        case CRANE_ROUTE_WAIT_LIFT_UP_AFTER_PLACE:
            leg = crane_route_current_leg();
            if (leg == NULL)
            {
                crane_route.state = CRANE_ROUTE_FINISHED;
                break;
            }
            return_slot = leg->next_pick_slot;
            crane_route_try_prepare_y(return_slot, leg->return_route);
            if (crane_route_lift_is_busy() == 0U)
            {
                if (crane_route.y_prepare_done == 0U)
                {
                    crane_route_prepare_y_for_slot(return_slot, leg->return_route);
                    crane_route.y_prepare_done = 1U;
                }
                crane_route.state = CRANE_ROUTE_RETURN_AFTER_PLACE;
            }
            break;

        case CRANE_ROUTE_RETURN_AFTER_PLACE:
            leg = crane_route_current_leg();
            if (leg == NULL)
            {
                crane_route.state = CRANE_ROUTE_FINISHED;
                break;
            }
            return_slot = leg->next_pick_slot;
            crane_route.current_slot = return_slot;
            crane_route_move_xy_to_slot(return_slot,
                                        leg->return_route,
                                        leg->return_release);
            crane_route.state = CRANE_ROUTE_WAIT_RETURN_XY;
            break;

        case CRANE_ROUTE_WAIT_RETURN_XY:
            if (crane_route_xy_is_busy() == 0U)
            {
                crane_route.dwell_tick = HAL_GetTick();
                crane_route.state = CRANE_ROUTE_LEG_DWELL;
            }
            break;

        case CRANE_ROUTE_LEG_DWELL:
            if ((HAL_GetTick() - crane_route.dwell_tick) >= CRANE_ROUTE_LEG_DWELL_MS)
            {
                crane_route.leg_index++;
                if (crane_route.leg_index >= CRANE_ROUTE_LEG_COUNT)
                {
                    crane_route.state = CRANE_ROUTE_FINISHED;
                }
                else
                {
                    crane_route.state = CRANE_ROUTE_MOVE_TO_PICK;
                }
            }
            break;

        default:
            crane_route.state = CRANE_ROUTE_IDLE;
            break;
    }
}
