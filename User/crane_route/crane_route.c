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
#define CRANE_ROUTE_PLACE_FOR_PICK3     7U
#define CRANE_ROUTE_PLACE_FOR_SECOND    4U
#define CRANE_ROUTE_PLACE_FOR_THIRD     8U
#define CRANE_ROUTE_PICK_COUNT          3U
#define CRANE_ROUTE_PLACE_COUNT         5U
#define CRANE_ROUTE_GOODS_GREEN         6U
#define CRANE_ROUTE_GOODS_WHITE_BEAN    7U
#define CRANE_ROUTE_GOODS_SOYBEAN       8U

#define CRANE_ROUTE_LIFT_PICK_1_POS     4.5f
#define CRANE_ROUTE_LIFT_PICK_2_POS     4.5f
#define CRANE_ROUTE_LIFT_PICK_3_POS     6.3f
#define CRANE_ROUTE_LIFT_PLACE_POS      1.5f
#define CRANE_ROUTE_LIFT_SAFE_POS       12.0f
#define CRANE_ROUTE_LIFT_APPROACH_CLEARANCE 4.0f
#define CRANE_ROUTE_LIFT_EXTREME_CLEARANCE  6.0f
#define CRANE_ROUTE_LEG_DWELL_MS        300U

/* Z 从工作低位抬高 2 后，Y 可以提前去绕行入口；X 仍等 Z 到安全位后再跑。 */
#define CRANE_ROUTE_Y_PREMOVE_DELTA     2.8f
#define CRANE_ROUTE_Z_GATE_TOL          1.0f

/* X 过第一个障碍点后，Z 可以提前下放到目标上方等待。 */
#define CRANE_ROUTE_X_GO_PREDROP_GATE      -750.0f
#define CRANE_ROUTE_X_RETURN_PREDROP_GATE   500.0f

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
    {-1285.0f, -15.5f, CRANE_ROUTE_LIFT_PICK_1_POS, CRANE_ROUTE_LIFT_SAFE_POS},//1
    {-1285.0f, 16.7f, CRANE_ROUTE_LIFT_PICK_2_POS, CRANE_ROUTE_LIFT_SAFE_POS},//2
    {-1130.0f, 1.0f, CRANE_ROUTE_LIFT_PICK_3_POS, CRANE_ROUTE_LIFT_SAFE_POS},//3

    {1000.0f, -24.6f, CRANE_ROUTE_LIFT_PLACE_POS, CRANE_ROUTE_LIFT_SAFE_POS},//4
    {1055.0f, -12.5f, CRANE_ROUTE_LIFT_PLACE_POS, CRANE_ROUTE_LIFT_SAFE_POS},//5
    {1055.0f, -1.0f, CRANE_ROUTE_LIFT_PLACE_POS, CRANE_ROUTE_LIFT_SAFE_POS},//6
    {1055.0f, 14.0f, CRANE_ROUTE_LIFT_PLACE_POS, CRANE_ROUTE_LIFT_SAFE_POS},//7
    {1000.0f, 25.5f, CRANE_ROUTE_LIFT_PLACE_POS, CRANE_ROUTE_LIFT_SAFE_POS},//8
};

#if (CRANE_ROUTE_CHASSIS_ONLY != 0U)
#define CRANE_ROUTE_CHASSIS_DEBUG_DWELL_MS 1000U

static const uint8_t crane_route_chassis_debug_slots[] =
{
    1U,
    6U,
    3U,
    8U,
    2U,
    7U,
    0U
};

static uint8_t crane_route_chassis_debug_index;
#define CRANE_ROUTE_CHASSIS_DEBUG_COUNT \
    ((uint8_t)(sizeof(crane_route_chassis_debug_slots) / sizeof(crane_route_chassis_debug_slots[0])))
#endif

typedef struct
{
    crane_route_state_e state;
    crane_route_leg_t leg[CRANE_ROUTE_LEG_COUNT];
    uint8_t leg_index;
    uint8_t current_slot;
    uint8_t y_prepare_done;
    uint8_t z_predrop_done;
    uint8_t beam_path_only;
    uint8_t extreme_return_slot;
    uint8_t pick_goods[CRANE_ROUTE_PICK_COUNT];
    uint8_t place_boxes[CRANE_ROUTE_PLACE_COUNT];
    uint8_t draw_valid;
    crane_route_action_e current_action;
    uint32_t dwell_tick;
} crane_route_t;

static crane_route_t crane_route;

#if (CRANE_ROUTE_CHASSIS_ONLY != 0U)
static void crane_route_chassis_debug_run_current(void)
{
    uint8_t slot;

    if (crane_route_chassis_debug_index >= CRANE_ROUTE_CHASSIS_DEBUG_COUNT)
    {
        crane_route.state = CRANE_ROUTE_FINISHED;
        pos_pid_sync_stop();
        return;
    }

    slot = crane_route_chassis_debug_slots[crane_route_chassis_debug_index];
    if (slot > CRANE_ROUTE_SLOT_COUNT)
    {
        crane_route.state = CRANE_ROUTE_FINISHED;
        pos_pid_sync_stop();
        return;
    }

    crane_route.current_slot = slot;
    crane_route.state = CRANE_ROUTE_MOVE_TO_PICK;
    pos_pid_sync_set_target(crane_route_slot_pose[slot].chassis_pos);
    pos_pid_sync_start();
}

static void crane_route_chassis_debug_process(void)
{
    if ((crane_route.state == CRANE_ROUTE_IDLE) ||
        (crane_route.state == CRANE_ROUTE_FINISHED))
    {
        return;
    }

    if (crane_route.state == CRANE_ROUTE_LEG_DWELL)
    {
        if ((HAL_GetTick() - crane_route.dwell_tick) < CRANE_ROUTE_CHASSIS_DEBUG_DWELL_MS)
        {
            return;
        }

        crane_route_chassis_debug_index++;
        crane_route_chassis_debug_run_current();
        return;
    }

    if (pos_pid_sync_is_busy() != 0U)
    {
        return;
    }

    crane_route.dwell_tick = HAL_GetTick();
    crane_route.state = CRANE_ROUTE_LEG_DWELL;
}
#endif

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

static uint8_t crane_route_is_valid_goods(uint8_t goods)
{
    return ((goods == CRANE_ROUTE_GOODS_GREEN) ||
            (goods == CRANE_ROUTE_GOODS_WHITE_BEAN) ||
            (goods == CRANE_ROUTE_GOODS_SOYBEAN)) ? 1U : 0U;
}

static uint8_t crane_route_target_box_for_goods(uint8_t goods)
{
    if (goods == CRANE_ROUTE_GOODS_SOYBEAN)
    {
        return 1U;
    }

    if (goods == CRANE_ROUTE_GOODS_GREEN)
    {
        return 2U;
    }

    if (goods == CRANE_ROUTE_GOODS_WHITE_BEAN)
    {
        return 3U;
    }

    return 0U;
}

static uint8_t crane_route_place_slot_for_box(uint8_t box)
{
    for (uint8_t i = 0U; i < CRANE_ROUTE_PLACE_COUNT; i++)
    {
        if (crane_route.place_boxes[i] == box)
        {
            return (uint8_t)(i + 4U);
        }
    }

    return 0U;
}

static uint8_t crane_route_place_for_pick(uint8_t pick_slot)
{
    uint8_t target_box;

    if ((pick_slot == 0U) || (pick_slot > CRANE_ROUTE_PICK_COUNT))
    {
        return 0U;
    }

    target_box = crane_route_target_box_for_goods(crane_route.pick_goods[pick_slot - 1U]);
    if (target_box == 0U)
    {
        return 0U;
    }

    return crane_route_place_slot_for_box(target_box);
}

static uint8_t crane_route_check_unique_range(const uint8_t *values,
                                              uint8_t count,
                                              uint8_t min_value,
                                              uint8_t max_value)
{
    for (uint8_t i = 0U; i < count; i++)
    {
        if ((values[i] < min_value) || (values[i] > max_value))
        {
            return 0U;
        }

        for (uint8_t j = (uint8_t)(i + 1U); j < count; j++)
        {
            if (values[i] == values[j])
            {
                return 0U;
            }
        }
    }

    return 1U;
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

    if (crane_route.draw_valid != 0U)
    {
        first_place = crane_route_place_for_pick(3U);
    }

    if (crane_route_is_valid_place(first_place) == 0U)
    {
        first_place = 4U;
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

    if (crane_route.draw_valid != 0U)
    {
        second_place = crane_route_place_for_pick(second_pick);
        third_place = crane_route_place_for_pick(third_pick);
    }

    if ((crane_route_is_valid_place(second_place) == 0U) ||
        (crane_route_is_valid_place(third_place) == 0U))
    {
        second_place = 5U;
        third_place = 6U;
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

static float crane_route_lift_approach_pos(uint8_t slot)
{
    float approach_pos;
    float clearance;

    if ((slot == 0U) || (slot > CRANE_ROUTE_SLOT_COUNT))
    {
        return CRANE_ROUTE_LIFT_SAFE_POS;
    }

    clearance = ((slot == 4U) || (slot == 8U)) ?
                CRANE_ROUTE_LIFT_EXTREME_CLEARANCE :
                CRANE_ROUTE_LIFT_APPROACH_CLEARANCE;
    approach_pos = crane_route_slot_pose[slot].lift_work_pos +
                   clearance;

    if (approach_pos > crane_route_slot_pose[slot].lift_safe_pos)
    {
        approach_pos = crane_route_slot_pose[slot].lift_safe_pos;
    }

    return approach_pos;
}

static void crane_route_move_lift_to_approach(uint8_t slot)
{
    crane_route_move_lift_to(crane_route_lift_approach_pos(slot));
}

static void crane_route_try_predrop_lift_after_x_gate(uint8_t target_slot,
                                                      uint8_t moving_to_place)
{
#if ((CRANE_ROUTE_CHASSIS_ONLY == 0U) && (CRANE_ROUTE_BEAM_ONLY == 0U) && (CRANE_ROUTE_NO_CHASSIS == 0U) && (CRANE_ROUTE_LIFT_ONLY == 0U))
    float x_now;

    if ((crane_route.z_predrop_done != 0U) ||
        (target_slot == 0U) ||
        (target_slot > CRANE_ROUTE_SLOT_COUNT) ||
        (crane_route_is_beam_path_only() != 0U))
    {
        return;
    }

    x_now = pos_pid_sync_get_current_pos();
    if (((moving_to_place != 0U) && (x_now >= CRANE_ROUTE_X_GO_PREDROP_GATE)) ||
        ((moving_to_place == 0U) && (x_now <= CRANE_ROUTE_X_RETURN_PREDROP_GATE)))
    {
        crane_route_move_lift_to_approach(target_slot);
        crane_route.z_predrop_done = 1U;
    }
#else
    (void)target_slot;
    (void)moving_to_place;
#endif
}

static uint8_t crane_route_lift_is_busy(void)
{
    if (crane_route_is_beam_path_only() != 0U)
    {
        return 0U;
    }

    return lift_ctrl_is_busy();
}

static void crane_route_config_servo3_for_xy(uint8_t slot,
                                             xy_route_type_e route_type,
                                             xy_release_mode_e release_mode)
{
    float target_angle = servo3_path_angle_for_slot(slot);
    uint8_t entry_enable = 0U;
    uint8_t exit_enable = 0U;

    if (crane_route_is_beam_path_only() != 0U)
    {
        return;
    }

    if ((route_type == XY_ROUTE_UP) || (route_type == XY_ROUTE_DOWN))
    {
        (void)release_mode;
        entry_enable = 1U;
    }
    else if ((route_type == XY_ROUTE_CENTER_UP_EXIT) ||
             (route_type == XY_ROUTE_CENTER_DOWN_EXIT))
    {
        exit_enable = 1U;
    }
    else
    {
        servo3_path_release_angle(target_angle);
    }

    xy_route_set_servo3_triggers(entry_enable, target_angle,
                                 exit_enable, target_angle);

    if ((crane_route.current_action == CRANE_ROUTE_ACTION_PLACE) &&
        (servo3_path_is_extreme_slot(slot) != 0U))
    {
        xy_route_set_servo3_target_wait(1U,
                                        servo3_path_safe_y_for_slot(slot),
                                        target_angle);
    }
    else
    {
        xy_route_set_servo3_target_wait(0U, 0.0f, target_angle);
    }
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
#elif (CRANE_ROUTE_NO_CHASSIS != 0U)
    crane_route_config_servo3_for_xy(slot, route_type, release_mode);
    xy_route_start_y_only(crane_route_slot_pose[slot].beam_pos,
                          route_type,
                          release_mode);
#elif (CRANE_ROUTE_LIFT_ONLY != 0U)
    (void)route_type;
    (void)release_mode;
#else
    crane_route_config_servo3_for_xy(slot, route_type, release_mode);

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

static void crane_route_move_xy_after_extreme_place(uint8_t slot,
                                                     xy_route_type_e route_type,
                                                     xy_release_mode_e release_mode,
                                                     uint8_t extreme_slot)
{
#if ((CRANE_ROUTE_CHASSIS_ONLY == 0U) && (CRANE_ROUTE_BEAM_ONLY == 0U) && (CRANE_ROUTE_NO_CHASSIS == 0U) && (CRANE_ROUTE_LIFT_ONLY == 0U))
    if (crane_route_is_beam_path_only() == 0U)
    {
        xy_route_start_extreme_return(crane_route_slot_pose[slot].chassis_pos,
                                      crane_route_slot_pose[slot].beam_pos,
                                      route_type,
                                      release_mode,
                                      servo3_path_safe_y_for_slot(extreme_slot),
                                      servo3_path_angle_for_slot(slot));
        return;
    }
#else
    (void)extreme_slot;
#endif

    /* 单轴调试模式保持原有行为。 */
    crane_route_move_xy_to_slot(slot, route_type, release_mode);
}

static uint8_t crane_route_xy_is_busy(void)
{
#if (CRANE_ROUTE_CHASSIS_ONLY != 0U)
    return pos_pid_sync_is_busy();
#elif (CRANE_ROUTE_BEAM_ONLY != 0U)
    return beam_ctrl_is_busy();
#elif (CRANE_ROUTE_NO_CHASSIS != 0U)
    return xy_route_is_busy();
#elif (CRANE_ROUTE_LIFT_ONLY != 0U)
    return 0U;
#else
    return xy_route_is_busy();
#endif
}

static void crane_route_prepare_y_for_slot(uint8_t slot, xy_route_type_e route_type)
{
#if ((CRANE_ROUTE_CHASSIS_ONLY == 0U) && (CRANE_ROUTE_BEAM_ONLY == 0U) && (CRANE_ROUTE_NO_CHASSIS == 0U) && (CRANE_ROUTE_LIFT_ONLY == 0U))
    if ((crane_route_is_beam_path_only() == 0U) && (slot <= CRANE_ROUTE_SLOT_COUNT))
    {
        xy_route_prepare_y(crane_route_slot_pose[slot].chassis_pos,
                           route_type,
                           crane_route_slot_pose[slot].beam_pos);
    }
#elif (CRANE_ROUTE_NO_CHASSIS != 0U)
    if (slot <= CRANE_ROUTE_SLOT_COUNT)
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

static void crane_route_try_prepare_extreme_safe_y(uint8_t extreme_slot)
{
    float lift_now;
    float y_premove_pos;

    if (crane_route.y_prepare_done != 0U)
    {
        return;
    }

    lift_now = lift_ctrl_get_current_pos();
    y_premove_pos = crane_route_slot_pose[extreme_slot].lift_work_pos +
                    CRANE_ROUTE_Y_PREMOVE_DELTA;

    if (lift_now >= (y_premove_pos - CRANE_ROUTE_Z_GATE_TOL))
    {
        beam_ctrl_set_target(servo3_path_safe_y_for_slot(extreme_slot));
        beam_ctrl_start();
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
        claw_close_pick();
    }
    else
    {
        claw_open();
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

uint8_t crane_route_set_draw_result(const uint8_t pick_goods[3], const uint8_t place_boxes[5])
{
    if ((pick_goods == NULL) || (place_boxes == NULL))
    {
        return 0U;
    }

    if (crane_route_check_unique_range(pick_goods,
                                       CRANE_ROUTE_PICK_COUNT,
                                       CRANE_ROUTE_GOODS_GREEN,
                                       CRANE_ROUTE_GOODS_SOYBEAN) == 0U)
    {
        return 0U;
    }

    for (uint8_t i = 0U; i < CRANE_ROUTE_PICK_COUNT; i++)
    {
        if (crane_route_is_valid_goods(pick_goods[i]) == 0U)
        {
            return 0U;
        }
    }

    if (crane_route_check_unique_range(place_boxes,
                                       CRANE_ROUTE_PLACE_COUNT,
                                       1U,
                                       CRANE_ROUTE_PLACE_COUNT) == 0U)
    {
        return 0U;
    }

    (void)memcpy(crane_route.pick_goods, pick_goods, sizeof(crane_route.pick_goods));
    (void)memcpy(crane_route.place_boxes, place_boxes, sizeof(crane_route.place_boxes));
    crane_route.draw_valid = 1U;

    return 1U;
}

void crane_route_start(void)
{
    memset(crane_route.leg, 0, sizeof(crane_route.leg));
    crane_route.leg_index = 0U;
    crane_route.current_slot = 0U;
    crane_route.y_prepare_done = 0U;
    crane_route.z_predrop_done = 0U;
    crane_route.extreme_return_slot = 0U;
    crane_route.current_action = CRANE_ROUTE_ACTION_PICK;
    crane_route.dwell_tick = HAL_GetTick();
    crane_route_stop_xy();
#if (CRANE_ROUTE_CHASSIS_ONLY != 0U)
    crane_route_chassis_debug_index = 0U;
    crane_route_chassis_debug_run_current();
#else
    lift_ctrl_stop();
    claw_close();
    crane_route.state = CRANE_ROUTE_BUILD_PLAN;
#endif
}

void crane_route_stop(void)
{
    crane_route.state = CRANE_ROUTE_IDLE;
    crane_route_stop_xy();
#if (CRANE_ROUTE_CHASSIS_ONLY != 0U)
    crane_route_chassis_debug_index = 0U;
#else
    lift_ctrl_stop();
#endif
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
#if (CRANE_ROUTE_CHASSIS_ONLY != 0U)
    float target_x = (slot <= CRANE_ROUTE_SLOT_COUNT) ? crane_route_slot_pose[slot].chassis_pos : 0.0f;
    float target_y = 0.0f;
#else
    float target_x = xy_route_get_target_x();
    float target_y = xy_route_get_target_y();

    if ((target_x == 0.0f) && (target_y == 0.0f) &&
        (slot > 0U) && (slot <= CRANE_ROUTE_SLOT_COUNT))
    {
        target_x = crane_route_slot_pose[slot].chassis_pos;
        target_y = crane_route_slot_pose[slot].beam_pos;
    }
#endif

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
#if (CRANE_ROUTE_CHASSIS_ONLY != 0U)
    float target_x = (slot <= CRANE_ROUTE_SLOT_COUNT) ? crane_route_slot_pose[slot].chassis_pos : 0.0f;
    float target_y = 0.0f;
    float target_z = 0.0f;
#else
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
        else if (((crane_route.state == CRANE_ROUTE_WAIT_PLACE_XY) ||
                  (crane_route.state == CRANE_ROUTE_WAIT_RETURN_XY)) &&
                 (crane_route.z_predrop_done != 0U))
        {
            target_z = crane_route_lift_approach_pos(slot);
        }
        else
        {
            target_z = crane_route_slot_pose[slot].lift_safe_pos;
        }
    }
#endif

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
#if (CRANE_ROUTE_CHASSIS_ONLY != 0U)
    crane_route_chassis_debug_process();
    return;
#else
    crane_route_leg_t *leg = crane_route_current_leg();
    uint8_t return_slot;
    uint8_t already_at_pick;

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
            already_at_pick = (crane_route.current_slot == leg->pick_slot) ? 1U : 0U;
            crane_route.current_slot = leg->pick_slot;
            crane_route.current_action = CRANE_ROUTE_ACTION_PICK;
            if (already_at_pick == 0U)
            {
                if ((crane_route.leg_index == 0U) && (leg->pick_slot == 3U))
                {
                    crane_route_move_lift_to_approach(leg->pick_slot);
                }
                else
                {
                    crane_route_move_lift_to(crane_route_slot_pose[leg->pick_slot].lift_safe_pos);
                }
            }
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
            claw_open();
            crane_route_move_lift_to(crane_route_slot_pose[crane_route.current_slot].lift_work_pos);
            crane_route.state = CRANE_ROUTE_WAIT_LIFT_DOWN_PICK;
            break;

        case CRANE_ROUTE_WAIT_LIFT_DOWN_PICK:
            if ((crane_route_lift_is_busy() == 0U) &&
                (claw_is_busy() == 0U))
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
                (claw_is_busy() == 0U))
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
            crane_route.z_predrop_done = 0U;
            crane_route_move_xy_to_slot(leg->place_slot,
                                        leg->go_route,
                                        leg->go_release);
            crane_route.state = CRANE_ROUTE_WAIT_PLACE_XY;
            break;

        case CRANE_ROUTE_WAIT_PLACE_XY:
            crane_route_try_predrop_lift_after_x_gate(crane_route.current_slot, 1U);
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
                (claw_is_busy() == 0U))
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
            if (servo3_path_is_extreme_slot(leg->place_slot) != 0U)
            {
                /* 4/8：Z 后半程与 Y 退出到旋转安全位并行。 */
                crane_route_try_prepare_extreme_safe_y(leg->place_slot);
            }
            else
            {
                crane_route_try_prepare_y(return_slot, leg->return_route);
            }
            if (crane_route_lift_is_busy() == 0U)
            {
                claw_close();
                if ((crane_route.y_prepare_done == 0U) &&
                    (servo3_path_is_extreme_slot(leg->place_slot) == 0U))
                {
                    crane_route_prepare_y_for_slot(return_slot, leg->return_route);
                    crane_route.y_prepare_done = 1U;
                }
                if (servo3_path_is_extreme_slot(leg->place_slot) != 0U)
                {
                    crane_route.extreme_return_slot = leg->place_slot;
                    /* 若 Z 先到，继续等待 Y 到极限位的旋转安全点。 */
                    if (beam_ctrl_is_busy() == 0U)
                    {
                        crane_route.state = CRANE_ROUTE_RETURN_AFTER_PLACE;
                    }
                }
                else
                {
                    crane_route.state = CRANE_ROUTE_RETURN_AFTER_PLACE;
                }
            }
            break;

        case CRANE_ROUTE_MOVE_EXTREME_SAFE_Y:
            if (crane_route_is_beam_path_only() != 0U)
            {
                crane_route.state = CRANE_ROUTE_SERVO_RETURN_PICK;
                break;
            }
            beam_ctrl_set_target(servo3_path_safe_y_for_slot(crane_route.extreme_return_slot));
            beam_ctrl_start();
            crane_route.state = CRANE_ROUTE_WAIT_EXTREME_SAFE_Y;
            break;

        case CRANE_ROUTE_WAIT_EXTREME_SAFE_Y:
            if ((crane_route_is_beam_path_only() != 0U) ||
                (beam_ctrl_is_busy() == 0U))
            {
                crane_route.state = CRANE_ROUTE_SERVO_RETURN_PICK;
            }
            break;

        case CRANE_ROUTE_SERVO_RETURN_PICK:
            servo3_path_release_pick_area();
            crane_route.state = CRANE_ROUTE_WAIT_SERVO_RETURN_PICK;
            break;

        case CRANE_ROUTE_WAIT_SERVO_RETURN_PICK:
            if (servo3_path_is_arrived() != 0U)
            {
                crane_route.extreme_return_slot = 0U;
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
            crane_route.z_predrop_done = 0U;
            if (crane_route.extreme_return_slot != 0U)
            {
                crane_route_move_xy_after_extreme_place(return_slot,
                                                        leg->return_route,
                                                        leg->return_release,
                                                        crane_route.extreme_return_slot);
                crane_route.extreme_return_slot = 0U;
            }
            else
            {
                crane_route_move_xy_to_slot(return_slot,
                                            leg->return_route,
                                            leg->return_release);
            }
            crane_route.state = CRANE_ROUTE_WAIT_RETURN_XY;
            break;

        case CRANE_ROUTE_WAIT_RETURN_XY:
            crane_route_try_predrop_lift_after_x_gate(crane_route.current_slot, 0U);
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
#endif
}
