#include "headfile.h"

/* 豆子编号与视觉箱号的既有对应关系。 */
#define CRANE_ROUTE_GOODS_GREEN               6U
#define CRANE_ROUTE_GOODS_WHITE_BEAN          7U
#define CRANE_ROUTE_GOODS_SOYBEAN             8U

/* Z 轴仍使用当前达妙 2325；以下均为现有位置单位。 */
#define CRANE_ROUTE_LIFT_PICK_1_POS           7.7f
#define CRANE_ROUTE_LIFT_PICK_2_POS           4.2f
#define CRANE_ROUTE_LIFT_PICK_3_POS          10.5f
#define CRANE_ROUTE_LIFT_PLACE_POS            2.5f
#define CRANE_ROUTE_PICK_APPROACH_CLEARANCE   8.0f
#define CRANE_ROUTE_PICK_BOX_CLEARANCE        5.8f
#define CRANE_ROUTE_Z_TOL                      1.0f

/*
 * 新统一上绕路径的三个 X 门槛。数值先继承旧坐标系作为首版占位，
 * 实车必须分别标定“障碍1前50 / 障碍2前100 / 障碍2后50”。
 */
#define CRANE_ROUTE_X_OBS1_PRE_50           -750.0f
#define CRANE_ROUTE_X_OBS2_PRE_100           500.0f
#define CRANE_ROUTE_X_OBS2_POST_50           750.0f

/* 两类放置 X：4/8 先在外侧站释放，其余在内侧站释放。 */
#define CRANE_ROUTE_X_EXTREME_STATION       1015.0f
#define CRANE_ROUTE_X_REMAINING_STATION     1103.5f

/*
 * 三套 Y 坐标必须独立标定。斗子上电时放在两侧机械安全位并保存零点，
 * 所以第一版斗子停放位为 0；其余数值只用于验证状态机和低速空载联调。
 */
#define CRANE_ROUTE_UPPER_HOPPER_STOW_Y        0.0f
#define CRANE_ROUTE_LOWER_HOPPER_STOW_Y        0.0f
#define CRANE_ROUTE_CLAW_LOAD_UPPER_Y         -25.0f
#define CRANE_ROUTE_CLAW_LOAD_LOWER_Y          25.0f

#define CRANE_ROUTE_CLAW_UPPER_SAFE_Y          -8.0f
#define CRANE_ROUTE_UPPER_UPPER_SAFE_Y         -8.0f
#define CRANE_ROUTE_LOWER_UPPER_SAFE_Y         -8.0f
#define CRANE_ROUTE_CLAW_LOWER_TRANS_Y           8.0f
#define CRANE_ROUTE_UPPER_LOWER_TRANS_Y          8.0f
#define CRANE_ROUTE_LOWER_LOWER_TRANS_Y          8.0f

#define CRANE_ROUTE_FEEDBACK_TIMEOUT_MS        250U
#define CRANE_ROUTE_SERVO_TIMEOUT_MS         10000U
#define CRANE_ROUTE_MOTION_TIMEOUT_MS       120000U
#define CRANE_ROUTE_GATE_TIMEOUT_MS          10000U
#define CRANE_ROUTE_GATE_RELEASE_HOLD_MS       500U

#define CRANE_RELEASE_UPPER_MASK              0x01U
#define CRANE_RELEASE_CLAW_MASK               0x02U
#define CRANE_RELEASE_LOWER_MASK              0x04U
#define CRANE_RELEASE_ALL_MASK                0x07U

typedef struct
{
    uint8_t pick_slot;
    uint8_t place_slot;
} crane_load_plan_t;

typedef struct
{
    crane_load_plan_t upper;
    crane_load_plan_t claw;
    crane_load_plan_t lower;
    uint8_t pick_order[CRANE_ROUTE_PICK_COUNT];
} crane_task_plan_t;

typedef struct
{
    crane_route_state_e state;
    crane_route_fault_e fault;
    crane_task_plan_t plan;
    uint8_t pick_goods[CRANE_ROUTE_PICK_COUNT];
    uint8_t place_boxes[CRANE_ROUTE_PLACE_COUNT];
    uint8_t draw_valid;
    uint8_t beam_path_only;
    uint8_t pick_index;
    uint8_t current_slot;
    uint8_t next_x_started;
    uint8_t release_mask;
    uint8_t gate_cycle_mask;
    uint8_t gate_open_seen_mask;
    uint8_t gate_close_commanded_mask;
    uint32_t upper_gate_open_tick;
    uint32_t lower_gate_open_tick;
    uint32_t state_tick;
    float target_x;
    float target_claw_y;
    float target_upper_y;
    float target_lower_y;
    float target_z;
} crane_route_t;

static crane_slot_pose_t crane_route_slot_pose[CRANE_ROUTE_SLOT_COUNT + 1U] =
{
    {0.0f, 0.0f, 0.0f, 0.0f},
    {-1265.0f, -15.95f, CRANE_ROUTE_LIFT_PICK_1_POS, CRANE_ROUTE_LIFT_SAFE_POS},
    {-1265.0f,  16.25f, CRANE_ROUTE_LIFT_PICK_2_POS, CRANE_ROUTE_LIFT_SAFE_POS},
    {-1091.2f,    0.40f, CRANE_ROUTE_LIFT_PICK_3_POS, CRANE_ROUTE_LIFT_SAFE_POS},
    { 1015.0f,  -24.10f, CRANE_ROUTE_LIFT_PLACE_POS, CRANE_ROUTE_LIFT_SAFE_POS},
    { 1103.5f,  -12.70f, CRANE_ROUTE_LIFT_PLACE_POS, CRANE_ROUTE_LIFT_SAFE_POS},
    { 1103.5f,    0.10f, CRANE_ROUTE_LIFT_PLACE_POS, CRANE_ROUTE_LIFT_SAFE_POS},
    { 1103.5f,   13.30f, CRANE_ROUTE_LIFT_PLACE_POS, CRANE_ROUTE_LIFT_SAFE_POS},
    { 1015.0f,   25.15f, CRANE_ROUTE_LIFT_PLACE_POS, CRANE_ROUTE_LIFT_SAFE_POS},
};

/* 同一箱号的三个机构 Y 不共用变量，便于逐项标定。 */
static float crane_route_upper_place_y[CRANE_ROUTE_SLOT_COUNT + 1U] =
{
    0.0f, 0.0f, 0.0f, 0.0f, -24.10f, -12.70f, 0.10f, 0.0f, 0.0f
};

static float crane_route_claw_place_y[CRANE_ROUTE_SLOT_COUNT + 1U] =
{
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -12.70f, 0.10f, 13.30f, 0.0f
};

static float crane_route_lower_place_y[CRANE_ROUTE_SLOT_COUNT + 1U] =
{
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.10f, 13.30f, 25.15f
};

static crane_route_t crane_route;

static float crane_route_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static void crane_route_enter_state(crane_route_state_e state)
{
    crane_route.state = state;
    crane_route.state_tick = HAL_GetTick();
}

static uint8_t crane_route_state_timed_out(uint32_t timeout_ms)
{
    return ((HAL_GetTick() - crane_route.state_tick) >= timeout_ms) ? 1U : 0U;
}

static uint8_t crane_route_motor_feedback_fresh(motor_num motor_index)
{
    return ((dm_motor_feedback_is_valid(motor_index) != 0U) &&
            (dm_motor_feedback_age_ms(motor_index) <=
             CRANE_ROUTE_FEEDBACK_TIMEOUT_MS)) ? 1U : 0U;
}

static uint8_t crane_route_all_motor_feedback_fresh(void)
{
    return ((crane_route_motor_feedback_fresh(Motor1) != 0U) &&
            (crane_route_motor_feedback_fresh(Motor2) != 0U) &&
            (crane_route_motor_feedback_fresh(Motor3) != 0U) &&
            (crane_route_motor_feedback_fresh(Motor4) != 0U) &&
            (crane_route_motor_feedback_fresh(Motor5) != 0U) &&
            (crane_route_motor_feedback_fresh(Motor6) != 0U)) ? 1U : 0U;
}

static void crane_route_stop_axes(void)
{
    pos_pid_sync_stop();
    beam_ctrl_stop();
    upper_hopper_y_ctrl_stop();
    lower_hopper_y_ctrl_stop();
    lift_ctrl_stop();
}

static void crane_route_fault_stop(crane_route_fault_e fault)
{
    crane_route.fault = fault;
    crane_route_stop_axes();
    claw_close();
    upper_hopper_gate_close();
    lower_hopper_gate_close();
    crane_route_enter_state(CRANE_ROUTE_FAULT);
}

static void crane_route_fault_from_wait(void)
{
    crane_route_fault_stop((crane_route_all_motor_feedback_fresh() != 0U) ?
                           CRANE_ROUTE_FAULT_MOTION_TIMEOUT :
                           CRANE_ROUTE_FAULT_FEEDBACK_TIMEOUT);
}

static void crane_route_move_x(float target)
{
    crane_route.target_x = target;
#if ((CRANE_ROUTE_NO_CHASSIS == 0U) && (CRANE_ROUTE_BEAM_ONLY == 0U) && (CRANE_ROUTE_LIFT_ONLY == 0U))
    pos_pid_sync_set_target(target);
    pos_pid_sync_start();
#else
    (void)target;
#endif
}

static void crane_route_hold_x(void)
{
    crane_route_move_x(pos_pid_sync_get_current_pos());
}

static void crane_route_move_claw_y(float target)
{
    crane_route.target_claw_y = target;
#if ((CRANE_ROUTE_BEAM_ONLY == 0U) && (CRANE_ROUTE_LIFT_ONLY == 0U))
    beam_ctrl_set_target(target);
    beam_ctrl_start();
#else
    (void)target;
#endif
}

static void crane_route_move_upper_y(float target)
{
    crane_route.target_upper_y = target;
#if (CRANE_ROUTE_LIFT_ONLY == 0U)
    upper_hopper_y_ctrl_set_target(target);
    upper_hopper_y_ctrl_start();
#else
    (void)target;
#endif
}

static void crane_route_move_lower_y(float target)
{
    crane_route.target_lower_y = target;
#if (CRANE_ROUTE_LIFT_ONLY == 0U)
    lower_hopper_y_ctrl_set_target(target);
    lower_hopper_y_ctrl_start();
#else
    (void)target;
#endif
}

static void crane_route_move_z(float target)
{
    crane_route.target_z = target;
#if ((CRANE_ROUTE_BEAM_ONLY == 0U) && (CRANE_ROUTE_LIFT_ONLY == 0U))
    lift_ctrl_set_target(target);
    lift_ctrl_start();
#else
    (void)target;
#endif
}

static uint8_t crane_route_x_arrived(void)
{
#if ((CRANE_ROUTE_NO_CHASSIS != 0U) || (CRANE_ROUTE_BEAM_ONLY != 0U) || (CRANE_ROUTE_LIFT_ONLY != 0U))
    return 1U;
#else
    (void)pos_pid_sync_is_busy();
    return ((crane_route_motor_feedback_fresh(Motor1) != 0U) &&
            (crane_route_motor_feedback_fresh(Motor2) != 0U) &&
            (pos_pid_sync_is_arrived() != 0U)) ? 1U : 0U;
#endif
}

static uint8_t crane_route_claw_y_arrived(void)
{
#if ((CRANE_ROUTE_BEAM_ONLY != 0U) || (CRANE_ROUTE_LIFT_ONLY != 0U))
    return 1U;
#else
    (void)beam_ctrl_is_busy();
    return ((crane_route_motor_feedback_fresh(Motor3) != 0U) &&
            (beam_ctrl_is_arrived() != 0U)) ? 1U : 0U;
#endif
}

static uint8_t crane_route_upper_y_arrived(void)
{
#if (CRANE_ROUTE_LIFT_ONLY != 0U)
    return 1U;
#else
    (void)upper_hopper_y_ctrl_is_busy();
    return ((crane_route_motor_feedback_fresh(Motor5) != 0U) &&
            (upper_hopper_y_ctrl_is_arrived() != 0U)) ? 1U : 0U;
#endif
}

static uint8_t crane_route_lower_y_arrived(void)
{
#if (CRANE_ROUTE_LIFT_ONLY != 0U)
    return 1U;
#else
    (void)lower_hopper_y_ctrl_is_busy();
    return ((crane_route_motor_feedback_fresh(Motor6) != 0U) &&
            (lower_hopper_y_ctrl_is_arrived() != 0U)) ? 1U : 0U;
#endif
}

static uint8_t crane_route_z_arrived(void)
{
#if ((CRANE_ROUTE_BEAM_ONLY != 0U) || (CRANE_ROUTE_LIFT_ONLY != 0U))
    return 1U;
#else
    (void)lift_ctrl_is_busy();
    return ((crane_route_motor_feedback_fresh(Motor4) != 0U) &&
            (lift_ctrl_is_arrived() != 0U)) ? 1U : 0U;
#endif
}

static uint8_t crane_route_all_y_arrived(void)
{
    return ((crane_route_claw_y_arrived() != 0U) &&
            (crane_route_upper_y_arrived() != 0U) &&
            (crane_route_lower_y_arrived() != 0U)) ? 1U : 0U;
}

static uint8_t crane_route_x_reached(float trigger)
{
    return (pos_pid_sync_get_current_pos() >= trigger) ? 1U : 0U;
}

static uint8_t crane_route_check_unique_range(const uint8_t *values,
                                              uint8_t count,
                                              uint8_t min_value,
                                              uint8_t max_value)
{
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < count; i++)
    {
        if ((values[i] < min_value) || (values[i] > max_value))
        {
            return 0U;
        }

        for (j = (uint8_t)(i + 1U); j < count; j++)
        {
            if (values[i] == values[j])
            {
                return 0U;
            }
        }
    }

    return 1U;
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
    uint8_t i;

    for (i = 0U; i < CRANE_ROUTE_PLACE_COUNT; i++)
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

    if ((pick_slot < 1U) || (pick_slot > CRANE_ROUTE_PICK_COUNT))
    {
        return 0U;
    }

    target_box = crane_route_target_box_for_goods(crane_route.pick_goods[pick_slot - 1U]);
    return crane_route_place_slot_for_box(target_box);
}

static void crane_route_swap_load(crane_load_plan_t *a, crane_load_plan_t *b)
{
    crane_load_plan_t temp = *a;
    *a = *b;
    *b = temp;
}

static uint8_t crane_route_build_plan(void)
{
    crane_load_plan_t sorted[CRANE_ROUTE_PICK_COUNT];
    uint8_t i;
    uint8_t j;

    if (crane_route.draw_valid == 0U)
    {
        return 0U;
    }

    for (i = 0U; i < CRANE_ROUTE_PICK_COUNT; i++)
    {
        sorted[i].pick_slot = (uint8_t)(i + 1U);
        sorted[i].place_slot = crane_route_place_for_pick(sorted[i].pick_slot);
        if ((sorted[i].place_slot < 4U) || (sorted[i].place_slot > 8U))
        {
            return 0U;
        }
    }

    for (i = 0U; i < CRANE_ROUTE_PICK_COUNT; i++)
    {
        for (j = (uint8_t)(i + 1U); j < CRANE_ROUTE_PICK_COUNT; j++)
        {
            if (sorted[j].place_slot < sorted[i].place_slot)
            {
                crane_route_swap_load(&sorted[i], &sorted[j]);
            }
        }
    }

    crane_route.plan.upper = sorted[0];
    crane_route.plan.claw = sorted[1];
    crane_route.plan.lower = sorted[2];

    if ((crane_route.plan.upper.place_slot < 4U) ||
        (crane_route.plan.upper.place_slot > 6U) ||
        (crane_route.plan.claw.place_slot < 5U) ||
        (crane_route.plan.claw.place_slot > 7U) ||
        (crane_route.plan.lower.place_slot < 6U) ||
        (crane_route.plan.lower.place_slot > 8U))
    {
        return 0U;
    }

    switch (crane_route.plan.claw.pick_slot)
    {
        case 1U:
            crane_route.plan.pick_order[0] = 3U;
            crane_route.plan.pick_order[1] = 2U;
            crane_route.plan.pick_order[2] = 1U;
            break;

        case 2U:
            crane_route.plan.pick_order[0] = 3U;
            crane_route.plan.pick_order[1] = 1U;
            crane_route.plan.pick_order[2] = 2U;
            break;

        case 3U:
            crane_route.plan.pick_order[0] = 1U;
            crane_route.plan.pick_order[1] = 2U;
            crane_route.plan.pick_order[2] = 3U;
            break;

        default:
            return 0U;
    }

    return 1U;
}

static uint8_t crane_route_config_valid(void)
{
    uint8_t upper_gate_available;
    uint8_t lower_gate_available;

    if (!((CRANE_ROUTE_X_OBS1_PRE_50 < CRANE_ROUTE_X_OBS2_PRE_100) &&
          (CRANE_ROUTE_X_OBS2_PRE_100 < CRANE_ROUTE_X_OBS2_POST_50) &&
          (CRANE_ROUTE_X_OBS2_POST_50 < CRANE_ROUTE_X_EXTREME_STATION) &&
          (CRANE_ROUTE_X_EXTREME_STATION < CRANE_ROUTE_X_REMAINING_STATION)))
    {
        return 0U;
    }

    upper_gate_available = upper_hopper_gate_is_hw_ready();
    lower_gate_available = lower_hopper_gate_is_hw_ready();
#if (CRANE_ROUTE_ALLOW_GATE_SIMULATION != 0U)
    upper_gate_available = (uint8_t)(upper_gate_available |
        upper_hopper_gate_is_simulation_enabled());
    lower_gate_available = (uint8_t)(lower_gate_available |
        lower_hopper_gate_is_simulation_enabled());
#endif

    return ((upper_gate_available != 0U) &&
            (lower_gate_available != 0U)) ? 1U : 0U;
}

static float crane_route_pick_approach_z(uint8_t slot)
{
    float approach = crane_route_slot_pose[slot].lift_work_pos +
                     CRANE_ROUTE_PICK_APPROACH_CLEARANCE;

    if (approach > crane_route_slot_pose[slot].lift_safe_pos)
    {
        approach = crane_route_slot_pose[slot].lift_safe_pos;
    }
    return approach;
}

static float crane_route_pick_box_clear_z(uint8_t slot)
{
    float clear = crane_route_slot_pose[slot].lift_work_pos +
                  CRANE_ROUTE_PICK_BOX_CLEARANCE;

    if (clear > crane_route_slot_pose[slot].lift_safe_pos)
    {
        clear = crane_route_slot_pose[slot].lift_safe_pos;
    }
    return clear;
}

static crane_carrier_e crane_route_carrier_for_pick(uint8_t pick_slot)
{
    if (pick_slot == crane_route.plan.upper.pick_slot)
    {
        return CRANE_CARRIER_UPPER_HOPPER;
    }
    if (pick_slot == crane_route.plan.lower.pick_slot)
    {
        return CRANE_CARRIER_LOWER_HOPPER;
    }
    return CRANE_CARRIER_CLAW;
}

static float crane_route_load_y_for_pick(uint8_t pick_slot)
{
    return (crane_route_carrier_for_pick(pick_slot) == CRANE_CARRIER_UPPER_HOPPER) ?
           CRANE_ROUTE_CLAW_LOAD_UPPER_Y : CRANE_ROUTE_CLAW_LOAD_LOWER_Y;
}

static void crane_route_prepare_pick(uint8_t pick_index)
{
    uint8_t slot = crane_route.plan.pick_order[pick_index];

    crane_route.pick_index = pick_index;
    crane_route.current_slot = slot;
    crane_route.next_x_started = 0U;
    crane_route_move_x(crane_route_slot_pose[slot].chassis_pos);
    crane_route_move_claw_y(crane_route_slot_pose[slot].beam_pos);
    crane_route_move_upper_y(CRANE_ROUTE_UPPER_HOPPER_STOW_Y);
    crane_route_move_lower_y(CRANE_ROUTE_LOWER_HOPPER_STOW_Y);
    crane_route_move_z(crane_route_pick_approach_z(slot));
    servo3_path_release_pick_area();
    crane_route_enter_state(CRANE_ROUTE_WAIT_PICK_APPROACH);
}

static void crane_route_try_start_next_pick_x(void)
{
    uint8_t next_slot;
    float next_x;

    if ((crane_route.next_x_started != 0U) ||
        (crane_route.pick_index >= (CRANE_ROUTE_PICK_COUNT - 1U)))
    {
        return;
    }

    if (lift_ctrl_get_current_pos() <
        (crane_route_pick_box_clear_z(crane_route.current_slot) - CRANE_ROUTE_Z_TOL))
    {
        return;
    }

    next_slot = crane_route.plan.pick_order[crane_route.pick_index + 1U];
    next_x = crane_route_slot_pose[next_slot].chassis_pos;

    if (crane_route_absf(next_x - crane_route_slot_pose[crane_route.current_slot].chassis_pos) > 1.0f)
    {
        crane_route_move_x(next_x);
    }
    crane_route.next_x_started = 1U;
}

static void crane_route_move_all_upper_safe(void)
{
    crane_route_move_claw_y(CRANE_ROUTE_CLAW_UPPER_SAFE_Y);
    crane_route_move_upper_y(CRANE_ROUTE_UPPER_UPPER_SAFE_Y);
    crane_route_move_lower_y(CRANE_ROUTE_LOWER_UPPER_SAFE_Y);
}

static void crane_route_move_all_lower_transition(void)
{
    crane_route_move_claw_y(CRANE_ROUTE_CLAW_LOWER_TRANS_Y);
    crane_route_move_upper_y(CRANE_ROUTE_UPPER_LOWER_TRANS_Y);
    crane_route_move_lower_y(CRANE_ROUTE_LOWER_LOWER_TRANS_Y);
}

static void crane_route_move_all_place_y(void)
{
    crane_route_move_upper_y(
        crane_route_upper_place_y[crane_route.plan.upper.place_slot]);
    crane_route_move_claw_y(
        crane_route_claw_place_y[crane_route.plan.claw.place_slot]);
    crane_route_move_lower_y(
        crane_route_lower_place_y[crane_route.plan.lower.place_slot]);
    crane_route_move_z(crane_route_slot_pose[crane_route.plan.claw.place_slot].lift_work_pos);
}

static uint8_t crane_route_extreme_mask(void)
{
    uint8_t mask = 0U;

    if (crane_route.plan.upper.place_slot == 4U)
    {
        mask |= CRANE_RELEASE_UPPER_MASK;
    }
    if (crane_route.plan.lower.place_slot == 8U)
    {
        mask |= CRANE_RELEASE_LOWER_MASK;
    }
    return mask;
}

static uint8_t crane_route_remaining_coords_arrived(void)
{
    uint8_t ready = 1U;

    if ((crane_route.release_mask & CRANE_RELEASE_UPPER_MASK) == 0U)
    {
        ready = (uint8_t)(ready & crane_route_upper_y_arrived());
    }
    if ((crane_route.release_mask & CRANE_RELEASE_CLAW_MASK) == 0U)
    {
        ready = (uint8_t)(ready & crane_route_claw_y_arrived());
        ready = (uint8_t)(ready & crane_route_z_arrived());
    }
    if ((crane_route.release_mask & CRANE_RELEASE_LOWER_MASK) == 0U)
    {
        ready = (uint8_t)(ready & crane_route_lower_y_arrived());
    }

    return ready;
}

static void crane_route_start_gate_cycle(uint8_t mask)
{
    uint8_t new_mask = (uint8_t)(mask & (uint8_t)(~crane_route.gate_cycle_mask));

    crane_route.gate_cycle_mask |= new_mask;
    crane_route.gate_open_seen_mask &= (uint8_t)(~new_mask);
    crane_route.gate_close_commanded_mask &= (uint8_t)(~new_mask);

    if ((new_mask & CRANE_RELEASE_UPPER_MASK) != 0U)
    {
        upper_hopper_gate_open();
    }
    if ((new_mask & CRANE_RELEASE_LOWER_MASK) != 0U)
    {
        lower_hopper_gate_open();
    }
}

static uint8_t crane_route_process_gate_cycle(void)
{
    uint32_t now = HAL_GetTick();

    if (((crane_route.gate_cycle_mask & CRANE_RELEASE_UPPER_MASK) != 0U) &&
        ((crane_route.release_mask & CRANE_RELEASE_UPPER_MASK) == 0U))
    {
        if ((crane_route.gate_open_seen_mask & CRANE_RELEASE_UPPER_MASK) == 0U)
        {
            if (upper_hopper_gate_is_open() != 0U)
            {
                crane_route.gate_open_seen_mask |= CRANE_RELEASE_UPPER_MASK;
                crane_route.upper_gate_open_tick = now;
            }
        }
        else if ((crane_route.gate_close_commanded_mask & CRANE_RELEASE_UPPER_MASK) == 0U)
        {
            if ((now - crane_route.upper_gate_open_tick) >= CRANE_ROUTE_GATE_RELEASE_HOLD_MS)
            {
                upper_hopper_gate_close();
                crane_route.gate_close_commanded_mask |= CRANE_RELEASE_UPPER_MASK;
            }
        }
        else if (upper_hopper_gate_is_closed() != 0U)
        {
            crane_route.release_mask |= CRANE_RELEASE_UPPER_MASK;
        }
    }

    if (((crane_route.gate_cycle_mask & CRANE_RELEASE_LOWER_MASK) != 0U) &&
        ((crane_route.release_mask & CRANE_RELEASE_LOWER_MASK) == 0U))
    {
        if ((crane_route.gate_open_seen_mask & CRANE_RELEASE_LOWER_MASK) == 0U)
        {
            if (lower_hopper_gate_is_open() != 0U)
            {
                crane_route.gate_open_seen_mask |= CRANE_RELEASE_LOWER_MASK;
                crane_route.lower_gate_open_tick = now;
            }
        }
        else if ((crane_route.gate_close_commanded_mask & CRANE_RELEASE_LOWER_MASK) == 0U)
        {
            if ((now - crane_route.lower_gate_open_tick) >= CRANE_ROUTE_GATE_RELEASE_HOLD_MS)
            {
                lower_hopper_gate_close();
                crane_route.gate_close_commanded_mask |= CRANE_RELEASE_LOWER_MASK;
            }
        }
        else if (lower_hopper_gate_is_closed() != 0U)
        {
            crane_route.release_mask |= CRANE_RELEASE_LOWER_MASK;
        }
    }

    return (((crane_route.release_mask & crane_route.gate_cycle_mask) ==
             crane_route.gate_cycle_mask) ? 1U : 0U);
}

void crane_route_init(void)
{
    memset(&crane_route, 0, sizeof(crane_route));
    crane_route.state = CRANE_ROUTE_IDLE;
    crane_route.beam_path_only = CRANE_ROUTE_BEAM_PATH_ONLY_DEFAULT;
}

uint8_t crane_route_set_draw_result(const uint8_t pick_goods[3],
                                    const uint8_t place_boxes[5])
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

    if (crane_route_check_unique_range(place_boxes,
                                       CRANE_ROUTE_PLACE_COUNT,
                                       1U,
                                       CRANE_ROUTE_PLACE_COUNT) == 0U)
    {
        return 0U;
    }

    memcpy(crane_route.pick_goods, pick_goods, sizeof(crane_route.pick_goods));
    memcpy(crane_route.place_boxes, place_boxes, sizeof(crane_route.place_boxes));
    crane_route.draw_valid = 1U;
    return 1U;
}

void crane_route_start(void)
{
    crane_route.fault = CRANE_ROUTE_FAULT_NONE;
    crane_route.pick_index = 0U;
    crane_route.current_slot = 0U;
    crane_route.next_x_started = 0U;
    crane_route.release_mask = 0U;
    crane_route.gate_cycle_mask = 0U;
    crane_route.gate_open_seen_mask = 0U;
    crane_route.gate_close_commanded_mask = 0U;
    crane_route.target_x = pos_pid_sync_get_current_pos();
    crane_route.target_claw_y = beam_ctrl_get_current_pos();
    crane_route.target_upper_y = upper_hopper_y_ctrl_get_current_pos();
    crane_route.target_lower_y = lower_hopper_y_ctrl_get_current_pos();
    crane_route.target_z = lift_ctrl_get_current_pos();

    upper_hopper_gate_close();
    lower_hopper_gate_close();
    claw_close();
    crane_route_enter_state(CRANE_ROUTE_BUILD_PLAN);
}

void crane_route_stop(void)
{
    crane_route_stop_axes();
    crane_route.fault = CRANE_ROUTE_FAULT_NONE;
    crane_route_enter_state(CRANE_ROUTE_IDLE);
}

void crane_route_process(void)
{
    uint8_t slot;
    uint8_t extreme_mask;
    uint8_t remaining_gate_mask;
    uint8_t previous_gate_cycle_mask;

    /*
     * 任一参与联动的电机反馈失鲜都立即停轴，禁止继续使用旧位置等待
     * 120 s 动作超时。ID 未正确配置或总线掉线时也会在起步前拦截。
     */
    if ((crane_route.state != CRANE_ROUTE_IDLE) &&
        (crane_route.state != CRANE_ROUTE_FINISHED) &&
        (crane_route.state != CRANE_ROUTE_FAULT) &&
        (crane_route_all_motor_feedback_fresh() == 0U))
    {
        crane_route_fault_stop(CRANE_ROUTE_FAULT_FEEDBACK_TIMEOUT);
        return;
    }

    switch (crane_route.state)
    {
        case CRANE_ROUTE_IDLE:
        case CRANE_ROUTE_FINISHED:
        case CRANE_ROUTE_FAULT:
            break;

        case CRANE_ROUTE_BUILD_PLAN:
            if (crane_route_build_plan() == 0U)
            {
                crane_route_fault_stop(CRANE_ROUTE_FAULT_PLAN);
                break;
            }
            if (crane_route_config_valid() == 0U)
            {
                crane_route_fault_stop(CRANE_ROUTE_FAULT_CONFIG);
                break;
            }
            servo3_path_release_pick_area();
            crane_route_prepare_pick(0U);
            break;

        case CRANE_ROUTE_WAIT_PICK_APPROACH:
            if ((crane_route_x_arrived() != 0U) &&
                (crane_route_claw_y_arrived() != 0U) &&
                (crane_route_upper_y_arrived() != 0U) &&
                (crane_route_lower_y_arrived() != 0U) &&
                (crane_route_z_arrived() != 0U) &&
                (servo3_path_is_arrived() != 0U) &&
                (claw_is_closed() != 0U))
            {
                crane_route_enter_state(CRANE_ROUTE_PICK_DESCEND);
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_MOTION_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_from_wait();
            }
            break;

        case CRANE_ROUTE_PICK_DESCEND:
            claw_open();
            crane_route_move_z(crane_route_slot_pose[crane_route.current_slot].lift_work_pos);
            crane_route_enter_state(CRANE_ROUTE_WAIT_PICK_DESCEND);
            break;

        case CRANE_ROUTE_WAIT_PICK_DESCEND:
            if ((crane_route_z_arrived() != 0U) && (claw_is_open() != 0U))
            {
                crane_route_enter_state(CRANE_ROUTE_PICK_CLOSE);
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_MOTION_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_from_wait();
            }
            break;

        case CRANE_ROUTE_PICK_CLOSE:
            claw_close_pick();
            crane_route_enter_state(CRANE_ROUTE_WAIT_PICK_CLOSE);
            break;

        case CRANE_ROUTE_WAIT_PICK_CLOSE:
            if (claw_is_closed() != 0U)
            {
                crane_route_enter_state(CRANE_ROUTE_PICK_LIFT_SAFE);
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_SERVO_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_stop(CRANE_ROUTE_FAULT_SERVO_TIMEOUT);
            }
            break;

        case CRANE_ROUTE_PICK_LIFT_SAFE:
            crane_route.next_x_started = 0U;
            crane_route_move_z(CRANE_ROUTE_LIFT_SAFE_POS);
            crane_route_enter_state(CRANE_ROUTE_WAIT_PICK_LIFT_SAFE);
            break;

        case CRANE_ROUTE_WAIT_PICK_LIFT_SAFE:
            crane_route_try_start_next_pick_x();
            if (crane_route_z_arrived() != 0U)
            {
                servo3_path_release_place_for_pick(crane_route.current_slot);
                crane_route_enter_state(
                    (crane_route.pick_index < (CRANE_ROUTE_PICK_COUNT - 1U)) ?
                    CRANE_ROUTE_WAIT_ROTATE_TO_LOAD :
                    CRANE_ROUTE_WAIT_FINAL_ROTATE_PLACE);
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_MOTION_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_from_wait();
            }
            break;

        case CRANE_ROUTE_WAIT_ROTATE_TO_LOAD:
            if (servo3_path_is_arrived() != 0U)
            {
                crane_route_enter_state(CRANE_ROUTE_MOVE_CLAW_TO_HOPPER);
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_SERVO_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_stop(CRANE_ROUTE_FAULT_SERVO_TIMEOUT);
            }
            break;

        case CRANE_ROUTE_MOVE_CLAW_TO_HOPPER:
            crane_route_move_claw_y(crane_route_load_y_for_pick(crane_route.current_slot));
            crane_route_enter_state(CRANE_ROUTE_WAIT_CLAW_AT_HOPPER);
            break;

        case CRANE_ROUTE_WAIT_CLAW_AT_HOPPER:
            if (crane_route_claw_y_arrived() != 0U)
            {
                crane_route_enter_state(CRANE_ROUTE_RELEASE_TO_HOPPER);
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_MOTION_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_from_wait();
            }
            break;

        case CRANE_ROUTE_RELEASE_TO_HOPPER:
            claw_open();
            crane_route_enter_state(CRANE_ROUTE_WAIT_RELEASE_TO_HOPPER);
            break;

        case CRANE_ROUTE_WAIT_RELEASE_TO_HOPPER:
            if (claw_is_open() != 0U)
            {
                crane_route_enter_state(CRANE_ROUTE_CLOSE_AND_ROTATE_BACK);
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_SERVO_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_stop(CRANE_ROUTE_FAULT_SERVO_TIMEOUT);
            }
            break;

        case CRANE_ROUTE_CLOSE_AND_ROTATE_BACK:
            claw_close();
            servo3_path_release_pick_from_place(crane_route.current_slot);
            crane_route_enter_state(CRANE_ROUTE_WAIT_CLOSE_AND_ROTATE_BACK);
            break;

        case CRANE_ROUTE_WAIT_CLOSE_AND_ROTATE_BACK:
            if ((claw_is_closed() != 0U) && (servo3_path_is_arrived() != 0U))
            {
                crane_route_prepare_pick((uint8_t)(crane_route.pick_index + 1U));
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_SERVO_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_stop(CRANE_ROUTE_FAULT_SERVO_TIMEOUT);
            }
            break;

        case CRANE_ROUTE_WAIT_FINAL_ROTATE_PLACE:
            if (servo3_path_is_arrived() != 0U)
            {
                crane_route_hold_x();
                crane_route_enter_state(CRANE_ROUTE_MOVE_ALL_TO_UPPER_SAFE);
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_SERVO_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_stop(CRANE_ROUTE_FAULT_SERVO_TIMEOUT);
            }
            break;

        case CRANE_ROUTE_MOVE_ALL_TO_UPPER_SAFE:
            crane_route_move_all_upper_safe();
            crane_route_enter_state(CRANE_ROUTE_WAIT_ALL_UPPER_SAFE);
            break;

        case CRANE_ROUTE_WAIT_ALL_UPPER_SAFE:
            if ((crane_route_all_y_arrived() != 0U) &&
                (crane_route_z_arrived() != 0U) &&
                (crane_route_x_arrived() != 0U))
            {
                crane_route_enter_state(CRANE_ROUTE_START_BYPASS_X);
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_MOTION_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_from_wait();
            }
            break;

        case CRANE_ROUTE_START_BYPASS_X:
            extreme_mask = crane_route_extreme_mask();
            crane_route_move_x((extreme_mask != 0U) ?
                               CRANE_ROUTE_X_EXTREME_STATION :
                               CRANE_ROUTE_X_REMAINING_STATION);
            crane_route_enter_state(CRANE_ROUTE_WAIT_OBSTACLE1_TRIGGER);
            break;

        case CRANE_ROUTE_WAIT_OBSTACLE1_TRIGGER:
            if (crane_route_x_reached(CRANE_ROUTE_X_OBS1_PRE_50) != 0U)
            {
                crane_route_move_all_lower_transition();
                crane_route_enter_state(CRANE_ROUTE_WAIT_OBSTACLE2_CHECK);
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_MOTION_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_from_wait();
            }
            break;

        case CRANE_ROUTE_WAIT_OBSTACLE2_CHECK:
            if (crane_route_x_reached(CRANE_ROUTE_X_OBS2_PRE_100) != 0U)
            {
                if (crane_route_all_y_arrived() != 0U)
                {
                    crane_route_enter_state(CRANE_ROUTE_WAIT_OBSTACLE2_EXIT);
                }
                else
                {
                    crane_route_hold_x();
                    crane_route_enter_state(CRANE_ROUTE_HOLD_X_WAIT_LOWER_Y);
                }
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_MOTION_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_from_wait();
            }
            break;

        case CRANE_ROUTE_HOLD_X_WAIT_LOWER_Y:
            if ((crane_route_all_y_arrived() != 0U) && (crane_route_x_arrived() != 0U))
            {
                extreme_mask = crane_route_extreme_mask();
                crane_route_move_x((extreme_mask != 0U) ?
                                   CRANE_ROUTE_X_EXTREME_STATION :
                                   CRANE_ROUTE_X_REMAINING_STATION);
                crane_route_enter_state(CRANE_ROUTE_WAIT_OBSTACLE2_EXIT);
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_MOTION_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_from_wait();
            }
            break;

        case CRANE_ROUTE_WAIT_OBSTACLE2_EXIT:
            if (crane_route_x_reached(CRANE_ROUTE_X_OBS2_POST_50) != 0U)
            {
                crane_route_enter_state(CRANE_ROUTE_MOVE_ALL_TO_PLACE);
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_MOTION_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_from_wait();
            }
            break;

        case CRANE_ROUTE_MOVE_ALL_TO_PLACE:
            crane_route_move_all_place_y();
            crane_route_enter_state(CRANE_ROUTE_WAIT_EXTREME_STATION);
            break;

        case CRANE_ROUTE_WAIT_EXTREME_STATION:
            extreme_mask = crane_route_extreme_mask();
            if (extreme_mask == 0U)
            {
                crane_route_enter_state(CRANE_ROUTE_WAIT_REMAINING_COORDS);
                break;
            }

            if (crane_route_x_arrived() != 0U)
            {
                crane_route_enter_state(CRANE_ROUTE_WAIT_EXTREME_RELEASE);
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_MOTION_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_from_wait();
            }
            break;

        case CRANE_ROUTE_WAIT_EXTREME_RELEASE:
            extreme_mask = crane_route_extreme_mask();
            if (crane_route_state_timed_out(CRANE_ROUTE_MOTION_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_from_wait();
                break;
            }

            previous_gate_cycle_mask = crane_route.gate_cycle_mask;
            if (((extreme_mask & CRANE_RELEASE_UPPER_MASK) != 0U) &&
                ((crane_route.gate_cycle_mask & CRANE_RELEASE_UPPER_MASK) == 0U) &&
                (crane_route_upper_y_arrived() != 0U))
            {
                crane_route_start_gate_cycle(CRANE_RELEASE_UPPER_MASK);
            }
            if (((extreme_mask & CRANE_RELEASE_LOWER_MASK) != 0U) &&
                ((crane_route.gate_cycle_mask & CRANE_RELEASE_LOWER_MASK) == 0U) &&
                (crane_route_lower_y_arrived() != 0U))
            {
                crane_route_start_gate_cycle(CRANE_RELEASE_LOWER_MASK);
            }

            /* 两个斗门分别解锁；最后一个开始动作后单独计舵机超时。 */
            if (((previous_gate_cycle_mask & extreme_mask) != extreme_mask) &&
                ((crane_route.gate_cycle_mask & extreme_mask) == extreme_mask))
            {
                crane_route.state_tick = HAL_GetTick();
            }

            (void)crane_route_process_gate_cycle();
            if ((crane_route.release_mask & extreme_mask) == extreme_mask)
            {
                crane_route_enter_state(CRANE_ROUTE_MOVE_TO_REMAINING_STATION);
            }
            else if (((crane_route.gate_cycle_mask & extreme_mask) == extreme_mask) &&
                     (crane_route_state_timed_out(CRANE_ROUTE_GATE_TIMEOUT_MS) != 0U))
            {
                crane_route_fault_stop(CRANE_ROUTE_FAULT_GATE_UNAVAILABLE);
            }
            break;

        case CRANE_ROUTE_MOVE_TO_REMAINING_STATION:
            crane_route_move_x(CRANE_ROUTE_X_REMAINING_STATION);
            crane_route_enter_state(CRANE_ROUTE_WAIT_REMAINING_COORDS);
            break;

        case CRANE_ROUTE_WAIT_REMAINING_COORDS:
            if ((crane_route_x_arrived() != 0U) &&
                (crane_route_remaining_coords_arrived() != 0U))
            {
                remaining_gate_mask = (uint8_t)((~crane_route.release_mask) &
                    (CRANE_RELEASE_UPPER_MASK | CRANE_RELEASE_LOWER_MASK));
                crane_route_start_gate_cycle(remaining_gate_mask);
                if ((crane_route.release_mask & CRANE_RELEASE_CLAW_MASK) == 0U)
                {
                    claw_open();
                }
                crane_route_enter_state(CRANE_ROUTE_WAIT_REMAINING_RELEASE);
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_MOTION_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_from_wait();
            }
            break;

        case CRANE_ROUTE_WAIT_REMAINING_RELEASE:
            if (((crane_route.release_mask & CRANE_RELEASE_CLAW_MASK) == 0U) &&
                (claw_is_open() != 0U))
            {
                crane_route.release_mask |= CRANE_RELEASE_CLAW_MASK;
            }
            (void)crane_route_process_gate_cycle();
            if ((crane_route.release_mask & CRANE_RELEASE_ALL_MASK) ==
                CRANE_RELEASE_ALL_MASK)
            {
                crane_route_enter_state(CRANE_ROUTE_LIFT_RETURN_CLEAR);
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_GATE_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_stop(CRANE_ROUTE_FAULT_GATE_UNAVAILABLE);
            }
            break;

        case CRANE_ROUTE_LIFT_RETURN_CLEAR:
            crane_route_move_z(CRANE_ROUTE_LIFT_SAFE_POS);
            crane_route_enter_state(CRANE_ROUTE_WAIT_LIFT_RETURN_CLEAR);
            break;

        case CRANE_ROUTE_WAIT_LIFT_RETURN_CLEAR:
            if (crane_route_z_arrived() != 0U)
            {
                crane_route_enter_state(CRANE_ROUTE_MOVE_ALL_RETURN_UPPER);
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_MOTION_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_from_wait();
            }
            break;

        case CRANE_ROUTE_MOVE_ALL_RETURN_UPPER:
            crane_route_move_all_upper_safe();
            crane_route_enter_state(CRANE_ROUTE_WAIT_ALL_RETURN_UPPER);
            break;

        case CRANE_ROUTE_WAIT_ALL_RETURN_UPPER:
            if (crane_route_all_y_arrived() != 0U)
            {
                crane_route_enter_state(CRANE_ROUTE_RETURN_X_CENTER);
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_MOTION_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_from_wait();
            }
            break;

        case CRANE_ROUTE_RETURN_X_CENTER:
            crane_route.current_slot = 0U;
            crane_route_move_x(crane_route_slot_pose[0U].chassis_pos);
            crane_route_enter_state(CRANE_ROUTE_WAIT_X_CENTER);
            break;

        case CRANE_ROUTE_WAIT_X_CENTER:
            if (crane_route_x_arrived() != 0U)
            {
                crane_route_enter_state(CRANE_ROUTE_FINISHED);
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_MOTION_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_from_wait();
            }
            break;

        default:
            crane_route_fault_stop(CRANE_ROUTE_FAULT_CONFIG);
            break;
    }

    /* Keep compiler diagnostics useful when slot is only needed in debug builds. */
    slot = crane_route.current_slot;
    (void)slot;
}

void crane_route_set_beam_path_only(uint8_t enable)
{
    crane_route.beam_path_only = (enable != 0U) ? 1U : 0U;
}

uint8_t crane_route_get_beam_path_only(void)
{
    return crane_route.beam_path_only;
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

void crane_route_set_slot_lift_pos(uint8_t slot,
                                   float lift_work_pos,
                                   float lift_safe_pos)
{
    if ((slot == 0U) || (slot > CRANE_ROUTE_SLOT_COUNT))
    {
        return;
    }
    crane_route_slot_pose[slot].lift_work_pos = lift_work_pos;
    crane_route_slot_pose[slot].lift_safe_pos = lift_safe_pos;
}

uint8_t crane_route_set_carrier_place_y(crane_carrier_e carrier,
                                        uint8_t slot,
                                        float y_pos)
{
    if (slot > CRANE_ROUTE_SLOT_COUNT)
    {
        return 0U;
    }

    if ((carrier == CRANE_CARRIER_UPPER_HOPPER) && (slot >= 4U) && (slot <= 6U))
    {
        crane_route_upper_place_y[slot] = y_pos;
        return 1U;
    }
    if ((carrier == CRANE_CARRIER_CLAW) && (slot >= 5U) && (slot <= 7U))
    {
        crane_route_claw_place_y[slot] = y_pos;
        return 1U;
    }
    if ((carrier == CRANE_CARRIER_LOWER_HOPPER) && (slot >= 6U) && (slot <= 8U))
    {
        crane_route_lower_place_y[slot] = y_pos;
        return 1U;
    }
    return 0U;
}

void crane_route_get_current_target(float *x, float *y)
{
    if (x != NULL)
    {
        *x = crane_route.target_x;
    }
    if (y != NULL)
    {
        *y = crane_route.target_claw_y;
    }
}

void crane_route_get_current_pose_target(float *x, float *y, float *z)
{
    crane_route_get_current_target(x, y);
    if (z != NULL)
    {
        *z = crane_route.target_z;
    }
}

void crane_route_get_all_y_targets(float *claw_y,
                                   float *upper_hopper_y,
                                   float *lower_hopper_y)
{
    if (claw_y != NULL)
    {
        *claw_y = crane_route.target_claw_y;
    }
    if (upper_hopper_y != NULL)
    {
        *upper_hopper_y = crane_route.target_upper_y;
    }
    if (lower_hopper_y != NULL)
    {
        *lower_hopper_y = crane_route.target_lower_y;
    }
}

crane_route_state_e crane_route_get_state(void)
{
    return crane_route.state;
}

crane_route_fault_e crane_route_get_fault(void)
{
    return crane_route.fault;
}

uint8_t crane_route_is_finished(void)
{
    return ((crane_route.state == CRANE_ROUTE_FINISHED) ||
            (crane_route.state == CRANE_ROUTE_FAULT)) ? 1U : 0U;
}

uint8_t crane_route_get_current_slot(void)
{
    return crane_route.current_slot;
}
