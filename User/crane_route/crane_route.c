#include "headfile.h"
#include "xy_route/xy_route.h"

/* 视觉货号：黄豆 6，白芸豆 7，绿豆 8。 */
#define CRANE_ROUTE_GOODS_SOYBEAN             6U
#define CRANE_ROUTE_GOODS_GREEN               8U
#define CRANE_ROUTE_GOODS_WHITE_BEAN          7U

/* Z 轴正方向为抬升；白芸豆取物时在标定工作位的基础上再下探 0.1。 */
#define CRANE_ROUTE_WHITE_BEAN_PICK_EXTRA_LOWER 0.2f

/* Z 轴仍使用当前达妙 2325；以下均为现有位置单位。 */
#define CRANE_ROUTE_LIFT_PICK_1_POS           2.2f
#define CRANE_ROUTE_LIFT_PICK_2_POS           1.05f
#define CRANE_ROUTE_LIFT_PICK_3_POS           3.2f
#define CRANE_ROUTE_LIFT_PLACE_POS            2.8f
#define CRANE_ROUTE_PICK_APPROACH_CLEARANCE   4.0f//安全接近高度--目标位置+高度
#define CRANE_ROUTE_PICK_BOX_CLEARANCE        2.6f//脱盒高度，代表夹爪超过目标位置+脱盒高度爪子的就可以移动
#define CRANE_ROUTE_Z_TOL                     0.1f//到位容忍值
/* 斗子转交后下一颗取物：爪子 Y 距目标进入该范围，才允许 Z 下到接近高度。 */
#define CRANE_ROUTE_NEXT_PICK_Y_RELEASE_DISTANCE 8.0f

/*
 * 夹爪取物侧下绕入口：首趟取 1 号在安全旋转前、首趟取 3 号在
 * X 接近入口前，都先进入此 Y 安全通道。3 号若未到位则暂停 X，
 * 等夹爪到位后再同时恢复 X 并释放夹爪去 3 号实际取物 Y。
 */
#define CRANE_ROUTE_PICK_LOWER_ENTRY_Y          8.0f
#define CRANE_ROUTE_PICK3_X_ENTRY_WAIT_MARGIN  25.0f

/* 避障门槛由 xy_route 统一标定的两处障碍物坐标推导，避免重复维护。 */
#define CRANE_ROUTE_X_OBS1_PRE_10 \
    (XY_ROUTE_X_ENTRY_PICK_SIDE + 10.0f)
#define CRANE_ROUTE_X_OBS2_PRE_30 \
    (XY_ROUTE_X_ENTRY_PLACE_SIDE - 30.0f)
#define CRANE_ROUTE_X_OBS2_POST_30 \
    (XY_ROUTE_X_ENTRY_PLACE_SIDE + 30.0f)

/* 两类放置 X：4/8 先在外侧站释放，其余在内侧站释放。 */
#define CRANE_ROUTE_X_EXTREME_STATION       866.0f//4，8号放置位置
#define CRANE_ROUTE_X_REMAINING_STATION     1054.0f//5,6,7号放置位置//1028.5

/*
 * 三套 Y 坐标必须独立标定。斗子上电时放在两侧机械安全位并保存零点，
 * 所以第一版斗子停放位为 0；其余数值只用于验证状态机和低速空载联调。
 */
/* 取物阶段料斗停放 Y：两只料斗保持收回，避免进入夹爪取物工作区。 */
#define CRANE_ROUTE_UPPER_HOPPER_STOW_Y        0.0f
#define CRANE_ROUTE_LOWER_HOPPER_STOW_Y        0.0f

/*
 * 夹爪向料斗转交物料时的 Y 位置。
 * 若当前物料由上料斗承接，夹爪移动到 LOAD_UPPER_Y；
 * 若由下料斗承接，则移动到 LOAD_LOWER_Y。
 */
#define CRANE_ROUTE_CLAW_LOAD_UPPER_Y         16.0f
#define CRANE_ROUTE_CLAW_LOAD_LOWER_Y         -16.5f
/* 跨中线转交时的提前旋转触发点；使用爪子 Y 的实际位置判断。 */
#define CRANE_ROUTE_CLAW_Y_ROTATE_RELEASE_POS  0.0f

/*
 * 三套 Y 机构共用的上侧安全通道坐标。
 * claw_obstacle_debug 通过 crane_route_get_upper_safe_y() 读取此处数值，
 * 禁止在调试模块中复制标定值。
 */
#define CRANE_ROUTE_CLAW_UPPER_SAFE_Y          12.8f//三个的上侧安全通道坐标
#define CRANE_ROUTE_UPPER_UPPER_SAFE_Y         -3.2f
#define CRANE_ROUTE_LOWER_UPPER_SAFE_Y         -32.5f

/*
 * 底盘接近第二个 X 向障碍物时，三套 Y 机构切换到“下侧过渡通道”坐标。
 * 若 Y 轴未及时到位，路线会暂停 X 轴，等待三套机构全部进入该安全位置。
 */
#define CRANE_ROUTE_CLAW_LOWER_TRANS_Y           -11.3f//三个的下侧过渡通道坐标
#define CRANE_ROUTE_UPPER_LOWER_TRANS_Y          29.4f
#define CRANE_ROUTE_LOWER_LOWER_TRANS_Y          1.0f

/* 夹爪和旋转舵机在规定时间内未到位时的超时保护。 */
#define CRANE_ROUTE_SERVO_TIMEOUT_MS         10000U
/* 上/下料斗旋转到放豆角度后，夹爪开爪前的稳定等待时间。 */
#define CRANE_ROUTE_HOPPER_OPEN_DELAY_MS       600U
/* 夹爪在上/下斗内张到 40° 后的放豆保持时间。 */
#define CRANE_ROUTE_HOPPER_RELEASE_HOLD_MS      500U
/* X、三套 Y、Z 任一位置动作的最大允许持续时间。 */
#define CRANE_ROUTE_MOTION_TIMEOUT_MS       120000U
/* 上、下料斗门从接到开门命令到确认打开的最大允许时间。 */
#define CRANE_ROUTE_GATE_TIMEOUT_MS          10000U
/* 斗门确认打开后保持的漏豆时间；时间到进入下一步，但斗门保持打开。 */
#define CRANE_ROUTE_GATE_RELEASE_HOLD_MS       600U
/*
 * 放料完成位图：上料斗、夹爪、下料斗各占一个 bit。
 * 状态机用该位图支持两端工位先放料、其余工位后放料的并行/分步流程。
 */
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
    uint8_t final_pick_y_return_started;
    uint8_t hopper_rotate_started;
    uint8_t first_pick_prestart_active;
    uint8_t first_pick_rotate_released;
    uint8_t release_mask;
    uint8_t gate_cycle_mask;
    uint8_t gate_open_seen_mask;
    uint32_t upper_gate_open_tick;
    uint32_t lower_gate_open_tick;
    uint32_t hopper_release_open_tick;
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
    {-1206.0f,  16.2f, CRANE_ROUTE_LIFT_PICK_1_POS, CRANE_ROUTE_LIFT_SAFE_POS},
    {-1206.0f,  -16.2f, CRANE_ROUTE_LIFT_PICK_2_POS, CRANE_ROUTE_LIFT_SAFE_POS},
    {-1028.2f,   0.0f, CRANE_ROUTE_LIFT_PICK_3_POS, CRANE_ROUTE_LIFT_SAFE_POS},
    { 1015.0f,  -24.10f, CRANE_ROUTE_LIFT_PLACE_POS, CRANE_ROUTE_LIFT_SAFE_POS},
    { 1103.5f,  -12.70f, CRANE_ROUTE_LIFT_PLACE_POS, CRANE_ROUTE_LIFT_SAFE_POS},
    { 1103.5f,    0.10f, CRANE_ROUTE_LIFT_PLACE_POS, CRANE_ROUTE_LIFT_SAFE_POS},
    { 1103.5f,   13.30f, CRANE_ROUTE_LIFT_PLACE_POS, CRANE_ROUTE_LIFT_SAFE_POS},
    { 1015.0f,   25.15f, CRANE_ROUTE_LIFT_PLACE_POS, CRANE_ROUTE_LIFT_SAFE_POS},
};

/* 同一箱号的三个机构 Y 不共用变量，便于逐项标定。 */
static float crane_route_upper_place_y[CRANE_ROUTE_SLOT_COUNT + 1U] =
{
    0.0f, 0.0f, 0.0f, 0.0f, -12.3f, 7.1f, 22.0f, 0.0f, 0.0f
};//上料斗的放置Y坐标（只有4，5，6号位置）

static float crane_route_claw_place_y[CRANE_ROUTE_SLOT_COUNT + 1U] =
{
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 12.8f, 0.0f, -12.8f, 0.0f
};//爪子的放置Y坐标（只有5，6，7号位置）

static float crane_route_lower_place_y[CRANE_ROUTE_SLOT_COUNT + 1U] =
{
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -21.5f, -5.8f, 12.2f
};//下料斗的放置Y坐标（只有6，7，8号位置）

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
    crane_route_fault_stop(CRANE_ROUTE_FAULT_MOTION_TIMEOUT);
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
    return pos_pid_sync_is_arrived();
#endif
}

static uint8_t crane_route_claw_y_arrived(void)
{
#if ((CRANE_ROUTE_BEAM_ONLY != 0U) || (CRANE_ROUTE_LIFT_ONLY != 0U))
    return 1U;
#else
    (void)beam_ctrl_is_busy();
    return beam_ctrl_is_arrived();
#endif
}

static uint8_t crane_route_claw_y_near_target(float target)
{
#if ((CRANE_ROUTE_BEAM_ONLY != 0U) || (CRANE_ROUTE_LIFT_ONLY != 0U))
    (void)target;
    return 1U;
#else
    return (crane_route_absf(beam_ctrl_get_current_pos() - target) <=
            CRANE_ROUTE_NEXT_PICK_Y_RELEASE_DISTANCE) ? 1U : 0U;
#endif
}

static uint8_t crane_route_upper_y_arrived(void)
{
#if (CRANE_ROUTE_LIFT_ONLY != 0U)
    return 1U;
#else
    (void)upper_hopper_y_ctrl_is_busy();
    return upper_hopper_y_ctrl_is_arrived();
#endif
}

static uint8_t crane_route_lower_y_arrived(void)
{
#if (CRANE_ROUTE_LIFT_ONLY != 0U)
    return 1U;
#else
    (void)lower_hopper_y_ctrl_is_busy();
    return lower_hopper_y_ctrl_is_arrived();
#endif
}

static uint8_t crane_route_z_arrived(void)
{
#if ((CRANE_ROUTE_BEAM_ONLY != 0U) || (CRANE_ROUTE_LIFT_ONLY != 0U))
    return 1U;
#else
    (void)lift_ctrl_is_busy();
    return lift_ctrl_is_arrived();
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

/* 与旧 xy_route 的取物侧下绕入口判定保持一致：X 向负方向运行时，
 * 在入口前 25 个位置单位开始检查夹爪 Y，给 X 留出停车余量。 */
static uint8_t crane_route_x_near_pick3_lower_entry(void)
{
    return (pos_pid_sync_get_current_pos() <=
            (XY_ROUTE_X_ENTRY_PICK_SIDE + CRANE_ROUTE_PICK3_X_ENTRY_WAIT_MARGIN)) ? 1U : 0U;
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

    if (!((CRANE_ROUTE_X_OBS1_PRE_10 < CRANE_ROUTE_X_OBS2_PRE_30) &&
          (CRANE_ROUTE_X_OBS2_PRE_30 < CRANE_ROUTE_X_OBS2_POST_30) &&
          (CRANE_ROUTE_X_OBS2_POST_30 < CRANE_ROUTE_X_EXTREME_STATION) &&
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

/* 仅白芸豆的实际取物下行深度额外下探；接近、脱盒和安全高度均不变。 */
static float crane_route_pick_work_z(uint8_t slot)
{
    float work = crane_route_slot_pose[slot].lift_work_pos;

    if (crane_route.pick_goods[slot - 1U] == CRANE_ROUTE_GOODS_WHITE_BEAN)
    {
        work -= CRANE_ROUTE_WHITE_BEAN_PICK_EXTRA_LOWER;
    }
    return work;
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

/*
 * 最后一颗若由夹爪从 1/2 号位直接携带，需先在脱盒高度同步回 Y 原点，
 * 再允许爪子旋转到最终放置角度。最后取 3 号以及两颗料斗转运均不走此流程。
 */
static uint8_t crane_route_final_pick_needs_y_return(void)
{
    return ((crane_route.pick_index == (CRANE_ROUTE_PICK_COUNT - 1U)) &&
            (crane_route_carrier_for_pick(crane_route.current_slot) ==
             CRANE_CARRIER_CLAW) &&
            ((crane_route.current_slot == 1U) ||
             (crane_route.current_slot == 2U))) ? 1U : 0U;
}

static float crane_route_load_y_for_pick(uint8_t pick_slot)
{
    return (crane_route_carrier_for_pick(pick_slot) == CRANE_CARRIER_UPPER_HOPPER) ?
           CRANE_ROUTE_CLAW_LOAD_UPPER_Y : CRANE_ROUTE_CLAW_LOAD_LOWER_Y;
}

/*
 * 前两颗豆子转交料斗时，旋转路径由“承接料斗类别”决定，而不是取物槽决定：
 * 上料斗沿用 1 号位路径 135 -> 270；下料斗沿用 2 号位路径 135 -> 5。
 * 最后一颗由夹爪直接携带，不调用本函数，仍按取物槽选择最终放置路径。
 */
static uint8_t crane_route_rotate_to_hopper_for_pick(uint8_t pick_slot)
{
    crane_carrier_e carrier = crane_route_carrier_for_pick(pick_slot);

    if (carrier == CRANE_CARRIER_UPPER_HOPPER)
    {
        servo3_path_release_angle(SERVO3_PICK1_PLACE_ANGLE_DEG);
        return 1U;
    }
    if (carrier == CRANE_CARRIER_LOWER_HOPPER)
    {
        servo3_path_release_angle(SERVO3_PICK2_PLACE_ANGLE_DEG);
        return 1U;
    }

    return 0U;
}

/*
 * 仅两种跨越 Y=0 的转交允许提前旋转：
 *   取 1 号 -> 下料斗：爪子 Y 从正侧向负侧运动，实际位置 <= 0 时释放；
 *   取 2 号 -> 上料斗：爪子 Y 从负侧向正侧运动，实际位置 >= 0 时释放。
 * 其余组合继续等待爪子 Y 到达料斗位置后再旋转。
 */
static uint8_t crane_route_hopper_early_rotate_ready(uint8_t pick_slot)
{
    crane_carrier_e carrier = crane_route_carrier_for_pick(pick_slot);
    float current_y = beam_ctrl_get_current_pos();

    if ((pick_slot == 1U) &&
        (carrier == CRANE_CARRIER_LOWER_HOPPER))
    {
        return (current_y <= CRANE_ROUTE_CLAW_Y_ROTATE_RELEASE_POS) ? 1U : 0U;
    }

    if ((pick_slot == 2U) &&
        (carrier == CRANE_CARRIER_UPPER_HOPPER))
    {
        return (current_y >= CRANE_ROUTE_CLAW_Y_ROTATE_RELEASE_POS) ? 1U : 0U;
    }

    return 0U;
}

static uint8_t crane_route_start_hopper_rotation(void)
{
    if (crane_route.hopper_rotate_started != 0U)
    {
        return 1U;
    }

    if (crane_route_rotate_to_hopper_for_pick(crane_route.current_slot) == 0U)
    {
        return 0U;
    }

    crane_route.hopper_rotate_started = 1U;
    return 1U;
}

static void crane_route_prepare_pick(uint8_t pick_index)
{
    uint8_t slot = crane_route.plan.pick_order[pick_index];
    uint8_t first_pick_is_slot3 = ((pick_index == 0U) && (slot == 3U)) ? 1U : 0U;
    uint8_t first_pick_is_slot1 = ((pick_index == 0U) && (slot == 1U)) ? 1U : 0U;
    uint8_t first_pick_needs_safe_rotate =
        ((pick_index == 0U) && ((slot == 1U) || (slot == 2U))) ? 1U : 0U;

    crane_route.pick_index = pick_index;
    crane_route.current_slot = slot;
    crane_route.next_x_started = 0U;
    crane_route.final_pick_y_return_started = 0U;
    crane_route.hopper_rotate_started = 0U;
    crane_route_move_x(crane_route_slot_pose[slot].chassis_pos);
    crane_route_move_upper_y(CRANE_ROUTE_UPPER_HOPPER_STOW_Y);
    crane_route_move_lower_y(CRANE_ROUTE_LOWER_HOPPER_STOW_Y);

    /*
     * 首趟取 1/2 时，旋转爪在低位直接转向会与料斗干涉。
     * 正式比赛中，视觉结果后的 0.5 s 已经让 Z 预先抬升，并在
     * Z 到离地高度后开始转向。首趟取 1 号时，正式起步让 X/Y
     * 直接同时前往 1 号目标位，不再经过下绕入口。
     * 首趟 3 号仍走其原有的取物侧下绕流程，不进入本分支。
     */
    if (first_pick_needs_safe_rotate != 0U)
    {
        crane_route_move_z(CRANE_ROUTE_LIFT_SAFE_POS);
        if ((first_pick_is_slot1 != 0U) &&
            (crane_route.first_pick_prestart_active != 0U))
        {
            crane_route_move_claw_y(crane_route_slot_pose[slot].beam_pos);
            crane_route_enter_state((crane_route.first_pick_rotate_released != 0U) ?
                                    CRANE_ROUTE_WAIT_FIRST_PICK_ROTATE :
                                    CRANE_ROUTE_WAIT_FIRST_PICK_LIFT_SAFE);
        }
        else
        {
            if (slot == 1U)
            {
                crane_route_move_claw_y(CRANE_ROUTE_PICK_LOWER_ENTRY_Y);
            }
            crane_route_enter_state(CRANE_ROUTE_WAIT_FIRST_PICK_LIFT_SAFE);
        }
    }
    else if (first_pick_is_slot3 != 0U)
    {
        crane_route_move_claw_y(CRANE_ROUTE_PICK_LOWER_ENTRY_Y);
        crane_route_move_z(crane_route_pick_approach_z(slot));
        if (crane_route.first_pick_prestart_active == 0U)
        {
            servo3_path_release_pick_area();
        }
        crane_route_enter_state(CRANE_ROUTE_WAIT_PICK3_LOWER_ENTRY);
    }
    else
    {
        crane_route_move_claw_y(crane_route_slot_pose[slot].beam_pos);
        crane_route_move_z(crane_route_pick_approach_z(slot));
        servo3_path_release_pick_area();
        crane_route_enter_state(CRANE_ROUTE_WAIT_PICK_APPROACH);
    }
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

static void crane_route_start_gate_cycle(uint8_t mask)
{
    uint8_t new_mask = (uint8_t)(mask & (uint8_t)(~crane_route.gate_cycle_mask));

    crane_route.gate_cycle_mask |= new_mask;
    crane_route.gate_open_seen_mask &= (uint8_t)(~new_mask);

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
        if (upper_hopper_gate_is_open() != 0U)
        {
            if ((crane_route.gate_open_seen_mask & CRANE_RELEASE_UPPER_MASK) == 0U)
            {
                crane_route.gate_open_seen_mask |= CRANE_RELEASE_UPPER_MASK;
                crane_route.upper_gate_open_tick = now;
            }
            else if ((now - crane_route.upper_gate_open_tick) >=
                     CRANE_ROUTE_GATE_RELEASE_HOLD_MS)
            {
                crane_route.release_mask |= CRANE_RELEASE_UPPER_MASK;
            }
        }
    }

    if (((crane_route.gate_cycle_mask & CRANE_RELEASE_LOWER_MASK) != 0U) &&
        ((crane_route.release_mask & CRANE_RELEASE_LOWER_MASK) == 0U))
    {
        if (lower_hopper_gate_is_open() != 0U)
        {
            if ((crane_route.gate_open_seen_mask & CRANE_RELEASE_LOWER_MASK) == 0U)
            {
                crane_route.gate_open_seen_mask |= CRANE_RELEASE_LOWER_MASK;
                crane_route.lower_gate_open_tick = now;
            }
            else if ((now - crane_route.lower_gate_open_tick) >=
                     CRANE_ROUTE_GATE_RELEASE_HOLD_MS)
            {
                crane_route.release_mask |= CRANE_RELEASE_LOWER_MASK;
            }
        }
    }

    return (((crane_route.release_mask & crane_route.gate_cycle_mask) ==
             crane_route.gate_cycle_mask) ? 1U : 0U);
}

/*
 * 5/6/7 号位的斗子释放与夹爪下放解耦：
 * 底盘 X 到共用放置工位后，上/下斗只等待各自 Y 到位就开门。
 * 已在 4/8 号位释放的斗子会被 release/gate_cycle 位图过滤。
 */
static void crane_route_process_remaining_gate_release(void)
{
    uint8_t gate_mask = 0U;

    if (crane_route_x_arrived() != 0U)
    {
        if (((crane_route.release_mask & CRANE_RELEASE_UPPER_MASK) == 0U) &&
            ((crane_route.gate_cycle_mask & CRANE_RELEASE_UPPER_MASK) == 0U) &&
            (crane_route_upper_y_arrived() != 0U))
        {
            gate_mask |= CRANE_RELEASE_UPPER_MASK;
        }
        if (((crane_route.release_mask & CRANE_RELEASE_LOWER_MASK) == 0U) &&
            ((crane_route.gate_cycle_mask & CRANE_RELEASE_LOWER_MASK) == 0U) &&
            (crane_route_lower_y_arrived() != 0U))
        {
            gate_mask |= CRANE_RELEASE_LOWER_MASK;
        }

        crane_route_start_gate_cycle(gate_mask);
    }

    (void)crane_route_process_gate_cycle();
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
                                       6U,
                                       8U) == 0U)
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

void crane_route_prestart_begin(void)
{
    crane_route.first_pick_prestart_active = 0U;
    crane_route.first_pick_rotate_released = 0U;

    /* 与正式 BUILD_PLAN 共用同一套分配规则，避免预准备和正式路线首趟不一致。 */
    if (crane_route_build_plan() == 0U)
    {
        return;
    }

    crane_route.first_pick_prestart_active = 1U;
    crane_route_move_z(CRANE_ROUTE_LIFT_SAFE_POS);
}

void crane_route_prestart_process(void)
{
    if ((crane_route.first_pick_prestart_active == 0U) ||
        (crane_route.first_pick_rotate_released != 0U))
    {
        return;
    }

    if (lift_ctrl_get_current_pos() >= CRANE_ROUTE_FIRST_PICK_ROTATE_RELEASE_Z)
    {
        servo3_path_release_pick_area();
        crane_route.first_pick_rotate_released = 1U;
    }
}

void crane_route_start(void)
{
    crane_route.fault = CRANE_ROUTE_FAULT_NONE;
    crane_route.pick_index = 0U;
    crane_route.current_slot = 0U;
    crane_route.next_x_started = 0U;
    crane_route.final_pick_y_return_started = 0U;
    crane_route.hopper_rotate_started = 0U;
    crane_route.release_mask = 0U;
    crane_route.gate_cycle_mask = 0U;
    crane_route.gate_open_seen_mask = 0U;
    crane_route.upper_gate_open_tick = 0U;
    crane_route.lower_gate_open_tick = 0U;
    crane_route.hopper_release_open_tick = 0U;
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
    uint8_t previous_gate_cycle_mask;

    /* 若 0.5 s 内尚未抬到离地高度，正式起步后继续只等待/触发旋转。 */
    crane_route_prestart_process();

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
            crane_route_prepare_pick(0U);
            break;

        /* 首趟 1 的正式比赛路径只等待 Z 越过离地阈值后启动的取物朝向旋转。 */
        case CRANE_ROUTE_WAIT_FIRST_PICK_LIFT_SAFE:
            if ((crane_route.current_slot == 1U) &&
                (crane_route.first_pick_prestart_active != 0U))
            {
                if (crane_route.first_pick_rotate_released != 0U)
                {
                    crane_route_enter_state(CRANE_ROUTE_WAIT_FIRST_PICK_ROTATE);
                }
                else if (crane_route_state_timed_out(CRANE_ROUTE_MOTION_TIMEOUT_MS) != 0U)
                {
                    crane_route_fault_from_wait();
                }
                break;
            }

            if ((crane_route_z_arrived() != 0U) &&
                ((crane_route.current_slot != 1U) ||
                 (crane_route_claw_y_arrived() != 0U)))
            {
                if (crane_route.current_slot != 1U)
                {
                    crane_route_move_claw_y(
                        crane_route_slot_pose[crane_route.current_slot].beam_pos);
                }
                servo3_path_release_pick_area();
                crane_route_enter_state(CRANE_ROUTE_WAIT_FIRST_PICK_ROTATE);
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_MOTION_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_from_wait();
            }
            break;

        /* 旋转完成后，Z 下到取物接近高度；首趟 1 的 Y 已与 X 同时直达目标。 */
        case CRANE_ROUTE_WAIT_FIRST_PICK_ROTATE:
            if (servo3_path_is_arrived() != 0U)
            {
                if (crane_route.current_slot == 1U)
                {
                    crane_route_move_claw_y(
                        crane_route_slot_pose[crane_route.current_slot].beam_pos);
                }
                crane_route_move_z(
                    crane_route_pick_approach_z(crane_route.current_slot));
                crane_route_enter_state(CRANE_ROUTE_WAIT_PICK_APPROACH);
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_SERVO_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_stop(CRANE_ROUTE_FAULT_SERVO_TIMEOUT);
            }
            break;

        /*
         * 仅首趟 3 号需要走取物侧下绕：X 继续朝 3 号走，夹爪 Y 先到
         * 下绕入口。X 接近入口时，Y 已到位就直接放行至 3 号目标 Y；
         * 否则保持 X，直到 Y 到位。
         */
        case CRANE_ROUTE_WAIT_PICK3_LOWER_ENTRY:
            if (crane_route_x_near_pick3_lower_entry() != 0U)
            {
                if (crane_route_claw_y_arrived() != 0U)
                {
                    crane_route_move_claw_y(
                        crane_route_slot_pose[crane_route.current_slot].beam_pos);
                    crane_route_enter_state(CRANE_ROUTE_WAIT_PICK_APPROACH);
                }
                else
                {
                    crane_route_hold_x();
                    crane_route_enter_state(CRANE_ROUTE_HOLD_X_WAIT_PICK3_LOWER_Y);
                }
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_MOTION_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_from_wait();
            }
            break;

        case CRANE_ROUTE_HOLD_X_WAIT_PICK3_LOWER_Y:
            if (crane_route_claw_y_arrived() != 0U)
            {
                crane_route_move_x(
                    crane_route_slot_pose[crane_route.current_slot].chassis_pos);
                crane_route_move_claw_y(
                    crane_route_slot_pose[crane_route.current_slot].beam_pos);
                crane_route_enter_state(CRANE_ROUTE_WAIT_PICK_APPROACH);
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_MOTION_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_from_wait();
            }
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
            crane_route_move_z(crane_route_pick_work_z(crane_route.current_slot));
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

            /*
             * 最后一颗从 1/2 号抓取时：Z 越过脱盒高度便立即让爪子 Y
             * 回原点；Y 与仍在上升的 Z 并行，到齐后才旋转爪子。
             */
            if ((crane_route_final_pick_needs_y_return() != 0U) &&
                (crane_route.final_pick_y_return_started == 0U) &&
                (lift_ctrl_get_current_pos() >=
                 (crane_route_pick_box_clear_z(crane_route.current_slot) -
                  CRANE_ROUTE_Z_TOL)))
            {
                crane_route_move_claw_y(crane_route_slot_pose[0U].beam_pos);
                crane_route.final_pick_y_return_started = 1U;
            }

            if ((crane_route_z_arrived() != 0U) &&
                ((crane_route_final_pick_needs_y_return() == 0U) ||
                 ((crane_route.final_pick_y_return_started != 0U) &&
                  (crane_route_claw_y_arrived() != 0U))))
            {
                if (crane_route_carrier_for_pick(crane_route.current_slot) ==
                    CRANE_CARRIER_CLAW)
                {
                    /*
                     * 最后一颗由夹爪直接携带：保持原逻辑，按取物槽
                     * 选择 1/2/3 号位对应的最终旋转路径。
                     */
                    servo3_path_release_place_for_pick(crane_route.current_slot);
                    crane_route_enter_state(CRANE_ROUTE_WAIT_FINAL_ROTATE_PLACE);
                }
                else
                {
                    /*
                     * 转交上/下料斗时，Z 到安全高度后先启动夹爪 Y；
                     * 符合跨 Y=0 条件的两种组合允许途中提前旋转。
                     */
                    crane_route_enter_state(CRANE_ROUTE_MOVE_CLAW_TO_HOPPER);
                }
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_MOTION_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_from_wait();
            }
            break;

        case CRANE_ROUTE_WAIT_ROTATE_TO_LOAD:
            if (servo3_path_is_arrived() != 0U)
            {
                crane_route_enter_state(CRANE_ROUTE_WAIT_HOPPER_OPEN_DELAY);
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_SERVO_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_stop(CRANE_ROUTE_FAULT_SERVO_TIMEOUT);
            }
            break;

        case CRANE_ROUTE_MOVE_CLAW_TO_HOPPER:
            crane_route.hopper_rotate_started = 0U;
            crane_route_move_claw_y(crane_route_load_y_for_pick(crane_route.current_slot));
            crane_route_enter_state(CRANE_ROUTE_WAIT_CLAW_AT_HOPPER);
            break;

        case CRANE_ROUTE_WAIT_CLAW_AT_HOPPER:
        {
            uint8_t claw_y_arrived = crane_route_claw_y_arrived();

            /*
             * 跨中线组合在实际 Y 越过 0 后立即下发旋转目标，Y 继续运动；
             * 其余组合仍在 Y 到位时才下发旋转目标。
             */
            if ((crane_route.hopper_rotate_started == 0U) &&
                ((claw_y_arrived != 0U) ||
                 (crane_route_hopper_early_rotate_ready(
                      crane_route.current_slot) != 0U)))
            {
                if (crane_route_start_hopper_rotation() == 0U)
                {
                    crane_route_fault_stop(CRANE_ROUTE_FAULT_CONFIG);
                    break;
                }
            }

            /* 放豆前仍保持原屏障：Y 必须到料斗位，旋转也必须确认完成。 */
            if ((claw_y_arrived != 0U) &&
                (crane_route.hopper_rotate_started != 0U))
            {
                crane_route_enter_state(CRANE_ROUTE_WAIT_ROTATE_TO_LOAD);
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_MOTION_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_from_wait();
            }
            break;
        }

        case CRANE_ROUTE_WAIT_HOPPER_OPEN_DELAY:
            if ((HAL_GetTick() - crane_route.state_tick) >=
                CRANE_ROUTE_HOPPER_OPEN_DELAY_MS)
            {
                crane_route_enter_state(CRANE_ROUTE_RELEASE_TO_HOPPER);
            }
            break;

        case CRANE_ROUTE_RELEASE_TO_HOPPER:
            crane_route.hopper_release_open_tick = 0U;
            claw_open_hopper();
            crane_route_enter_state(CRANE_ROUTE_WAIT_RELEASE_TO_HOPPER);
            break;

        case CRANE_ROUTE_WAIT_RELEASE_TO_HOPPER:
            if (claw_is_hopper_open() != 0U)
            {
                if (crane_route.hopper_release_open_tick == 0U)
                {
                    crane_route.hopper_release_open_tick = HAL_GetTick();
                }
                else if ((HAL_GetTick() - crane_route.hopper_release_open_tick) >=
                         CRANE_ROUTE_HOPPER_RELEASE_HOLD_MS)
                {
                    crane_route_enter_state(CRANE_ROUTE_CLOSE_AND_ROTATE_BACK);
                }
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_SERVO_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_stop(CRANE_ROUTE_FAULT_SERVO_TIMEOUT);
            }
            break;

        case CRANE_ROUTE_CLOSE_AND_ROTATE_BACK:
            claw_close();
            /*
             * 料斗放豆完成后按同一路径返回取物朝向：
             * 上料斗 270->135；下料斗 5->135。
             */
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
                if ((crane_route.current_slot == 1U) ||
                    (crane_route.current_slot == 2U))
                {
                    /*
                     * 最后从 1/2 号取物时，X 必须先回到 3 号取物 X 位，
                     * 才允许爪子、上斗和下斗同时进入上避障通道。
                     */
                    crane_route_move_x(crane_route_slot_pose[3U].chassis_pos);
                    crane_route_enter_state(CRANE_ROUTE_WAIT_FINAL_PICK_X_AT_SLOT3);
                }
                else
                {
                    /* 最后取 3 号时，X 已在 3 号位，保持原有避障顺序。 */
                    crane_route_hold_x();
                    crane_route_enter_state(CRANE_ROUTE_MOVE_ALL_TO_UPPER_SAFE);
                }
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_SERVO_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_stop(CRANE_ROUTE_FAULT_SERVO_TIMEOUT);
            }
            break;

        case CRANE_ROUTE_WAIT_FINAL_PICK_X_AT_SLOT3:
            if (crane_route_x_arrived() != 0U)
            {
                crane_route_enter_state(CRANE_ROUTE_MOVE_ALL_TO_UPPER_SAFE);
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_MOTION_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_from_wait();
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
            if (crane_route_x_reached(CRANE_ROUTE_X_OBS1_PRE_10) != 0U)
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
            if (crane_route_x_reached(CRANE_ROUTE_X_OBS2_PRE_30) != 0U)
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
            if (crane_route_x_reached(CRANE_ROUTE_X_OBS2_POST_30) != 0U)
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
            /* 斗门只等待共用 X 和各自 Y，与夹爪 Y/Z 并行释放。 */
            crane_route_process_remaining_gate_release();

            /* Z 最终下放只等待承载夹爪的 X 与横梁 Y。 */
            if ((crane_route_x_arrived() != 0U) &&
                (crane_route_claw_y_arrived() != 0U))
            {
                crane_route_enter_state(CRANE_ROUTE_PLACE_DESCEND);
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_MOTION_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_from_wait();
            }
            break;

        /* 爪子 X/Y 到最终放置位后，Z 只到可在盒子上方放豆的标定高度。 */
        case CRANE_ROUTE_PLACE_DESCEND:
            crane_route_process_remaining_gate_release();
            crane_route_move_z(CRANE_ROUTE_LIFT_PLACE_POS);
            crane_route_enter_state(CRANE_ROUTE_WAIT_PLACE_DESCEND);
            break;

        case CRANE_ROUTE_WAIT_PLACE_DESCEND:
            crane_route_process_remaining_gate_release();
            if (crane_route_z_arrived() != 0U)
            {
                /* 最终放置与放入料斗统一：夹爪只张到 40°。 */
                claw_open_hopper();
                crane_route_enter_state(CRANE_ROUTE_WAIT_REMAINING_RELEASE);
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_MOTION_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_from_wait();
            }
            break;

        case CRANE_ROUTE_WAIT_REMAINING_RELEASE:
            crane_route_process_remaining_gate_release();

            if (((crane_route.release_mask & CRANE_RELEASE_CLAW_MASK) == 0U) &&
                (claw_is_hopper_open() != 0U))
            {
                crane_route.release_mask |= CRANE_RELEASE_CLAW_MASK;
            }
            if ((crane_route.release_mask & CRANE_RELEASE_ALL_MASK) ==
                CRANE_RELEASE_ALL_MASK)
            {
                /* 三路豆子均已放完：保持当前位置和开门/开爪状态，直接结束比赛。 */
                crane_route_enter_state(CRANE_ROUTE_FINISHED);
            }
            else if (crane_route_state_timed_out(CRANE_ROUTE_GATE_TIMEOUT_MS) != 0U)
            {
                crane_route_fault_stop(CRANE_ROUTE_FAULT_GATE_UNAVAILABLE);
            }
            break;

        case CRANE_ROUTE_LIFT_RETURN_CLEAR:
            /* 放豆完成后不再下行/停留在脱盒高度，直接回全局安全高度。 */
            crane_route_move_z(CRANE_ROUTE_LIFT_SAFE_POS);
            crane_route_enter_state(CRANE_ROUTE_WAIT_LIFT_RETURN_CLEAR);
            break;

        case CRANE_ROUTE_WAIT_LIFT_RETURN_CLEAR:
            if (crane_route_z_arrived() != 0U)
            {
                /* Z 已回安全高度后闭爪，再保持原有三套 Y 的上侧回避流程。 */
                claw_close();
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
            if ((crane_route_all_y_arrived() != 0U) &&
                (crane_route_z_arrived() != 0U) &&
                (claw_is_closed() != 0U))
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

float crane_route_get_slot_chassis_pos(uint8_t slot)
{
    if ((slot == 0U) || (slot > CRANE_ROUTE_SLOT_COUNT))
    {
        return 0.0f;
    }

    return crane_route_slot_pose[slot].chassis_pos;
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

void crane_route_get_upper_safe_y(float *claw_y,
                                  float *upper_hopper_y,
                                  float *lower_hopper_y)
{
    if (claw_y != NULL)
    {
        *claw_y = CRANE_ROUTE_CLAW_UPPER_SAFE_Y;
    }
    if (upper_hopper_y != NULL)
    {
        *upper_hopper_y = CRANE_ROUTE_UPPER_UPPER_SAFE_Y;
    }
    if (lower_hopper_y != NULL)
    {
        *lower_hopper_y = CRANE_ROUTE_LOWER_UPPER_SAFE_Y;
    }
}

void crane_route_get_lower_transition_y(float *claw_y,
                                        float *upper_hopper_y,
                                        float *lower_hopper_y)
{
    if (claw_y != NULL)
    {
        *claw_y = CRANE_ROUTE_CLAW_LOWER_TRANS_Y;
    }
    if (upper_hopper_y != NULL)
    {
        *upper_hopper_y = CRANE_ROUTE_UPPER_LOWER_TRANS_Y;
    }
    if (lower_hopper_y != NULL)
    {
        *lower_hopper_y = CRANE_ROUTE_LOWER_LOWER_TRANS_Y;
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
