#include "headfile.h"

/*
 * XY 绕障协调器
 *
 * 核心时序：
 * - Y 轴可以先去绕行入口 Y。
 * - X 轴启动后尽量放开跑。
 * - X 到入口前检查 Y 是否到入口 Y，没到就停 X 等 Y。
 * - 完整绕桩段：入口通过后 Y 去出口 Y，X 到出口时再检查 Y。
 * - 对角线优化段：入口通过后 Y 直接去最终目标 Y，不再等出口。
 */

/* X 入口/出口点：-X 侧靠近取货区，+X 侧靠近放置区。 */
#define XY_ROUTE_X_ENTRY_PICK_SIDE       -750.0f
#define XY_ROUTE_X_ENTRY_PLACE_SIDE     500.0f

/* 上绕：入口在 +20，出口在 -20。 */
#define XY_ROUTE_Y_UP_ENTRY               -8.0f
#define XY_ROUTE_Y_UP_EXIT               8.0f

/* 下绕：入口在 -20，出口在 +20。 */
#define XY_ROUTE_Y_DOWN_ENTRY            8.0f
#define XY_ROUTE_Y_DOWN_EXIT              -8.0f

/* X 到入口前提前多少开始检查 Y，给 X 停车留余量。 */
#define XY_ROUTE_X_WAIT_MARGIN            25.0f

/* 入口/出口 Y 到位容差。 */
#define XY_ROUTE_Y_ENTRY_TOL               2.0f
#define XY_ROUTE_Y_EXIT_TOL                2.0f

typedef enum
{
    XY_ROUTE_SIDE_PLACE = 0,
    XY_ROUTE_SIDE_PICK
} xy_route_side_e;

typedef struct
{
    xy_route_state_e state;
    xy_route_type_e route_type;
    xy_release_mode_e release_mode;
    float start_x;
    float start_y;
    float target_x;
    float target_y;
    float entry_x;
    float entry_y;
    float exit_x;
    float exit_y;
    int8_t x_dir;
    uint8_t busy;
    uint8_t y_only;
    uint8_t servo3_entry_enable;
    uint8_t servo3_exit_enable;
    uint8_t servo3_entry_done;
    uint8_t servo3_exit_done;
    uint8_t servo3_entry_wait_enable;
    uint8_t servo3_wait_enable;
    uint8_t servo3_wait_done;
    uint8_t servo3_wait_at_entry_gate;
    xy_route_state_e servo3_wait_next_state;
    float servo3_entry_angle_deg;
    float servo3_exit_angle_deg;
    float servo3_wait_y;
    float servo3_wait_angle_deg;
} xy_route_t;

typedef struct
{
    uint8_t entry_enable;
    uint8_t exit_enable;
    uint8_t entry_wait_enable;
    uint8_t wait_enable;
    float entry_angle_deg;
    float exit_angle_deg;
    float wait_y;
    float wait_angle_deg;
} xy_route_servo3_pending_t;

static xy_route_t xy_route;
static xy_route_servo3_pending_t xy_route_servo3_pending;

static float xy_route_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static uint8_t xy_route_is_bypass(xy_route_type_e route_type)
{
    return ((route_type == XY_ROUTE_UP) || (route_type == XY_ROUTE_DOWN)) ? 1U : 0U;
}

static uint8_t xy_route_is_center_bypass(xy_route_type_e route_type)
{
    return ((route_type == XY_ROUTE_CENTER_UP_EXIT) ||
            (route_type == XY_ROUTE_CENTER_DOWN_EXIT)) ? 1U : 0U;
}

static xy_route_side_e xy_route_side_from_x(float x, float target_x)
{
    if (x <= XY_ROUTE_X_ENTRY_PICK_SIDE)
    {
        return XY_ROUTE_SIDE_PICK;
    }

    if (x >= XY_ROUTE_X_ENTRY_PLACE_SIDE)
    {
        return XY_ROUTE_SIDE_PLACE;
    }

    return (target_x <= 0.0f) ? XY_ROUTE_SIDE_PICK : XY_ROUTE_SIDE_PLACE;
}

static float xy_route_x_for_side(xy_route_side_e side)
{
    return (side == XY_ROUTE_SIDE_PICK) ? XY_ROUTE_X_ENTRY_PICK_SIDE : XY_ROUTE_X_ENTRY_PLACE_SIDE;
}

static xy_route_side_e xy_route_other_side(xy_route_side_e side)
{
    return (side == XY_ROUTE_SIDE_PICK) ? XY_ROUTE_SIDE_PLACE : XY_ROUTE_SIDE_PICK;
}

static float xy_route_gate_y_for_side(xy_route_type_e route_type,
                                      xy_route_side_e side,
                                      float final_y)
{
    if (route_type == XY_ROUTE_UP)
    {
        return (side == XY_ROUTE_SIDE_PICK) ? XY_ROUTE_Y_UP_ENTRY : XY_ROUTE_Y_UP_EXIT;
    }

    if (route_type == XY_ROUTE_DOWN)
    {
        return (side == XY_ROUTE_SIDE_PICK) ? XY_ROUTE_Y_DOWN_ENTRY : XY_ROUTE_Y_DOWN_EXIT;
    }

    return final_y;
}

static float xy_route_entry_y_for(xy_route_type_e route_type, float final_y)
{
    if (route_type == XY_ROUTE_UP)
    {
        return XY_ROUTE_Y_UP_ENTRY;
    }

    if (route_type == XY_ROUTE_DOWN)
    {
        return XY_ROUTE_Y_DOWN_ENTRY;
    }

    if (route_type == XY_ROUTE_CENTER_UP_EXIT)
    {
        return XY_ROUTE_Y_UP_EXIT;
    }

    if (route_type == XY_ROUTE_CENTER_DOWN_EXIT)
    {
        return XY_ROUTE_Y_DOWN_EXIT;
    }

    return final_y;
}

static float xy_route_exit_y_for(xy_route_type_e route_type, float final_y)
{
    if (route_type == XY_ROUTE_UP)
    {
        return XY_ROUTE_Y_UP_EXIT;
    }

    if (route_type == XY_ROUTE_DOWN)
    {
        return XY_ROUTE_Y_DOWN_EXIT;
    }

    if (route_type == XY_ROUTE_CENTER_UP_EXIT)
    {
        return XY_ROUTE_Y_UP_EXIT;
    }

    if (route_type == XY_ROUTE_CENTER_DOWN_EXIT)
    {
        return XY_ROUTE_Y_DOWN_EXIT;
    }

    return final_y;
}

static void xy_route_config_gate(float target_x, float target_y, xy_route_type_e route_type)
{
    xy_route_side_e entry_side = xy_route_side_from_x(xy_route.start_x, target_x);
    xy_route_side_e target_side = xy_route_side_from_x(target_x, target_x);
    xy_route_side_e exit_side = (entry_side == target_side) ? entry_side : xy_route_other_side(entry_side);

    xy_route.entry_x = xy_route_x_for_side(entry_side);
    xy_route.exit_x = xy_route_x_for_side(exit_side);
    xy_route.entry_y = xy_route_gate_y_for_side(route_type, entry_side, target_y);
    xy_route.exit_y = xy_route_gate_y_for_side(route_type, exit_side, target_y);
}

static uint8_t xy_route_y_at_entry(void)
{
    float y_now = beam_ctrl_get_current_pos();
    return (xy_route_absf(y_now - xy_route.entry_y) <= XY_ROUTE_Y_ENTRY_TOL) ? 1U : 0U;
}

static uint8_t xy_route_y_at_exit(void)
{
    float y_now = beam_ctrl_get_current_pos();
    return (xy_route_absf(y_now - xy_route.exit_y) <= XY_ROUTE_Y_EXIT_TOL) ? 1U : 0U;
}

static uint8_t xy_route_x_near_entry(void)
{
    float x_now = pos_pid_sync_get_current_pos();

    if (xy_route.x_dir >= 0)
    {
        return (x_now >= (xy_route.entry_x - XY_ROUTE_X_WAIT_MARGIN)) ? 1U : 0U;
    }

    return (x_now <= (xy_route.entry_x + XY_ROUTE_X_WAIT_MARGIN)) ? 1U : 0U;
}

static uint8_t xy_route_x_at_exit(void)
{
    float x_now = pos_pid_sync_get_current_pos();

    if (xy_route.x_dir >= 0)
    {
        return (x_now >= xy_route.exit_x) ? 1U : 0U;
    }

    return (x_now <= xy_route.exit_x) ? 1U : 0U;
}

static uint8_t xy_route_x_past_exit(void)
{
    return xy_route_x_at_exit();
}

static void xy_route_hold_x_at_current(void)
{
    float x_now = pos_pid_sync_get_current_pos();
    pos_pid_sync_set_target(x_now);
    pos_pid_sync_start();
}

static void xy_route_run_x_to_target(void)
{
    pos_pid_sync_set_target(xy_route.target_x);
    pos_pid_sync_start();
}

static void xy_route_run_y_to(float target_y)
{
    beam_ctrl_set_target(target_y);
    beam_ctrl_start();
}

static void xy_route_load_servo3_pending(void)
{
    xy_route.servo3_entry_enable = xy_route_servo3_pending.entry_enable;
    xy_route.servo3_exit_enable = xy_route_servo3_pending.exit_enable;
    xy_route.servo3_entry_wait_enable = xy_route_servo3_pending.entry_wait_enable;
    xy_route.servo3_wait_enable = xy_route_servo3_pending.wait_enable;
    xy_route.servo3_entry_angle_deg = xy_route_servo3_pending.entry_angle_deg;
    xy_route.servo3_exit_angle_deg = xy_route_servo3_pending.exit_angle_deg;
    xy_route.servo3_wait_y = xy_route_servo3_pending.wait_y;
    xy_route.servo3_wait_angle_deg = xy_route_servo3_pending.wait_angle_deg;
    xy_route.servo3_entry_done = 0U;
    xy_route.servo3_exit_done = 0U;
    xy_route.servo3_wait_done = 0U;
    xy_route.servo3_wait_next_state = XY_ROUTE_DONE;
    memset(&xy_route_servo3_pending, 0, sizeof(xy_route_servo3_pending));
}

static void xy_route_trigger_servo3_entry(void)
{
    if ((xy_route.servo3_entry_enable != 0U) && (xy_route.servo3_entry_done == 0U))
    {
        servo3_path_release_angle(xy_route.servo3_entry_angle_deg);
        xy_route.servo3_entry_done = 1U;
    }
}

static void xy_route_trigger_servo3_exit(void)
{
    if ((xy_route.servo3_exit_enable != 0U) && (xy_route.servo3_exit_done == 0U))
    {
        servo3_path_release_angle(xy_route.servo3_exit_angle_deg);
        xy_route.servo3_exit_done = 1U;
    }
}

static void xy_route_run_final_y_or_wait(xy_route_state_e next_state)
{
    if ((xy_route.servo3_wait_enable != 0U) && (xy_route.servo3_wait_done == 0U))
    {
        xy_route_run_y_to(xy_route.servo3_wait_y);
        xy_route.servo3_wait_next_state = next_state;
        xy_route.state = XY_ROUTE_SERVO_WAIT_SLOT;
        return;
    }

    xy_route_run_y_to(xy_route.target_y);
    xy_route.state = next_state;
}

/* 入口第一段旋转结束后，才允许 X/Y 进入后续绕障段。 */
static void xy_route_continue_after_entry(void)
{
    if (xy_route.y_only == 0U)
    {
        xy_route_run_x_to_target();
    }

    if (xy_route.release_mode == XY_RELEASE_AFTER_ENTRY)
    {
        xy_route_run_final_y_or_wait(XY_ROUTE_BYPASS_TO_TARGET);
    }
    else
    {
        xy_route_run_y_to(xy_route.exit_y);
        xy_route.state = (xy_route.y_only != 0U) ?
                         XY_ROUTE_BYPASS_WAIT_EXIT_Y :
                         XY_ROUTE_BYPASS_TO_EXIT;
    }
}

/* 4/8 放置：入口先旋到 180 度，旋转期间固定 X/Y。 */
static void xy_route_handle_entry(void)
{
    xy_route_trigger_servo3_entry();

    if ((xy_route.servo3_entry_wait_enable != 0U) &&
        (servo3_path_is_arrived() == 0U))
    {
        if (xy_route.y_only == 0U)
        {
            xy_route_hold_x_at_current();
        }
        xy_route.state = XY_ROUTE_SERVO_WAIT_ENTRY_ROTATE;
        return;
    }

    xy_route_continue_after_entry();
}

static void xy_route_start_common(float target_x,
                                  float target_y,
                                  xy_route_type_e route_type,
                                  xy_release_mode_e release_mode)
{
    xy_route.start_x = pos_pid_sync_get_current_pos();
    xy_route.start_y = beam_ctrl_get_current_pos();
    xy_route.target_x = target_x;
    xy_route.target_y = target_y;
    xy_route.route_type = route_type;
    xy_route.release_mode = release_mode;
    xy_route.x_dir = (target_x >= xy_route.start_x) ? 1 : -1;
    xy_route.busy = 1U;
    xy_route.y_only = 0U;
    xy_route.servo3_wait_at_entry_gate = 0U;
    xy_route_load_servo3_pending();
}

void xy_route_init(void)
{
    memset(&xy_route, 0, sizeof(xy_route));
    memset(&xy_route_servo3_pending, 0, sizeof(xy_route_servo3_pending));
    xy_route.state = XY_ROUTE_IDLE;
}

void xy_route_set_servo3_triggers(uint8_t entry_enable, float entry_angle_deg,
                                  uint8_t exit_enable, float exit_angle_deg)
{
    xy_route_servo3_pending.entry_enable = (entry_enable != 0U) ? 1U : 0U;
    xy_route_servo3_pending.exit_enable = (exit_enable != 0U) ? 1U : 0U;
    xy_route_servo3_pending.entry_angle_deg = entry_angle_deg;
    xy_route_servo3_pending.exit_angle_deg = exit_angle_deg;
}

void xy_route_set_servo3_target_wait(uint8_t enable, float wait_y, float angle_deg)
{
    xy_route_servo3_pending.wait_enable = (enable != 0U) ? 1U : 0U;
    xy_route_servo3_pending.wait_y = wait_y;
    xy_route_servo3_pending.wait_angle_deg = angle_deg;
}

void xy_route_prepare_y(float target_x, xy_route_type_e route_type, float final_y)
{
    float start_x = pos_pid_sync_get_current_pos();
    xy_route_side_e entry_side = xy_route_side_from_x(start_x, target_x);
    float y_target = xy_route_gate_y_for_side(route_type, entry_side, final_y);

    if (xy_route_is_center_bypass(route_type) != 0U)
    {
        y_target = xy_route_exit_y_for(route_type, final_y);
    }

    xy_route_run_y_to(y_target);
}

void xy_route_start(float target_x,
                    float target_y,
                    xy_route_type_e route_type,
                    xy_release_mode_e release_mode)
{
    xy_route_start_common(target_x, target_y, route_type, release_mode);

    if (xy_route_is_center_bypass(route_type) != 0U)
    {
        xy_route.exit_x = xy_route_x_for_side(xy_route_side_from_x(xy_route.start_x, target_x));
        xy_route.exit_y = xy_route_exit_y_for(route_type, target_y);
        xy_route_run_y_to(xy_route.exit_y);
        xy_route_run_x_to_target();
        xy_route.state = XY_ROUTE_CENTER_TO_EXIT;
        return;
    }

    if (xy_route_is_bypass(route_type) == 0U)
    {
        xy_route_run_x_to_target();
        xy_route_run_final_y_or_wait(XY_ROUTE_DIRECT_RUN);
        return;
    }

    xy_route_config_gate(target_x, target_y, route_type);
    xy_route_run_y_to(xy_route.entry_y);
    xy_route_run_x_to_target();
    xy_route.state = XY_ROUTE_BYPASS_TO_ENTRY;
}

void xy_route_set_servo3_entry_wait(uint8_t enable)
{
    xy_route_servo3_pending.entry_wait_enable = (enable != 0U) ? 1U : 0U;
}

void xy_route_start_extreme_return(float target_x,
                                   float target_y,
                                   xy_route_type_e route_type,
                                   xy_release_mode_e release_mode,
                                   float safe_y,
                                   float servo3_angle_deg)
{
    /* 这一路径不使用常规入口/出口舵机触发，避免提前回转。 */
    xy_route_set_servo3_triggers(0U, 0.0f, 0U, 0.0f);
    xy_route_set_servo3_entry_wait(0U);
    xy_route_set_servo3_target_wait(1U, safe_y, servo3_angle_deg);
    xy_route_start_common(target_x, target_y, route_type, release_mode);
    xy_route.servo3_wait_at_entry_gate = 1U;

    if (xy_route_is_center_bypass(route_type) != 0U)
    {
        xy_route.exit_x = xy_route_x_for_side(xy_route_side_from_x(xy_route.start_x, target_x));
        xy_route.exit_y = xy_route_exit_y_for(route_type, target_y);
        xy_route_run_y_to(safe_y);
        xy_route_run_x_to_target();
        xy_route.state = XY_ROUTE_CENTER_TO_EXIT;
        return;
    }

    if (xy_route_is_bypass(route_type) == 0U)
    {
        /* 直线返程：X 起动后立刻在安全 Y 点回转 servo3。 */
        xy_route_run_x_to_target();
        xy_route_run_final_y_or_wait(XY_ROUTE_DIRECT_RUN);
        return;
    }

    /* 绕障返程：先让 X 从极限位走到第一个入口，再回转 servo3。 */
    xy_route_config_gate(target_x, target_y, route_type);
    xy_route_run_y_to(safe_y);
    xy_route_run_x_to_target();
    xy_route.state = XY_ROUTE_BYPASS_TO_ENTRY;
}

void xy_route_start_y_only(float target_y,
                           xy_route_type_e route_type,
                           xy_release_mode_e release_mode)
{
    xy_route.start_y = beam_ctrl_get_current_pos();
    xy_route.target_x = pos_pid_sync_get_current_pos();
    xy_route.target_y = target_y;
    xy_route.route_type = route_type;
    xy_route.release_mode = release_mode;
    xy_route.busy = 1U;
    xy_route.y_only = 1U;
    xy_route.servo3_wait_at_entry_gate = 0U;
    xy_route_load_servo3_pending();

    xy_route.entry_y = xy_route_entry_y_for(route_type, target_y);
    xy_route.exit_y = xy_route_exit_y_for(route_type, target_y);

    if (xy_route_is_center_bypass(route_type) != 0U)
    {
        xy_route_run_y_to(xy_route.exit_y);
        xy_route.state = XY_ROUTE_CENTER_WAIT_EXIT_Y;
        return;
    }

    if (xy_route_is_bypass(route_type) == 0U)
    {
        xy_route_run_final_y_or_wait(XY_ROUTE_DIRECT_RUN);
        return;
    }

    xy_route_run_y_to(xy_route.entry_y);
    xy_route.state = XY_ROUTE_BYPASS_WAIT_ENTRY_Y;
}

void xy_route_stop(void)
{
    xy_route.state = XY_ROUTE_IDLE;
    xy_route.busy = 0U;
    if (xy_route.y_only == 0U)
    {
        pos_pid_sync_stop();
    }
    beam_ctrl_stop();
    xy_route.y_only = 0U;
}

void xy_route_process(void)
{
    switch (xy_route.state)
    {
        case XY_ROUTE_IDLE:
        case XY_ROUTE_DONE:
            break;

        case XY_ROUTE_DIRECT_RUN:
            if (((xy_route.y_only != 0U) || (pos_pid_sync_is_busy() == 0U)) &&
                (beam_ctrl_is_busy() == 0U))
            {
                xy_route.busy = 0U;
                xy_route.state = XY_ROUTE_DONE;
            }
            break;

        case XY_ROUTE_BYPASS_TO_ENTRY:
            if (xy_route_x_near_entry() != 0U)
            {
                if ((xy_route.servo3_wait_at_entry_gate != 0U) &&
                    (xy_route.servo3_wait_enable != 0U) &&
                    (xy_route.servo3_wait_done == 0U))
                {
                    xy_route_hold_x_at_current();
                    xy_route.servo3_wait_next_state = XY_ROUTE_BYPASS_WAIT_ENTRY_Y;
                    xy_route.state = XY_ROUTE_SERVO_WAIT_SLOT;
                }
                else
                if (xy_route_y_at_entry() != 0U)
                {
                    xy_route_handle_entry();
                }
                else
                {
                    xy_route_hold_x_at_current();
                    xy_route.state = XY_ROUTE_BYPASS_WAIT_ENTRY_Y;
                }
            }
            break;

        case XY_ROUTE_BYPASS_WAIT_ENTRY_Y:
            if (xy_route_y_at_entry() != 0U)
            {
                xy_route_handle_entry();
            }
            break;

        case XY_ROUTE_BYPASS_TO_EXIT:
            if (xy_route_x_at_exit() != 0U)
            {
                if (xy_route_y_at_exit() != 0U)
                {
                    xy_route_trigger_servo3_exit();
                    xy_route_run_final_y_or_wait(XY_ROUTE_BYPASS_TO_TARGET);
                }
                else
                {
                    xy_route_hold_x_at_current();
                    xy_route.state = XY_ROUTE_BYPASS_WAIT_EXIT_Y;
                }
            }
            break;

        case XY_ROUTE_BYPASS_WAIT_EXIT_Y:
            if (xy_route_y_at_exit() != 0U)
            {
                xy_route_trigger_servo3_exit();
                if (xy_route.y_only == 0U)
                {
                    xy_route_run_x_to_target();
                }
                xy_route_run_final_y_or_wait(XY_ROUTE_BYPASS_TO_TARGET);
            }
            break;

        case XY_ROUTE_BYPASS_TO_TARGET:
            if (((xy_route.y_only != 0U) || (pos_pid_sync_is_busy() == 0U)) &&
                (beam_ctrl_is_busy() == 0U))
            {
                xy_route.busy = 0U;
                xy_route.state = XY_ROUTE_DONE;
            }
            break;

        case XY_ROUTE_SERVO_WAIT_ENTRY_ROTATE:
            if (servo3_path_is_arrived() != 0U)
            {
                xy_route_continue_after_entry();
            }
            break;

        case XY_ROUTE_SERVO_WAIT_SLOT:
            if (beam_ctrl_is_busy() == 0U)
            {
                servo3_path_release_angle(xy_route.servo3_wait_angle_deg);
                if (servo3_path_is_arrived() != 0U)
                {
                    xy_route.servo3_wait_done = 1U;
                    if (xy_route.servo3_wait_next_state == XY_ROUTE_BYPASS_WAIT_ENTRY_Y)
                    {
                        xy_route_run_y_to(xy_route.entry_y);
                    }
                    else if (xy_route.servo3_wait_next_state == XY_ROUTE_CENTER_WAIT_EXIT_Y)
                    {
                        xy_route_run_y_to(xy_route.exit_y);
                    }
                    else
                    {
                        xy_route_run_y_to(xy_route.target_y);
                    }
                    xy_route.state = xy_route.servo3_wait_next_state;
                }
            }
            break;

        case XY_ROUTE_CENTER_TO_EXIT:
            if (xy_route_x_past_exit() != 0U)
            {
                if ((xy_route.servo3_wait_at_entry_gate != 0U) &&
                    (xy_route.servo3_wait_enable != 0U) &&
                    (xy_route.servo3_wait_done == 0U))
                {
                    xy_route_hold_x_at_current();
                    xy_route.servo3_wait_next_state = XY_ROUTE_CENTER_WAIT_EXIT_Y;
                    xy_route.state = XY_ROUTE_SERVO_WAIT_SLOT;
                }
                else
                if (xy_route_y_at_exit() != 0U)
                {
                    xy_route_trigger_servo3_exit();
                    xy_route_run_final_y_or_wait(XY_ROUTE_CENTER_TO_TARGET);
                }
                else
                {
                    xy_route_hold_x_at_current();
                    xy_route.state = XY_ROUTE_CENTER_WAIT_EXIT_Y;
                }
            }
            break;

        case XY_ROUTE_CENTER_WAIT_EXIT_Y:
            if (xy_route_y_at_exit() != 0U)
            {
                xy_route_trigger_servo3_exit();
                if (xy_route.y_only == 0U)
                {
                    xy_route_run_x_to_target();
                }
                xy_route_run_final_y_or_wait(XY_ROUTE_CENTER_TO_TARGET);
            }
            break;

        case XY_ROUTE_CENTER_TO_TARGET:
            if (((xy_route.y_only != 0U) || (pos_pid_sync_is_busy() == 0U)) &&
                (beam_ctrl_is_busy() == 0U))
            {
                xy_route.busy = 0U;
                xy_route.state = XY_ROUTE_DONE;
            }
            break;

        default:
            xy_route_stop();
            break;
    }
}

uint8_t xy_route_is_busy(void)
{
    return xy_route.busy;
}

uint8_t xy_route_is_done(void)
{
    return (xy_route.state == XY_ROUTE_DONE) ? 1U : 0U;
}

xy_route_state_e xy_route_get_state(void)
{
    return xy_route.state;
}

float xy_route_get_target_x(void)
{
    return xy_route.target_x;
}

float xy_route_get_target_y(void)
{
    return xy_route.target_y;
}
