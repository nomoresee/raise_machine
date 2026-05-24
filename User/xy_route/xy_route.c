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

/* X 入口/出口点：+X 侧靠近取货区，-X 侧靠近放置区。 */
#define XY_ROUTE_X_ENTRY_PICK_SIDE       400.0f
#define XY_ROUTE_X_ENTRY_PLACE_SIDE     -400.0f

/* 上绕：入口在 +20，出口在 -20。 */
#define XY_ROUTE_Y_UP_ENTRY               20.0f
#define XY_ROUTE_Y_UP_EXIT               -20.0f

/* 下绕：入口在 -20，出口在 +20。 */
#define XY_ROUTE_Y_DOWN_ENTRY            -20.0f
#define XY_ROUTE_Y_DOWN_EXIT              20.0f

/* X 到入口前提前多少开始检查 Y，给 X 停车留余量。 */
#define XY_ROUTE_X_WAIT_MARGIN            50.0f

/* 入口/出口 Y 到位容差。 */
#define XY_ROUTE_Y_ENTRY_TOL               3.0f
#define XY_ROUTE_Y_EXIT_TOL                3.0f

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
} xy_route_t;

static xy_route_t xy_route;

static float xy_route_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static uint8_t xy_route_is_bypass(xy_route_type_e route_type)
{
    return ((route_type == XY_ROUTE_UP) || (route_type == XY_ROUTE_DOWN)) ? 1U : 0U;
}

static xy_route_side_e xy_route_side_from_x(float x, float target_x)
{
    if (x >= XY_ROUTE_X_ENTRY_PICK_SIDE)
    {
        return XY_ROUTE_SIDE_PICK;
    }

    if (x <= XY_ROUTE_X_ENTRY_PLACE_SIDE)
    {
        return XY_ROUTE_SIDE_PLACE;
    }

    return (target_x >= 0.0f) ? XY_ROUTE_SIDE_PICK : XY_ROUTE_SIDE_PLACE;
}

static float xy_route_x_for_side(xy_route_side_e side)
{
    return (side == XY_ROUTE_SIDE_PICK) ? XY_ROUTE_X_ENTRY_PICK_SIDE : XY_ROUTE_X_ENTRY_PLACE_SIDE;
}

static xy_route_side_e xy_route_other_side(xy_route_side_e side)
{
    return (side == XY_ROUTE_SIDE_PICK) ? XY_ROUTE_SIDE_PLACE : XY_ROUTE_SIDE_PICK;
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

    return final_y;
}

static void xy_route_config_gate(float target_x, float target_y, xy_route_type_e route_type)
{
    xy_route_side_e entry_side = xy_route_side_from_x(xy_route.start_x, target_x);
    xy_route_side_e target_side = xy_route_side_from_x(target_x, target_x);
    xy_route_side_e exit_side = (entry_side == target_side) ? entry_side : xy_route_other_side(entry_side);

    xy_route.entry_x = xy_route_x_for_side(entry_side);
    xy_route.exit_x = xy_route_x_for_side(exit_side);
    xy_route.entry_y = xy_route_entry_y_for(route_type, target_y);
    xy_route.exit_y = xy_route_exit_y_for(route_type, target_y);
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

void xy_route_init(void)
{
    memset(&xy_route, 0, sizeof(xy_route));
    xy_route.state = XY_ROUTE_IDLE;
}

void xy_route_prepare_y(xy_route_type_e route_type, float final_y)
{
    float y_target = xy_route_entry_y_for(route_type, final_y);
    xy_route_run_y_to(y_target);
}

void xy_route_start(float target_x,
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

    if (xy_route_is_bypass(route_type) == 0U)
    {
        xy_route_run_x_to_target();
        xy_route_run_y_to(target_y);
        xy_route.state = XY_ROUTE_DIRECT_RUN;
        return;
    }

    xy_route_config_gate(target_x, target_y, route_type);
    xy_route_run_y_to(xy_route.entry_y);
    xy_route_run_x_to_target();
    xy_route.state = XY_ROUTE_BYPASS_TO_ENTRY;
}

void xy_route_stop(void)
{
    xy_route.state = XY_ROUTE_IDLE;
    xy_route.busy = 0U;
    pos_pid_sync_stop();
    beam_ctrl_stop();
}

void xy_route_process(void)
{
    switch (xy_route.state)
    {
        case XY_ROUTE_IDLE:
        case XY_ROUTE_DONE:
            break;

        case XY_ROUTE_DIRECT_RUN:
            if ((pos_pid_sync_is_busy() == 0U) && (beam_ctrl_is_busy() == 0U))
            {
                xy_route.busy = 0U;
                xy_route.state = XY_ROUTE_DONE;
            }
            break;

        case XY_ROUTE_BYPASS_TO_ENTRY:
            if (xy_route_x_near_entry() != 0U)
            {
                if (xy_route_y_at_entry() != 0U)
                {
                    if (xy_route.release_mode == XY_RELEASE_AFTER_ENTRY)
                    {
                        xy_route_run_y_to(xy_route.target_y);
                        xy_route.state = XY_ROUTE_BYPASS_TO_TARGET;
                    }
                    else
                    {
                        xy_route_run_y_to(xy_route.exit_y);
                        xy_route.state = XY_ROUTE_BYPASS_TO_EXIT;
                    }
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
                xy_route_run_x_to_target();

                if (xy_route.release_mode == XY_RELEASE_AFTER_ENTRY)
                {
                    xy_route_run_y_to(xy_route.target_y);
                    xy_route.state = XY_ROUTE_BYPASS_TO_TARGET;
                }
                else
                {
                    xy_route_run_y_to(xy_route.exit_y);
                    xy_route.state = XY_ROUTE_BYPASS_TO_EXIT;
                }
            }
            break;

        case XY_ROUTE_BYPASS_TO_EXIT:
            if (xy_route_x_at_exit() != 0U)
            {
                if (xy_route_y_at_exit() != 0U)
                {
                    xy_route_run_y_to(xy_route.target_y);
                    xy_route.state = XY_ROUTE_BYPASS_TO_TARGET;
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
                xy_route_run_x_to_target();
                xy_route_run_y_to(xy_route.target_y);
                xy_route.state = XY_ROUTE_BYPASS_TO_TARGET;
            }
            break;

        case XY_ROUTE_BYPASS_TO_TARGET:
            if ((pos_pid_sync_is_busy() == 0U) && (beam_ctrl_is_busy() == 0U))
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
