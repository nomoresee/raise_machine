#include "headfile.h"

/*
 * XY 路径协调器。
 *
 * 控制原则：
 * - X 轴是主轴。Z 轴抬到安全高度后，X 尽可能按 pos_pid_sync 的速度直接向目标 X 运动。
 * - Y 轴是配合轴。Z 抬升期间，Y 可以提前向上绕/下绕安全线运动。
 * - 如果 Y 比 X 更早到安全线，X 不减速继续跑。
 * - 如果 X 到障碍入口附近时 Y 还没到安全线，X 暂停在当前位置等 Y，到位后继续跑。
 */

/* 上绕/下绕的 Y 安全线坐标。实车调试时优先标定这两个值。 */
#define XY_ROUTE_Y_UP_SAFE             10.0f
#define XY_ROUTE_Y_DOWN_SAFE          -10.0f

/* Y 到安全线的允许误差：|当前Y - 安全线Y| <= 该值，就认为横梁已经进入绕行安全范围。 */
#define XY_ROUTE_Y_SAFE_TOL             5.0f

/*
 * 两个障碍物在 X 轴上的判断点。
 * 不做障碍区间，只用两个 X 点做门限：
 * - X 靠近第一个会遇到的障碍点前，检查 Y 是否已经到安全线。
 * - X 越过第二个障碍点后，Y 才允许离开安全线去最终目标。
 */
#define XY_ROUTE_X_OBS1_POINT        -400.0f//障碍2，靠近放物点
#define XY_ROUTE_X_OBS2_POINT         400.0f//障碍1，靠近取物点

/*
 * 绕障出口 X 点。
 * X 到达出口点后，Y 直接离开安全线去最终目标，不再额外加等待裕量。
 */
#define XY_ROUTE_X_EXIT_NEG          -400.0f//出口2，靠近放物点
#define XY_ROUTE_X_EXIT_POS           400.0f//出口1，靠近取物点

/*
 * X 到障碍边界前的提前检查距离。
 * 值越大：X 越早检查/等待，更稳但可能慢。
 * 值越小：X 更贴近障碍才检查，更快但对刹停距离和 Y 到位速度要求更高。
 */
#define XY_ROUTE_X_WAIT_MARGIN         50.0f

typedef struct
{
    xy_route_state_e state;    /* 当前 XY 协调状态 */
    xy_route_type_e route_type;/* 当前路线类型：直线/上绕/下绕/回中 */
    xy_release_mode_e release_mode; /* Y 释放时机：等出口，或入口后直接去目标 */
    float start_x;             /* 本段启动时的 X 位置，用于判断 X 运动方向 */
    float start_y;             /* 本段启动时的 Y 位置，当前仅用于调试观察 */
    float target_x;            /* 本段最终 X 目标 */
    float target_y;            /* 本段最终 Y 目标 */
    float safe_y;              /* 绕障时 Y 需要提前到达的安全线 */
    int8_t x_dir;              /* X 运动方向：+1 表示向 X 增大方向，-1 表示向 X 减小方向 */
    uint8_t busy;              /* 1：本段正在运行；0：本段空闲或已完成 */
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

/* 根据路线类型得到 Y 预备目标：绕行走安全线，直线/回中直接走最终 Y。 */
static float xy_route_get_safe_y(xy_route_type_e route_type, float final_y)
{
    if (route_type == XY_ROUTE_UP)
    {
        return XY_ROUTE_Y_UP_SAFE;
    }
    if (route_type == XY_ROUTE_DOWN)
    {
        return XY_ROUTE_Y_DOWN_SAFE;
    }
    return final_y;
}

static uint8_t xy_route_y_is_safe(void)
{
    float y_now = beam_ctrl_get_current_pos();
    return (xy_route_absf(y_now - xy_route.safe_y) <= XY_ROUTE_Y_SAFE_TOL) ? 1U : 0U;
}

/*
 * 根据 X 运动方向，计算两个关键门限：
 * - first_enter_x：X 靠近第一个障碍前的检查点。
 * - last_exit_x：X 越过最后一个障碍后的释放点。
 */
static uint8_t xy_route_get_obstacle_gate(float *first_enter_x, float *last_exit_x)
{
    float obs_low;
    float obs_high;

    if ((first_enter_x == NULL) || (last_exit_x == NULL))
    {
        return 0U;
    }

    if (XY_ROUTE_X_OBS1_POINT <= XY_ROUTE_X_OBS2_POINT)
    {
        obs_low = XY_ROUTE_X_OBS1_POINT;
        obs_high = XY_ROUTE_X_OBS2_POINT;
    }
    else
    {
        obs_low = XY_ROUTE_X_OBS2_POINT;
        obs_high = XY_ROUTE_X_OBS1_POINT;
    }

    if (xy_route.x_dir >= 0)
    {
        *first_enter_x = obs_low - XY_ROUTE_X_WAIT_MARGIN;
        *last_exit_x = XY_ROUTE_X_EXIT_POS;
    }
    else
    {
        *first_enter_x = obs_high + XY_ROUTE_X_WAIT_MARGIN;
        *last_exit_x = XY_ROUTE_X_EXIT_NEG;
    }

    return 1U;
}

/* 判断 X 是否已经接近障碍入口，需要开始检查 Y 是否在安全线内。 */
static uint8_t xy_route_x_near_obstacle(void)
{
    float x_now = pos_pid_sync_get_current_pos();
    float first_enter_x;
    float last_exit_x;

    if (xy_route_get_obstacle_gate(&first_enter_x, &last_exit_x) == 0U)
    {
        return 0U;
    }

    if (xy_route.x_dir >= 0)
    {
        return (x_now >= first_enter_x) ? 1U : 0U;
    }

    return (x_now <= first_enter_x) ? 1U : 0U;
}

/* 判断 X 是否已经越过障碍区，越过后 Y 才允许离开安全线去最终目标。 */
static uint8_t xy_route_x_has_crossed_obstacle(void)
{
    float x_now = pos_pid_sync_get_current_pos();
    float first_enter_x;
    float last_exit_x;

    if (xy_route_get_obstacle_gate(&first_enter_x, &last_exit_x) == 0U)
    {
        return 1U;
    }

    if (xy_route.x_dir >= 0)
    {
        return (x_now >= last_exit_x) ? 1U : 0U;
    }

    return (x_now <= last_exit_x) ? 1U : 0U;
}

static void xy_route_hold_x_at_current(void)
{
    float x_now = pos_pid_sync_get_current_pos();
    /* 用当前位置作为 X 目标，实现“停在障碍入口附近等 Y”。 */
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
    float y_target = xy_route_get_safe_y(route_type, final_y);
    /* 给 crane_route 提前调用：Z 抬升时，Y 先去绕行安全线。 */
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
    xy_route.safe_y = xy_route_get_safe_y(route_type, target_y);
    xy_route.x_dir = (target_x >= xy_route.start_x) ? 1 : -1;
    xy_route.busy = 1U;

    if (xy_route_is_bypass(route_type) == 0U)
    {
        /* 直线/回中：不需要绕障门限，X/Y 同时去最终点。 */
        xy_route_run_x_to_target();
        xy_route_run_y_to(target_y);
        xy_route.state = XY_ROUTE_DIRECT_RUN;
        return;
    }

    /* 上绕/下绕：Y 先去安全线，X 仍然直接向目标 X 跑。 */
    xy_route_run_y_to(xy_route.safe_y);
    xy_route_run_x_to_target();
    xy_route.state = XY_ROUTE_BYPASS_PREPARE;
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

        case XY_ROUTE_BYPASS_PREPARE:
            if (xy_route_x_near_obstacle() != 0U)
            {
                if (xy_route_y_is_safe() != 0U)
                {
                    /* Y 已在安全线内，X 不停顿，直接穿越障碍区。 */
                    if (xy_route.release_mode == XY_RELEASE_AFTER_ENTRY)
                    {
                        xy_route_run_y_to(xy_route.target_y);
                    }
                    xy_route.state = XY_ROUTE_BYPASS_CROSS;
                }
                else
                {
                    /* Y 还没到安全线，X 在当前位置短暂停车等待。 */
                    xy_route_hold_x_at_current();
                    xy_route.state = XY_ROUTE_BYPASS_WAIT_Y;
                }
            }
            break;

        case XY_ROUTE_BYPASS_WAIT_Y:
            if (xy_route_y_is_safe() != 0U)
            {
                /* Y 到位后，X 恢复向最终目标运动。 */
                if (xy_route.release_mode == XY_RELEASE_AFTER_ENTRY)
                {
                    xy_route_run_y_to(xy_route.target_y);
                }
                xy_route_run_x_to_target();
                xy_route.state = XY_ROUTE_BYPASS_CROSS;
            }
            break;

        case XY_ROUTE_BYPASS_CROSS:
            if (xy_route.release_mode == XY_RELEASE_AFTER_ENTRY)
            {
                if ((pos_pid_sync_is_busy() == 0U) && (beam_ctrl_is_busy() == 0U))
                {
                    xy_route.busy = 0U;
                    xy_route.state = XY_ROUTE_DONE;
                }
            }
            else if (xy_route_x_has_crossed_obstacle() != 0U)
            {
                /* X 已经越过障碍区，Y 可以离开安全线，去最终放置/取货 Y。 */
                xy_route_run_y_to(xy_route.target_y);
                xy_route.state = XY_ROUTE_BYPASS_ALIGN_TARGET;
            }
            break;

        case XY_ROUTE_BYPASS_ALIGN_TARGET:
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
