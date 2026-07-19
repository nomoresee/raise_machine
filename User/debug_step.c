#include "debug_step.h"

#include "beam_ctrl/beam_ctrl.h"
#include "dm_motor_ctrl.h"
#include "gpio.h"
#include "lift_ctrl/lift_ctrl.h"
#include "lower_hopper_y_ctrl/lower_hopper_y_ctrl.h"
#include "motor_angle/motor_angle.h"
#include "pos_sync/pos_pid_sync.h"
#include "upper_hopper_y_ctrl/upper_hopper_y_ctrl.h"
#include "vofa_debug/vofa_debug.h"

/* 按键时间参数：所有长按事件在松手时才执行，避免 6 s 回零时先翻向。 */
#define DEBUG_STEP_KEY_DEBOUNCE_MS       30U
#define DEBUG_STEP_DOUBLE_CLICK_MS      350U
#define DEBUG_STEP_DIRECTION_HOLD_MS   3000U
#define DEBUG_STEP_RETURN_ZERO_HOLD_MS 6000U

/*
 * ======================== 用户调试配置区 ========================
 * 修改以下四项后重新下载程序即可。
 *
 * debug_step_motor_select：选择唯一允许运动的机构：
 *   1 = 底盘 X：Motor1 与 Motor2 同步运动；
 *   2 = 夹爪横梁 Y：Motor3；
 *   3 = 上料斗 Y：Motor5；
 *   4 = 下料斗 Y：Motor6；
 *   5 = 夹爪升降 Z：Motor4。
 *
 * debug_step_coarse_distance：单击 START 的步进距离，用于快速接近目标。
 * debug_step_fine_distance：双击 START 的步进距离，用于最终精细定位。
 * debug_step_direction：+1 为机构正方向，-1 为机构反方向；长按 3~6 秒可切换。
 * 长按至少 6 秒后松开：所选机构自动回到坐标 0，不会改写电机内部零点。
 */
uint8_t debug_step_motor_select = (uint8_t)DEBUG_STEP_CLAW_Z;
float debug_step_coarse_distance = 2.0f;
float debug_step_fine_distance = 100.0f;
int8_t debug_step_direction = 1;

typedef struct
{
    uint8_t initialized;
    uint8_t active_select;
    uint8_t target_initialized;
    uint8_t key_down;
    uint8_t waiting_single;
    uint32_t key_change_tick;
    uint32_t press_tick;
    uint32_t click_tick;
    float target_pos;
    float step_origin_pos;
} debug_step_state_t;

static debug_step_state_t debug_step;

static uint8_t debug_step_selection_valid(uint8_t selection)
{
    return ((selection >= (uint8_t)DEBUG_STEP_CHASSIS_X) &&
            (selection <= (uint8_t)DEBUG_STEP_CLAW_Z)) ? 1U : 0U;
}

static float debug_step_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static void debug_step_stop_all_axes(void)
{
    pos_pid_sync_stop();
    beam_ctrl_stop();
    upper_hopper_y_ctrl_stop();
    lower_hopper_y_ctrl_stop();
    lift_ctrl_stop();
}

/* 目标只能在该轴已经收到 CAN 反馈后才允许建立，避免上电时错误地以 0 为基准。 */
static uint8_t debug_step_feedback_ready(uint8_t selection)
{
    switch (selection)
    {
        case DEBUG_STEP_CHASSIS_X:
            return (uint8_t)(dm_motor_feedback_is_valid(Motor1) &&
                             dm_motor_feedback_is_valid(Motor2));
        case DEBUG_STEP_CLAW_Y:
            return dm_motor_feedback_is_valid(Motor3);
        case DEBUG_STEP_UPPER_HOPPER_Y:
            return dm_motor_feedback_is_valid(Motor5);
        case DEBUG_STEP_LOWER_HOPPER_Y:
            return dm_motor_feedback_is_valid(Motor6);
        case DEBUG_STEP_CLAW_Z:
            return dm_motor_feedback_is_valid(Motor4);
        default:
            return 0U;
    }
}

float debug_step_get_current_pos(void)
{
    switch (debug_step.active_select)
    {
        case DEBUG_STEP_CHASSIS_X:      return pos_pid_sync_get_current_pos();
        case DEBUG_STEP_CLAW_Y:         return beam_ctrl_get_current_pos();
        case DEBUG_STEP_UPPER_HOPPER_Y: return upper_hopper_y_ctrl_get_current_pos();
        case DEBUG_STEP_LOWER_HOPPER_Y: return lower_hopper_y_ctrl_get_current_pos();
        case DEBUG_STEP_CLAW_Z:         return lift_ctrl_get_current_pos();
        default:                         return 0.0f;
    }
}

float debug_step_get_target_pos(void)
{
    return debug_step.target_pos;
}

/*
 * VOFA 速度采用电机侧单位，便于直接比较 motor[].para.vel 与
 * motor[].ctrl.vel_set。底盘选择号包含两台同步电机，因此取两者绝对值的平均值。
 */
static float debug_step_get_feedback_motor_vel(void)
{
    if (debug_step.active_select == DEBUG_STEP_CHASSIS_X)
    {
        return 0.5f * (debug_step_absf(motor[Motor1].para.vel) +
                       debug_step_absf(motor[Motor2].para.vel));
    }

    switch (debug_step.active_select)
    {
        case DEBUG_STEP_CLAW_Y:         return debug_step_absf(motor[Motor3].para.vel);
        case DEBUG_STEP_UPPER_HOPPER_Y: return debug_step_absf(motor[Motor5].para.vel);
        case DEBUG_STEP_LOWER_HOPPER_Y: return debug_step_absf(motor[Motor6].para.vel);
        case DEBUG_STEP_CLAW_Z:         return debug_step_absf(motor[Motor4].para.vel);
        default:                         return 0.0f;
    }
}

static float debug_step_get_command_motor_vel(void)
{
    if (debug_step.active_select == DEBUG_STEP_CHASSIS_X)
    {
        return 0.5f * (debug_step_absf(motor[Motor1].ctrl.vel_set) +
                       debug_step_absf(motor[Motor2].ctrl.vel_set));
    }

    switch (debug_step.active_select)
    {
        case DEBUG_STEP_CLAW_Y:         return debug_step_absf(motor[Motor3].ctrl.vel_set);
        case DEBUG_STEP_UPPER_HOPPER_Y: return debug_step_absf(motor[Motor5].ctrl.vel_set);
        case DEBUG_STEP_LOWER_HOPPER_Y: return debug_step_absf(motor[Motor6].ctrl.vel_set);
        case DEBUG_STEP_CLAW_Z:         return debug_step_absf(motor[Motor4].ctrl.vel_set);
        default:                         return 0.0f;
    }
}

static void debug_step_select_axis(uint8_t selection)
{
    if (debug_step_selection_valid(selection) == 0U)
    {
        return;
    }

    debug_step_stop_all_axes();
    debug_step.active_select = selection;
    debug_step.target_initialized = 0U;
}

static void debug_step_send_target(void)
{
    switch (debug_step.active_select)
    {
        case DEBUG_STEP_CHASSIS_X:
            pos_pid_sync_set_target(debug_step.target_pos);
            pos_pid_sync_start();
            break;

        case DEBUG_STEP_CLAW_Y:
            beam_ctrl_set_target(debug_step.target_pos);
            beam_ctrl_start();
            break;

        case DEBUG_STEP_UPPER_HOPPER_Y:
            upper_hopper_y_ctrl_set_target(debug_step.target_pos);
            upper_hopper_y_ctrl_start();
            break;

        case DEBUG_STEP_LOWER_HOPPER_Y:
            lower_hopper_y_ctrl_set_target(debug_step.target_pos);
            lower_hopper_y_ctrl_start();
            break;

        case DEBUG_STEP_CLAW_Z:
            lift_ctrl_set_target(debug_step.target_pos);
            lift_ctrl_start();
            break;

        default:
            break;
    }
}

static void debug_step_send_absolute_target(float target_pos)
{
    debug_step.target_pos = target_pos;
    debug_step_send_target();
}

static void debug_step_move(float distance)
{
    if (distance > 0.0f)
    {
        /*
         * 每次单击都以当前 motor_angle_get() 实际位置为基准，不累计旧目标。
         * 这样实际位置为 5、步进为 20、方向为 -1 时，命令目标必为 -15。
         */
        debug_step.step_origin_pos = debug_step_get_current_pos();
        debug_step_send_absolute_target(debug_step.step_origin_pos +
                                        ((debug_step_direction >= 0) ? distance : -distance));
    }
}

static void debug_step_return_to_zero(void)
{
    debug_step_send_absolute_target(0.0f);
}

static uint8_t debug_step_start_button_down(void)
{
    return (HAL_GPIO_ReadPin(START_BTN_GPIO_Port, START_BTN_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

static void debug_step_handle_release(uint32_t hold_ms, uint32_t now)
{
    if (hold_ms >= DEBUG_STEP_RETURN_ZERO_HOLD_MS)
    {
        debug_step.waiting_single = 0U;
        debug_step_return_to_zero();
        return;
    }

    if (hold_ms >= DEBUG_STEP_DIRECTION_HOLD_MS)
    {
        debug_step.waiting_single = 0U;
        debug_step_direction = (debug_step_direction >= 0) ? -1 : 1;
        return;
    }

    if ((debug_step.waiting_single != 0U) &&
        ((now - debug_step.click_tick) <= DEBUG_STEP_DOUBLE_CLICK_MS))
    {
        /* 第一击已立即执行粗步进；第二击改写为同一起点的细步进目标。 */
        float fine = debug_step_absf(debug_step_fine_distance);

        debug_step.waiting_single = 0U;
        debug_step_send_absolute_target(debug_step.step_origin_pos +
                                        ((debug_step_direction >= 0) ? fine : -fine));
        return;
    }

    /* 单击松手立刻执行粗步进；waiting_single 只用于等待可能的第二击。 */
    debug_step.waiting_single = 1U;
    debug_step.click_tick = now;
    debug_step_move(debug_step_absf(debug_step_coarse_distance));
}

void debug_step_init(void)
{
    debug_step.initialized = 1U;
    debug_step.active_select = 0U;
    debug_step.target_initialized = 0U;
    debug_step.step_origin_pos = 0.0f;
    debug_step.key_down = 0U;
    debug_step.waiting_single = 0U;
    debug_step.key_change_tick = HAL_GetTick();
    debug_step.press_tick = 0U;
    debug_step.click_tick = 0U;

    if (debug_step_direction >= 0)
    {
        debug_step_direction = 1;
    }
    else
    {
        debug_step_direction = -1;
    }

    debug_step_stop_all_axes();
}

void debug_step_process(void)
{
    uint32_t now;
    uint8_t key_down;

    if (debug_step.initialized == 0U)
    {
        debug_step_init();
    }

    if (debug_step_selection_valid(debug_step_motor_select) == 0U)
    {
        return;
    }

    if (debug_step.active_select != debug_step_motor_select)
    {
        debug_step_select_axis(debug_step_motor_select);
    }

    now = HAL_GetTick();

    /*
     * 首次有效反馈到达后，把调试目标锁定到当前实际位置。
     * 之后每次单击才是在该真实位置基础上做相对步进。
     */
    if (debug_step.target_initialized == 0U)
    {
        if (debug_step_feedback_ready(debug_step.active_select) != 0U)
        {
            debug_step.target_pos = debug_step_get_current_pos();
            debug_step.step_origin_pos = debug_step.target_pos;
            debug_step.target_initialized = 1U;
        }
        else
        {
            /* 反馈尚未就绪时吞掉按键状态，防止松手被误判成一次步进。 */
            debug_step.key_down = debug_step_start_button_down();
            debug_step.key_change_tick = now;
            return;
        }
    }

    key_down = debug_step_start_button_down();

    if (key_down != debug_step.key_down)
    {
        if ((now - debug_step.key_change_tick) < DEBUG_STEP_KEY_DEBOUNCE_MS)
        {
            return;
        }

        debug_step.key_change_tick = now;
        debug_step.key_down = key_down;

        if (key_down != 0U)
        {
            debug_step.press_tick = now;
        }
        else
        {
            debug_step_handle_release(now - debug_step.press_tick, now);
        }
    }

    if ((debug_step.waiting_single != 0U) &&
        ((now - debug_step.click_tick) > DEBUG_STEP_DOUBLE_CLICK_MS))
    {
        /* 粗步进已在第一击执行；双击识别窗口到期后只清除标志。 */
        debug_step.waiting_single = 0U;
    }
}

void debug_step_vofa_process(void)
{
    if ((debug_step.initialized == 0U) || (debug_step.target_initialized == 0U))
    {
        return;
    }

    vofa_debug_step_process(debug_step.active_select,
                            debug_step_get_current_pos(),
                            debug_step.target_pos,
                            debug_step_get_feedback_motor_vel(),
                            debug_step_get_command_motor_vel());
}
