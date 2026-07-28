#include "headfile.h"

/*
 * 底盘按键测试顺序：按 START 后，3 -> 4 -> 1 -> 6 -> 2 -> 8 -> 中心。
 * 槽位 X 坐标不在这里复制，统一从当前 crane_route 的料位表读取。
 */
static const uint8_t chassis_debug_slots[] = {3U, 4U, 1U, 6U, 2U, 8U};
#define CHASSIS_DEBUG_DWELL_MS 10000U

typedef struct
{
    hcan_t *hcan;
    uint8_t initialized;
    uint8_t start_requested;
    uint8_t motion_started;
    uint8_t finished;
    uint8_t sequence_index;
    uint8_t start_button_down;
    uint8_t dwell_active;
    uint32_t dwell_tick;
} chassis_debug_t;

static chassis_debug_t chassis_debug;

static uint8_t chassis_debug_feedback_ready(void)
{
    return (uint8_t)(dm_motor_feedback_is_valid(Motor1) &&
                     dm_motor_feedback_is_valid(Motor2));
}

static uint8_t chassis_debug_start_button_down(void)
{
    return (HAL_GPIO_ReadPin(START_BTN_GPIO_Port, START_BTN_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

static void chassis_debug_start_target(float target_pos)
{
    chassis_debug.dwell_active = 0U;
    pos_pid_sync_set_target(target_pos);
    pos_pid_sync_start();
}

void chassis_debug_init(hcan_t *hcan)
{
    memset(&chassis_debug, 0, sizeof(chassis_debug));
    chassis_debug.hcan = hcan;
    chassis_debug.initialized = 1U;
    /* 必须经历一次松开再按下，避免上电时按键已被按住就直接启动。 */
    chassis_debug.start_button_down = chassis_debug_start_button_down();

    /*
     * 本模式只允许底盘的 Motor1、Motor2 工作。
     * Motor3~Motor6 明确下使能，且主循环不调用它们的控制器。
     */
    pos_pid_sync_stop();
    beam_ctrl_stop();
    upper_hopper_y_ctrl_stop();
    lower_hopper_y_ctrl_stop();
    lift_ctrl_stop();

    dm_motor_disable(hcan, &motor[Motor3]);
    dm_motor_disable(hcan, &motor[Motor4]);
    dm_motor_disable(hcan, &motor[Motor5]);
    dm_motor_disable(hcan, &motor[Motor6]);
}

void chassis_debug_process(void)
{
    float target_pos;
    uint8_t start_button_down;
    uint32_t now_ms;

    if (chassis_debug.finished != 0U)
    {
        return;
    }

    start_button_down = chassis_debug_start_button_down();
    if ((start_button_down != 0U) && (chassis_debug.start_button_down == 0U))
    {
        chassis_debug.start_requested = 1U;
    }
    chassis_debug.start_button_down = start_button_down;

    if ((chassis_debug.initialized == 0U) || (chassis_debug.motion_started == 0U))
    {
        if ((chassis_debug.initialized == 0U) ||
            (chassis_debug.start_requested == 0U) ||
            (chassis_debug_feedback_ready() == 0U))
        {
            return;
        }

        chassis_debug.motion_started = 1U;
        chassis_debug.sequence_index = 0U;
        chassis_debug_start_target(
            crane_route_get_slot_chassis_pos(chassis_debug_slots[chassis_debug.sequence_index]));
        return;
    }

    /*
     * is_arrived() 只读取上次状态，不会更新到位判定；
     * is_busy() 会依据当前两台电机的位置和同步误差刷新该状态。
     */
    if (pos_pid_sync_is_busy() != 0U)
    {
        return;
    }

    /*
     * 两台底盘电机已同步到位后原地保持 10 s，再切换到下一段。
     * 计时从同步控制器确认“非 busy”的时刻开始。
     */
    now_ms = HAL_GetTick();
    if (chassis_debug.dwell_active == 0U)
    {
        chassis_debug.dwell_active = 1U;
        chassis_debug.dwell_tick = now_ms;
        return;
    }
    if ((now_ms - chassis_debug.dwell_tick) < CHASSIS_DEBUG_DWELL_MS)
    {
        return;
    }

    chassis_debug.sequence_index++;
    if (chassis_debug.sequence_index < (uint8_t)(sizeof(chassis_debug_slots) /
                                                  sizeof(chassis_debug_slots[0])))
    {
        target_pos = crane_route_get_slot_chassis_pos(
            chassis_debug_slots[chassis_debug.sequence_index]);
        chassis_debug_start_target(target_pos);
        return;
    }

    if (chassis_debug.sequence_index == (uint8_t)(sizeof(chassis_debug_slots) /
                                                   sizeof(chassis_debug_slots[0])))
    {
        /* 最后一站是底盘统一坐标系的中心。 */
        chassis_debug_start_target(0.0f);
        return;
    }

    /* 中心到位后停止同步控制；到位阶段已经下发 0 速度保持帧。 */
    pos_pid_sync_stop();
    chassis_debug.finished = 1U;
}

void chassis_debug_vofa_process(void)
{
    /*
     * 串口每行输出：M1 实时位置、M1 目标位置、M2 实时位置、M2 目标位置。
     */
    vofa_debug_chassis_process(motor_angle_get(Motor1),
                               motor[Motor1].ctrl.pos_set,
                               motor[Motor1].para.vel,
                               motor[Motor1].ctrl.vel_set,
                               motor_angle_get(Motor2),
                               motor[Motor2].ctrl.pos_set,
                               motor[Motor2].para.vel,
                               motor[Motor2].ctrl.vel_set);
}
