#include "headfile.h"
#include "claw_obstacle_debug.h"

#define CLAW_OBSTACLE_DEBUG_DWELL_MS           5000U
#define CLAW_OBSTACLE_DEBUG_KEY_DEBOUNCE_MS     250U
#define CLAW_OBSTACLE_DEBUG_TELEMETRY_MS          20U

typedef enum
{
    CLAW_OBSTACLE_DEBUG_WAIT_PLACE_ORIENTATION = 0,
    CLAW_OBSTACLE_DEBUG_WAIT_START,
    CLAW_OBSTACLE_DEBUG_WAIT_LIFT_SAFE,
    CLAW_OBSTACLE_DEBUG_WAIT_UPPER_Y,
    CLAW_OBSTACLE_DEBUG_HOLD_UPPER_Y,
    CLAW_OBSTACLE_DEBUG_WAIT_LOWER_Y,
    CLAW_OBSTACLE_DEBUG_HOLD_LOWER_Y,
    CLAW_OBSTACLE_DEBUG_WAIT_INITIAL_Y,
    CLAW_OBSTACLE_DEBUG_WAIT_INITIAL_Z
} claw_obstacle_debug_state_e;

typedef struct
{
    claw_obstacle_debug_state_e state;
    uint8_t key_down;
    uint32_t key_tick;
    uint32_t dwell_tick;
    uint32_t telemetry_tick;
    float initial_claw_y;
    float initial_upper_hopper_y;
    float initial_lower_hopper_y;
    float initial_z;
    float target_claw_y;
    float target_upper_hopper_y;
    float target_lower_hopper_y;
} claw_obstacle_debug_t;

static claw_obstacle_debug_t claw_obstacle_debug;

static uint8_t claw_obstacle_debug_start_pressed(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t down = (HAL_GPIO_ReadPin(START_BTN_GPIO_Port, START_BTN_Pin) ==
                    GPIO_PIN_RESET) ? 1U : 0U;

    if ((now - claw_obstacle_debug.key_tick) <
        CLAW_OBSTACLE_DEBUG_KEY_DEBOUNCE_MS)
    {
        return 0U;
    }

    if ((down != 0U) && (claw_obstacle_debug.key_down == 0U))
    {
        claw_obstacle_debug.key_down = 1U;
        claw_obstacle_debug.key_tick = now;
        return 1U;
    }

    claw_obstacle_debug.key_down = down;
    return 0U;
}

static uint8_t claw_obstacle_debug_feedback_ready(void)
{
    return (uint8_t)(dm_motor_feedback_is_valid(Motor3) &&
                     dm_motor_feedback_is_valid(Motor4) &&
                     dm_motor_feedback_is_valid(Motor5) &&
                     dm_motor_feedback_is_valid(Motor6));
}

static uint8_t claw_obstacle_debug_all_y_arrived(void)
{
    return (uint8_t)((beam_ctrl_is_arrived() != 0U) &&
                     (upper_hopper_y_ctrl_is_arrived() != 0U) &&
                     (lower_hopper_y_ctrl_is_arrived() != 0U));
}

static void claw_obstacle_debug_move_y(float claw_y,
                                        float upper_hopper_y,
                                        float lower_hopper_y)
{
    claw_obstacle_debug.target_claw_y = claw_y;
    claw_obstacle_debug.target_upper_hopper_y = upper_hopper_y;
    claw_obstacle_debug.target_lower_hopper_y = lower_hopper_y;
    beam_ctrl_set_target(claw_y);
    upper_hopper_y_ctrl_set_target(upper_hopper_y);
    lower_hopper_y_ctrl_set_target(lower_hopper_y);
    beam_ctrl_start();
    upper_hopper_y_ctrl_start();
    lower_hopper_y_ctrl_start();
}

static void claw_obstacle_debug_move_to_upper_safe_y(void)
{
    float claw_y;
    float upper_hopper_y;
    float lower_hopper_y;

    crane_route_get_upper_safe_y(&claw_y, &upper_hopper_y, &lower_hopper_y);
    claw_obstacle_debug_move_y(claw_y, upper_hopper_y, lower_hopper_y);
}

static void claw_obstacle_debug_move_to_lower_safe_y(void)
{
    float claw_y;
    float upper_hopper_y;
    float lower_hopper_y;

    crane_route_get_lower_transition_y(&claw_y, &upper_hopper_y, &lower_hopper_y);
    claw_obstacle_debug_move_y(claw_y, upper_hopper_y, lower_hopper_y);
}

static void claw_obstacle_debug_capture_initial_pose(void)
{
    claw_obstacle_debug.initial_claw_y = beam_ctrl_get_current_pos();
    claw_obstacle_debug.initial_upper_hopper_y =
        upper_hopper_y_ctrl_get_current_pos();
    claw_obstacle_debug.initial_lower_hopper_y =
        lower_hopper_y_ctrl_get_current_pos();
    claw_obstacle_debug.initial_z = lift_ctrl_get_current_pos();
    claw_obstacle_debug.target_claw_y = claw_obstacle_debug.initial_claw_y;
    claw_obstacle_debug.target_upper_hopper_y =
        claw_obstacle_debug.initial_upper_hopper_y;
    claw_obstacle_debug.target_lower_hopper_y =
        claw_obstacle_debug.initial_lower_hopper_y;
}

void claw_obstacle_debug_init(void)
{
    memset(&claw_obstacle_debug, 0, sizeof(claw_obstacle_debug));

    /* 禁止 X 或遗留路线命令参与本调试，只允许本模块控制 Motor3~Motor6。 */
    pos_pid_sync_stop();
    beam_ctrl_stop();
    upper_hopper_y_ctrl_stop();
    lower_hopper_y_ctrl_stop();
    lift_ctrl_stop();

    /* 全程固定在置物区初始朝向，不进入取物区。 */
    servo3_path_release_angle(SERVO3_PLACE_DEFAULT_ANGLE_DEG);
    claw_obstacle_debug.state = CLAW_OBSTACLE_DEBUG_WAIT_PLACE_ORIENTATION;
    claw_obstacle_debug.key_tick = HAL_GetTick();
}

void claw_obstacle_debug_process(void)
{
    /* 不论是否在执行，始终把旋转爪锁定在置物区初始朝向。 */
    servo3_path_release_angle(SERVO3_PLACE_DEFAULT_ANGLE_DEG);

    switch (claw_obstacle_debug.state)
    {
        case CLAW_OBSTACLE_DEBUG_WAIT_PLACE_ORIENTATION:
            if (servo3_path_is_arrived() != 0U)
            {
                claw_obstacle_debug.state = CLAW_OBSTACLE_DEBUG_WAIT_START;
            }
            break;

        case CLAW_OBSTACLE_DEBUG_WAIT_START:
            if (claw_obstacle_debug_feedback_ready() != 0U)
            {
                /* 空闲时将当前实际位置作为目标显示，避免遥测显示旧目标。 */
                claw_obstacle_debug.target_claw_y = beam_ctrl_get_current_pos();
                claw_obstacle_debug.target_upper_hopper_y =
                    upper_hopper_y_ctrl_get_current_pos();
                claw_obstacle_debug.target_lower_hopper_y =
                    lower_hopper_y_ctrl_get_current_pos();
            }
            if ((claw_obstacle_debug_start_pressed() != 0U) &&
                (claw_obstacle_debug_feedback_ready() != 0U))
            {
                claw_obstacle_debug_capture_initial_pose();
                lift_ctrl_set_target(CRANE_ROUTE_LIFT_SAFE_POS);
                lift_ctrl_start();
                claw_obstacle_debug.state = CLAW_OBSTACLE_DEBUG_WAIT_LIFT_SAFE;
            }
            break;

        case CLAW_OBSTACLE_DEBUG_WAIT_LIFT_SAFE:
            if (lift_ctrl_is_arrived() != 0U)
            {
                claw_obstacle_debug_move_to_upper_safe_y();
                claw_obstacle_debug.state = CLAW_OBSTACLE_DEBUG_WAIT_UPPER_Y;
            }
            break;

        case CLAW_OBSTACLE_DEBUG_WAIT_UPPER_Y:
            if (claw_obstacle_debug_all_y_arrived() != 0U)
            {
                claw_obstacle_debug.dwell_tick = HAL_GetTick();
                claw_obstacle_debug.state = CLAW_OBSTACLE_DEBUG_HOLD_UPPER_Y;
            }
            break;

        case CLAW_OBSTACLE_DEBUG_HOLD_UPPER_Y:
            if ((HAL_GetTick() - claw_obstacle_debug.dwell_tick) >=
                CLAW_OBSTACLE_DEBUG_DWELL_MS)
            {
                claw_obstacle_debug_move_to_lower_safe_y();
                claw_obstacle_debug.state = CLAW_OBSTACLE_DEBUG_WAIT_LOWER_Y;
            }
            break;

        case CLAW_OBSTACLE_DEBUG_WAIT_LOWER_Y:
            if (claw_obstacle_debug_all_y_arrived() != 0U)
            {
                claw_obstacle_debug.dwell_tick = HAL_GetTick();
                claw_obstacle_debug.state = CLAW_OBSTACLE_DEBUG_HOLD_LOWER_Y;
            }
            break;

        case CLAW_OBSTACLE_DEBUG_HOLD_LOWER_Y:
            if ((HAL_GetTick() - claw_obstacle_debug.dwell_tick) >=
                CLAW_OBSTACLE_DEBUG_DWELL_MS)
            {
                claw_obstacle_debug_move_y(claw_obstacle_debug.initial_claw_y,
                                           claw_obstacle_debug.initial_upper_hopper_y,
                                           claw_obstacle_debug.initial_lower_hopper_y);
                claw_obstacle_debug.state = CLAW_OBSTACLE_DEBUG_WAIT_INITIAL_Y;
            }
            break;

        case CLAW_OBSTACLE_DEBUG_WAIT_INITIAL_Y:
            if (claw_obstacle_debug_all_y_arrived() != 0U)
            {
                lift_ctrl_set_target(claw_obstacle_debug.initial_z);
                lift_ctrl_start();
                claw_obstacle_debug.state = CLAW_OBSTACLE_DEBUG_WAIT_INITIAL_Z;
            }
            break;

        case CLAW_OBSTACLE_DEBUG_WAIT_INITIAL_Z:
            if (lift_ctrl_is_arrived() != 0U)
            {
                claw_obstacle_debug.state = CLAW_OBSTACLE_DEBUG_WAIT_START;
            }
            break;

        default:
            claw_obstacle_debug.state = CLAW_OBSTACLE_DEBUG_WAIT_START;
            break;
    }
}

void claw_obstacle_debug_telemetry_process(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - claw_obstacle_debug.telemetry_tick) <
        CLAW_OBSTACLE_DEBUG_TELEMETRY_MS)
    {
        return;
    }
    claw_obstacle_debug.telemetry_tick = now;

    /* USART1 CSV：夹爪实际、夹爪目标、上斗实际、上斗目标、下斗实际、下斗目标。 */
    printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
           (double)beam_ctrl_get_current_pos(),
           (double)claw_obstacle_debug.target_claw_y,
           (double)upper_hopper_y_ctrl_get_current_pos(),
           (double)claw_obstacle_debug.target_upper_hopper_y,
           (double)lower_hopper_y_ctrl_get_current_pos(),
           (double)claw_obstacle_debug.target_lower_hopper_y);
}
