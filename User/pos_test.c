#include "headfile.h"

/* Adjustable motion parameters. All position/velocity values use output-side rad. */
#define POS_REDUCTION_NUM              1.5f
#define POS_REDUCTION_DEN              10.0f

#define POS_TARGET_POS_RAD             300.0f
#define POS_TARGET_NEG_RAD             (-400.0f)
#define POS_TARGET_ZERO_RAD            0.0f
#define POS_PROFILE_VEL_LIMIT          40.0f
#define POS_PROFILE_ACC                8.0f
#define POS_HOLD_TIME_MS               1000U

/* Adjustable synchronization parameters. */
#define POS_SYNC_READY_POS_EPS         0.05f
#define POS_SYNC_READY_VEL_EPS         0.30f
#define POS_SYNC_READY_TIME_MS         300U
#define POS_SYNC_ALIGN_VEL             5.0f
#define POS_SYNC_KP                    0.50f
#define POS_SYNC_CORR_LIMIT            20.0f
#define POS_SYNC_POS_FAULT_LIMIT       50.0f
#define POS_SYNC_VEL_FAULT_LIMIT       50.0f
#define POS_SYNC_FAULT_DELAY_MS        1000U

#define POS_PROFILE_POS_EPS            0.01f
#define POS_PROFILE_DT_MAX             0.05f
#define POS_FB_WRAP_HALF_RAD           12.5f
#define POS_FB_WRAP_RAD                25.0f

#define MOTOR1_POS_DIR                 1.0f
#define MOTOR2_POS_DIR                 (-1.0f)

static hcan_t *pos_hcan = NULL;
static motor_num pos_motor1 = Motor1;
static motor_num pos_motor2 = Motor2;

static uint32_t print_tick = 0;
static uint32_t state_tick = 0;
static uint32_t profile_tick = 0;
static uint32_t sync_ready_tick = 0;
static uint32_t sync_fault_start_tick = 0;

static float output_ref = POS_TARGET_ZERO_RAD;
static float output_vel_ref = 0.0f;

static float motor1_pos_unwrap = 0.0f;
static float motor2_pos_unwrap = 0.0f;
static float motor1_pos_last = 0.0f;
static float motor2_pos_last = 0.0f;
static float motor1_pos_zero = 0.0f;
static float motor2_pos_zero = 0.0f;

static float motor1_track_pos = 0.0f;
static float motor2_track_pos = 0.0f;
static float motor1_track_vel = 0.0f;
static float motor2_track_vel = 0.0f;
static float sync_pos_err = 0.0f;
static float sync_vel_err = 0.0f;

static pos_test_state_e pos_state = POS_TEST_SYNC_ZERO;
static pos_test_fault_e pos_fault = POS_TEST_FAULT_NONE;

static float pos_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float pos_clampf(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

static float pos_wrap_delta(float delta)
{
    if (delta > POS_FB_WRAP_HALF_RAD)
    {
        delta -= POS_FB_WRAP_RAD;
    }
    else if (delta < -POS_FB_WRAP_HALF_RAD)
    {
        delta += POS_FB_WRAP_RAD;
    }

    return delta;
}

static float pos_motor_to_output(float motor_pos)
{
    return motor_pos * POS_REDUCTION_NUM / POS_REDUCTION_DEN;
}

static float pos_output_to_motor(float output_pos)
{
    return output_pos * POS_REDUCTION_DEN / POS_REDUCTION_NUM;
}

static uint8_t pos_motor_state_is_fault(int state)
{
    return ((state == 5) || (state == 6) || (state >= 8)) ? 1U : 0U;
}

static void pos_feedback_reset(void)
{
    motor1_pos_last = motor[pos_motor1].para.pos;
    motor2_pos_last = motor[pos_motor2].para.pos;
    motor1_pos_unwrap = motor1_pos_last;
    motor2_pos_unwrap = motor2_pos_last;
    motor1_pos_zero = motor1_pos_unwrap;
    motor2_pos_zero = motor2_pos_unwrap;

    motor1_track_pos = 0.0f;
    motor2_track_pos = 0.0f;
    motor1_track_vel = 0.0f;
    motor2_track_vel = 0.0f;
    sync_pos_err = 0.0f;
    sync_vel_err = 0.0f;
}

static void pos_feedback_update(void)
{
    motor1_pos_unwrap += pos_wrap_delta(motor[pos_motor1].para.pos - motor1_pos_last);
    motor2_pos_unwrap += pos_wrap_delta(motor[pos_motor2].para.pos - motor2_pos_last);
    motor1_pos_last = motor[pos_motor1].para.pos;
    motor2_pos_last = motor[pos_motor2].para.pos;

    motor1_track_pos = MOTOR1_POS_DIR * pos_motor_to_output(motor1_pos_unwrap - motor1_pos_zero);
    motor2_track_pos = MOTOR2_POS_DIR * pos_motor_to_output(motor2_pos_unwrap - motor2_pos_zero);
    motor1_track_vel = MOTOR1_POS_DIR * pos_motor_to_output(motor[pos_motor1].para.vel);
    motor2_track_vel = MOTOR2_POS_DIR * pos_motor_to_output(motor[pos_motor2].para.vel);
    sync_pos_err = motor1_track_pos - motor2_track_pos;
    sync_vel_err = motor1_track_vel - motor2_track_vel;
}

static void pos_set_fault(pos_test_fault_e fault)
{
    if (pos_fault != POS_TEST_FAULT_NONE)
    {
        return;
    }

    pos_fault = fault;
    pos_state = POS_TEST_FAULT;
    output_vel_ref = 0.0f;
    motor[pos_motor1].ctrl.vel_set = 0.0f;
    motor[pos_motor2].ctrl.vel_set = 0.0f;

    if (pos_hcan != NULL)
    {
        HAL_TIM_Base_Stop_IT(&htim3);
        dm_motor_disable(pos_hcan, &motor[pos_motor1]);
        dm_motor_disable(pos_hcan, &motor[pos_motor2]);
    }
}

static uint8_t pos_fault_guard_update(uint32_t now)
{
    if (pos_motor_state_is_fault(motor[pos_motor1].para.state) ||
        pos_motor_state_is_fault(motor[pos_motor2].para.state))
    {
        pos_set_fault(POS_TEST_FAULT_MOTOR_STATE);
        return 1U;
    }

    if ((now - sync_fault_start_tick) < POS_SYNC_FAULT_DELAY_MS)
    {
        return 0U;
    }

    if (pos_absf(sync_pos_err) > POS_SYNC_POS_FAULT_LIMIT)
    {
        pos_set_fault(POS_TEST_FAULT_POS_SYNC);
        return 1U;
    }

    if (pos_absf(sync_vel_err) > POS_SYNC_VEL_FAULT_LIMIT)
    {
        pos_set_fault(POS_TEST_FAULT_VEL_SYNC);
        return 1U;
    }

    return 0U;
}

static void pos_apply_output_ref(uint8_t sync_enable)
{
    float correction = 0.0f;
    float motor1_output_cmd;
    float motor2_output_cmd;
    float output_vel_cmd;
    uint32_t primask;

    if (sync_enable != 0U)
    {
        correction = POS_SYNC_KP * sync_pos_err;
        correction = pos_clampf(correction, -POS_SYNC_CORR_LIMIT, POS_SYNC_CORR_LIMIT);
    }

    motor1_output_cmd = output_ref - correction;
    motor2_output_cmd = output_ref + correction;
    output_vel_cmd = pos_absf(output_vel_ref);

    if ((output_vel_cmd < POS_SYNC_ALIGN_VEL) &&
        ((pos_absf(output_ref - motor1_track_pos) > POS_SYNC_READY_POS_EPS) ||
         (pos_absf(output_ref - motor2_track_pos) > POS_SYNC_READY_POS_EPS) ||
         (pos_absf(sync_pos_err) > POS_SYNC_READY_POS_EPS)))
    {
        output_vel_cmd = POS_SYNC_ALIGN_VEL;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    motor[pos_motor1].ctrl.mode = pos_mode;
    motor[pos_motor1].ctrl.pos_set = MOTOR1_POS_DIR * pos_output_to_motor(motor1_output_cmd);
    motor[pos_motor1].ctrl.vel_set = pos_output_to_motor(output_vel_cmd);
    motor[pos_motor2].ctrl.mode = pos_mode;
    motor[pos_motor2].ctrl.pos_set = MOTOR2_POS_DIR * pos_output_to_motor(motor2_output_cmd);
    motor[pos_motor2].ctrl.vel_set = pos_output_to_motor(output_vel_cmd);
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void pos_hold_output(float output_pos)
{
    output_ref = output_pos;
    output_vel_ref = 0.0f;
    pos_apply_output_ref(1U);
}

static void pos_profile_start(float start_pos, uint32_t now)
{
    output_ref = start_pos;
    output_vel_ref = 0.0f;
    profile_tick = now;
    pos_apply_output_ref(1U);
}

static uint8_t pos_profile_update(float target_pos, uint32_t now)
{
    float dt = (now - profile_tick) / 1000.0f;
    float error;
    float distance;
    float direction;
    float speed;
    float stop_distance;
    float step;

    if (dt <= 0.0f)
    {
        pos_apply_output_ref(1U);
        return 0U;
    }

    if (dt > POS_PROFILE_DT_MAX)
    {
        dt = POS_PROFILE_DT_MAX;
    }

    profile_tick = now;
    error = target_pos - output_ref;
    distance = pos_absf(error);
    speed = pos_absf(output_vel_ref);

    if ((distance <= POS_PROFILE_POS_EPS) && (speed <= POS_PROFILE_POS_EPS))
    {
        pos_hold_output(target_pos);
        return 1U;
    }

    direction = (error >= 0.0f) ? 1.0f : -1.0f;
    stop_distance = (speed * speed) / (2.0f * POS_PROFILE_ACC);

    if (stop_distance >= distance)
    {
        speed -= POS_PROFILE_ACC * dt;
    }
    else
    {
        speed += POS_PROFILE_ACC * dt;
    }

    speed = pos_clampf(speed, 0.0f, POS_PROFILE_VEL_LIMIT);
    step = speed * dt;

    if (step >= distance)
    {
        pos_hold_output(target_pos);
        return 1U;
    }

    output_vel_ref = direction * speed;
    output_ref += output_vel_ref * dt;
    pos_apply_output_ref(1U);
    return 0U;
}

static uint8_t pos_sync_zero_ready(uint32_t now)
{
    if ((pos_absf(motor1_track_pos - POS_TARGET_ZERO_RAD) <= POS_SYNC_READY_POS_EPS) &&
        (pos_absf(motor2_track_pos - POS_TARGET_ZERO_RAD) <= POS_SYNC_READY_POS_EPS) &&
        (pos_absf(sync_pos_err) <= POS_SYNC_READY_POS_EPS) &&
        (pos_absf(sync_vel_err) <= POS_SYNC_READY_VEL_EPS))
    {
        if (sync_ready_tick == 0U)
        {
            sync_ready_tick = now;
        }

        if ((now - sync_ready_tick) >= POS_SYNC_READY_TIME_MS)
        {
            return 1U;
        }
    }
    else
    {
        sync_ready_tick = 0U;
    }

    return 0U;
}

static void pos_test_prepare_motor(motor_num motor_index)
{
    motor[motor_index].ctrl.mode = pos_mode;
    motor[motor_index].ctrl.pos_set = 0.0f;
    motor[motor_index].ctrl.vel_set = 0.0f;

    write_motor_data(motor[motor_index].id, RID_CMODE, pos_mode, 0, 0, 0);
    HAL_Delay(50);
    save_motor_data(motor[motor_index].id, RID_CMODE);
    HAL_Delay(50);
}

static void pos_test_send_zero_frames(uint16_t count)
{
    uint16_t i;

    for (i = 0U; i < count; i++)
    {
        pos_apply_output_ref(0U);
        pos_test_ctrl_send();
        HAL_Delay(5);
    }
}

void pos_test_init(hcan_t *hcan, motor_num motor_index)
{
    uint32_t now;

    pos_hcan = hcan;
    pos_motor1 = motor_index;
    pos_motor2 = Motor2;
    print_tick = 0U;
    state_tick = 0U;
    profile_tick = 0U;
    sync_ready_tick = 0U;
    sync_fault_start_tick = 0U;
    output_ref = POS_TARGET_ZERO_RAD;
    output_vel_ref = 0.0f;
    pos_state = POS_TEST_SYNC_ZERO;
    pos_fault = POS_TEST_FAULT_NONE;

    pos_test_prepare_motor(pos_motor1);
    pos_test_prepare_motor(pos_motor2);

    save_pos_zero(pos_hcan, motor[pos_motor1].id, POS_MODE);
    HAL_Delay(300);
    save_pos_zero(pos_hcan, motor[pos_motor2].id, POS_MODE);
    HAL_Delay(300);

    dm_motor_clear_err(pos_hcan, &motor[pos_motor1]);
    HAL_Delay(100);
    dm_motor_clear_err(pos_hcan, &motor[pos_motor2]);
    HAL_Delay(100);

    pos_feedback_reset();
    pos_apply_output_ref(0U);

    dm_motor_enable(pos_hcan, &motor[pos_motor1]);
    HAL_Delay(50);
    dm_motor_enable(pos_hcan, &motor[pos_motor2]);
    HAL_Delay(50);
    pos_test_send_zero_frames(20U);

    now = HAL_GetTick();
    pos_feedback_reset();
    state_tick = now;
    profile_tick = now;
    sync_fault_start_tick = now;
    sync_ready_tick = 0U;
}

void pos_test_ctrl_send(void)
{
    if (pos_hcan == NULL)
    {
        return;
    }

    dm_motor_ctrl_send(pos_hcan, &motor[pos_motor1]);
    dm_motor_ctrl_send(pos_hcan, &motor[pos_motor2]);
}

void pos_test_update(void)
{
    uint32_t now;

    if (pos_hcan == NULL)
    {
        return;
    }

    now = HAL_GetTick();
    pos_feedback_update();

    if (pos_fault_guard_update(now) != 0U)
    {
        return;
    }

    switch (pos_state)
    {
        case POS_TEST_SYNC_ZERO:
            pos_hold_output(POS_TARGET_ZERO_RAD);
            if (pos_sync_zero_ready(now) != 0U)
            {
                state_tick = now;
                pos_profile_start(POS_TARGET_ZERO_RAD, now);
                pos_state = POS_TEST_MOVE_POS;
            }
            break;

        case POS_TEST_MOVE_POS:
            if (pos_profile_update(POS_TARGET_POS_RAD, now) != 0U)
            {
                state_tick = now;
                pos_state = POS_TEST_HOLD_POS;
            }
            break;

        case POS_TEST_HOLD_POS:
            pos_hold_output(POS_TARGET_POS_RAD);
            if ((now - state_tick) >= POS_HOLD_TIME_MS)
            {
                state_tick = now;
                pos_profile_start(POS_TARGET_POS_RAD, now);
                pos_state = POS_TEST_MOVE_NEG;
            }
            break;

        case POS_TEST_MOVE_NEG:
            if (pos_profile_update(POS_TARGET_NEG_RAD, now) != 0U)
            {
                state_tick = now;
                pos_state = POS_TEST_HOLD_NEG;
            }
            break;

        case POS_TEST_HOLD_NEG:
            pos_hold_output(POS_TARGET_NEG_RAD);
            if ((now - state_tick) >= POS_HOLD_TIME_MS)
            {
                state_tick = now;
                pos_profile_start(POS_TARGET_NEG_RAD, now);
                pos_state = POS_TEST_MOVE_ZERO;
            }
            break;

        case POS_TEST_MOVE_ZERO:
            if (pos_profile_update(POS_TARGET_ZERO_RAD, now) != 0U)
            {
                state_tick = now;
                pos_state = POS_TEST_HOLD_ZERO;
            }
            break;

        case POS_TEST_HOLD_ZERO:
            pos_hold_output(POS_TARGET_ZERO_RAD);
            if ((now - state_tick) >= POS_HOLD_TIME_MS)
            {
                state_tick = now;
                pos_profile_start(POS_TARGET_ZERO_RAD, now);
                pos_state = POS_TEST_MOVE_POS;
            }
            break;

        case POS_TEST_FAULT:
            output_vel_ref = 0.0f;
            motor[pos_motor1].ctrl.vel_set = 0.0f;
            motor[pos_motor2].ctrl.vel_set = 0.0f;
            break;

        default:
            pos_hold_output(POS_TARGET_ZERO_RAD);
            pos_state = POS_TEST_SYNC_ZERO;
            break;
    }
}

void pos_test_print_100ms(void)
{
    if ((HAL_GetTick() - print_tick) < 100U)
    {
        return;
    }

    print_tick = HAL_GetTick();
    printf("%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%X,%X\r\n",
           pos_state,
           pos_fault,
           output_ref,
           output_vel_ref,
           motor1_track_pos,
           motor2_track_pos,
           motor1_track_vel,
           motor2_track_vel,
           sync_pos_err,
           motor[pos_motor1].para.state,
           motor[pos_motor2].para.state);
}

void pos_test_get_status(pos_test_status_t *status)
{
    if (status == NULL)
    {
        return;
    }

    status->state = pos_state;
    status->fault = pos_fault;
    status->ref_pos = output_ref;
    status->ref_vel = output_vel_ref;
    status->motor1_pos = motor1_track_pos;
    status->motor2_pos = motor2_track_pos;
    status->motor1_vel = motor1_track_vel;
    status->motor2_vel = motor2_track_vel;
    status->pos_err = sync_pos_err;
    status->vel_err = sync_vel_err;
}

float pos_test_get_motor1_pos(void)
{
    return motor1_track_pos;
}

float pos_test_get_motor2_pos(void)
{
    return motor2_track_pos;
}

float pos_test_get_sync_pos_err(void)
{
    return sync_pos_err;
}
