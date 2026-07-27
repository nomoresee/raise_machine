#include "headfile.h"

#define LIFT_CTRL_PERIOD_MS        10U
#define LIFT_CTRL_REDUCTION_RATIO  30.0f  /* 电机侧转角 / 升降输出端转角 */
#define LIFT_CTRL_POS_RATIO        (1.0f / LIFT_CTRL_REDUCTION_RATIO)

/* 以下所有位置、速度均为升降输出端单位，而非 Motor4 电机侧单位。 */
#define LIFT_CTRL_REACH_TOL        0.10f
#define LIFT_CTRL_REACH_HOLD_MS    80U
#define LIFT_CTRL_DEFAULT_MAX_VEL  (30.0f / LIFT_CTRL_REDUCTION_RATIO)
#define LIFT_CTRL_DIR              1.0f

/*
 * 升降 3519 单轴闭环参数。
 * 控制结构与底盘 pos_pid_sync 相同：位置 P 环 + 速度 P 环 + 距离/刹车约束。
 * 唯一移除的是底盘双电机才需要的同步平衡环。
 */
#define LIFT_CTRL_POS_KP           0.20f
#define LIFT_CTRL_POS_KI           0.0f
#define LIFT_CTRL_POS_KD           0.0f
#define LIFT_CTRL_POS_OUT_MAX      (5.0f / LIFT_CTRL_REDUCTION_RATIO)
#define LIFT_CTRL_POS_OUT_MIN     (-5.0f / LIFT_CTRL_REDUCTION_RATIO)
/* 与底盘 POS_PID_SYNC_VEL_KP 保持一致，避免接近目标时速度上限突变。 */
#define LIFT_CTRL_VEL_KP           0.40f
#define LIFT_CTRL_VEL_KI           0.0f
#define LIFT_CTRL_VEL_KD           0.06f
#define LIFT_CTRL_VEL_OUT_MAX      (40.0f / LIFT_CTRL_REDUCTION_RATIO)
#define LIFT_CTRL_VEL_OUT_MIN      0.05f

/*
 * 升降行程远小于底盘：缩短减速区以扩大全速运行区。
 * 减速范围仍按 30:1 换算到输出端坐标；到位死区与最低速度则按机构
 * 实际可重复精度设置，避免轻微碰撞后因静摩擦卡在过小的理论误差内。
 */
#define LIFT_CTRL_DECEL_RANGE      (3.0f / LIFT_CTRL_REDUCTION_RATIO)
#define LIFT_CTRL_SETTLE_TOL       0.10f
#define LIFT_CTRL_DECEL_MIN_VEL    0.05f
#define LIFT_CTRL_DECEL_ACCEL      (200.0f / LIFT_CTRL_REDUCTION_RATIO)

typedef struct
{
    hcan_t *hcan;
    motor_num motor_index;
    uint32_t ctrl_tick;
    uint32_t reach_tick;
    float target_pos;
    float current_pos;
    float max_vel;
    float reach_tol;
    uint8_t enabled;
    uint8_t busy;
    uint8_t arrived;
    pid_para_t pos_pid;
    pid_para_t vel_pid;
} lift_ctrl_t;

static lift_ctrl_t lift_ctrl;

static float lift_ctrl_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float lift_ctrl_clampf(float value, float min_value, float max_value)
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

/* 与底盘 pos_pid_sync_pid_init() 相同的 PID 初始化方式。 */
static void lift_ctrl_pid_init(pid_para_t *pid,
                               float kp,
                               float ki,
                               float kd,
                               float out_max,
                               float out_min)
{
    pid_para_init(pid);
    pid_reset(pid, kp, ki, kd);
    pid_limit_init(pid, 1.0f, -1.0f, out_max, out_min);
    pid->ctrl_period = (float)LIFT_CTRL_PERIOD_MS * 0.001f;
}

void lift_ctrl_init(hcan_t *hcan, motor_num motor_index)
{
    memset(&lift_ctrl, 0, sizeof(lift_ctrl));

    lift_ctrl.hcan = hcan;
    lift_ctrl.motor_index = motor_index;
    lift_ctrl.max_vel = LIFT_CTRL_DEFAULT_MAX_VEL;
    lift_ctrl.reach_tol = LIFT_CTRL_REACH_TOL;
    lift_ctrl.arrived = 1U;
    lift_ctrl.ctrl_tick = HAL_GetTick();
    lift_ctrl.reach_tick = HAL_GetTick();

    lift_ctrl_pid_init(&lift_ctrl.pos_pid,
                       LIFT_CTRL_POS_KP,
                       LIFT_CTRL_POS_KI,
                       LIFT_CTRL_POS_KD,
                       LIFT_CTRL_POS_OUT_MAX,
                       LIFT_CTRL_POS_OUT_MIN);
    lift_ctrl_pid_init(&lift_ctrl.vel_pid,
                       LIFT_CTRL_VEL_KP,
                       LIFT_CTRL_VEL_KI,
                       LIFT_CTRL_VEL_KD,
                       LIFT_CTRL_VEL_OUT_MAX,
                       LIFT_CTRL_VEL_OUT_MIN);

    motor[motor_index].ctrl.mode = pos_mode;
    (void)motor_angle_register(motor_index);
    (void)motor_angle_set_pos_ratio(motor_index, LIFT_CTRL_POS_RATIO);
}

void lift_ctrl_set_target(float target_pos)
{
    lift_ctrl.target_pos = target_pos;
    lift_ctrl.busy = 1U;
    lift_ctrl.arrived = 0U;
    lift_ctrl.reach_tick = HAL_GetTick();
    pid_clear(&lift_ctrl.pos_pid);
    pid_clear(&lift_ctrl.vel_pid);
}

void lift_ctrl_set_max_vel(float max_vel)
{
    if (max_vel < 0.0f)
    {
        max_vel = 0.0f;
    }

    lift_ctrl.max_vel = max_vel;
}

void lift_ctrl_start(void)
{
    lift_ctrl.enabled = 1U;
    lift_ctrl.busy = 1U;
    lift_ctrl.arrived = 0U;
    lift_ctrl.reach_tick = HAL_GetTick();
}

void lift_ctrl_stop(void)
{
    lift_ctrl.enabled = 0U;
    lift_ctrl.busy = 0U;
    lift_ctrl.arrived = 1U;
}

float lift_ctrl_get_current_pos(void)
{
    lift_ctrl.current_pos = LIFT_CTRL_DIR * motor_angle_get(lift_ctrl.motor_index);
    return lift_ctrl.current_pos;
}

uint8_t lift_ctrl_is_busy(void)
{
    uint32_t now_tick;
    float pos_error;

    if ((lift_ctrl.enabled == 0U) || (lift_ctrl.hcan == NULL))
    {
        return 0U;
    }

    now_tick = HAL_GetTick();
    pos_error = lift_ctrl_absf(lift_ctrl.target_pos - lift_ctrl_get_current_pos());

    if (pos_error <= lift_ctrl.reach_tol)
    {
        if ((now_tick - lift_ctrl.reach_tick) >= LIFT_CTRL_REACH_HOLD_MS)
        {
            lift_ctrl.busy = 0U;
            lift_ctrl.arrived = 1U;
        }
    }
    else
    {
        lift_ctrl.busy = 1U;
        lift_ctrl.arrived = 0U;
        lift_ctrl.reach_tick = now_tick;
    }

    return lift_ctrl.busy;
}

uint8_t lift_ctrl_is_arrived(void)
{
    (void)lift_ctrl_is_busy();
    return lift_ctrl.arrived;
}

void lift_ctrl_process(void)
{
    motor_t *motor_ptr;
    uint32_t now_tick;
    float target_error;
    float current_vel;
    float pos_offset;
    float cmd_output_vel;
    float cmd_motor_vel;
    float motor_cmd_pos;

    if ((lift_ctrl.hcan == NULL) || (lift_ctrl.enabled == 0U))
    {
        return;
    }

    now_tick = HAL_GetTick();
    if ((now_tick - lift_ctrl.ctrl_tick) < LIFT_CTRL_PERIOD_MS)
    {
        return;
    }
    lift_ctrl.ctrl_tick = now_tick;

    motor_ptr = &motor[lift_ctrl.motor_index];
    lift_ctrl.current_pos = lift_ctrl_get_current_pos();
    target_error = lift_ctrl.target_pos - lift_ctrl.current_pos;

    /*
     * 直接采用 3519 的速度反馈，并转换到机构输出端单位。
     * 不能用“10 ms 位置差分”：CAN 反馈并非严格每 10 ms 抵达，差分值会在
     * 0 和瞬时大值之间跳变，进而让速度 PID 把 vel_set 反复压为 0。
     */
    current_vel = lift_ctrl_absf(motor_ptr->para.vel) * LIFT_CTRL_POS_RATIO;

    if (lift_ctrl_absf(target_error) <= LIFT_CTRL_SETTLE_TOL)
    {
        pos_offset = 0.0f;
        cmd_output_vel = 0.0f;
        pid_clear(&lift_ctrl.pos_pid);
        pid_clear(&lift_ctrl.vel_pid);
    }
    else
    {
        float abs_error = lift_ctrl_absf(target_error);
        float ramp_vel = lift_ctrl.max_vel;
        float brake_vel;

        /* 位置 P 环产生小幅前瞻位置修正，逻辑与底盘 pos_offset 相同。 */
        pos_offset = parallel_pid_ctrl(&lift_ctrl.pos_pid,
                                       lift_ctrl.target_pos,
                                       lift_ctrl.current_pos);
        /* 速度 P 环：距离越大允许速度越高；实际越快时自动收紧速度命令。 */
        cmd_output_vel = parallel_pid_ctrl(&lift_ctrl.vel_pid,
                                           abs_error,
                                           current_vel);
        cmd_output_vel = lift_ctrl_clampf(cmd_output_vel, 0.0f, lift_ctrl.max_vel);

        /* 第一层：进入升降专用减速区后按剩余距离线性降低允许速度。 */
        if (abs_error < LIFT_CTRL_DECEL_RANGE)
        {
            ramp_vel = lift_ctrl.max_vel * (abs_error / LIFT_CTRL_DECEL_RANGE);
            if ((abs_error > LIFT_CTRL_SETTLE_TOL) &&
                (ramp_vel < LIFT_CTRL_DECEL_MIN_VEL))
            {
                ramp_vel = LIFT_CTRL_DECEL_MIN_VEL;
            }
        }

        /* 第二层：v=sqrt(2*a*s)，保证以 LIFT_CTRL_DECEL_ACCEL 能在剩余距离内刹停。 */
        brake_vel = sqrtf(2.0f * LIFT_CTRL_DECEL_ACCEL * abs_error);
        if (brake_vel < ramp_vel)
        {
            ramp_vel = brake_vel;
        }
        if (cmd_output_vel > ramp_vel)
        {
            cmd_output_vel = ramp_vel;
        }
    }

    motor_cmd_pos = lift_ctrl.target_pos + pos_offset;
    /* 输出端速度乘减速比后，才是位置模式要求的电机侧速度上限。 */
    cmd_motor_vel = cmd_output_vel / LIFT_CTRL_POS_RATIO;
    motor_ptr->ctrl.mode = pos_mode;
    motor_ptr->ctrl.pos_set = motor_angle_to_raw_pos(lift_ctrl.motor_index,
                                                     LIFT_CTRL_DIR * motor_cmd_pos);
    motor_ptr->ctrl.vel_set = cmd_motor_vel;
    pos_ctrl(lift_ctrl.hcan, motor_ptr->id, motor_ptr->ctrl.pos_set, motor_ptr->ctrl.vel_set);

    (void)lift_ctrl_is_busy();
}
