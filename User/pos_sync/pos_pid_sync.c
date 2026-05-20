#include "headfile.h"

#define POS_PID_SYNC_PERIOD_MS       20U///pid计算周期

#define POS_PID_SYNC_PRINT_MS        100U//打印周期

#define POS_PID_SYNC_MOTOR1_DIR      1.0f
#define POS_PID_SYNC_MOTOR2_DIR     -1.0f
#define POS_PID_SYNC_MOTOR3_DIR      1.0f /* 与 beam_ctrl 中 BEAM_CTRL_DIR 保持一致 */
#define POS_PID_SYNC_MOTOR4_DIR      1.0f /* 与 lift_ctrl 中 LIFT_CTRL_DIR 保持一致 */

/* === 平衡环：保留原有双电机同步修正能力，不动 === */
#define POS_PID_SYNC_BAL_KP          0.08f//同步修正环，判断 Motor1 和 Motor2 之间差多少
#define POS_PID_SYNC_BAL_KI          0.0f
#define POS_PID_SYNC_BAL_KD          0.0f
#define POS_PID_SYNC_BAL_OUT_MAX     0.8f
#define POS_PID_SYNC_BAL_OUT_MIN    -0.8f

/* === 速度/换算/到位判定 ===
 *  pos_vel（main.c）语义不变：输出轴速度设定值，内部 × RATIO 换算为电机侧 rad/s。
 *  电机侧上限 MOTOR_VMAX（60 rad/s），超过会在 DM 12 位量化里环绕成乱值。
 *  因此 pos_vel > MOTOR_VMAX/RATIO/SPEED_GAIN（=0.6667）时，实际仍顶在 60 rad/s。
 *  注意：不要用 0.95 去钳 pos_vel 本身——会把 pos_vel=3 误钳成 0.95，体感慢约 3 倍。 */
#define POS_PID_SYNC_VEL_CMD_RATIO   30.0f
#define POS_PID_SYNC_VEL_SPEED_GAIN  3.0f    /* 相对旧版 pos_vel 的额外提速倍率（用户体感偏慢时启用） */
#define POS_PID_SYNC_MOTOR_VMAX      60.0f   /* 与 motor[].tmp.VMAX 保持一致 */
#define POS_PID_SYNC_VEL_OUT_MAX     (POS_PID_SYNC_MOTOR_VMAX / (POS_PID_SYNC_VEL_CMD_RATIO * POS_PID_SYNC_VEL_SPEED_GAIN))

#define POS_PID_SYNC_REACH_TOL       5.0f
#define POS_PID_SYNC_SYNC_REACH_TOL  2.0f
#define POS_PID_SYNC_REACH_HOLD_MS   80U
#define POS_PID_SYNC_SETTLE_TOL      1.0f
#define POS_PID_SYNC_SYNC_DEADBAND   0.8f

/* === 状态机调试目标 === */
#define POS_PID_SYNC_SM_DWELL_MS     2000U
#define POS_PID_SYNC_SM_REACH_TOL    5.0f

/* === 梯形轨迹规划参数（全程速度规划，替换"末端临时减速"） ===
 *
 *  ★ 单位约定（关键，不要搞错）：
 *      内部规划全部使用 "电机侧坐标系"，即与 motor_angle 同源：
 *      - ref_pos / target_pos : 电机侧 rad（累积多圈）
 *      - ref_vel              : 电机侧 rad/s
 *      - a_max                : 电机侧 rad/s²
 *    用户接口的 set_max_vel 仍接受 "输出轴速度"（原语义），内部乘 RATIO 后存为电机侧 rad/s。
 *    set_max_accel 直接接受 "电机侧 rad/s²"，方便和 VOFA 打印的 motor_vel 对照调参。
 *
 *  规划方式：在线 v² 律
 *  - 距目标 |Δs| > v_ref²/(2·a) 时：按 a 加速到 v_max（加速段）
 *  - 距目标 |Δs| > v_ref²/(2·a) 且 v_ref == v_max 时：匀速（巡航段）
 *  - 距目标 |Δs| ≤ v_ref²/(2·a) 时：按 a 减速直至 0（减速段）
 *  - 自动退化为三角形：当总距离不足 2·v_max²/(2a) 时，达不到 v_max 即开始减速
 *
 *  典型选值参考（v_max_motor ≈ 28.5 rad/s = 0.95 × 30）：
 *      a = 10 rad/s² → 加速段 ~2.85 s，到 v_max 走 ~40 rad（约 6 圈），柔和
 *      a = 30 rad/s² → 加速段 ~0.95 s，到 v_max 走 ~13 rad（约 2 圈），中等
 *      a = 60 rad/s² → 加速段 ~0.48 s，到 v_max 走 ~6.8 rad（约 1 圈），激进
 *  a 越大启停越快但易过冲，越小越柔但定位时间变长。 */
#define POS_PID_SYNC_TRAJ_ACCEL_UP_DEF 30.0f  /* 加速段加速度（电机侧 rad/s²），越大起步越快 */
#define POS_PID_SYNC_TRAJ_ACCEL_DN_DEF 20.0f  /* 减速段加速度，与加速分开，避免加快起步时末段变冲 */
#define POS_PID_SYNC_TRAJ_ACCEL_DEF    POS_PID_SYNC_TRAJ_ACCEL_UP_DEF /* set_max_accel 默认值 */
#define POS_PID_SYNC_TRAJ_VEL_MARGIN 1.08f   /* 巡航段 vel_limit 余量；减速段改用小余量，避免冲过目标 */
#define POS_PID_SYNC_TRAJ_VEL_MARGIN_DN 1.02f /* 已进入减速区时的 vel_limit 余量 */
#define POS_PID_SYNC_TRAJ_MIN_VEL    0.8f    /* 仅在中段防止卡死，末端不用 */
#define POS_PID_SYNC_TRAJ_BRAKE_GAIN 1.30f   /* 越大越早开始收速（加大减速力度） */
#define POS_PID_SYNC_BRAKE_STRONG    0.82f   /* 压低 sqrt(2as) 限速，等效刹车更狠，只乘一处 */
#define POS_PID_SYNC_CMD_SLEW_DN     45.0f   /* cmd_vel_limit 每周期最大下降量（rad/s），平滑减速指令 */
#define POS_PID_SYNC_DECEL_ZONE      300.0f  /* |target_error| 小于此值进入末端减速区（电机侧 rad） */
#define POS_PID_SYNC_APPROACH_DIST   120.0f  /* 剩余距离小于此值：进入低速爬行段 */
#define POS_PID_SYNC_APPROACH_VEL    2.0f    /* 爬行段入口最大速度（电机侧 rad/s），越近线性减小到 0 */
#define POS_PID_SYNC_OVERSHOOT_TOL   2.0f    /* 过冲超过此值则强制收速 */

typedef struct
{
    hcan_t *hcan;
    motor_num motor1_index;
    motor_num motor2_index;
    uint32_t ctrl_tick;
    uint32_t print_tick;
    uint32_t reach_tick;
    float target_pos;
    float max_motor_vel;          /* 梯形规划巡航速度上限（电机侧 rad/s，set_max_vel 换算后写入） */
    float max_output_accel;       /* 梯形规划的加/减速度（电机侧 rad/s²） */
    float current_pos;
    float reach_tol;
    uint8_t enabled;
    uint8_t busy;
    uint8_t arrived;
    pid_para_t balance_pid;       /* 只保留同步平衡环；位置外环与"伪速度环"被梯形规划取代 */

    /* === 梯形/简化S曲线规划状态 ===
     *  ref_pos：当前周期下发给驱动器的参考位置（沿规划轨迹推进）
     *  ref_vel：当前规划速度（带符号，方向 = 朝向目标） */
    float ref_pos;
    float ref_vel;
    float last_cmd_vel_limit;     /* 上一周期下发的速度上限，用于斜率限制 */
    uint8_t traj_inited;          /* 0=首次/新目标，需要把 ref_pos 锚到 avg_pos 重新规划 */
    uint8_t in_decel_zone;        /* 已进入按实际剩余距离减速的区间 */

    pos_pid_sync_vofa_snapshot_t vofa_snapshot;

    /* 底盘速度：与 motor_angle 同源（统一方向后 Δpos/Δt），避免两路驱动器 vel 刻度不一致 */
    float vofa_last_m1_raw;
    float vofa_last_m2_raw;
    float vofa_last_m3_raw;
    float vofa_last_m4_raw;
    uint32_t vofa_last_ms;
    uint8_t vofa_geom_inited;

    float track_m1;
    float track_m2;
    uint32_t track_ms;
    uint8_t track_inited;
} pos_pid_sync_t;

static pos_pid_sync_t pos_pid_sync;
static const float pos_pid_sync_sm_targets[] = {200.0f, 0.0f, -200.0f, 0.0f};

/**
***********************************************************************
* @brief:      pos_pid_sync_absf(float value)
* @param:      value：输入浮点数
* @retval:     float
* @details:    返回输入值的绝对值。
***********************************************************************
**/
static float pos_pid_sync_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

/**
***********************************************************************
* @brief:      pos_pid_sync_clampf(float value, float min_value, float max_value)
* @param:      value：待限幅的输入值
* @param:      min_value：输出下限
* @param:      max_value：输出上限
 * @retval:     float
* @details:    将输入值限制在指定范围内，避免位置修正量和速度指令超限。
***********************************************************************
**/
static float pos_pid_sync_clampf(float value, float min_value, float max_value)
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

/* 爬行段速度：距目标越近越低，在 APPROACH_DIST 处为 APPROACH_VEL，贴近目标时趋近 0（不在中途急停留误差） */
static float pos_pid_sync_creep_vel_cap(float abs_rem_err)
{
    float v_creep;

    if (abs_rem_err >= POS_PID_SYNC_APPROACH_DIST)
    {
        return POS_PID_SYNC_APPROACH_VEL;
    }
    if (abs_rem_err <= POS_PID_SYNC_SETTLE_TOL)
    {
        return 0.0f;
    }

    v_creep = POS_PID_SYNC_APPROACH_VEL * (abs_rem_err / POS_PID_SYNC_APPROACH_DIST);
    if (v_creep < 0.0f)
    {
        v_creep = 0.0f;
    }
    return v_creep;
}

/**
***********************************************************************
* @brief:      pos_pid_sync_pid_init(pid_para_t *pid, float kp, float ki, float kd, float out_max, float out_min)
* @param:      pid：指向 pid_para_t 结构体的指针
* @param:      kp：比例系数
* @param:      ki：积分系数
* @param:      kd：微分系数
* @param:      out_max：输出上限
* @param:      out_min：输出下限
* @retval:     void
* @details:    使用现有 PID 文件中的初始化、重置和限幅函数完成单个 PID 控制器配置。
***********************************************************************
**/
static void pos_pid_sync_pid_init(pid_para_t *pid,
                                  float kp,
                                  float ki,
                                  float kd,
                                  float out_max,
                                  float out_min)
{
    pid_para_init(pid);
    pid_reset(pid, kp, ki, kd);
    pid_limit_init(pid, 1.0f, -1.0f, out_max, out_min);
    pid->ctrl_period = (float)POS_PID_SYNC_PERIOD_MS * 0.001f;
}

/**
***********************************************************************
* @brief:      pos_pid_sync_send(motor_t *motor_ptr, float pos, float vel)
* @param:      motor_ptr：指向 motor_t 结构体的指针
* @param:      pos：位置模式下发目标位置
* @param:      vel：位置模式下发速度限制
* @retval:     void
* @details:    保存当前电机控制量，并通过原有 pos_ctrl 函数发送位置模式控制帧。
***********************************************************************
**/
/* 下发位置控制帧。
 *  pos      : 电机侧 rad（与 motor_angle 同单位，DM 驱动器内部支持多圈累积位置）
 *  motor_vel: 电机侧 rad/s（DM 驱动器内部会量化到 [-VMAX, +VMAX]，超出会环绕乱掉）
 *  这里再做一道兜底 clamp，避免量化溢出。 */
static void pos_pid_sync_send(motor_t *motor_ptr, float pos, float motor_vel)
{
    float vel_limit = motor_vel;

    if (vel_limit < 0.0f)
    {
        vel_limit = 0.0f;
    }
    if (vel_limit > POS_PID_SYNC_MOTOR_VMAX)
    {
        vel_limit = POS_PID_SYNC_MOTOR_VMAX;
    }

    motor_ptr->ctrl.mode = pos_mode;
    motor_ptr->ctrl.pos_set = pos;
    motor_ptr->ctrl.vel_set = vel_limit;
    pos_ctrl(pos_pid_sync.hcan, motor_ptr->id, motor_ptr->ctrl.pos_set, motor_ptr->ctrl.vel_set);
}

/**
***********************************************************************
* @brief:      pos_pid_sync_vofa_print(...)
* @details:    VOFA+ FireWater：目标/左右多圈位置/位置误差；5、6 列为与位置同源的几何速度（统一方向后 Δpos/Δt）。
***********************************************************************
**/
static void pos_pid_sync_vofa_print(float motor1_pos,
                                    float motor2_pos,
                                    float sync_error,
                                    float target_x,
                                    float target_y,
                                    float target_z,
                                    float motor3_pos,
                                    float motor4_pos)
{
    float motor1_show = POS_PID_SYNC_MOTOR1_DIR * motor1_pos;
    float motor2_show = POS_PID_SYNC_MOTOR2_DIR * motor2_pos;
    float target_error = pos_pid_sync_absf(pos_pid_sync_absf(motor1_show) - pos_pid_sync_absf(motor2_show));
    uint32_t now_ms = HAL_GetTick();
    float dt_s;
    float geom_m1_vel = 0.0f;
    float geom_m2_vel = 0.0f;
    float geom_m3_vel = 0.0f;
    float geom_m4_vel = 0.0f;

    (void)sync_error;

    if (pos_pid_sync.vofa_geom_inited != 0U)
    {
        dt_s = ((float)(now_ms - pos_pid_sync.vofa_last_ms)) * 0.001f;
        if (dt_s < 0.0005f)
        {
            dt_s = (float)POS_PID_SYNC_PRINT_MS * 0.001f;
        }
        geom_m1_vel = POS_PID_SYNC_MOTOR1_DIR * (motor1_pos - pos_pid_sync.vofa_last_m1_raw) / dt_s;
        geom_m2_vel = POS_PID_SYNC_MOTOR2_DIR * (motor2_pos - pos_pid_sync.vofa_last_m2_raw) / dt_s;
        geom_m3_vel = POS_PID_SYNC_MOTOR3_DIR * (motor3_pos - pos_pid_sync.vofa_last_m3_raw) / dt_s;
        geom_m4_vel = POS_PID_SYNC_MOTOR4_DIR * (motor4_pos - pos_pid_sync.vofa_last_m4_raw) / dt_s;
    }

    pos_pid_sync.vofa_last_m1_raw = motor1_pos;
    pos_pid_sync.vofa_last_m2_raw = motor2_pos;
    pos_pid_sync.vofa_last_m3_raw = motor3_pos;
    pos_pid_sync.vofa_last_m4_raw = motor4_pos;
    pos_pid_sync.vofa_last_ms = now_ms;
    pos_pid_sync.vofa_geom_inited = 1U;

    /* 缓存给 LCD 使用：与 VOFA 输出同源 */
    pos_pid_sync.vofa_snapshot.target_x = target_x;
    pos_pid_sync.vofa_snapshot.target_y = target_y;
    pos_pid_sync.vofa_snapshot.motor1_pos = motor1_show;
    pos_pid_sync.vofa_snapshot.motor2_pos = motor2_show;
    pos_pid_sync.vofa_snapshot.pos_error = target_error;
    pos_pid_sync.vofa_snapshot.motor3_pos = motor3_pos;
    pos_pid_sync.vofa_snapshot.motor1_vel = geom_m1_vel;
    pos_pid_sync.vofa_snapshot.motor2_vel = geom_m2_vel;
    pos_pid_sync.vofa_snapshot.motor3_vel = geom_m3_vel;
    pos_pid_sync.vofa_snapshot.motor4_pos = motor4_pos;
    pos_pid_sync.vofa_snapshot.motor4_vel = geom_m4_vel;
    pos_pid_sync.vofa_snapshot.valid = 1U;

    printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
           target_x,
           motor1_show,
           motor2_show,
           target_error,
           geom_m1_vel,
           geom_m2_vel,
           target_y,
           motor3_pos,
           geom_m3_vel,
           target_z,
           motor4_pos,
           geom_m4_vel);
}

uint8_t pos_pid_sync_get_vofa_snapshot(pos_pid_sync_vofa_snapshot_t *out)
{
    if (out == NULL)
    {
        return 0U;
    }

    if (pos_pid_sync.vofa_snapshot.valid == 0U)
    {
        return 0U;
    }

    *out = pos_pid_sync.vofa_snapshot;
    return 1U;
}

/**
***********************************************************************
* @brief:      pos_pid_sync_init(hcan_t *hcan, motor_num motor1_index, motor_num motor2_index)
* @param:      hcan：FDCAN 句柄
* @param:      motor1_index：第一台同步电机在 motor 数组中的索引
* @param:      motor2_index：第二台同步电机在 motor 数组中的索引
* @retval:     void
* @details:    初始化双电机位置同步控制模块，配置位置外环、同步修正环和速度给定环。
***********************************************************************
**/
void pos_pid_sync_init(hcan_t *hcan, motor_num motor1_index, motor_num motor2_index)
{
    memset(&pos_pid_sync, 0, sizeof(pos_pid_sync));

    pos_pid_sync.hcan = hcan;
    pos_pid_sync.motor1_index = motor1_index;
    pos_pid_sync.motor2_index = motor2_index;
    pos_pid_sync.max_motor_vel = POS_PID_SYNC_MOTOR_VMAX;
    pos_pid_sync.max_output_accel = POS_PID_SYNC_TRAJ_ACCEL_DEF;
    pos_pid_sync.reach_tol = POS_PID_SYNC_REACH_TOL;
    pos_pid_sync.arrived = 1U;
    pos_pid_sync.traj_inited = 0U;

    pos_pid_sync_pid_init(&pos_pid_sync.balance_pid,
                          POS_PID_SYNC_BAL_KP,
                          POS_PID_SYNC_BAL_KI,
                          POS_PID_SYNC_BAL_KD,
                          POS_PID_SYNC_BAL_OUT_MAX,
                          POS_PID_SYNC_BAL_OUT_MIN);

    motor[motor1_index].ctrl.mode = pos_mode;
    motor[motor2_index].ctrl.mode = pos_mode;
    (void)motor_angle_register(motor1_index);
    (void)motor_angle_register(motor2_index);
    pos_pid_sync.ctrl_tick = HAL_GetTick();
    pos_pid_sync.print_tick = HAL_GetTick();
    pos_pid_sync.reach_tick = HAL_GetTick();
}

/**
***********************************************************************
* @brief:      pos_pid_sync_set_target(float target_pos)
* @param:      target_pos：双电机公共目标位置
* @retval:     void
* @details:    设置两台电机同步运动的公共目标位置，单位与 pos_ctrl 的位置单位保持一致。
***********************************************************************
**/
void pos_pid_sync_set_target(float target_pos)
{
    pos_pid_sync.target_pos = target_pos;
    pos_pid_sync.busy = 1U;
    pos_pid_sync.arrived = 0U;
    pos_pid_sync.reach_tick = HAL_GetTick();
    /* 标记轨迹需要重新规划：下个 process 周期会把 ref_pos 锚到当前 avg_pos，
     * ref_vel 保留当前值（中途换目标时不会让电机急停） */
    pos_pid_sync.traj_inited = 0U;
    pos_pid_sync.last_cmd_vel_limit = 0.0f;
    pos_pid_sync.in_decel_zone = 0U;
    pos_pid_sync.ref_vel = 0.0f;
}

void pos_pid_sync_start(void)
{
    pos_pid_sync.enabled = 1U;
    pos_pid_sync.busy = 1U;
    pos_pid_sync.arrived = 0U;
    pos_pid_sync.reach_tick = HAL_GetTick();
    pos_pid_sync.vofa_geom_inited = 0U;
    pos_pid_sync.track_inited = 0U;
    pos_pid_sync.traj_inited = 0U;
    pos_pid_sync.last_cmd_vel_limit = 0.0f;
    pos_pid_sync.in_decel_zone = 0U;
}

void pos_pid_sync_stop(void)
{
    pos_pid_sync.enabled = 0U;
    pos_pid_sync.busy = 0U;
    pos_pid_sync.arrived = 1U;
    pos_pid_sync.vofa_geom_inited = 0U;
    pos_pid_sync.track_inited = 0U;
    pos_pid_sync.traj_inited = 0U;
    pos_pid_sync.ref_vel = 0.0f;
    pos_pid_sync.last_cmd_vel_limit = 0.0f;
    pos_pid_sync.in_decel_zone = 0U;
}

uint8_t pos_pid_sync_is_busy(void)
{
    uint32_t now_tick;
    float pos_error;
    float motor1_pos;
    float motor2_pos;
    float sync_error;

    if ((pos_pid_sync.enabled == 0U) || (pos_pid_sync.hcan == NULL))
    {
        return 0U;
    }

    now_tick = HAL_GetTick();
    pos_pid_sync.current_pos = pos_pid_sync_get_current_pos();
    pos_error = pos_pid_sync_absf(pos_pid_sync.target_pos - pos_pid_sync.current_pos);
    motor1_pos = POS_PID_SYNC_MOTOR1_DIR * motor_angle_get(pos_pid_sync.motor1_index);
    motor2_pos = POS_PID_SYNC_MOTOR2_DIR * motor_angle_get(pos_pid_sync.motor2_index);
    sync_error = pos_pid_sync_absf(motor1_pos - motor2_pos);

    if ((pos_error <= pos_pid_sync.reach_tol) &&
        (sync_error <= POS_PID_SYNC_SYNC_REACH_TOL))
    {
        if ((now_tick - pos_pid_sync.reach_tick) >= POS_PID_SYNC_REACH_HOLD_MS)
        {
            pos_pid_sync.busy = 0U;
            pos_pid_sync.arrived = 1U;
        }
    }
    else
    {
        pos_pid_sync.busy = 1U;
        pos_pid_sync.arrived = 0U;
        pos_pid_sync.reach_tick = now_tick;
    }

    return pos_pid_sync.busy;
}

uint8_t pos_pid_sync_is_arrived(void)
{
    return pos_pid_sync.arrived;
}

float pos_pid_sync_get_current_pos(void)
{
    float motor1_pos = POS_PID_SYNC_MOTOR1_DIR * motor_angle_get(pos_pid_sync.motor1_index);
    float motor2_pos = POS_PID_SYNC_MOTOR2_DIR * motor_angle_get(pos_pid_sync.motor2_index);

    pos_pid_sync.current_pos = 0.5f * (motor1_pos + motor2_pos);
    return pos_pid_sync.current_pos;
}

/**
***********************************************************************
* @brief:      pos_pid_sync_set_max_vel(float max_vel)
* @param:      max_vel：输出轴最大速度
* @retval:     void
* @details:    设置 pos_ctrl 下发的最大速度限制，内部会限制到模块允许范围内。
***********************************************************************
**/
void pos_pid_sync_set_max_vel(float max_vel)
{
    float motor_vel = max_vel * POS_PID_SYNC_VEL_CMD_RATIO * POS_PID_SYNC_VEL_SPEED_GAIN;

    if (motor_vel < 0.0f)
    {
        motor_vel = 0.0f;
    }
    if (motor_vel > POS_PID_SYNC_MOTOR_VMAX)
    {
        motor_vel = POS_PID_SYNC_MOTOR_VMAX;
    }
    pos_pid_sync.max_motor_vel = motor_vel;
}

/**
***********************************************************************
* @brief:      pos_pid_sync_set_max_accel(float max_accel)
* @param:      max_accel：梯形规划的最大加/减速度（电机侧 rad/s²，与 motor_angle 同源）
* @retval:     void
* @details:    设置全程轨迹规划的加减速度。值越大启停越快但易过冲，越小越柔但定位变慢。
*              典型值 10~60 rad/s²。传入 0 时退回默认值 POS_PID_SYNC_TRAJ_ACCEL_DEF。
***********************************************************************
**/
void pos_pid_sync_set_max_accel(float max_accel)
{
    if (max_accel <= 0.0f)
    {
        pos_pid_sync.max_output_accel = POS_PID_SYNC_TRAJ_ACCEL_DEF;
    }
    else
    {
        pos_pid_sync.max_output_accel = max_accel;
    }
}

/**
***********************************************************************
* @brief:      pos_pid_sync_target_state_machine(void)
* @retval:     void
* @details:    目标点状态机：1000 -> 0 -> -1000 -> 0，只执行一轮。
*              每次到达目标后保持 2s，最后停在 0 不再切换。
***********************************************************************
**/
void pos_pid_sync_target_state_machine(void)
{
    static uint8_t target_index = 0U;//当前走到目标数组的第几个点
    static uint8_t is_inited = 0U;//是否完成首次初始化
    static uint8_t wait_dwell = 0U;//是否进入到点后的停留阶段
    static uint8_t is_finished = 0U;//是否整轮执行结束
    static uint32_t dwell_tick = 0U;//进入停留阶段的时间戳
    uint32_t now_tick;
    float motor1_pos;
    float motor2_pos;
    float avg_pos;
    float target_pos;

    if (pos_pid_sync.hcan == NULL)
    {
        return;
    }

    now_tick = HAL_GetTick();

    if (is_inited == 0U)
    {
        target_index = 0U;
        wait_dwell = 0U;
        is_finished = 0U;
        dwell_tick = now_tick;
        is_inited = 1U;
        pos_pid_sync_set_target(pos_pid_sync_sm_targets[target_index]);
        return;
    }

    if (is_finished != 0U)
    {
        return;
    }

    target_pos = pos_pid_sync_sm_targets[target_index];

    if (wait_dwell != 0U)
    {
        if ((now_tick - dwell_tick) >= POS_PID_SYNC_SM_DWELL_MS)
        {
            uint8_t last_index = (uint8_t)((sizeof(pos_pid_sync_sm_targets) / sizeof(pos_pid_sync_sm_targets[0])) - 1U);
            if (target_index < last_index)
            {
                target_index++;
                pos_pid_sync_set_target(pos_pid_sync_sm_targets[target_index]);
                wait_dwell = 0U;
            }
            else
            {
                is_finished = 1U;
            }
        }
        return;
    }

    motor_angle_update();
    motor1_pos = POS_PID_SYNC_MOTOR1_DIR * motor_angle_get(pos_pid_sync.motor1_index);
    motor2_pos = POS_PID_SYNC_MOTOR2_DIR * motor_angle_get(pos_pid_sync.motor2_index);
    avg_pos = 0.5f * (motor1_pos + motor2_pos);

    if (pos_pid_sync_absf(target_pos - avg_pos) <= POS_PID_SYNC_SM_REACH_TOL)
    {
        dwell_tick = now_tick;
        wait_dwell = 1U;
    }
}

/**
***********************************************************************
* @brief:      pos_pid_sync_process(void)
* @retval:     void
* @details:    主循环周期调用的同步控制函数。根据两台电机实时位置和速度反馈，
*              通过位置 PID、同步 PID 和速度 PID 计算目标位置与速度，再调用 pos_ctrl 下发。
***********************************************************************
**/
void pos_pid_sync_process(void)
{
    if (pos_pid_sync.enabled == 0U)
    {
        return;
    }

    motor_t *motor1;        // 电机 1 的控制结构体指针，用于读取反馈并下发控制量。
    motor_t *motor2;        // 电机 2 的控制结构体指针，用于读取反馈并下发控制量。
    uint32_t now_tick;      // 当前系统毫秒计时，用来控制周期。
    float motor1_pos;       // 电机 1 按同步方向修正后的实际位置反馈。
    float motor2_pos;       // 电机 2 按同步方向修正后的实际位置反馈。
    float motor1_vel;       // 电机 1 按同步方向修正后的实际速度反馈（仅用于备用 avg_vel）。
    float motor2_vel;       // 电机 2 按同步方向修正后的实际速度反馈。
    float motor3_pos;       // 电机 3 位置（与 beam_ctrl 输出轴坐标一致）。
    float motor4_pos;       // 电机 4 升降位置（与 lift_ctrl 输出轴坐标一致）。
    float target_x;
    float target_y;
    float target_z;
    float target_error;     // target_pos - avg_pos（终点误差，仅用于到位/打印判断）
    float plan_error;       // target_pos - ref_pos（规划侧误差，驱动梯形演化）
    float sync_error;
    float avg_pos;          // 两台电机的平均位置，代表双电机系统的整体当前位置。
    float avg_vel;          // 两台电机速度绝对值的平均值，仅用于打印/调试。
    float balance_offset;   // 同步平衡环输出，用两电机位置差计算左右同步修正量。
    float cmd_vel_limit;    // 最终下发给位置模式的输出轴速度上限。
    float motor1_cmd_pos;   // 电机 1 在统一方向坐标系下的目标位置。
    float motor2_cmd_pos;   // 电机 2 在统一方向坐标系下的目标位置。
    float dt_s;             // 本周期时长（秒），用于梯形规划积分。

    if (pos_pid_sync.hcan == NULL)
    {
        return;
    }

    now_tick = HAL_GetTick();
    if ((now_tick - pos_pid_sync.ctrl_tick) < POS_PID_SYNC_PERIOD_MS)
    {
        return;
    }
    dt_s = (float)POS_PID_SYNC_PERIOD_MS * 0.001f;
    pos_pid_sync.ctrl_tick = now_tick;

    motor1 = &motor[pos_pid_sync.motor1_index];
    motor2 = &motor[pos_pid_sync.motor2_index];
    crane_route_get_current_pose_target(&target_x, &target_y, &target_z);
    motor_angle_update();

    motor1_pos = POS_PID_SYNC_MOTOR1_DIR * motor_angle_get(pos_pid_sync.motor1_index);
    motor2_pos = POS_PID_SYNC_MOTOR2_DIR * motor_angle_get(pos_pid_sync.motor2_index);
    motor1_vel = POS_PID_SYNC_MOTOR1_DIR * motor1->para.vel;
    motor2_vel = POS_PID_SYNC_MOTOR2_DIR * motor2->para.vel;
    motor3_pos = beam_ctrl_get_current_pos();
    motor4_pos = lift_ctrl_get_current_pos();
    avg_pos = 0.5f * (motor1_pos + motor2_pos);

    /* 几何速度（仅打印/调试用，规划不再依赖它） */
    if (pos_pid_sync.track_inited == 0U)
    {
        pos_pid_sync.track_m1 = motor1_pos;
        pos_pid_sync.track_m2 = motor2_pos;
        pos_pid_sync.track_ms = now_tick;
        pos_pid_sync.track_inited = 1U;
        avg_vel = 0.5f * (pos_pid_sync_absf(motor1_vel) + pos_pid_sync_absf(motor2_vel));
    }
    else
    {
        float dt_trk = ((float)(now_tick - pos_pid_sync.track_ms)) * 0.001f;
        if (dt_trk < 0.0005f)
        {
            avg_vel = 0.5f * (pos_pid_sync_absf(motor1_vel) + pos_pid_sync_absf(motor2_vel));
        }
        else
        {
            float g1 = (motor1_pos - pos_pid_sync.track_m1) / dt_trk;
            float g2 = (motor2_pos - pos_pid_sync.track_m2) / dt_trk;
            avg_vel = 0.5f * (pos_pid_sync_absf(g1) + pos_pid_sync_absf(g2));
        }
        pos_pid_sync.track_m1 = motor1_pos;
        pos_pid_sync.track_m2 = motor2_pos;
        pos_pid_sync.track_ms = now_tick;
    }

    target_error = pos_pid_sync.target_pos - avg_pos;
    sync_error = motor1_pos - motor2_pos;

    /* ============================================================
     *  梯形/三角形 在线轨迹规划
     *  ----------------------------------------------------------
     *  规则（带符号方向）：
     *    plan_error = target_pos - ref_pos
     *    若朝向目标的剩余距离 ≤ ref_vel² / (2·a)  → 减速：|ref_vel| -= a·dt
     *    否则若 |ref_vel| < v_max                 → 加速：朝向目标 +a·dt
     *    否则                                     → 匀速：保持 v_max
     *    ref_pos += ref_vel · dt
     *  这样自动出现「加速 → 匀速 → 减速 → 0」的梯形，
     *  距离不足时自动退化为三角形（达不到 v_max 即开始减速）。
     * ============================================================ */
    if (pos_pid_sync.traj_inited == 0U)
    {
        /* 首次或新目标：把 ref_pos 锚到当前实际位置，
         * ref_vel 保留（中途换目标时不会硬刹），但需要根据新目标方向重新评估。 */
        pos_pid_sync.ref_pos = avg_pos;
        pos_pid_sync.traj_inited = 1U;
        pid_clear(&pos_pid_sync.balance_pid);
    }

    plan_error = pos_pid_sync.target_pos - pos_pid_sync.ref_pos;

    {
        float v_max = pos_pid_sync.max_motor_vel;
        float a_up = pos_pid_sync.max_output_accel;   /* 加速段 */
        float a_dn = POS_PID_SYNC_TRAJ_ACCEL_DN_DEF;  /* 减速段 */
        /* 混合下发时必须用实际剩余距离，不能用 ref_pos：电机追的是 target，ref 会滞后导致减速晚、过冲抖 */
        float abs_rem_err = pos_pid_sync_absf(target_error);
        float dir_to_target = (target_error >= 0.0f) ? 1.0f : -1.0f;
        float curr_v = pos_pid_sync.ref_vel;
        float curr_speed = pos_pid_sync_absf(curr_v);
        float brake_dist;
        float dv_up = a_up * dt_s;
        float dv_dn = a_dn * dt_s;

        brake_dist = (curr_speed * curr_speed) / (2.0f * a_dn) * POS_PID_SYNC_TRAJ_BRAKE_GAIN;

        if (abs_rem_err <= POS_PID_SYNC_DECEL_ZONE)
        {
            pos_pid_sync.in_decel_zone = 1U;
        }

        /* 已过目标：实际位置误差符号与规划速度相反 → 立刻收 ref_vel，防止继续顶向目标 */
        if ((target_error * curr_v) < 0.0f)
        {
            if (curr_speed > (dv_dn * 2.0f))
            {
                curr_v -= dir_to_target * dv_dn * 2.0f;
            }
            else
            {
                curr_v = 0.0f;
            }
        }
        else if (abs_rem_err <= brake_dist)
        {
            if (curr_speed > dv_dn)
            {
                curr_v -= dir_to_target * dv_dn;
            }
            else
            {
                curr_v = 0.0f;
            }
        }
        else if (curr_speed < v_max)
        {
            curr_v += dir_to_target * dv_up;
            if (pos_pid_sync_absf(curr_v) > v_max)
            {
                curr_v = dir_to_target * v_max;
            }
        }
        else
        {
            curr_v = dir_to_target * v_max;
        }

        if ((abs_rem_err <= POS_PID_SYNC_SETTLE_TOL) &&
            (pos_pid_sync_absf(curr_v) <= dv_dn))
        {
            curr_v = 0.0f;
            pos_pid_sync.ref_pos = pos_pid_sync.target_pos;
        }
        else
        {
            pos_pid_sync.ref_pos += curr_v * dt_s;
        }
        pos_pid_sync.ref_vel = curr_v;

        /* 末段爬行：距目标越近 ref_vel 越低（线性收速） */
        if (abs_rem_err <= POS_PID_SYNC_APPROACH_DIST)
        {
            float v_creep = pos_pid_sync_creep_vel_cap(abs_rem_err);

            if (v_creep <= 0.0f)
            {
                pos_pid_sync.ref_vel = 0.0f;
            }
            else if (pos_pid_sync_absf(pos_pid_sync.ref_vel) > v_creep)
            {
                pos_pid_sync.ref_vel = dir_to_target * v_creep;
            }
        }

        (void)plan_error;
    }

    /* === 同步平衡环：末端缩小修正，避免减速时左右来回拧 === */
    if (pos_pid_sync_absf(target_error) <= POS_PID_SYNC_REACH_TOL)
    {
        float scale = pos_pid_sync_absf(target_error) / POS_PID_SYNC_REACH_TOL;

        if (pos_pid_sync_absf(sync_error) <= POS_PID_SYNC_SYNC_DEADBAND)
        {
            balance_offset = 0.0f;
            pid_clear(&pos_pid_sync.balance_pid);
        }
        else
        {
            balance_offset = parallel_pid_ctrl(&pos_pid_sync.balance_pid, 0.0f, sync_error) * scale;
        }
    }
    else if (pos_pid_sync_absf(sync_error) <= POS_PID_SYNC_SYNC_DEADBAND)
    {
        balance_offset = 0.0f;
        pid_clear(&pos_pid_sync.balance_pid);
    }
    else
    {
        balance_offset = parallel_pid_ctrl(&pos_pid_sync.balance_pid, 0.0f, sync_error);
    }

    /* === 下发：位置=target，速度=规划限速 + 物理可停距约束 + 斜率限制 === */
    {
        float abs_rem_err = pos_pid_sync_absf(target_error);
        float a_dn = POS_PID_SYNC_TRAJ_ACCEL_DN_DEF;
        float vel_margin = POS_PID_SYNC_TRAJ_VEL_MARGIN;
        float v_brake;
        float max_drop;

        if (pos_pid_sync.in_decel_zone != 0U)
        {
            vel_margin = POS_PID_SYNC_TRAJ_VEL_MARGIN_DN;
        }

        motor1_cmd_pos = pos_pid_sync.target_pos - balance_offset;
        motor2_cmd_pos = pos_pid_sync.target_pos + balance_offset;

        cmd_vel_limit = pos_pid_sync_absf(pos_pid_sync.ref_vel) * vel_margin;

        /* v = sqrt(2*a*s)：按真实剩余距离限制最高速度，混合模式防过冲的核心 */
        v_brake = sqrtf(2.0f * a_dn * abs_rem_err) * POS_PID_SYNC_BRAKE_STRONG;
        if (cmd_vel_limit > v_brake)
        {
            cmd_vel_limit = v_brake;
        }

        /* 末段爬行：剩余 < APPROACH_DIST 按距离线性限速，越近越慢 */
        if (abs_rem_err <= POS_PID_SYNC_APPROACH_DIST)
        {
            float v_creep = pos_pid_sync_creep_vel_cap(abs_rem_err);

            if (cmd_vel_limit > v_creep)
            {
                cmd_vel_limit = v_creep;
            }
        }

        /* 过冲：已越过目标则立刻 vel=0，避免再顶过去又回位 */
        if ((target_error * pos_pid_sync.ref_vel) < 0.0f)
        {
            cmd_vel_limit = 0.0f;
            pos_pid_sync.ref_vel = 0.0f;
            motor1_cmd_pos = pos_pid_sync.target_pos;
            motor2_cmd_pos = pos_pid_sync.target_pos;
            balance_offset = 0.0f;
            pid_clear(&pos_pid_sync.balance_pid);
        }

        /* 末端锁定：真正贴近目标且同步 OK 时 vel=0（不再提前 CREEP_STOP 急停留固定误差） */
        if ((abs_rem_err <= POS_PID_SYNC_SETTLE_TOL) &&
            (pos_pid_sync_absf(sync_error) <= POS_PID_SYNC_SYNC_DEADBAND))
        {
            cmd_vel_limit = 0.0f;
            pos_pid_sync.ref_vel = 0.0f;
            motor1_cmd_pos = pos_pid_sync.target_pos;
            motor2_cmd_pos = pos_pid_sync.target_pos;
            balance_offset = 0.0f;
            pid_clear(&pos_pid_sync.balance_pid);
        }
        else if ((abs_rem_err > POS_PID_SYNC_APPROACH_DIST) &&
                 (cmd_vel_limit < POS_PID_SYNC_TRAJ_MIN_VEL) &&
                 (pos_pid_sync_absf(pos_pid_sync.ref_vel) > 0.2f) &&
                 (abs_rem_err > POS_PID_SYNC_SETTLE_TOL))
        {
            cmd_vel_limit = POS_PID_SYNC_TRAJ_MIN_VEL;
        }

        if (cmd_vel_limit > POS_PID_SYNC_MOTOR_VMAX)
        {
            cmd_vel_limit = POS_PID_SYNC_MOTOR_VMAX;
        }

        /* 减速段斜率限制：禁止 vel_limit 一周期内掉太快，避免驱动器阶跃引发抖动 */
        max_drop = POS_PID_SYNC_CMD_SLEW_DN * dt_s;
        if (pos_pid_sync.last_cmd_vel_limit > 0.01f)
        {
            if (cmd_vel_limit < (pos_pid_sync.last_cmd_vel_limit - max_drop))
            {
                cmd_vel_limit = pos_pid_sync.last_cmd_vel_limit - max_drop;
            }
        }
        pos_pid_sync.last_cmd_vel_limit = cmd_vel_limit;
    }

    pos_pid_sync_send(motor1, POS_PID_SYNC_MOTOR1_DIR * motor1_cmd_pos, cmd_vel_limit);
    pos_pid_sync_send(motor2, POS_PID_SYNC_MOTOR2_DIR * motor2_cmd_pos, cmd_vel_limit);

    (void)target_error;
    (void)avg_vel;

    if ((now_tick - pos_pid_sync.print_tick) >= POS_PID_SYNC_PRINT_MS)
    {
        pos_pid_sync.print_tick = now_tick;
        pos_pid_sync_vofa_print(motor_angle_get(pos_pid_sync.motor1_index),
                                motor_angle_get(pos_pid_sync.motor2_index),
                                motor_angle_get(pos_pid_sync.motor1_index) +
                                motor_angle_get(pos_pid_sync.motor2_index),
                                target_x,
                                target_y,
                                target_z,
                                motor3_pos,
                                motor4_pos);
    }
}
