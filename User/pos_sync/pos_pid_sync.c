#include "headfile.h"

#define POS_PID_SYNC_PERIOD_MS       20U///pid计算周期

#define POS_PID_SYNC_PRINT_MS        100U//打印周期

#define POS_PID_SYNC_MOTOR1_DIR      1.0f
#define POS_PID_SYNC_MOTOR2_DIR     -1.0f
#define POS_PID_SYNC_MOTOR3_DIR      1.0f /* 与 beam_ctrl 中 BEAM_CTRL_DIR 保持一致 */
#define POS_PID_SYNC_MOTOR4_DIR      1.0f /* 与 lift_ctrl 中 LIFT_CTRL_DIR 保持一致 */

#define POS_PID_SYNC_POS_KP          0.20f//整体位置外环，判断两台电机平均位置离目标还有多远
#define POS_PID_SYNC_POS_KI          0.0f
#define POS_PID_SYNC_POS_KD          0.0f
#define POS_PID_SYNC_POS_OUT_MAX     5.0f
#define POS_PID_SYNC_POS_OUT_MIN    -5.0f

#define POS_PID_SYNC_BAL_KP          0.20f//同步修正环，判断 Motor1 和 Motor2 之间差多少
#define POS_PID_SYNC_BAL_KI          0.0f
#define POS_PID_SYNC_BAL_KD          0.0f
#define POS_PID_SYNC_BAL_OUT_MAX     2.0f
#define POS_PID_SYNC_BAL_OUT_MIN    -2.0f
#undef POS_PID_SYNC_BAL_KP
#undef POS_PID_SYNC_BAL_OUT_MAX
#undef POS_PID_SYNC_BAL_OUT_MIN
#define POS_PID_SYNC_BAL_KP          0.08f
#define POS_PID_SYNC_BAL_OUT_MAX     0.8f
#define POS_PID_SYNC_BAL_OUT_MIN    -0.8f

#define POS_PID_SYNC_VEL_KP          0.20f//速度给定环，根据位置误差和当前速度算下发速度
#define POS_PID_SYNC_VEL_KI          0.0f
#define POS_PID_SYNC_VEL_KD          0.0f
#define POS_PID_SYNC_VEL_OUT_MAX     20.0f
#define POS_PID_SYNC_VEL_OUT_MIN     0.5f
#define POS_PID_SYNC_VEL_CMD_RATIO   30.0f

#define POS_PID_SYNC_SM_DWELL_MS     2000U
#define POS_PID_SYNC_SM_REACH_TOL    5.0f
#define POS_PID_SYNC_REACH_TOL       5.0f
#define POS_PID_SYNC_SYNC_REACH_TOL  2.0f
#define POS_PID_SYNC_REACH_HOLD_MS   80U
#define POS_PID_SYNC_SETTLE_TOL      1.0f
#define POS_PID_SYNC_SYNC_DEADBAND   0.8f
#define POS_PID_SYNC_HOLD_VEL        0.0f

/* === 到位减速坡道：|err| < DECEL_RANGE 时，cmd_vel 上限按距离线性收缩到 0，
 *     再叠加按当前速度可停下的距离约束，避免过冲震荡 === */
#define POS_PID_SYNC_DECEL_RANGE     200.0f   /* 减速带宽度（与 motor_angle 位置单位一致） */
#define POS_PID_SYNC_DECEL_MIN_VEL   0.30f   /* 接近目标但未进入 SETTLE 时的最小限速，避免趴下 */
#define POS_PID_SYNC_DECEL_ACCEL     80.0f   /* 等效减速度（输出轴速度单位/秒），用于按 v^2/2a 估剩余距 */

typedef struct
{
    hcan_t *hcan;
    motor_num motor1_index;
    motor_num motor2_index;
    uint32_t ctrl_tick;
    uint32_t print_tick;
    uint32_t reach_tick;
    float target_pos;
    float max_output_vel;
    float current_pos;
    float reach_tol;
    uint8_t enabled;
    uint8_t busy;
    uint8_t arrived;
    pid_para_t pos_pid;
    pid_para_t balance_pid;
    pid_para_t vel_pid;

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
/* output_vel is output-shaft speed. pos_ctrl needs motor-side speed. */
static void pos_pid_sync_send(motor_t *motor_ptr, float pos, float output_vel)
{
    float output_vel_limit = pos_pid_sync_clampf(output_vel, 0.0f, pos_pid_sync.max_output_vel);
    float motor_cmd_vel = output_vel_limit * POS_PID_SYNC_VEL_CMD_RATIO;

    motor_ptr->ctrl.mode = pos_mode;
    motor_ptr->ctrl.pos_set = pos;
    motor_ptr->ctrl.vel_set = motor_cmd_vel;
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
    pos_pid_sync.max_output_vel = POS_PID_SYNC_VEL_OUT_MAX;
    pos_pid_sync.reach_tol = POS_PID_SYNC_REACH_TOL;
    pos_pid_sync.arrived = 1U;

    pos_pid_sync_pid_init(&pos_pid_sync.pos_pid,
                          POS_PID_SYNC_POS_KP,
                          POS_PID_SYNC_POS_KI,
                          POS_PID_SYNC_POS_KD,
                          POS_PID_SYNC_POS_OUT_MAX,
                          POS_PID_SYNC_POS_OUT_MIN);

    pos_pid_sync_pid_init(&pos_pid_sync.balance_pid,
                          POS_PID_SYNC_BAL_KP,
                          POS_PID_SYNC_BAL_KI,
                          POS_PID_SYNC_BAL_KD,
                          POS_PID_SYNC_BAL_OUT_MAX,
                          POS_PID_SYNC_BAL_OUT_MIN);

    pos_pid_sync_pid_init(&pos_pid_sync.vel_pid,
                          POS_PID_SYNC_VEL_KP,
                          POS_PID_SYNC_VEL_KI,
                          POS_PID_SYNC_VEL_KD,
                          POS_PID_SYNC_VEL_OUT_MAX,
                          POS_PID_SYNC_VEL_OUT_MIN);

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
    pid_clear(&pos_pid_sync.pos_pid);
    pid_clear(&pos_pid_sync.vel_pid);
}

void pos_pid_sync_start(void)
{
    pos_pid_sync.enabled = 1U;
    pos_pid_sync.busy = 1U;
    pos_pid_sync.arrived = 0U;
    pos_pid_sync.reach_tick = HAL_GetTick();
    pos_pid_sync.vofa_geom_inited = 0U;
    pos_pid_sync.track_inited = 0U;
}

void pos_pid_sync_stop(void)
{
    pos_pid_sync.enabled = 0U;
    pos_pid_sync.busy = 0U;
    pos_pid_sync.arrived = 1U;
    pos_pid_sync.vofa_geom_inited = 0U;
    pos_pid_sync.track_inited = 0U;
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
    pos_pid_sync.max_output_vel = pos_pid_sync_clampf(max_vel, 0.0f, POS_PID_SYNC_VEL_OUT_MAX);
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
    uint32_t now_tick;      // 当前系统毫秒计时，用来控制 PID 计算周期和打印周期。
    float motor1_pos;       // 电机 1 按同步方向修正后的实际位置反馈。
    float motor2_pos;       // 电机 2 按同步方向修正后的实际位置反馈。
    float motor1_vel;       // 电机 1 按同步方向修正后的实际速度反馈。
    float motor2_vel;       // 电机 2 按同步方向修正后的实际速度反馈。
    float motor3_pos;     // 电机 3 位置（与 beam_ctrl 输出轴坐标一致）。
    float motor4_pos;     // 电机 4 升降位置（与 lift_ctrl 输出轴坐标一致）。
    float target_x;
    float target_y;
    float target_z;
    float target_error;
    float sync_error;
    float avg_pos;          // 两台电机的平均位置，代表双电机系统的整体当前位置。
    float avg_vel;          // 两台电机速度绝对值的平均值，代表当前整体运动速度。
    float pos_offset;       // 位置外环输出，用平均位置误差计算整体位置补偿量。
    float balance_offset;   // 同步平衡环输出，用两电机位置差计算左右同步修正量。
    float cmd_vel;          // 最终下发给位置模式的输出轴速度限制。
    float motor1_cmd_pos;   // 电机 1 在统一方向坐标系下的目标位置。
    float motor2_cmd_pos;   // 电机 2 在统一方向坐标系下的目标位置。

    if (pos_pid_sync.hcan == NULL) // CAN 句柄未初始化时不能下发控制帧，直接退出保护。
    {
        return; // 等待 pos_pid_sync_init() 完成初始化后再运行同步控制。
    }

    now_tick = HAL_GetTick(); // 读取当前系统时间，单位为 ms。
    if ((now_tick - pos_pid_sync.ctrl_tick) < POS_PID_SYNC_PERIOD_MS) // 未到控制周期时不重复计算 PID。
    {
        return; // 保持上一次下发的控制量，避免控制频率过高。
    }
    pos_pid_sync.ctrl_tick = now_tick; // 记录本次控制时间，作为下一次周期判断的基准。

    motor1 = &motor[pos_pid_sync.motor1_index]; // 根据初始化时保存的索引找到电机 1。
    motor2 = &motor[pos_pid_sync.motor2_index]; // 根据初始化时保存的索引找到电机 2。
    crane_route_get_current_pose_target(&target_x, &target_y, &target_z);
    motor_angle_update(); // 更新已登记电机的多圈位置。

    motor1_pos = POS_PID_SYNC_MOTOR1_DIR * motor_angle_get(pos_pid_sync.motor1_index); // 读取电机 1 位置，并用方向系数统一正方向。
    motor2_pos = POS_PID_SYNC_MOTOR2_DIR * motor_angle_get(pos_pid_sync.motor2_index); // 读取电机 2 位置，并用方向系数统一正方向。
    motor1_vel = POS_PID_SYNC_MOTOR1_DIR * motor1->para.vel; // 驱动器反馈速度（仅作备用，avg_vel 优先用几何速度）
    motor2_vel = POS_PID_SYNC_MOTOR2_DIR * motor2->para.vel;
    motor3_pos = beam_ctrl_get_current_pos();
    motor4_pos = lift_ctrl_get_current_pos();
    avg_pos = 0.5f * (motor1_pos + motor2_pos); // 计算平均位置，用于判断整体距离目标位置还有多远。
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

    if ((pos_pid_sync_absf(target_error) <= POS_PID_SYNC_SETTLE_TOL) &&
        (pos_pid_sync_absf(sync_error) <= POS_PID_SYNC_SYNC_DEADBAND))
    {
        pos_offset = 0.0f;
        balance_offset = 0.0f;
        cmd_vel = POS_PID_SYNC_HOLD_VEL;
        pid_clear(&pos_pid_sync.pos_pid);
        pid_clear(&pos_pid_sync.balance_pid);
        pid_clear(&pos_pid_sync.vel_pid);
    }
    else
    {
    pos_offset = parallel_pid_ctrl(&pos_pid_sync.pos_pid, pos_pid_sync.target_pos, avg_pos); // 位置外环：目标位置与平均位置做 PID，输出整体位置补偿。
        if (pos_pid_sync_absf(sync_error) <= POS_PID_SYNC_SYNC_DEADBAND)
        {
            balance_offset = 0.0f;
            pid_clear(&pos_pid_sync.balance_pid);
        }
        else
        {
            balance_offset = parallel_pid_ctrl(&pos_pid_sync.balance_pid, 0.0f, sync_error);
        }
        cmd_vel = parallel_pid_ctrl(&pos_pid_sync.vel_pid, pos_pid_sync_absf(target_error), avg_vel);
        cmd_vel = pos_pid_sync_clampf(cmd_vel, 0.0f, pos_pid_sync.max_output_vel);

        /* ---- 到位减速坡道：进入 DECEL_RANGE 后，将 cmd_vel 上限按距离线性收紧 ---- */
        {
            float abs_err = pos_pid_sync_absf(target_error);
            float ramp_vel = pos_pid_sync.max_output_vel;

            if (abs_err < POS_PID_SYNC_DECEL_RANGE)
            {
                ramp_vel = pos_pid_sync.max_output_vel * (abs_err / POS_PID_SYNC_DECEL_RANGE);

                /* 还在工作区内时给个最小限速，避免被 PID/速度噪声卡死无法继续靠近 */
                if ((abs_err > POS_PID_SYNC_SETTLE_TOL) && (ramp_vel < POS_PID_SYNC_DECEL_MIN_VEL))
                {
                    ramp_vel = POS_PID_SYNC_DECEL_MIN_VEL;
                }
            }

            /* 二次约束：按 v = sqrt(2 * a * s) 计算「当前距离最大允许速度」，
             * 让减速过程平滑到 0，避免靠近时仍以坡道边界速度冲过去 */
            {
                float brake_vel = sqrtf(2.0f * POS_PID_SYNC_DECEL_ACCEL * abs_err);
                if (brake_vel < ramp_vel)
                {
                    ramp_vel = brake_vel;
                }
            }

            if (cmd_vel > ramp_vel)
            {
                cmd_vel = ramp_vel;
            }
            if (cmd_vel < 0.0f)
            {
                cmd_vel = 0.0f;
            }
        }
    }

    motor1_cmd_pos = pos_pid_sync.target_pos + pos_offset - balance_offset; // 电机 1 目标位置 = 公共目标 + 整体补偿 - 同步修正。
    motor2_cmd_pos = pos_pid_sync.target_pos + pos_offset + balance_offset; // 电机 2 目标位置 = 公共目标 + 整体补偿 + 同步修正。

    pos_pid_sync_send(motor1, POS_PID_SYNC_MOTOR1_DIR * motor1_cmd_pos, cmd_vel); // 将电机 1 目标位置转回实际方向，并通过位置模式下发。
    pos_pid_sync_send(motor2, POS_PID_SYNC_MOTOR2_DIR * motor2_cmd_pos, cmd_vel); // 将电机 2 目标位置转回实际方向，并通过位置模式下发。

    if ((now_tick - pos_pid_sync.print_tick) >= POS_PID_SYNC_PRINT_MS) // 到达打印周期后输出调试数据。
    {
        pos_pid_sync.print_tick = now_tick; // 更新时间戳，控制下一次 VOFA 打印间隔。
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
