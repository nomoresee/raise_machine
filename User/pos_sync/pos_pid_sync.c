#include "headfile.h"

#define POS_PID_SYNC_PERIOD_MS       20U///pid计算周期

#define POS_PID_SYNC_PRINT_MS        100U//打印周期

#define POS_PID_SYNC_MOTOR1_DIR      1.0f
#define POS_PID_SYNC_MOTOR2_DIR      1.0f

#define POS_PID_SYNC_POS_KP          0.20f//整体位置外环，判断两台电机平均位置离目标还有多远
#define POS_PID_SYNC_POS_KI          0.0f
#define POS_PID_SYNC_POS_KD          0.0f
#define POS_PID_SYNC_POS_OUT_MAX     5.0f
#define POS_PID_SYNC_POS_OUT_MIN    -5.0f

#define POS_PID_SYNC_BAL_KP          0.10f//同步修正环，判断 Motor1 和 Motor2 之间差多少
#define POS_PID_SYNC_BAL_KI          0.0f
#define POS_PID_SYNC_BAL_KD          0.0f
#define POS_PID_SYNC_BAL_OUT_MAX     2.0f
#define POS_PID_SYNC_BAL_OUT_MIN    -2.0f

#define POS_PID_SYNC_VEL_KP          0.50f//速度给定环，根据位置误差和当前速度算下发速度
#define POS_PID_SYNC_VEL_KI          0.0f
#define POS_PID_SYNC_VEL_KD          0.0f
#define POS_PID_SYNC_VEL_OUT_MAX     20.0f
#define POS_PID_SYNC_VEL_OUT_MIN     0.5f
#define POS_PID_SYNC_VEL_CMD_RATIO   30.0f

typedef struct
{
    hcan_t *hcan;
    motor_num motor1_index;
    motor_num motor2_index;
    uint32_t ctrl_tick;
    uint32_t print_tick;
    float target_pos;
    float max_output_vel;
    pid_para_t pos_pid;
    pid_para_t balance_pid;
    pid_para_t vel_pid;
} pos_pid_sync_t;

static pos_pid_sync_t pos_pid_sync;

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
    pid_limit_init(pid, 0.0f, 0.0f, out_max, out_min);
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
* @brief:      pos_pid_sync_vofa_print(float motor1_pos, float motor2_pos, float pos_error, float motor1_vel, float motor2_vel, float vel_error)
* @param:      motor1_pos：电机1反馈位置
* @param:      motor2_pos：电机2反馈位置
* @param:      pos_error：两台电机位置误差，计算方式为电机1位置减电机2位置
* @param:      motor1_vel：电机1反馈速度
* @param:      motor2_vel：电机2反馈速度
* @param:      vel_error：两台电机速度误差，计算方式为电机1速度减电机2速度
* @retval:     void
* @details:    使用 VOFA+ FireWater 格式输出数据。各通道使用英文逗号分隔，帧尾使用 \r\n。
***********************************************************************
**/
static void pos_pid_sync_vofa_print(float motor1_pos,
                                    float motor2_pos,
                                    float pos_error,
                                    float motor1_vel,
                                    float motor2_vel,
                                    float vel_error)
{
    printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
           motor1_pos,
           motor2_pos,
           pos_error,
           motor1_vel,
           motor2_vel,
           vel_error);
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
    motor_angle_init(motor1_index, motor2_index);
    pos_pid_sync.ctrl_tick = HAL_GetTick();
    pos_pid_sync.print_tick = HAL_GetTick();
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
* @brief:      pos_pid_sync_process(void)
* @retval:     void
* @details:    主循环周期调用的同步控制函数。根据两台电机实时位置和速度反馈，
*              通过位置 PID、同步 PID 和速度 PID 计算目标位置与速度，再调用 pos_ctrl 下发。
***********************************************************************
**/
void pos_pid_sync_process(void)
{
    motor_t *motor1;        // 电机 1 的控制结构体指针，用于读取反馈并下发控制量。
    motor_t *motor2;        // 电机 2 的控制结构体指针，用于读取反馈并下发控制量。
    uint32_t now_tick;      // 当前系统毫秒计时，用来控制 PID 计算周期和打印周期。
    float motor1_pos;       // 电机 1 按同步方向修正后的实际位置反馈。
    float motor2_pos;       // 电机 2 按同步方向修正后的实际位置反馈。
    float motor1_vel;       // 电机 1 按同步方向修正后的实际速度反馈。
    float motor2_vel;       // 电机 2 按同步方向修正后的实际速度反馈。
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
    motor_angle_update(); // 更新两台电机的多圈位置，保证后续位置反馈是最新值。

    motor1_pos = POS_PID_SYNC_MOTOR1_DIR * motor_angle_get(pos_pid_sync.motor1_index); // 读取电机 1 位置，并用方向系数统一正方向。
    motor2_pos = POS_PID_SYNC_MOTOR2_DIR * motor_angle_get(pos_pid_sync.motor2_index); // 读取电机 2 位置，并用方向系数统一正方向。
    motor1_vel = POS_PID_SYNC_MOTOR1_DIR * motor1->para.vel; // 读取电机 1 速度，并用方向系数统一正方向。
    motor2_vel = POS_PID_SYNC_MOTOR2_DIR * motor2->para.vel; // 读取电机 2 速度，并用方向系数统一正方向。
    avg_pos = 0.5f * (motor1_pos + motor2_pos); // 计算平均位置，用于判断整体距离目标位置还有多远。
    avg_vel = 0.5f * (pos_pid_sync_absf(motor1_vel) + pos_pid_sync_absf(motor2_vel)); // 计算平均速度幅值，用于速度环反馈。

    pos_offset = parallel_pid_ctrl(&pos_pid_sync.pos_pid, pos_pid_sync.target_pos, avg_pos); // 位置外环：目标位置与平均位置做 PID，输出整体位置补偿。
    balance_offset = parallel_pid_ctrl(&pos_pid_sync.balance_pid, 0.0f, motor1_pos - motor2_pos); // 同步环：目标位置差为 0，输出两台电机的反向修正量。
    cmd_vel = parallel_pid_ctrl(&pos_pid_sync.vel_pid, pos_pid_sync_absf(pos_offset), avg_vel); // 速度环：位置误差越大，允许速度越高；接近目标后速度降低。
    cmd_vel = pos_pid_sync_clampf(cmd_vel, 0.0f, pos_pid_sync.max_output_vel); // 限制输出轴速度，防止速度指令超出允许范围。

    motor1_cmd_pos = pos_pid_sync.target_pos + pos_offset - balance_offset; // 电机 1 目标位置 = 公共目标 + 整体补偿 - 同步修正。
    motor2_cmd_pos = pos_pid_sync.target_pos + pos_offset + balance_offset; // 电机 2 目标位置 = 公共目标 + 整体补偿 + 同步修正。

    pos_pid_sync_send(motor1, POS_PID_SYNC_MOTOR1_DIR * motor1_cmd_pos, cmd_vel); // 将电机 1 目标位置转回实际方向，并通过位置模式下发。
    pos_pid_sync_send(motor2, POS_PID_SYNC_MOTOR2_DIR * motor2_cmd_pos, cmd_vel); // 将电机 2 目标位置转回实际方向，并通过位置模式下发。

    if ((now_tick - pos_pid_sync.print_tick) >= POS_PID_SYNC_PRINT_MS) // 到达打印周期后输出调试数据。
    {
        pos_pid_sync.print_tick = now_tick; // 更新时间戳，控制下一次 VOFA 打印间隔。
        pos_pid_sync_vofa_print(motor1_pos,              // VOFA 通道 1：电机 1 统一方向后的位置反馈。
                                motor2_pos,              // VOFA 通道 2：电机 2 统一方向后的位置反馈。
                                motor1_pos - motor2_pos, // VOFA 通道 3：两台电机的位置同步误差。
                                motor1_vel,              // VOFA 通道 4：电机 1 统一方向后的速度反馈。
                                motor2_vel,              // VOFA 通道 5：电机 2 统一方向后的速度反馈。
                                motor1_vel - motor2_vel); // VOFA 通道 6：两台电机的速度误差。
    }
}
