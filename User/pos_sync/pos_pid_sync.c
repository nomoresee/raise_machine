#include "headfile.h"

#define POS_PID_SYNC_PERIOD_MS       20U
#define POS_PID_SYNC_PRINT_MS        100U

#define POS_PID_SYNC_MOTOR1_DIR      1.0f
#define POS_PID_SYNC_MOTOR2_DIR      1.0f

#define POS_PID_SYNC_POS_KP          0.20f
#define POS_PID_SYNC_POS_KI          0.0f
#define POS_PID_SYNC_POS_KD          0.0f
#define POS_PID_SYNC_POS_OUT_MAX     5.0f
#define POS_PID_SYNC_POS_OUT_MIN    -5.0f

#define POS_PID_SYNC_BAL_KP          0.10f
#define POS_PID_SYNC_BAL_KI          0.0f
#define POS_PID_SYNC_BAL_KD          0.0f
#define POS_PID_SYNC_BAL_OUT_MAX     2.0f
#define POS_PID_SYNC_BAL_OUT_MIN    -2.0f

#define POS_PID_SYNC_VEL_KP          0.50f
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
    motor_t *motor1;
    motor_t *motor2;
    uint32_t now_tick;
    float motor1_pos;
    float motor2_pos;
    float motor1_vel;
    float motor2_vel;
    float avg_pos;
    float avg_vel;
    float pos_offset;
    float balance_offset;
    float cmd_vel;
    float motor1_cmd_pos;
    float motor2_cmd_pos;

    if (pos_pid_sync.hcan == NULL)
    {
        return;
    }

    now_tick = HAL_GetTick();
    if ((now_tick - pos_pid_sync.ctrl_tick) < POS_PID_SYNC_PERIOD_MS)
    {
        return;
    }
    pos_pid_sync.ctrl_tick = now_tick;

    motor1 = &motor[pos_pid_sync.motor1_index];
    motor2 = &motor[pos_pid_sync.motor2_index];
    motor_angle_update();

    motor1_pos = POS_PID_SYNC_MOTOR1_DIR * motor_angle_get(pos_pid_sync.motor1_index);
    motor2_pos = POS_PID_SYNC_MOTOR2_DIR * motor_angle_get(pos_pid_sync.motor2_index);
    motor1_vel = POS_PID_SYNC_MOTOR1_DIR * motor1->para.vel;
    motor2_vel = POS_PID_SYNC_MOTOR2_DIR * motor2->para.vel;
    avg_pos = 0.5f * (motor1_pos + motor2_pos);
    avg_vel = 0.5f * (pos_pid_sync_absf(motor1_vel) + pos_pid_sync_absf(motor2_vel));

    pos_offset = parallel_pid_ctrl(&pos_pid_sync.pos_pid, pos_pid_sync.target_pos, avg_pos);
    balance_offset = parallel_pid_ctrl(&pos_pid_sync.balance_pid, 0.0f, motor1_pos - motor2_pos);
    cmd_vel = parallel_pid_ctrl(&pos_pid_sync.vel_pid, pos_pid_sync_absf(pos_offset), avg_vel);
    cmd_vel = pos_pid_sync_clampf(cmd_vel, 0.0f, pos_pid_sync.max_output_vel);

    motor1_cmd_pos = pos_pid_sync.target_pos + pos_offset - balance_offset;
    motor2_cmd_pos = pos_pid_sync.target_pos + pos_offset + balance_offset;

    pos_pid_sync_send(motor1, POS_PID_SYNC_MOTOR1_DIR * motor1_cmd_pos, cmd_vel);
    pos_pid_sync_send(motor2, POS_PID_SYNC_MOTOR2_DIR * motor2_cmd_pos, cmd_vel);

    if ((now_tick - pos_pid_sync.print_tick) >= POS_PID_SYNC_PRINT_MS)
    {
        pos_pid_sync.print_tick = now_tick;
        pos_pid_sync_vofa_print(motor1_pos,
                                motor2_pos,
                                motor1_pos - motor2_pos,
                                motor1_vel,
                                motor2_vel,
                                motor1_vel - motor2_vel);
    }
}
