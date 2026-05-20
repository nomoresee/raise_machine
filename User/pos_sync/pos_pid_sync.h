#ifndef __POS_PID_SYNC_H__
#define __POS_PID_SYNC_H__

#include "bsp_fdcan.h"
#include "dm_motor_drv.h"

/**
***********************************************************************
* @brief:      pos_pid_sync_init(hcan_t *hcan, motor_num motor1_index, motor_num motor2_index)
* @param:      hcan：FDCAN 句柄
* @param:      motor1_index：第一台同步电机在 motor 数组中的索引
* @param:      motor2_index：第二台同步电机在 motor 数组中的索引
* @retval:     void
* @details:    初始化基于 pos_ctrl 的双电机位置同步模块。
***********************************************************************
**/
void pos_pid_sync_init(hcan_t *hcan, motor_num motor1_index, motor_num motor2_index);

/**
***********************************************************************
* @brief:      pos_pid_sync_set_target(float target_pos)
* @param:      target_pos：双电机公共目标位置
* @retval:     void
* @details:    设置同步控制目标位置，单位与 pos_ctrl 的位置参数一致。
***********************************************************************
**/
void pos_pid_sync_set_target(float target_pos);
void pos_pid_sync_start(void);
void pos_pid_sync_stop(void);
uint8_t pos_pid_sync_is_busy(void);
uint8_t pos_pid_sync_is_arrived(void);
float pos_pid_sync_get_current_pos(void);

/**
***********************************************************************
* @brief:      pos_pid_sync_set_max_vel(float max_vel)
* @param:      max_vel：输出轴最大速度
* @retval:     void
* @details:    设置输出轴速度设定值（与改轨迹规划前 pos_vel 语义相同）。
*              内部 × RATIO(30) 换算为电机侧 rad/s，并钳到 MOTOR_VMAX(60)。
*              例如 pos_vel=1.0 → 60 rad/s；pos_vel=3.0 → 仍顶在 60 rad/s（与旧版一致）。
***********************************************************************
**/
void pos_pid_sync_set_max_vel(float max_vel);

/**
***********************************************************************
* @brief:      pos_pid_sync_set_max_accel(float max_accel)
* @param:      max_accel：梯形规划的最大加/减速度（电机侧 rad/s²，与 motor_angle/VOFA 同源）
* @retval:     void
* @details:    设置全程速度规划的加/减速度。典型值 10~60 rad/s²。
*              到 v_max 所用时间 ≈ max_motor_vel / max_accel。
*              传入 0 时退回默认值。
***********************************************************************
**/
void pos_pid_sync_set_max_accel(float max_accel);

/**
***********************************************************************
* @brief:      pos_pid_sync_target_state_machine(void)
* @retval:     void
* @details:    目标点状态机：1000 -> 0 -> -1000 -> 0，只跑一轮，每段停 2s。
***********************************************************************
**/
void pos_pid_sync_target_state_machine(void);

/**
***********************************************************************
* @brief:      pos_pid_sync_process(void)
* @retval:     void
* @details:    主循环中持续调用，按内部周期计算同步修正并调用 pos_ctrl 下发控制量。
***********************************************************************
**/
void pos_pid_sync_process(void);

/* ===== VOFA/LCD 调试快照 ===== */
typedef struct
{
    float target_x;
    float target_y;
    float motor1_pos;
    float motor2_pos;
    float pos_error;
    float motor3_pos;
    float motor1_vel; /* 底盘：与 motor_angle 同源的几何速度（统一方向 Δpos/Δt），非驱动器 vel 原始值 */
    float motor2_vel;
    float motor3_vel; /* 与 beam_ctrl 输出轴位置同源的几何速度 Δpos/Δt */
    float motor4_pos;
    float motor4_vel; /* 与 lift_ctrl 输出轴位置同源的几何速度 Δpos/Δt */
    uint8_t valid;
} pos_pid_sync_vofa_snapshot_t;

/**
 * @brief 获取最新一次 VOFA 打印同源的数据快照，用于 LCD 实时显示
 * @return 1: 成功 0: 当前无有效数据
 */
uint8_t pos_pid_sync_get_vofa_snapshot(pos_pid_sync_vofa_snapshot_t *out);

#endif
