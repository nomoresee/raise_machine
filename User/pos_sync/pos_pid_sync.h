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

/**
***********************************************************************
* @brief:      pos_pid_sync_set_max_vel(float max_vel)
* @param:      max_vel：输出轴最大速度
* @retval:     void
* @details:    设置输出轴反馈速度允许达到的最大值，内部会换算为电机侧速度下发。
***********************************************************************
**/
void pos_pid_sync_set_max_vel(float max_vel);

/**
***********************************************************************
* @brief:      pos_pid_sync_process(void)
* @retval:     void
* @details:    主循环中持续调用，按内部周期计算同步修正并调用 pos_ctrl 下发控制量。
***********************************************************************
**/
void pos_pid_sync_process(void);

#endif
