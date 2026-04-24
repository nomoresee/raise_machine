#ifndef __MOTOR_ANGLE_H__
#define __MOTOR_ANGLE_H__

#include "dm_motor_drv.h"

/**
***********************************************************************
* @brief:      motor_angle_init(motor_num motor1_index, motor_num motor2_index)
* @param:      motor1_index：第一台电机在 motor 数组中的索引
* @param:      motor2_index：第二台电机在 motor 数组中的索引
* @retval:     void
* @details:    初始化两台电机的多圈实际输出轴角度计算模块。
***********************************************************************
**/
void motor_angle_init(motor_num motor1_index, motor_num motor2_index);

/**
***********************************************************************
* @brief:      motor_angle_reset(void)
* @retval:     void
* @details:    清零多圈角度累计状态，下次更新时会重新以当前反馈位置作为起点。
***********************************************************************
**/
void motor_angle_reset(void);

/**
***********************************************************************
* @brief:      motor_angle_update(void)
* @retval:     void
* @details:    根据电机当前映射位置反馈更新连续实际转动角度。
***********************************************************************
**/
void motor_angle_update(void);

/**
***********************************************************************
* @brief:      motor_angle_get(motor_num motor_index)
* @param:      motor_index：电机在 motor 数组中的索引
* @retval:     float
* @details:    获取指定电机展开后的连续实际输出轴角度，单位与电机位置反馈单位一致。
***********************************************************************
**/
float motor_angle_get(motor_num motor_index);

#endif
