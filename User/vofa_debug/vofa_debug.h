#ifndef __VOFA_DEBUG_H__
#define __VOFA_DEBUG_H__

#include "dm_motor_drv.h"
#include <stdint.h>

typedef struct
{
    float target_x;
    float target_y;
    float motor1_pos;
    float motor2_pos;
    float pos_error;
    float motor3_pos;
    float motor1_vel;
    float motor2_vel;
    float motor3_vel;
    float motor4_pos;
    float motor4_vel;
    float target_upper_y;
    float target_lower_y;
    float motor5_pos;
    float motor6_pos;
    float motor5_vel;
    float motor6_vel;
    uint32_t motor5_feedback_age_ms;
    uint32_t motor6_feedback_age_ms;
    uint16_t route_state;
    uint16_t route_fault;
    uint8_t valid;
} vofa_debug_snapshot_t;

void vofa_debug_init(motor_num motor1_index,
                     motor_num motor2_index,
                     motor_num motor3_index,
                     motor_num motor4_index,
                     motor_num motor5_index,
                     motor_num motor6_index);
void vofa_debug_process(void);

/**
 * @brief 单步调试专用 VOFA 输出。
 * @param motor_select 当前调试选择号，仅用于调用端语义。
 * @param actual_pos 由 motor_angle_get() 展开得到的实际机构位置。
 * @param target_pos 当前单步调试的机构目标位置。
 * @param feedback_motor_vel 当前选中电机的速度反馈绝对值，单位为电机侧速度单位。
 * @param command_motor_vel 当前下发给该电机位置模式的速度上限，单位为电机侧速度单位。
 * @note USART1 每 200 ms 输出一行：
 *       actual_pos,target_pos,feedback_motor_vel,command_motor_vel\r\n。
 *       在 VOFA 中依次对应实际位置、目标位置、实际速度、速度指令。
 */
void vofa_debug_step_process(uint8_t motor_select,
                             float actual_pos,
                             float target_pos,
                             float feedback_motor_vel,
                             float command_motor_vel);

/**
 * @brief 底盘双电机单步调试专用 VOFA 输出。
 * @note USART1 每 20 ms 输出一行八个浮点数，顺序为：
 *       M1实际位置、M1目标位置、M1反馈速度、M1下发速度、
 *       M2实际位置、M2目标位置、M2反馈速度、M2下发速度。
 *       均为电机原始坐标；Motor2 未做方向翻转，方便直接对应 CAN ID 2。
 */
void vofa_debug_chassis_process(float motor1_actual_pos,
                                float motor1_target_pos,
                                float motor1_feedback_vel,
                                float motor1_command_vel,
                                float motor2_actual_pos,
                                float motor2_target_pos,
                                float motor2_feedback_vel,
                                float motor2_command_vel);

/**
 * @brief 底盘自动调试的精简 VOFA 输出。
 * @note USART1 每 20 ms 输出：
 *       M1实际位置,M1实际速度,M1目标位置,M1下发速度,
 *       M2实际位置,M2实际速度,M2目标位置,M2下发速度。
 */
void vofa_debug_chassis_feedback_process(float motor1_actual_pos,
                                         float motor1_feedback_vel,
                                         float motor1_target_pos,
                                         float motor1_command_vel,
                                         float motor2_actual_pos,
                                         float motor2_feedback_vel,
                                         float motor2_target_pos,
                                         float motor2_command_vel);
uint8_t vofa_debug_get_snapshot(vofa_debug_snapshot_t *out);

#endif /* __VOFA_DEBUG_H__ */
