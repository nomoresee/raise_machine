#ifndef __SERVO_H__
#define __SERVO_H__

#include "tim.h"
#include <stdint.h>

#define SERVO_MIN_ANGLE_DEG      0.0f
#define SERVO1_MAX_ANGLE_DEG     270.0f
#define SERVO2_MAX_ANGLE_DEG     180.0f
#define SERVO_MIN_PULSE_US       500U
#define SERVO_MID_PULSE_US       1500U
#define SERVO_MAX_PULSE_US       2500U
#define SERVO_SYNC_STEP_DELAY_MS 20U
#define SERVO1_SYNC_STEP_DEG     2.0f
#define SERVO2_SYNC_STEP_DEG     2.0f

float servo_clamp_angle(float angle_deg);
uint32_t servo_angle_to_compare(float angle_deg);
float servo_compare_to_angle(uint32_t compare);
uint32_t servo_pulse_us_to_compare(uint32_t pulse_us);
uint32_t servo_compare_to_pulse_us(uint32_t compare);
float servo_write_angle(TIM_HandleTypeDef *htim, uint32_t channel, float angle_deg);
void servo_set_angle(TIM_HandleTypeDef *htim, uint32_t channel, float angle_deg);
void servo_set_pulse_us(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t pulse_us);
void servo1_set_angle(float angle_deg);
void servo1_set_pulse_us(uint32_t pulse_us);
void servo2_set_angle(float angle_deg);
void servo2_set_pulse_us(uint32_t pulse_us);
void servo_sync_move(float servo1_target_deg, float servo2_target_deg);
void servo_sync_move_custom(float servo1_target_deg, float servo2_target_deg,
                            float servo1_step_deg, float servo2_step_deg,
                            uint32_t step_delay_ms);

#endif
