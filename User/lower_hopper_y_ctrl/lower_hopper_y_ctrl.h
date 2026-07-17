#ifndef __LOWER_HOPPER_Y_CTRL_H__
#define __LOWER_HOPPER_Y_CTRL_H__

#include "bsp_fdcan.h"
#include "dm_motor_drv.h"

void lower_hopper_y_ctrl_init(hcan_t *hcan, motor_num motor_index);
void lower_hopper_y_ctrl_set_target(float target_pos);
void lower_hopper_y_ctrl_set_max_vel(float max_vel);
void lower_hopper_y_ctrl_start(void);
void lower_hopper_y_ctrl_stop(void);
void lower_hopper_y_ctrl_process(void);
uint8_t lower_hopper_y_ctrl_is_busy(void);
uint8_t lower_hopper_y_ctrl_is_arrived(void);
float lower_hopper_y_ctrl_get_current_pos(void);
float lower_hopper_y_ctrl_get_target_pos(void);

#endif /* __LOWER_HOPPER_Y_CTRL_H__ */
