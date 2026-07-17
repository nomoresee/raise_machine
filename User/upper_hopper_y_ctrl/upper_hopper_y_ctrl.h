#ifndef __UPPER_HOPPER_Y_CTRL_H__
#define __UPPER_HOPPER_Y_CTRL_H__

#include "bsp_fdcan.h"
#include "dm_motor_drv.h"

void upper_hopper_y_ctrl_init(hcan_t *hcan, motor_num motor_index);
void upper_hopper_y_ctrl_set_target(float target_pos);
void upper_hopper_y_ctrl_set_max_vel(float max_vel);
void upper_hopper_y_ctrl_start(void);
void upper_hopper_y_ctrl_stop(void);
void upper_hopper_y_ctrl_process(void);
uint8_t upper_hopper_y_ctrl_is_busy(void);
uint8_t upper_hopper_y_ctrl_is_arrived(void);
float upper_hopper_y_ctrl_get_current_pos(void);
float upper_hopper_y_ctrl_get_target_pos(void);

#endif /* __UPPER_HOPPER_Y_CTRL_H__ */
