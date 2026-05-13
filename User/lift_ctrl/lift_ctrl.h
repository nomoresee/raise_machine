#ifndef __LIFT_CTRL_H__
#define __LIFT_CTRL_H__

#include "bsp_fdcan.h"
#include "dm_motor_drv.h"

void lift_ctrl_init(hcan_t *hcan, motor_num motor_index);
void lift_ctrl_set_target(float target_pos);
void lift_ctrl_set_max_vel(float max_vel);
void lift_ctrl_start(void);
void lift_ctrl_stop(void);
void lift_ctrl_process(void);
uint8_t lift_ctrl_is_busy(void);
uint8_t lift_ctrl_is_arrived(void);
float lift_ctrl_get_current_pos(void);

#endif
