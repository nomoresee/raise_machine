#ifndef __BEAM_CTRL_H__
#define __BEAM_CTRL_H__

#include "bsp_fdcan.h"
#include "dm_motor_drv.h"

void beam_ctrl_init(hcan_t *hcan, motor_num motor_index);
void beam_ctrl_set_target(float target_pos);
void beam_ctrl_set_max_vel(float max_vel);
void beam_ctrl_start(void);
void beam_ctrl_stop(void);
void beam_ctrl_process(void);
uint8_t beam_ctrl_is_busy(void);
uint8_t beam_ctrl_is_arrived(void);
float beam_ctrl_get_current_pos(void);

#endif
