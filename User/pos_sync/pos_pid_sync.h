#ifndef __POS_PID_SYNC_H__
#define __POS_PID_SYNC_H__

#include "bsp_fdcan.h"
#include "dm_motor_drv.h"

void pos_pid_sync_init(hcan_t *hcan, motor_num motor1_index, motor_num motor2_index);
void pos_pid_sync_set_target(float target_pos);
void pos_pid_sync_start(void);
void pos_pid_sync_stop(void);
uint8_t pos_pid_sync_is_busy(void);
uint8_t pos_pid_sync_is_arrived(void);
float pos_pid_sync_get_current_pos(void);
void pos_pid_sync_set_max_vel(float max_vel);
void pos_pid_sync_target_state_machine(void);
void pos_pid_sync_process(void);

#endif /* __POS_PID_SYNC_H__ */
