#ifndef __POS_TEST_H__
#define __POS_TEST_H__

#include "bsp_fdcan.h"
#include "dm_motor_drv.h"

typedef enum
{
    POS_TEST_SYNC_ZERO = 0,
    POS_TEST_MOVE_POS,
    POS_TEST_HOLD_POS,
    POS_TEST_MOVE_NEG,
    POS_TEST_HOLD_NEG,
    POS_TEST_MOVE_ZERO,
    POS_TEST_HOLD_ZERO,
    POS_TEST_FAULT
} pos_test_state_e;

typedef enum
{
    POS_TEST_FAULT_NONE = 0,
    POS_TEST_FAULT_MOTOR_STATE,
    POS_TEST_FAULT_POS_SYNC,
    POS_TEST_FAULT_VEL_SYNC
} pos_test_fault_e;

typedef struct
{
    pos_test_state_e state;
    pos_test_fault_e fault;
    float ref_pos;
    float ref_vel;
    float motor1_pos;
    float motor2_pos;
    float motor1_vel;
    float motor2_vel;
    float pos_err;
    float vel_err;
} pos_test_status_t;

void pos_test_init(hcan_t *hcan, motor_num motor_index);
void pos_test_ctrl_send(void);
void pos_test_update(void);
void pos_test_print_100ms(void);
void pos_test_get_status(pos_test_status_t *status);
float pos_test_get_motor1_pos(void);
float pos_test_get_motor2_pos(void);
float pos_test_get_sync_pos_err(void);

#endif
