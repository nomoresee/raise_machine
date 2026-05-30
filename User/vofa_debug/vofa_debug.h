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
    uint8_t valid;
} vofa_debug_snapshot_t;

void vofa_debug_init(motor_num motor1_index,
                     motor_num motor2_index,
                     motor_num motor3_index,
                     motor_num motor4_index);
void vofa_debug_process(void);
uint8_t vofa_debug_get_snapshot(vofa_debug_snapshot_t *out);

#endif /* __VOFA_DEBUG_H__ */
