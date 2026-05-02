#ifndef __CRANE_ROUTE_H__
#define __CRANE_ROUTE_H__

#include "main.h"

#define CRANE_ROUTE_SLOT_COUNT  8U

typedef struct
{
    float chassis_pos;
    float beam_pos;
} crane_slot_pose_t;

typedef enum
{
    CRANE_ROUTE_IDLE = 0,
    CRANE_ROUTE_LOAD_STEP,
    CRANE_ROUTE_WAIT_ARRIVE,
    CRANE_ROUTE_STEP_DWELL,
    CRANE_ROUTE_FINISHED
} crane_route_state_e;

void crane_route_init(void);
void crane_route_start(void);
void crane_route_stop(void);
void crane_route_process(void);
void crane_route_set_slot_pose(uint8_t slot, float chassis_pos, float beam_pos);
void crane_route_get_current_target(float *x, float *y);
crane_route_state_e crane_route_get_state(void);
uint8_t crane_route_is_finished(void);
uint8_t crane_route_get_current_slot(void);

#endif
