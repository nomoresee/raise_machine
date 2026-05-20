#ifndef __CRANE_ROUTE_H__
#define __CRANE_ROUTE_H__

#include "main.h"

#define CRANE_ROUTE_SLOT_COUNT  8U

/* 0：未装舵机，抓取/放置仅 4 号到位后延时再抬升；1：启用舵机夹爪 */
#define CRANE_ROUTE_USE_SERVO       0U
#define CRANE_ROUTE_PICK_DWELL_MS   1000U  /* 无舵机时：升降工作位停留时间（ms） */

/* 1：仅横梁按路线槽位 beam_pos 逐点走位，底盘/升降/夹爪不动作；0：整机联动 */
#define CRANE_ROUTE_BEAM_ONLY       1U

typedef struct
{
    float chassis_pos;
    float beam_pos;
    float lift_work_pos;
    float lift_safe_pos;
} crane_slot_pose_t;

typedef enum
{
    CRANE_ROUTE_ACTION_PICK = 0,
    CRANE_ROUTE_ACTION_PLACE
} crane_route_action_e;

typedef enum
{
    CRANE_ROUTE_IDLE = 0,
    CRANE_ROUTE_LOAD_STEP,
    CRANE_ROUTE_WAIT_XY_ARRIVE,
    CRANE_ROUTE_LIFT_DOWN,
    CRANE_ROUTE_WAIT_LIFT_DOWN_ARRIVE,
    CRANE_ROUTE_GRIPPER_ACT,
    CRANE_ROUTE_GRIPPER_HOLD,
    CRANE_ROUTE_LIFT_UP_SAFE,
    CRANE_ROUTE_WAIT_LIFT_UP_ARRIVE,
    CRANE_ROUTE_STEP_DWELL,
    CRANE_ROUTE_FINISHED
} crane_route_state_e;

void crane_route_init(void);
void crane_route_start(void);
void crane_route_stop(void);
void crane_route_process(void);
void crane_route_set_slot_pose(uint8_t slot, float chassis_pos, float beam_pos);
void crane_route_set_slot_lift_pos(uint8_t slot, float lift_work_pos, float lift_safe_pos);
void crane_route_get_current_target(float *x, float *y);
void crane_route_get_current_pose_target(float *x, float *y, float *z);
crane_route_state_e crane_route_get_state(void);
uint8_t crane_route_is_finished(void);
uint8_t crane_route_get_current_slot(void);

#endif
