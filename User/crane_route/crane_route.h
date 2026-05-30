#ifndef __CRANE_ROUTE_H__
#define __CRANE_ROUTE_H__

#include "main.h"

#define CRANE_ROUTE_SLOT_COUNT  8U

/* 0：未装舵机，抓取/放置仅 4 号到位后延时再抬升；1：启用舵机夹爪 */
#define CRANE_ROUTE_USE_SERVO       1U
#define CRANE_ROUTE_PICK_DWELL_MS   1000U  /* 无舵机时：升降工作位停留时间（ms） */

/* 1：仅横梁按路线槽位 beam_pos 逐点走位，底盘/升降/夹爪不动作；0：整机联动 */
#define CRANE_ROUTE_BEAM_ONLY       0U

/* 1: keep chassis still, but run planned Y path with lift and servo3 coordinated. */
#define CRANE_ROUTE_NO_CHASSIS      0U

/* 1: run planned route on beam(Y) only; chassis, lift and gripper stay still. */
#define CRANE_ROUTE_BEAM_PATH_ONLY_DEFAULT  0U

/* 1: chassis-only debug, only run pos_pid_sync along chassis_pos route */
#define CRANE_ROUTE_CHASSIS_ONLY    0U

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
    CRANE_ROUTE_BUILD_PLAN,
    CRANE_ROUTE_MOVE_TO_PICK,
    CRANE_ROUTE_WAIT_PICK_XY,
    CRANE_ROUTE_LIFT_DOWN_PICK,
    CRANE_ROUTE_WAIT_LIFT_DOWN_PICK,
    CRANE_ROUTE_GRIPPER_PICK,
    CRANE_ROUTE_GRIPPER_PICK_HOLD,
    CRANE_ROUTE_LIFT_UP_AFTER_PICK,
    CRANE_ROUTE_WAIT_LIFT_UP_AFTER_PICK,
    CRANE_ROUTE_MOVE_TO_PLACE,
    CRANE_ROUTE_WAIT_PLACE_XY,
    CRANE_ROUTE_LIFT_DOWN_PLACE,
    CRANE_ROUTE_WAIT_LIFT_DOWN_PLACE,
    CRANE_ROUTE_GRIPPER_PLACE,
    CRANE_ROUTE_GRIPPER_PLACE_HOLD,
    CRANE_ROUTE_LIFT_UP_AFTER_PLACE,
    CRANE_ROUTE_WAIT_LIFT_UP_AFTER_PLACE,
    CRANE_ROUTE_MOVE_EXTREME_SAFE_Y,
    CRANE_ROUTE_WAIT_EXTREME_SAFE_Y,
    CRANE_ROUTE_SERVO_RETURN_PICK,
    CRANE_ROUTE_WAIT_SERVO_RETURN_PICK,
    CRANE_ROUTE_RETURN_AFTER_PLACE,
    CRANE_ROUTE_WAIT_RETURN_XY,
    CRANE_ROUTE_LEG_DWELL,
    CRANE_ROUTE_FINISHED
} crane_route_state_e;

void crane_route_init(void);
void crane_route_start(void);
void crane_route_stop(void);
void crane_route_process(void);
void crane_route_set_beam_path_only(uint8_t enable);
uint8_t crane_route_get_beam_path_only(void);
void crane_route_set_slot_pose(uint8_t slot, float chassis_pos, float beam_pos);
void crane_route_set_slot_lift_pos(uint8_t slot, float lift_work_pos, float lift_safe_pos);
void crane_route_get_current_target(float *x, float *y);
void crane_route_get_current_pose_target(float *x, float *y, float *z);
crane_route_state_e crane_route_get_state(void);
uint8_t crane_route_is_finished(void);
uint8_t crane_route_get_current_slot(void);

#endif
