#ifndef __CRANE_ROUTE_H__
#define __CRANE_ROUTE_H__

#include "main.h"

#define CRANE_ROUTE_SLOT_COUNT  8U

/* 夹爪模式：0=无舵机夹爪，仅按 dwell 时间等待；1=启用舵机夹爪。 */
#define CRANE_ROUTE_USE_SERVO       1U
#define CRANE_ROUTE_PICK_DWELL_MS   1000U  /* 无舵机夹爪时，工作位停留时间（ms） */

/* 调试模式开关：一次只建议打开一个。 */
/* 1=只跑横梁 Y 轴槽位，底盘/升降/夹爪不动作。 */
#define CRANE_ROUTE_BEAM_ONLY       0U

/* 1=底盘 X 不动，只跑规划好的 Y/Z/servo3 协调动作。 */
#define CRANE_ROUTE_NO_CHASSIS      0U

/* 1=只调试升降 Z 轴，底盘/横梁/舵机保持不动。 */
#define CRANE_ROUTE_LIFT_ONLY       0U

/*
 * 1=按 START 按键执行独立的 Z 轴步进测试；0=正常全局协调路线。
 * 此模式不请求视觉结果、不启动 X/Y 路线，也不驱动 servo3。
 * 与 CRANE_ROUTE_LIFT_ONLY 二选一，测试完成后改回 0U 即可恢复全局协调模式。
 */
#ifndef CRANE_ROUTE_Z_STEP_TEST_ENABLE
#define CRANE_ROUTE_Z_STEP_TEST_ENABLE  0U
#endif

/* Z 轴步进测试参数（单位与 lift_ctrl 当前位置一致）。 */
#define CRANE_ROUTE_LIFT_SAFE_POS        21.0f
#define CRANE_ROUTE_Z_TEST_DROP_POS       4.2f  /* 第二次按键：放货测试高度 */
#define CRANE_ROUTE_Z_TEST_PICK_POS       2.5f  /* 第三次按键：取货测试高度 */

#if ((CRANE_ROUTE_LIFT_ONLY != 0U) && (CRANE_ROUTE_Z_STEP_TEST_ENABLE != 0U))
#error "CRANE_ROUTE_LIFT_ONLY and CRANE_ROUTE_Z_STEP_TEST_ENABLE cannot both be enabled"
#endif

/* 1=按完整路线规划只跑 Y 轴，底盘/升降/夹爪不动作。 */
#define CRANE_ROUTE_BEAM_PATH_ONLY_DEFAULT  0U
#define CRANE_ROUTE_BEAM_TEST_DWELL_MS       2000U /* 纯 Y 标定路线普通点停留时间 */
#define CRANE_ROUTE_BEAM_TEST_EXTREME_DWELL_MS 5000U /* 4/8 号位停留时间 */

#if ((CRANE_ROUTE_BEAM_PATH_ONLY_DEFAULT != 0U) && (CRANE_ROUTE_Z_STEP_TEST_ENABLE != 0U))
#error "CRANE_ROUTE_BEAM_PATH_ONLY_DEFAULT and CRANE_ROUTE_Z_STEP_TEST_ENABLE cannot both be enabled"
#endif

/* 1=底盘 X 单测，按 START 后不等视觉，直接跑固定底盘路线。 */
#define CRANE_ROUTE_CHASSIS_ONLY    0U

/* 单个槽位的三轴目标位置。slot 0 代表中心/回零点。 */
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
    CRANE_ROUTE_IDLE = 0,              /* 空闲，等待启动 */
    CRANE_ROUTE_BUILD_PLAN,            /* 根据抽签/默认结果生成三趟任务 */
    CRANE_ROUTE_MOVE_TO_PICK,          /* XY 移动到取货槽 */
    CRANE_ROUTE_WAIT_PICK_XY,
    CRANE_ROUTE_LIFT_DOWN_PICK,        /* Z 下降到取货高度 */
    CRANE_ROUTE_WAIT_LIFT_DOWN_PICK,
    CRANE_ROUTE_GRIPPER_PICK,          /* 夹爪抓取 */
    CRANE_ROUTE_GRIPPER_PICK_HOLD,
    CRANE_ROUTE_LIFT_UP_AFTER_PICK,    /* 抓取后抬升到安全高度 */
    CRANE_ROUTE_WAIT_LIFT_UP_AFTER_PICK,
    CRANE_ROUTE_MOVE_TO_PLACE,         /* XY 移动到放置槽 */
    CRANE_ROUTE_WAIT_PLACE_XY,
    CRANE_ROUTE_LIFT_DOWN_PLACE,       /* Z 下降到放置高度 */
    CRANE_ROUTE_WAIT_LIFT_DOWN_PLACE,
    CRANE_ROUTE_GRIPPER_PLACE,         /* 夹爪释放 */
    CRANE_ROUTE_GRIPPER_PLACE_HOLD,
    CRANE_ROUTE_LIFT_UP_AFTER_PLACE,   /* 放置后抬升 */
    CRANE_ROUTE_WAIT_LIFT_UP_AFTER_PLACE,
    CRANE_ROUTE_WAIT_EXTREME_RETURN_MID_ANGLE, /* 4/8 绕障返程前等待 180 度 */
    CRANE_ROUTE_MOVE_EXTREME_SAFE_Y,   /* 4/8 极限槽位的 Y 安全等待 */
    CRANE_ROUTE_WAIT_EXTREME_SAFE_Y,
    CRANE_ROUTE_SERVO_RETURN_PICK,     /* 延长杆回取货区方向 */
    CRANE_ROUTE_WAIT_SERVO_RETURN_PICK,
    CRANE_ROUTE_RETURN_AFTER_PLACE,    /* 空载返回下一取货槽或中心 */
    CRANE_ROUTE_WAIT_RETURN_XY,
    CRANE_ROUTE_BEAM_TEST_DWELL,       /* 纯 Y 标定路线到点观察等待 */
    CRANE_ROUTE_LEG_DWELL,             /* 每趟之间的短暂停顿 */
    CRANE_ROUTE_FINISHED
} crane_route_state_e;

void crane_route_init(void);
uint8_t crane_route_set_draw_result(const uint8_t pick_goods[3], const uint8_t place_boxes[5]);
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
