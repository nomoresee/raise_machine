#ifndef __CRANE_ROUTE_H__
#define __CRANE_ROUTE_H__

#include "main.h"
#include <stdint.h>

#define CRANE_ROUTE_SLOT_COUNT             8U
#define CRANE_ROUTE_PICK_COUNT             3U
#define CRANE_ROUTE_PLACE_COUNT            5U

/* 调试模式开关：一次只建议打开一个；正常比赛全部保持 0。 */
#define CRANE_ROUTE_BEAM_ONLY              0U
#define CRANE_ROUTE_NO_CHASSIS              0U
#define CRANE_ROUTE_LIFT_ONLY              0U
#define CRANE_ROUTE_BEAM_PATH_ONLY_DEFAULT 0U
#define CRANE_ROUTE_CHASSIS_ONLY           0U

/* 独立 Z 轴步进测试仍沿用当前 2325 电机及 lift_ctrl。 */
#ifndef CRANE_ROUTE_Z_STEP_TEST_ENABLE
#define CRANE_ROUTE_Z_STEP_TEST_ENABLE     0U
#endif

#define CRANE_ROUTE_LIFT_SAFE_POS           8.0f
#define CRANE_ROUTE_Z_TEST_DROP_POS         4.2f
#define CRANE_ROUTE_Z_TEST_PICK_POS         2.5f
 
#if ((CRANE_ROUTE_LIFT_ONLY != 0U) && (CRANE_ROUTE_Z_STEP_TEST_ENABLE != 0U))
#error "CRANE_ROUTE_LIFT_ONLY and CRANE_ROUTE_Z_STEP_TEST_ENABLE cannot both be enabled"
#endif

/*
 * 两个斗门当前还没有实际 PWM 通道。第一版允许用模块内的模拟到位完成空载
 * 路线验证；装豆实测前必须将其改为 0，并配置两个斗门模块的真实硬件输出。
 */
#define CRANE_ROUTE_ALLOW_GATE_SIMULATION  0U

typedef struct
{
    float chassis_pos;
    float beam_pos;
    float lift_work_pos;
    float lift_safe_pos;
} crane_slot_pose_t;

typedef enum
{
    CRANE_CARRIER_UPPER_HOPPER = 0,
    CRANE_CARRIER_CLAW,
    CRANE_CARRIER_LOWER_HOPPER
} crane_carrier_e;

typedef enum
{
    CRANE_ROUTE_FAULT_NONE = 0,
    CRANE_ROUTE_FAULT_PLAN,
    CRANE_ROUTE_FAULT_CONFIG,
    CRANE_ROUTE_FAULT_MOTION_TIMEOUT,
    CRANE_ROUTE_FAULT_SERVO_TIMEOUT,
    CRANE_ROUTE_FAULT_GATE_UNAVAILABLE
} crane_route_fault_e;

typedef enum
{
    CRANE_ROUTE_IDLE = 0,
    CRANE_ROUTE_BUILD_PLAN,
    /* 首趟取 1/2：先抬 Z 到安全旋转高度，再允许旋转爪和横梁 Y 动作。 */
    CRANE_ROUTE_WAIT_FIRST_PICK_LIFT_SAFE,
    CRANE_ROUTE_WAIT_FIRST_PICK_ROTATE,
    CRANE_ROUTE_WAIT_PICK3_LOWER_ENTRY,
    CRANE_ROUTE_HOLD_X_WAIT_PICK3_LOWER_Y,
    CRANE_ROUTE_WAIT_NEXT_PICK_Y_NEAR,
    CRANE_ROUTE_WAIT_PICK_APPROACH,
    CRANE_ROUTE_PICK_DESCEND,
    CRANE_ROUTE_WAIT_PICK_DESCEND,
    CRANE_ROUTE_PICK_CLOSE,
    CRANE_ROUTE_WAIT_PICK_CLOSE,
    CRANE_ROUTE_PICK_LIFT_SAFE,
    CRANE_ROUTE_WAIT_PICK_LIFT_SAFE,
    CRANE_ROUTE_WAIT_ROTATE_TO_LOAD,
    CRANE_ROUTE_WAIT_HOPPER_OPEN_DELAY,
    CRANE_ROUTE_MOVE_CLAW_TO_HOPPER,
    CRANE_ROUTE_WAIT_CLAW_AT_HOPPER,
    CRANE_ROUTE_RELEASE_TO_HOPPER,
    CRANE_ROUTE_WAIT_RELEASE_TO_HOPPER,
    CRANE_ROUTE_CLOSE_AND_ROTATE_BACK,
    CRANE_ROUTE_WAIT_CLOSE_AND_ROTATE_BACK,
    CRANE_ROUTE_WAIT_FINAL_ROTATE_PLACE,
    /* 最后一颗从 1/2 号取物时，先让 X 回到 3 号位再释放三套 Y 上避障。 */
    CRANE_ROUTE_WAIT_FINAL_PICK_X_AT_SLOT3,
    CRANE_ROUTE_MOVE_ALL_TO_UPPER_SAFE,
    CRANE_ROUTE_WAIT_ALL_UPPER_SAFE,
    CRANE_ROUTE_START_BYPASS_X,
    CRANE_ROUTE_WAIT_OBSTACLE1_TRIGGER,
    CRANE_ROUTE_WAIT_OBSTACLE2_CHECK,
    CRANE_ROUTE_HOLD_X_WAIT_LOWER_Y,
    CRANE_ROUTE_WAIT_OBSTACLE2_EXIT,
    CRANE_ROUTE_MOVE_ALL_TO_PLACE,
    CRANE_ROUTE_WAIT_EXTREME_STATION,
    CRANE_ROUTE_WAIT_EXTREME_RELEASE,
    CRANE_ROUTE_MOVE_TO_REMAINING_STATION,
    CRANE_ROUTE_WAIT_REMAINING_COORDS,
    CRANE_ROUTE_PLACE_DESCEND,
    CRANE_ROUTE_WAIT_PLACE_DESCEND,
    CRANE_ROUTE_WAIT_REMAINING_RELEASE,
    CRANE_ROUTE_LIFT_RETURN_CLEAR,
    CRANE_ROUTE_WAIT_LIFT_RETURN_CLEAR,
    CRANE_ROUTE_MOVE_ALL_RETURN_UPPER,
    CRANE_ROUTE_WAIT_ALL_RETURN_UPPER,
    CRANE_ROUTE_RETURN_X_CENTER,
    CRANE_ROUTE_WAIT_X_CENTER,
    CRANE_ROUTE_FINISHED,
    CRANE_ROUTE_FAULT
} crane_route_state_e;

void crane_route_init(void);
uint8_t crane_route_set_draw_result(const uint8_t pick_goods[3],
                                    const uint8_t place_boxes[5]);
void crane_route_start(void);
void crane_route_stop(void);
void crane_route_process(void);

/* 保留原调试入口；新正式路线不依赖旧 xy_route。 */
void crane_route_set_beam_path_only(uint8_t enable);
uint8_t crane_route_get_beam_path_only(void);

void crane_route_set_slot_pose(uint8_t slot, float chassis_pos, float beam_pos);
void crane_route_set_slot_lift_pos(uint8_t slot,
                                   float lift_work_pos,
                                   float lift_safe_pos);
uint8_t crane_route_set_carrier_place_y(crane_carrier_e carrier,
                                        uint8_t slot,
                                        float y_pos);
float crane_route_get_slot_chassis_pos(uint8_t slot);

void crane_route_get_current_target(float *x, float *y);
void crane_route_get_current_pose_target(float *x, float *y, float *z);
void crane_route_get_all_y_targets(float *claw_y,
                                   float *upper_hopper_y,
                                   float *lower_hopper_y);

/* 返回路径文件中唯一标定的三套 Y 上侧避障坐标。 */
void crane_route_get_upper_safe_y(float *claw_y,
                                  float *upper_hopper_y,
                                  float *lower_hopper_y);

/* 返回路径文件中唯一标定的三套 Y 下侧避障坐标。 */
void crane_route_get_lower_transition_y(float *claw_y,
                                        float *upper_hopper_y,
                                        float *lower_hopper_y);

crane_route_state_e crane_route_get_state(void);
crane_route_fault_e crane_route_get_fault(void);
uint8_t crane_route_is_finished(void);
uint8_t crane_route_get_current_slot(void);

#endif /* __CRANE_ROUTE_H__ */
