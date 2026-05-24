#ifndef __XY_ROUTE_H__
#define __XY_ROUTE_H__

#include "main.h"

typedef enum
{
    XY_ROUTE_DIRECT = 0, /* 直线：X/Y 同时去目标点，不做绕障约束 */
    XY_ROUTE_UP,         /* 上绕：入口 Y 为 +20，出口 Y 为 -20 */
    XY_ROUTE_DOWN,       /* 下绕：入口 Y 为 -20，出口 Y 为 +20 */
    XY_ROUTE_CENTER      /* 回中：当前按直线回 0 号中心点处理 */
} xy_route_type_e;

typedef enum
{
    XY_RELEASE_AFTER_EXIT = 0, /* 完整绕桩：到出口后，Y 再去最终目标 */
    XY_RELEASE_AFTER_ENTRY     /* 对角线优化：入口通过后，Y 直接去最终目标 */
} xy_release_mode_e;

typedef enum
{
    XY_ROUTE_IDLE = 0,
    XY_ROUTE_DIRECT_RUN,          /* 直线运行，等待 X/Y 都到位 */
    XY_ROUTE_BYPASS_TO_ENTRY,     /* X 跑向入口，Y 应已提前去入口 Y */
    XY_ROUTE_BYPASS_WAIT_ENTRY_Y, /* X 到入口附近但 Y 未到入口，暂停 X 等 Y */
    XY_ROUTE_BYPASS_TO_EXIT,      /* 完整绕桩：X 跑向出口，Y 去出口 Y */
    XY_ROUTE_BYPASS_WAIT_EXIT_Y,  /* X 到出口附近但 Y 未到出口，暂停 X 等 Y */
    XY_ROUTE_BYPASS_TO_TARGET,    /* 绕桩完成后，X/Y 去最终目标 */
    XY_ROUTE_DONE
} xy_route_state_e;

void xy_route_init(void);
void xy_route_prepare_y(xy_route_type_e route_type, float final_y);
void xy_route_start(float target_x,
                    float target_y,
                    xy_route_type_e route_type,
                    xy_release_mode_e release_mode);
void xy_route_start_y_only(float target_y,
                           xy_route_type_e route_type,
                           xy_release_mode_e release_mode);
void xy_route_stop(void);
void xy_route_process(void);
uint8_t xy_route_is_busy(void);
uint8_t xy_route_is_done(void);
xy_route_state_e xy_route_get_state(void);
float xy_route_get_target_x(void);
float xy_route_get_target_y(void);

#endif
