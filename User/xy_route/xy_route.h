#ifndef __XY_ROUTE_H__
#define __XY_ROUTE_H__

#include "main.h"

typedef enum
{
    XY_ROUTE_DIRECT = 0, /* 直线：X/Y 同时去目标点，不做绕障约束 */
    XY_ROUTE_UP,         /* 上绕：Y 先去上绕安全线，X 通过障碍区前检查 Y 是否到位 */
    XY_ROUTE_DOWN,       /* 下绕：Y 先去下绕安全线，X 通过障碍区前检查 Y 是否到位 */
    XY_ROUTE_CENTER      /* 回中：当前按直线回 0 号中心点处理 */
} xy_route_type_e;

typedef enum
{
    XY_RELEASE_AFTER_EXIT = 0, /* 完整通道：X 到出口点后，Y 再去最终目标 */
    XY_RELEASE_AFTER_ENTRY     /* 对角线优化：X 过入口且 Y 安全后，Y 立即去最终目标 */
} xy_release_mode_e;

typedef enum
{
    XY_ROUTE_IDLE = 0,
    XY_ROUTE_DIRECT_RUN,          /* 直线运行，等待 X/Y 都到位 */
    XY_ROUTE_BYPASS_PREPARE,      /* 绕行准备：X 放开跑，Y 抢先去安全线 */
    XY_ROUTE_BYPASS_WAIT_Y,       /* X 到障碍入口附近但 Y 未到位，暂停 X 等 Y */
    XY_ROUTE_BYPASS_CROSS,        /* Y 已到安全线，X 继续穿越障碍区 */
    XY_ROUTE_BYPASS_ALIGN_TARGET, /* X 已越过障碍区，Y 去最终目标点 */
    XY_ROUTE_DONE
} xy_route_state_e;

void xy_route_init(void);
void xy_route_prepare_y(xy_route_type_e route_type, float final_y);
void xy_route_start(float target_x,
                    float target_y,
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
