#ifndef __XY_ROUTE_H__
#define __XY_ROUTE_H__

#include "main.h"

/* X 轴两处避障物的标定坐标：取货侧为第一个障碍物，放置侧为第二个障碍物。 */
#define XY_ROUTE_X_ENTRY_PICK_SIDE       -750.0f
#define XY_ROUTE_X_ENTRY_PLACE_SIDE       500.0f

typedef enum
{
    XY_ROUTE_DIRECT = 0,          /* X/Y 直接去目标点 */
    XY_ROUTE_UP,                  /* 上绕障碍 */
    XY_ROUTE_DOWN,                /* 下绕障碍 */
    XY_ROUTE_CENTER,              /* 预留中心路线 */
    XY_ROUTE_CENTER_UP_EXIT,      /* 从中间回中心时走上出口 */
    XY_ROUTE_CENTER_DOWN_EXIT     /* 从中间回中心时走下出口 */
} xy_route_type_e;

typedef enum
{
    XY_RELEASE_AFTER_EXIT = 0,    /* 到绕行出口后再放开 Y 到最终目标 */
    XY_RELEASE_AFTER_ENTRY        /* 过绕行入口后立即放开 Y 到最终目标 */
} xy_release_mode_e;

typedef enum
{
    XY_ROUTE_IDLE = 0,            /* 空闲 */
    XY_ROUTE_DIRECT_RUN,          /* 直线路线执行中 */
    XY_ROUTE_BYPASS_TO_ENTRY,     /* 绕障：X 前往入口 */
    XY_ROUTE_BYPASS_WAIT_ENTRY_Y, /* 绕障：等待 Y 到入口 */
    XY_ROUTE_BYPASS_TO_EXIT,      /* 绕障：X/Y 前往出口 */
    XY_ROUTE_BYPASS_WAIT_EXIT_Y,  /* 绕障：等待 Y 到出口 */
    XY_ROUTE_BYPASS_TO_TARGET,    /* 绕障后前往最终目标 */
    XY_ROUTE_SERVO_WAIT_ENTRY_ROTATE, /* 入口处等待 servo3 第一阶段旋转 */
    XY_ROUTE_SERVO_WAIT_SLOT,     /* 极限槽位等待 servo3 安全旋转 */
    XY_ROUTE_CENTER_TO_EXIT,      /* 回中心路线：前往出口 */
    XY_ROUTE_CENTER_WAIT_EXIT_Y,  /* 回中心路线：等待 Y 到出口 */
    XY_ROUTE_CENTER_TO_TARGET,    /* 回中心路线：前往中心 */
    XY_ROUTE_DONE                 /* 本段路线完成 */
} xy_route_state_e;

void xy_route_init(void);
void xy_route_prepare_y(float target_x, xy_route_type_e route_type, float final_y);
void xy_route_start(float target_x,
                    float target_y,
                    xy_route_type_e route_type,
                    xy_release_mode_e release_mode);
/*
 * 从极限放置位返程：Y 已在 safe_y 时先放行 X。
 * 直线段立即回转 servo3；绕障段到第一个入口/出口门限才回转，
 * 回转完成前 Y 始终停在 safe_y。
 */
void xy_route_start_extreme_return(float target_x,
                                   float target_y,
                                   xy_route_type_e route_type,
                                   xy_release_mode_e release_mode,
                                   float safe_y,
                                   float servo3_angle_deg);
void xy_route_start_y_only(float target_y,
                           xy_route_type_e route_type,
                           xy_release_mode_e release_mode);
void xy_route_set_servo3_triggers(uint8_t entry_enable, float entry_angle_deg,
                                  uint8_t exit_enable, float exit_angle_deg);
void xy_route_set_servo3_entry_wait(uint8_t enable);
void xy_route_set_servo3_target_wait(uint8_t enable, float wait_y, float angle_deg);
void xy_route_stop(void);
void xy_route_process(void);
uint8_t xy_route_is_busy(void);
uint8_t xy_route_is_done(void);
xy_route_state_e xy_route_get_state(void);
float xy_route_get_target_x(void);
float xy_route_get_target_y(void);

#endif
