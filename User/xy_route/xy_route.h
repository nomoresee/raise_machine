#ifndef __XY_ROUTE_H__
#define __XY_ROUTE_H__

#include "main.h"

typedef enum
{
    XY_ROUTE_DIRECT = 0,
    XY_ROUTE_UP,
    XY_ROUTE_DOWN,
    XY_ROUTE_CENTER,
    XY_ROUTE_CENTER_UP_EXIT,
    XY_ROUTE_CENTER_DOWN_EXIT
} xy_route_type_e;

typedef enum
{
    XY_RELEASE_AFTER_EXIT = 0,
    XY_RELEASE_AFTER_ENTRY
} xy_release_mode_e;

typedef enum
{
    XY_ROUTE_IDLE = 0,
    XY_ROUTE_DIRECT_RUN,
    XY_ROUTE_BYPASS_TO_ENTRY,
    XY_ROUTE_BYPASS_WAIT_ENTRY_Y,
    XY_ROUTE_BYPASS_TO_EXIT,
    XY_ROUTE_BYPASS_WAIT_EXIT_Y,
    XY_ROUTE_BYPASS_TO_TARGET,
    XY_ROUTE_SERVO_WAIT_SLOT,
    XY_ROUTE_CENTER_TO_EXIT,
    XY_ROUTE_CENTER_WAIT_EXIT_Y,
    XY_ROUTE_CENTER_TO_TARGET,
    XY_ROUTE_DONE
} xy_route_state_e;

void xy_route_init(void);
void xy_route_prepare_y(float target_x, xy_route_type_e route_type, float final_y);
void xy_route_start(float target_x,
                    float target_y,
                    xy_route_type_e route_type,
                    xy_release_mode_e release_mode);
void xy_route_start_y_only(float target_y,
                           xy_route_type_e route_type,
                           xy_release_mode_e release_mode);
void xy_route_set_servo3_triggers(uint8_t entry_enable, float entry_angle_deg,
                                  uint8_t exit_enable, float exit_angle_deg);
void xy_route_set_servo3_target_wait(uint8_t enable, float wait_y, float angle_deg);
void xy_route_stop(void);
void xy_route_process(void);
uint8_t xy_route_is_busy(void);
uint8_t xy_route_is_done(void);
xy_route_state_e xy_route_get_state(void);
float xy_route_get_target_x(void);
float xy_route_get_target_y(void);

#endif
