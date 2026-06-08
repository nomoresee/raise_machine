#ifndef __CLAW_H__
#define __CLAW_H__

#include "main.h"
#include <stdint.h>

#define CLAW_OPEN_ANGLE_DEG                  0.0f
#define CLAW_CLOSE_ANGLE_DEG                90.0f
#define CLAW_LEFT_OPEN_ANGLE_DEG             CLAW_OPEN_ANGLE_DEG
#define CLAW_LEFT_CLOSE_ANGLE_DEG            CLAW_CLOSE_ANGLE_DEG
#define CLAW_RIGHT_OPEN_ANGLE_DEG            CLAW_OPEN_ANGLE_DEG
#define CLAW_RIGHT_CLOSE_ANGLE_DEG           CLAW_CLOSE_ANGLE_DEG

#define CLAW_NORMAL_SPEED_DEG_PER_SEC      180.0f
#define CLAW_PICK_CLOSE_SPEED_DEG_PER_SEC   90.0f
#define CLAW_SETTLE_MARGIN_MS              120U
#define CLAW_ANGLE_TOL_DEG                   1.0f

typedef enum
{
    CLAW_ACTION_OPEN = 0,
    CLAW_ACTION_CLOSE_NORMAL,
    CLAW_ACTION_CLOSE_PICK
} claw_action_e;

void claw_init(void);
void claw_open(void);
void claw_close(void);
void claw_close_pick(void);
void claw_set_angle(float left_angle_deg, float right_angle_deg, float speed_deg_per_sec);
void claw_process(void);
uint8_t claw_is_busy(void);
uint8_t claw_is_open(void);
uint8_t claw_is_closed(void);

#endif /* __CLAW_H__ */
