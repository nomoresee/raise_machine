#ifndef __LOWER_HOPPER_GATE_H__
#define __LOWER_HOPPER_GATE_H__

#include <stdint.h>

/*
 * 第一版安全占位配置：
 *   HW_READY=0          -> 不向任何定时器/PWM 通道写入。
 *   SIMULATION_ENABLE=1 -> 仅在软件中按时间估算开合到位，便于联调状态机。
 *
 * 实物舵机的 PWM 通道、开合角度未确定前，不得将 HW_READY 改为 1。
 */
#define LOWER_HOPPER_GATE_HW_READY             1U
#define LOWER_HOPPER_GATE_SIMULATION_ENABLE     0U

/* 待实车标定的占位参数。 */
#define LOWER_HOPPER_GATE_CLOSE_ANGLE_DEG       80.0f
#define LOWER_HOPPER_GATE_OPEN_ANGLE_DEG        180.0f
#define LOWER_HOPPER_GATE_SPEED_DEG_PER_SEC    180.0f
#define LOWER_HOPPER_GATE_SETTLE_MARGIN_MS     120U
#define LOWER_HOPPER_GATE_ANGLE_TOL_DEG          1.0f

void lower_hopper_gate_init(void);
void lower_hopper_gate_open(void);
void lower_hopper_gate_close(void);
void lower_hopper_gate_process(void);
uint8_t lower_hopper_gate_is_busy(void);
uint8_t lower_hopper_gate_is_open(void);
uint8_t lower_hopper_gate_is_closed(void);
uint8_t lower_hopper_gate_is_hw_ready(void);
uint8_t lower_hopper_gate_is_simulation_enabled(void);

#endif /* __LOWER_HOPPER_GATE_H__ */
