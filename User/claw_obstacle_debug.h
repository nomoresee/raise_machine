#ifndef CLAW_OBSTACLE_DEBUG_H
#define CLAW_OBSTACLE_DEBUG_H

/*
 * 爪子避障闭环调试开关。
 * 0：关闭，恢复正常比赛/其它调试模式。
 * 1：START 触发 Z 安全抬升、三套 Y 上/下避障、回原位、Z 回原位。
 *
 * 启用时应保持 CHASSIS_DEBUG_MODE 和 DEBUG_STEP_MODE_ENABLE 为 0。
 */
#ifndef CLAW_OBSTACLE_DEBUG_MODE
#define CLAW_OBSTACLE_DEBUG_MODE 0U
#endif

void claw_obstacle_debug_init(void);
void claw_obstacle_debug_process(void);

/* USART1 CSV：夹爪实际、夹爪目标、上斗实际、上斗目标、下斗实际、下斗目标。 */
void claw_obstacle_debug_telemetry_process(void);

#endif /* CLAW_OBSTACLE_DEBUG_H */
