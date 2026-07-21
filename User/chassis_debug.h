#ifndef CHASSIS_DEBUG_H
#define CHASSIS_DEBUG_H

#include "bsp_fdcan.h"

/*
 * 底盘按键调试模式。
 * 1：按 START 后，仅 Motor1、Motor2 按槽位序列运动；其余达妙电机被禁用。
 * 0：关闭底盘自动调试，恢复其它运行模式。
 */
#ifndef CHASSIS_DEBUG_MODE
#define CHASSIS_DEBUG_MODE 0U
#endif

void chassis_debug_init(hcan_t *hcan);
void chassis_debug_process(void);
void chassis_debug_vofa_process(void);

#endif /* CHASSIS_DEBUG_H */
