#ifndef APP_START_H
#define APP_START_H

#include "main.h"

/*
 * 0=正常比赛模式：按 START 后经 UART7 请求并等待树莓派视觉结果。
 * 1=手动视觉结果测试：不使用 UART7，按 START 后蜂鸣 0.5s，
 *   再按 app_start.c 中的固定数据包启动路线。
 */
#ifndef APP_START_MANUAL_DRAW_MODE
#define APP_START_MANUAL_DRAW_MODE  0U
#endif

void app_start_init(void);
void app_start_process(void);

#endif
