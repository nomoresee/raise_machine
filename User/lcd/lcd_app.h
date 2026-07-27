#ifndef __LCD_APP_H__
#define __LCD_APP_H__

#include "stdint.h"

/* 比赛运行期间关闭 LCD，避免阻塞式逐字节 SPI 刷新占用主循环。 */
#define LCD_APP_ENABLE  0U

/**
 * @brief LCD 应用层初始化（画框/标题/静态文字）
 */
void lcd_app_init(void);

/**
 * @brief 周期刷新 LCD 上的实时参数（建议在 while(1) 中调用）
 */
void lcd_app_update(void);

#endif
