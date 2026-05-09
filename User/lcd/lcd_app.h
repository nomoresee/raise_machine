#ifndef __LCD_APP_H__
#define __LCD_APP_H__

#include "stdint.h"

/**
 * @brief LCD 应用层初始化（画框/标题/静态文字）
 */
void lcd_app_init(void);

/**
 * @brief 周期刷新 LCD 上的实时参数（建议在 while(1) 中调用）
 */
void lcd_app_update(void);

#endif

