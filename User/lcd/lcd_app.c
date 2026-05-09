#include "lcd_app.h"

#include "headfile.h"
#include "pos_sync/pos_pid_sync.h"
#include "stdio.h"

/* 由 pos_pid_sync 在其控制周期内更新（本文件只读） */
extern uint8_t pos_pid_sync_get_vofa_snapshot(pos_pid_sync_vofa_snapshot_t *out);

static uint8_t lcd_app_inited = 0U;
static uint32_t lcd_app_tick = 0U;

static void lcd_app_draw_static(void)
{
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
    LCD_DrawRectangle(0, 0, LCD_W - 1, LCD_H - 1, WHITE);
    LCD_DrawRectangle(6, 6, LCD_W - 7, LCD_H - 7, GBLUE);

    LCD_ShowString(16, 14, (uint8_t *)"DM CTRL - LIVE", YELLOW, BLACK, 16, 0);
    LCD_ShowString(16, 34, (uint8_t *)"VOFA SNAPSHOT", GREEN, BLACK, 16, 0);

    /* 标签（只画一次） */
    LCD_ShowString(16, 60, (uint8_t *)"TXY:", WHITE, BLACK, 16, 0);
    LCD_ShowString(16, 80, (uint8_t *)"M1P:", WHITE, BLACK, 16, 0);
    LCD_ShowString(16, 100, (uint8_t *)"M2P:", WHITE, BLACK, 16, 0);
    LCD_ShowString(16, 120, (uint8_t *)"ERR:", WHITE, BLACK, 16, 0);
    LCD_ShowString(16, 140, (uint8_t *)"M3P:", WHITE, BLACK, 16, 0);
    LCD_ShowString(16, 160, (uint8_t *)"V1V2:", WHITE, BLACK, 16, 0);
    LCD_ShowString(16, 180, (uint8_t *)"V3:", WHITE, BLACK, 16, 0);
}

void lcd_app_init(void)
{
    LCD_Init();
    lcd_app_draw_static();
    lcd_app_inited = 1U;
    lcd_app_tick = HAL_GetTick();
}

static void lcd_app_print_line(uint16_t x, uint16_t y, const char *text, uint16_t color)
{
    /* 统一用字符串覆盖刷新，背景色为黑，避免大面积 Fill 造成卡顿/闪烁 */
    LCD_ShowString(x, y, (const uint8_t *)text, color, BLACK, 16, 0);
}

void lcd_app_update(void)
{
    pos_pid_sync_vofa_snapshot_t s;
    uint32_t now = HAL_GetTick();
    char line[48];

    if (lcd_app_inited == 0U)
    {
        return;
    }

    /* 提升刷新到 20Hz：更顺滑，同时避免占用过高 */
    if ((now - lcd_app_tick) < 50U)
    {
        return;
    }
    lcd_app_tick = now;

    if (pos_pid_sync_get_vofa_snapshot(&s) == 0U || s.valid == 0U)
    {
        /* 未拿到数据：覆盖同一行即可 */
        lcd_app_print_line(70, 60, "WAITING...                ", YELLOW);
        return;
    }

    /* 用固定宽度行覆盖，防止残留字符 */
    (void)snprintf(line, sizeof(line), "%8.2f %8.2f   ", (double)s.target_x, (double)s.target_y);
    lcd_app_print_line(70, 60, line, CYAN);

    (void)snprintf(line, sizeof(line), "%9.2f          ", (double)s.motor1_pos);
    lcd_app_print_line(70, 80, line, WHITE);

    (void)snprintf(line, sizeof(line), "%9.2f          ", (double)s.motor2_pos);
    lcd_app_print_line(70, 100, line, WHITE);

    (void)snprintf(line, sizeof(line), "%9.2f          ", (double)s.pos_error);
    lcd_app_print_line(70, 120, line, YELLOW);

    (void)snprintf(line, sizeof(line), "%9.2f          ", (double)s.motor3_pos);
    lcd_app_print_line(70, 140, line, WHITE);

    (void)snprintf(line, sizeof(line), "%8.2f %8.2f   ", (double)s.motor1_vel, (double)s.motor2_vel);
    lcd_app_print_line(70, 160, line, GREEN);

    (void)snprintf(line, sizeof(line), "%8.2f          ", (double)s.motor3_vel);
    lcd_app_print_line(70, 180, line, GREEN);
}

