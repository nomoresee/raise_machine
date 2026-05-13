#include "lcd_app.h"

#include "headfile.h"
#include "pi_uart_rx.h"
#include "pos_sync/pos_pid_sync.h"
#include "stdio.h"
#include "string.h"

/* 由 pos_pid_sync 在其控制周期内更新（本文件只读） */
extern uint8_t pos_pid_sync_get_vofa_snapshot(pos_pid_sync_vofa_snapshot_t *out);

typedef enum
{
    LCD_PAGE_MAIN = 0,
    LCD_PAGE_PI = 1,
} lcd_page_t;

#define LCD_TEXT_COLS 32U
#define LCD_PI_TEXT_Y0 52U
#define LCD_PI_LINE_H 18U

static uint8_t lcd_app_inited = 0U;
static uint32_t lcd_app_tick = 0U;
static lcd_page_t s_page = LCD_PAGE_MAIN;
static lcd_page_t s_layout_page = (lcd_page_t)0xFFU;
static uint8_t s_btn_down = 0U;
static uint32_t s_btn_last_ms = 0U;
static char s_pi_last_drawn[256];

static void lcd_app_draw_static_main(void)
{
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
    LCD_DrawRectangle(0, 0, LCD_W - 1, LCD_H - 1, WHITE);
    LCD_DrawRectangle(6, 6, LCD_W - 7, LCD_H - 7, GBLUE);

    LCD_ShowString(16, 14, (uint8_t *)"DM CTRL - LIVE", YELLOW, BLACK, 16, 0);
    LCD_ShowString(16, 34, (uint8_t *)"VOFA SNAPSHOT", GREEN, BLACK, 16, 0);

    LCD_ShowString(16, 60, (uint8_t *)"TXY:", WHITE, BLACK, 16, 0);
    LCD_ShowString(16, 80, (uint8_t *)"M1P:", WHITE, BLACK, 16, 0);
    LCD_ShowString(16, 100, (uint8_t *)"M2P:", WHITE, BLACK, 16, 0);
    LCD_ShowString(16, 120, (uint8_t *)"ERR:", WHITE, BLACK, 16, 0);
    LCD_ShowString(16, 140, (uint8_t *)"M3P:", WHITE, BLACK, 16, 0);
    LCD_ShowString(16, 160, (uint8_t *)"V1V2:", WHITE, BLACK, 16, 0);
    LCD_ShowString(16, 180, (uint8_t *)"V3:", WHITE, BLACK, 16, 0);
    LCD_ShowString(16, 200, (uint8_t *)"M4P:", WHITE, BLACK, 16, 0);
    LCD_ShowString(16, 220, (uint8_t *)"V4:", WHITE, BLACK, 16, 0);
}

static void lcd_app_draw_static_pi(void)
{
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
    LCD_DrawRectangle(0, 0, LCD_W - 1, LCD_H - 1, WHITE);
    LCD_DrawRectangle(6, 6, LCD_W - 7, LCD_H - 7, MAGENTA);

    LCD_ShowString(16, 14, (uint8_t *)"PAGE 2: UART7 / Pi", YELLOW, BLACK, 16, 0);
    LCD_ShowString(16, 32, (uint8_t *)"115200 8N1 line (CRLF)", GREEN, BLACK, 16, 0);
}

static void lcd_app_apply_layout(void)
{
    if (s_layout_page == s_page)
    {
        return;
    }
    if (s_page == LCD_PAGE_MAIN)
    {
        lcd_app_draw_static_main();
    }
    else
    {
        lcd_app_draw_static_pi();
    }
    s_layout_page = s_page;
}

static void lcd_app_poll_page_button(void)
{
    uint32_t t = HAL_GetTick();
    if ((t - s_btn_last_ms) < 280U)
    {
        return;
    }

    GPIO_PinState st = HAL_GPIO_ReadPin(LCD_PAGE_BTN_GPIO_Port, LCD_PAGE_BTN_Pin);
    uint8_t down = (st == GPIO_PIN_RESET) ? 1U : 0U;

    if ((down != 0U) && (s_btn_down == 0U))
    {
        s_page = (s_page == LCD_PAGE_MAIN) ? LCD_PAGE_PI : LCD_PAGE_MAIN;
        s_layout_page = (lcd_page_t)0xFFU;
        s_btn_last_ms = t;
    }
    s_btn_down = down;
}

static void lcd_app_print_line(uint16_t x, uint16_t y, const char *text, uint16_t color)
{
    LCD_ShowString(x, y, (const uint8_t *)text, color, BLACK, 16, 0);
}

static void lcd_app_show_wrapped(uint16_t x0, uint16_t y0, const char *msg, uint16_t fg)
{
    size_t n = strlen(msg);
    size_t i = 0U;
    uint16_t y = y0;

    while ((i < n) && (y < (LCD_H - LCD_PI_LINE_H)))
    {
        char line[40];
        size_t take = n - i;
        if (take > LCD_TEXT_COLS)
        {
            take = LCD_TEXT_COLS;
        }
        (void)memcpy(line, msg + i, take);
        line[take] = '\0';
        lcd_app_print_line(x0, y, line, fg);
        i += take;
        y = (uint16_t)(y + LCD_PI_LINE_H);
    }
}

static void lcd_app_update_main_values(void)
{
    pos_pid_sync_vofa_snapshot_t s;
    char line[48];

    if (pos_pid_sync_get_vofa_snapshot(&s) == 0U || s.valid == 0U)
    {
        lcd_app_print_line(70, 60, "WAITING...                ", YELLOW);
        return;
    }

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

    (void)snprintf(line, sizeof(line), "%9.2f          ", (double)s.motor4_pos);
    lcd_app_print_line(70, 200, line, WHITE);

    (void)snprintf(line, sizeof(line), "%8.2f          ", (double)s.motor4_vel);
    lcd_app_print_line(70, 220, line, GREEN);
}

static void lcd_app_update_pi_text(void)
{
    char buf[256];

    /* 只有收到新的一行才刷新页面，避免“整块清屏+重画”造成肉眼可见频闪 */
    if (pi_uart_rx_take_new_line(buf, sizeof(buf)) == 0U)
    {
        return;
    }

    if (buf[0] == '\0')
    {
        (void)strncpy(buf, "(empty line)", sizeof(buf) - 1U);
        buf[sizeof(buf) - 1U] = '\0';
    }

    /* 如果内容没变，不重画 */
    if (strncmp(buf, s_pi_last_drawn, sizeof(s_pi_last_drawn)) == 0)
    {
        return;
    }
    (void)strncpy(s_pi_last_drawn, buf, sizeof(s_pi_last_drawn) - 1U);
    s_pi_last_drawn[sizeof(s_pi_last_drawn) - 1U] = '\0';

    /* 按行覆盖清理，避免大面积 Fill 带来的闪烁 */
    for (uint16_t y = LCD_PI_TEXT_Y0; y < (LCD_H - 8U); y = (uint16_t)(y + LCD_PI_LINE_H))
    {
        lcd_app_print_line(8, y, "                                ", BLACK);
    }
    lcd_app_show_wrapped(8, LCD_PI_TEXT_Y0, buf, CYAN);
}

void lcd_app_init(void)
{
    LCD_Init();
    HAL_Delay(30);
    s_page = LCD_PAGE_MAIN;
    s_layout_page = (lcd_page_t)0xFFU;
    (void)memset(s_pi_last_drawn, 0, sizeof(s_pi_last_drawn));
    lcd_app_apply_layout();
    lcd_app_inited = 1U;
    lcd_app_tick = HAL_GetTick();
}

void lcd_app_update(void)
{
    uint32_t now = HAL_GetTick();

    lcd_app_poll_page_button();

    if (lcd_app_inited == 0U)
    {
        return;
    }

    lcd_app_apply_layout();

    if ((now - lcd_app_tick) < 50U)
    {
        return;
    }
    lcd_app_tick = now;

    if (s_page == LCD_PAGE_MAIN)
    {
        lcd_app_update_main_values();
    }
    else
    {
        lcd_app_update_pi_text();
    }
}
