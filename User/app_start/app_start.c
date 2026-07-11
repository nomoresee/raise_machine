#include "app_start.h"

#include "crane_route.h"
#include "gpio.h"
#include "pi_uart_rx.h"
#include "tim.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

/*
 * 启动流程：
 * 1. 等待 START 按键。
 * 2. 整机模式下向树莓派发送取放结果请求；底盘单测模式下直接启动固定路线。
 * 3. 校验树莓派数据，写入 crane_route 后延时启动。
 * 4. 路线完成后回到等待按键状态。
 */
#define APP_START_KEY_DEBOUNCE_MS 250U
#define APP_START_BUZZER_MS       1500U
#define APP_START_BUZZER_PULSE    1000U  /* TIM12 ARR=1999，对应约 50% 占空比 */
#if (CRANE_ROUTE_CHASSIS_ONLY == 0U)
#define APP_START_DELAY_MS        8000U  /* 收到视觉结果后等待 8s 再启动 */
#define APP_START_PI_LINE_SIZE    128U
#define APP_PICK_COUNT            3U
#define APP_PLACE_COUNT           5U
#endif

typedef enum
{
    APP_START_WAIT_KEY = 0,        /* 等待启动按键 */
    APP_START_WAIT_PI_PACKET,      /* 等待树莓派返回取放结果 */
    APP_START_DELAY_BEFORE_RUN,    /* 给现场留出的启动缓冲时间 */
    APP_START_RUNNING              /* crane_route 正在执行 */
} app_start_state_t;

static app_start_state_t app_start_state;
static uint8_t app_start_key_down;
static uint32_t app_start_key_tick;
static uint8_t app_start_buzzer_active;
static uint32_t app_start_buzzer_tick;
#if (CRANE_ROUTE_CHASSIS_ONLY == 0U)
static uint32_t app_start_delay_tick;
static char app_start_pi_line[APP_START_PI_LINE_SIZE];
#endif

static uint8_t app_start_key_pressed(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t down = (HAL_GPIO_ReadPin(START_BTN_GPIO_Port, START_BTN_Pin) == GPIO_PIN_RESET) ? 1U : 0U;

    if ((now - app_start_key_tick) < APP_START_KEY_DEBOUNCE_MS)
    {
        return 0U;
    }

    if ((down != 0U) && (app_start_key_down == 0U))
    {
        app_start_key_tick = now;
        app_start_key_down = down;
        return 1U;
    }

    app_start_key_down = down;
    return 0U;
}

#if (CRANE_ROUTE_CHASSIS_ONLY == 0U)
static void app_start_send_request(void)
{
    /* G = Get result，请求树莓派发送 START;PICK=...;PLACE=...;END 数据帧。 */
    uint8_t cmd = 'G';
    (void)HAL_UART_Transmit(&huart7, &cmd, 1U, 100U);
}

/* 非阻塞蜂鸣：由 app_start_process() 周期调用，到时自动关闭。 */
static void app_start_buzzer_start(void)
{
    (void)HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, APP_START_BUZZER_PULSE);
    app_start_buzzer_tick = HAL_GetTick();
    app_start_buzzer_active = 1U;
}

static void app_start_buzzer_process(void)
{
    if ((app_start_buzzer_active != 0U) &&
        ((HAL_GetTick() - app_start_buzzer_tick) >= APP_START_BUZZER_MS))
    {
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0U);
        (void)HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
        app_start_buzzer_active = 0U;
    }
}

static uint8_t app_start_check_unique_range(const uint8_t *values,
                                            uint8_t count,
                                            uint8_t min_value,
                                            uint8_t max_value)
{
    for (uint8_t i = 0U; i < count; i++)
    {
        if ((values[i] < min_value) || (values[i] > max_value))
        {
            return 0U;
        }

        for (uint8_t j = (uint8_t)(i + 1U); j < count; j++)
        {
            if (values[i] == values[j])
            {
                return 0U;
            }
        }
    }

    return 1U;
}

static uint8_t app_start_parse_pi_packet(const char *line,
                                         uint8_t pick_goods[APP_PICK_COUNT],
                                         uint8_t place_boxes[APP_PLACE_COUNT])
{
    unsigned int pick0;
    unsigned int pick1;
    unsigned int pick2;
    unsigned int place0;
    unsigned int place1;
    unsigned int place2;
    unsigned int place3;
    unsigned int place4;
    char tail;
    int matched;

    if (line == NULL)
    {
        return 0U;
    }

    matched = sscanf(line,
                     "START;PICK=%u,%u,%u;PLACE=%u,%u,%u,%u,%u;END%c",
                     &pick0,
                     &pick1,
                     &pick2,
                     &place0,
                     &place1,
                     &place2,
                     &place3,
                     &place4,
                     &tail);
    if (matched != 8)
    {
        return 0U;
    }

    /* 树莓派 PICK 顺序为实物 2,3,1；内部统一存为取货槽 1,2,3。 */
    pick_goods[0] = (uint8_t)pick2;
    pick_goods[1] = (uint8_t)pick0;
    pick_goods[2] = (uint8_t)pick1;
    /* PLACE 顺序已经是槽位 4,5,6,7,8，可直接保存。 */
    place_boxes[0] = (uint8_t)place0;
    place_boxes[1] = (uint8_t)place1;
    place_boxes[2] = (uint8_t)place2;
    place_boxes[3] = (uint8_t)place3;
    place_boxes[4] = (uint8_t)place4;

    if (app_start_check_unique_range(pick_goods, APP_PICK_COUNT, 6U, 8U) == 0U)
    {
        return 0U;
    }

    if (app_start_check_unique_range(place_boxes, APP_PLACE_COUNT, 1U, 5U) == 0U)
    {
        return 0U;
    }

    return 1U;
}
#endif

void app_start_init(void)
{
    app_start_state = APP_START_WAIT_KEY;
    app_start_key_down = 0U;
    app_start_key_tick = 0U;
    app_start_buzzer_active = 0U;
    app_start_buzzer_tick = 0U;
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0U);
#if (CRANE_ROUTE_CHASSIS_ONLY == 0U)
    app_start_delay_tick = 0U;
    (void)memset(app_start_pi_line, 0, sizeof(app_start_pi_line));
#endif
}

void app_start_process(void)
{
#if (CRANE_ROUTE_CHASSIS_ONLY == 0U)
    uint8_t pick_goods[APP_PICK_COUNT];
    uint8_t place_boxes[APP_PLACE_COUNT];
#endif

    app_start_buzzer_process();

    switch (app_start_state)
    {
        case APP_START_WAIT_KEY:
            if (app_start_key_pressed() != 0U)
            {
#if (CRANE_ROUTE_CHASSIS_ONLY != 0U)
                crane_route_start();
                app_start_state = APP_START_RUNNING;
#else
                app_start_send_request();
                app_start_state = APP_START_WAIT_PI_PACKET;
#endif
            }
            break;

#if (CRANE_ROUTE_CHASSIS_ONLY == 0U)
        case APP_START_WAIT_PI_PACKET:
            if (pi_uart_rx_take_new_line(app_start_pi_line, sizeof(app_start_pi_line)) != 0U)
            {
                if (app_start_parse_pi_packet(app_start_pi_line, pick_goods, place_boxes) != 0U)
                {
                    if (crane_route_set_draw_result(pick_goods, place_boxes) != 0U)
                    {
                        /* 视觉数据已完整解析、范围校验并成功写入路线。 */
                        app_start_buzzer_start();
                        app_start_delay_tick = HAL_GetTick();
                        app_start_state = APP_START_DELAY_BEFORE_RUN;
                    }
                }
            }
            break;

        case APP_START_DELAY_BEFORE_RUN:
            if ((HAL_GetTick() - app_start_delay_tick) >= APP_START_DELAY_MS)
            {
                crane_route_start();
                app_start_state = APP_START_RUNNING;
            }
            break;
#endif

        case APP_START_RUNNING:
            crane_route_process();
            if (crane_route_is_finished() != 0U)
            {
                app_start_state = APP_START_WAIT_KEY;
            }
            break;

        default:
            app_start_state = APP_START_WAIT_KEY;
            break;
    }
}
