#ifndef DEBUG_STEP_H
#define DEBUG_STEP_H

#include <stdint.h>

/*
 * 单步位置调试模式开关。
 * 0：正常比赛/路线模式，START 按键由 app_start_process() 使用。
 * 1：单步调试模式，START 按键由 debug_step_process() 使用。
 */
#ifndef DEBUG_STEP_MODE_ENABLE
#define DEBUG_STEP_MODE_ENABLE 1U
#endif

/*
 * 可选的调试对象编号。
 * 填入 debug_step_motor_select 的数字不是 CAN ID，而是本调试模块的选择号。
 */
typedef enum
{
    DEBUG_STEP_CHASSIS_X = 1U,       /* 1：Motor1 + Motor2，底盘 X 双电机同步 */
    DEBUG_STEP_CLAW_Y = 2U,          /* 2：Motor3，夹爪横梁 Y */
    DEBUG_STEP_UPPER_HOPPER_Y = 3U,  /* 3：Motor5，上料斗 Y */
    DEBUG_STEP_LOWER_HOPPER_Y = 4U,  /* 4：Motor6，下料斗 Y */
    DEBUG_STEP_CLAW_Z = 5U           /* 5：Motor4，夹爪升降 Z */
} debug_step_motor_e;

/*
 * 以下四个变量是单步调试的主要配置入口，可直接在此处或调试器中修改。
 *
 * 编号对应：
 *   1 = 底盘 X（Motor1、Motor2 同步）
 *   2 = 夹爪横梁 Y（Motor3）
 *   3 = 上料斗 Y（Motor5）
 *   4 = 下料斗 Y（Motor6）
 *   5 = 夹爪升降 Z（Motor4）
 *
 * 例：选择升降 Z 轴并让单击走 1、双击走 0.1：
 *   debug_step_motor_select = 5U;
 *   debug_step_coarse_distance = 1.0f;
 *   debug_step_fine_distance = 0.1f;
 *   debug_step_direction = 1;
 *
 * 更改 debug_step_motor_select 后，模块会自动停止旧轴，并以新轴当前位置
 * 作为新的步进基准，避免切换对象时发生跳动。
 */
extern uint8_t debug_step_motor_select;   /* 选择号 1~5；只允许当前选中机构运动。 */
extern float debug_step_coarse_distance;  /* 单击步进距离：用于快速接近目标；必须大于 0。 */
extern float debug_step_fine_distance;    /* 双击步进距离：用于精细定位；必须大于 0。 */
extern int8_t debug_step_direction;       /* +1：机构正方向；-1：机构反方向；3~6 秒长按可切换。 */

/**
 * @brief 初始化单步调试模块并停止全部位置轴。
 * @note 仅当 DEBUG_STEP_MODE_ENABLE 为 1 时应由 main 调用。
 */
void debug_step_init(void);

/**
 * @brief 处理 START 按键和单步位置命令；应在主循环中高频调用。
 *
 * 单击：松手立即按 coarse_distance 步进；双击：最终按 fine_distance 步进；
 * 按住 3~6 s 后松开：翻转 direction；按住不少于 6 s 后松开：目标回到 0。
 */
void debug_step_process(void);

/**
 * @brief 输出单步调试 VOFA 数据到 USART1。
 * @note 每行有四个通道：实际多圈位置、目标位置、反馈电机速度、下发速度上限。
 */
void debug_step_vofa_process(void);

/**
 * @brief 返回当前被选中机构在控制坐标系下的实际位置。
 * @return 底盘返回双电机方向修正后的平均位置；其余返回对应单轴位置。
 */
float debug_step_get_current_pos(void);

/**
 * @brief 返回当前被选中机构的调试目标位置。
 */
float debug_step_get_target_pos(void);

#endif /* DEBUG_STEP_H */
