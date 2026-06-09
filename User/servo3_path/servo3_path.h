#ifndef __SERVO3_PATH_H__
#define __SERVO3_PATH_H__

#include "main.h"
#include <stdint.h>

/*
 * servo3 / 延长杆实车精调参数。
 *
 * 角度定义：
 *   0 度   -> 指向 8 号位置
 *   270 度 -> 指向取物区
 *   0 -> 270 为顺时针转动
 */
#define SERVO3_PICK_AREA_ANGLE_DEG       270.0f
#define SERVO3_PLACE_MID_ANGLE_DEG        90.0f
#define SERVO3_SLOT4_ANGLE_DEG           0.0f
#define SERVO3_SLOT8_ANGLE_DEG             180.0f

/*
 * 4/8 号位置附近的 Y 轴安全等待点。
 * Y 到达该点后，servo3 才允许旋转，避免延长杆旋转时打到左右机械支撑。
 */
#define SERVO3_SLOT4_SAFE_Y               -15.0f
#define SERVO3_SLOT8_SAFE_Y                15.0f

/*
 * servo3 没有位置反馈，到位判断只能按时间估算：
 *   等待时间 = 角度变化量 / SERVO3_ROTATE_SPEED_DEG_PER_SEC
 * SERVO3_ANGLE_TOL_DEG 用于过滤很小的重复角度下发，目标角变化小于该值时不重新计时。
 */
#define SERVO3_ROTATE_SPEED_DEG_PER_SEC  270.0f
#define SERVO3_ANGLE_TOL_DEG               1.0f

void servo3_path_init(void);
void servo3_path_release_angle(float angle_deg);
void servo3_path_release_pick_area(void);
void servo3_path_release_for_slot(uint8_t slot);
uint8_t servo3_path_is_arrived(void);
uint8_t servo3_path_is_extreme_slot(uint8_t slot);
float servo3_path_angle_for_slot(uint8_t slot);
float servo3_path_safe_y_for_slot(uint8_t slot);

#endif /* __SERVO3_PATH_H__ */
