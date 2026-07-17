#ifndef __SERVO3_PATH_H__
#define __SERVO3_PATH_H__

#include "main.h"
#include <stdint.h>

/*
 * 新机械结构旋转路径参数。
 *
 * 270 度舵机通过外齿轮扩展为约 360 度机构行程。0 度和 270 度在
 * 机械端指向同一置物方向，但代表两条不同的旋转路径，不能取模合并。
 *
 * 取物区统一为 135 度：
 *   1 号取物后：135 -> 270
 *   2 号取物后：135 -> 0
 *   3 号取物后：首版先采用 135 -> 270，实车标定时只改下面一个宏。
 */
#define SERVO3_PICK_AREA_ANGLE_DEG          135.0f
#define SERVO3_PLACE_DEFAULT_ANGLE_DEG        0.0f
#define SERVO3_PICK1_PLACE_ANGLE_DEG         270.0f
#define SERVO3_PICK2_PLACE_ANGLE_DEG           0.0f
#define SERVO3_PICK3_PLACE_ANGLE_DEG         270.0f

/* servo3 无位置反馈，到位只能按角速度和余量估算。 */
#define SERVO3_ROTATE_SPEED_DEG_PER_SEC      270.0f
#define SERVO3_SETTLE_MARGIN_MS              120U
#define SERVO3_ANGLE_TOL_DEG                   1.0f

void servo3_path_init(void);
void servo3_path_release_angle(float angle_deg);
void servo3_path_release_pick_area(void);
void servo3_path_release_place_for_pick(uint8_t pick_slot);
void servo3_path_release_pick_from_place(uint8_t pick_slot);
uint8_t servo3_path_is_arrived(void);
float servo3_path_place_angle_for_pick(uint8_t pick_slot);
float servo3_path_get_current_angle(void);
float servo3_path_get_target_angle(void);

#endif /* __SERVO3_PATH_H__ */
