#include "headfile.h"

#define CRANE_ROUTE_STEP_COUNT     6U
#define CRANE_ROUTE_STEP_DWELL_MS  3000U /* 每个点位到达后停留 3s，改这里可调整/关闭 */

static crane_slot_pose_t crane_route_slot_pose[CRANE_ROUTE_SLOT_COUNT + 1U] =
{
    {0.0f, 0.0f}, /* unused slot 0 */
    {300.0f, 150.0f}, /* slot 1 */
    {300.0f, 0.0f}, /* slot 2 */
    {300.0f, -150.0f}, /* slot 3 */
    {-400.0f, 150.0f}, /* slot 4 */ 
    {-400.0f, 120.0f}, /* slot 5 */
    {-400.0f, 50.0f}, /* slot 6 */
    {-400.0f, -50.0f}, /* slot 7 */
    {-400.0f,-130.0f}, /* slot 8 */
};

static const uint8_t crane_route_steps[CRANE_ROUTE_STEP_COUNT] =
{
    1U, 4U, 5U,3U, 2U, 7U
};

typedef struct
{
    crane_route_state_e state;
    uint8_t step_index;
    uint8_t current_slot;
    uint32_t dwell_tick;
} crane_route_t;

static crane_route_t crane_route;

void crane_route_init(void)
{
    memset(&crane_route, 0, sizeof(crane_route));

    crane_route.state = CRANE_ROUTE_IDLE;
}

void crane_route_start(void)
{
    crane_route.step_index = 0U;
    crane_route.current_slot = 0U;
    crane_route.dwell_tick = HAL_GetTick();
    crane_route.state = CRANE_ROUTE_LOAD_STEP;
}

void crane_route_stop(void)
{
    crane_route.state = CRANE_ROUTE_IDLE;
    pos_pid_sync_stop();
    beam_ctrl_stop();
}

void crane_route_set_slot_pose(uint8_t slot, float chassis_pos, float beam_pos)
{
    if ((slot == 0U) || (slot > CRANE_ROUTE_SLOT_COUNT))
    {
        return;
    }

    crane_route_slot_pose[slot].chassis_pos = chassis_pos;
    crane_route_slot_pose[slot].beam_pos = beam_pos;
}

crane_route_state_e crane_route_get_state(void)
{
    return crane_route.state;
}

uint8_t crane_route_is_finished(void)
{
    return (crane_route.state == CRANE_ROUTE_FINISHED) ? 1U : 0U;
}

uint8_t crane_route_get_current_slot(void)
{
    return crane_route.current_slot;
}

void crane_route_get_current_target(float *x, float *y)
{
    uint8_t slot = crane_route.current_slot;
    float target_x = 0.0f;
    float target_y = 0.0f;

    if ((slot > 0U) && (slot <= CRANE_ROUTE_SLOT_COUNT))
    {
        target_x = crane_route_slot_pose[slot].chassis_pos;
        target_y = crane_route_slot_pose[slot].beam_pos;
    }

    if (x != NULL)
    {
        *x = target_x;
    }
    if (y != NULL)
    {
        *y = target_y;
    }
}

void crane_route_process(void)
{
    uint8_t slot;

    switch (crane_route.state)
    {
        case CRANE_ROUTE_IDLE:
        case CRANE_ROUTE_FINISHED:
            break;

        case CRANE_ROUTE_LOAD_STEP:
            if (crane_route.step_index >= CRANE_ROUTE_STEP_COUNT)
            {
                crane_route.state = CRANE_ROUTE_FINISHED;
                break;
            }

            slot = crane_route_steps[crane_route.step_index];
            if ((slot == 0U) || (slot > CRANE_ROUTE_SLOT_COUNT))
            {
                crane_route.state = CRANE_ROUTE_FINISHED;
                break;
            }

            crane_route.current_slot = slot;
            pos_pid_sync_set_target(crane_route_slot_pose[slot].chassis_pos);
            beam_ctrl_set_target(crane_route_slot_pose[slot].beam_pos);
            pos_pid_sync_start();
            beam_ctrl_start();
            crane_route.state = CRANE_ROUTE_WAIT_ARRIVE;
            break;

        case CRANE_ROUTE_WAIT_ARRIVE:
            if ((pos_pid_sync_is_busy() == 0U) && (beam_ctrl_is_busy() == 0U))
            {
                crane_route.dwell_tick = HAL_GetTick();
                crane_route.state = CRANE_ROUTE_STEP_DWELL;
            }
            break;

        case CRANE_ROUTE_STEP_DWELL:
            if ((HAL_GetTick() - crane_route.dwell_tick) >= CRANE_ROUTE_STEP_DWELL_MS)
            {
                crane_route.step_index++;
                crane_route.state = CRANE_ROUTE_LOAD_STEP;
            }
            break;

        default:
            crane_route.state = CRANE_ROUTE_IDLE;
            break;
    }
}
