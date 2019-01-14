/* RIOT includes */
#include "irq.h"
#include "log.h"
#include "xtimer.h"

/* Project includes */
#include "ctrl.h"
#include "utils.h"
#include "platform.h"

void ctrl_set_pose_reached(ctrl_t* ctrl)
{
    if (ctrl->common.pose_reached) {
        return;
    }

    ctrl->common.pose_reached = TRUE;
    printf("pose reached\n");
}

inline void ctrl_set_pose_intermediate(ctrl_t* ctrl, uint8_t intermediate)
{
    ctrl->common.pose_intermediate = intermediate;
}

inline void ctrl_set_allow_reverse(ctrl_t* ctrl, uint8_t allow)
{
    ctrl->common.allow_reverse = allow;
}

inline uint8_t ctrl_is_pose_reached(ctrl_t* ctrl)
{
    return ctrl->common.pose_reached;
}

inline void ctrl_set_pose_current(ctrl_t* ctrl, pose_t* pose_current)
{
    irq_disable();
    ctrl->common.pose_current = pose_current;
    irq_enable();
}

inline pose_t* ctrl_get_pose_current(ctrl_t* ctrl)
{
    return ctrl->common.pose_current;
}

inline void ctrl_set_pose_to_reach(ctrl_t* ctrl, pose_t* pose_order)
{
    irq_disable();
    if (!pose_equal(&ctrl->common.pose_order, pose_order)) {
        ctrl->common.pose_order = *pose_order;
        ctrl->common.pose_reached = FALSE;

        cons_printf("@robot@,@pose_order@,%u,%.0f,%.0f,%.0f\n",
                    ROBOT_ID,
                    pose_order->x,
                    pose_order->y,
                    pose_order->O);
    }
    irq_enable();
}

inline const pose_t* ctrl_get_pose_to_reach(ctrl_t* ctrl)
{
    return &ctrl->common.pose_order;
}

inline void ctrl_set_speed_current(ctrl_t* ctrl, polar_t* speed_current)
{
    irq_disable();

    ctrl->common.speed_current = speed_current;

    cons_printf("@robot@,@speed_current@,%u,%.0f,%.0f\n",
                ROBOT_ID,
                speed_current->distance,
                speed_current->angle);

    irq_enable();
}

inline const polar_t* ctrl_get_speed_current(ctrl_t* ctrl)
{
    return ctrl->common.speed_current;
}

inline void ctrl_set_speed_order(ctrl_t* ctrl, polar_t* speed_order)
{
    irq_disable();

    ctrl->common.speed_order = speed_order;

    cons_printf("@robot@,@speed_order@,%u,%.0f,%.0f\n",
                ROBOT_ID,
                speed_order->distance,
                speed_order->angle);

    irq_enable();
}

inline polar_t* ctrl_get_speed_order(ctrl_t* ctrl)
{
    return ctrl->common.speed_order;
}

void ctrl_set_mode(ctrl_t* ctrl, ctrl_mode_id_t new_mode)
{
    if (new_mode < CTRL_STATE_NUMOF) {
        for (int i = 0; i < CTRL_STATE_NUMOF; i++) {
            if (new_mode == ctrl->common.modes[i].mode_id) {
                ctrl->common.current_mode = &ctrl->common.modes[i];
                LOG_DEBUG("ctrl: New mode: %s\n", ctrl->common.current_mode->name);
                break;
            }
        }
    }
}

void *task_ctrl_update(void *arg)
{
    /* bot position on the 'table' (absolute position): */
    polar_t motor_command = { 0, 0 };

    ctrl_t *ctrl = (ctrl_t*)arg;
    LOG_DEBUG("ctrl: Controller started\n");

    for (;;) {
        xtimer_ticks32_t loop_start_time = xtimer_now();

        if (ctrl->common.current_mode) {
            if (ctrl->common.current_mode->ctrl_pre_mode_cb) {
                ctrl->common.current_mode->ctrl_pre_mode_cb(ctrl->common.pose_current, ctrl->common.speed_current, &motor_command);
            }

            switch(ctrl->common.current_mode->mode_id) {
                case CTRL_STATE_INGAME:
                    motor_command = ctrl->conf.ctrl_ingame_cb(ctrl);
                    break;
                default:
                    LOG_ERROR("ctrl: Unhandled mode !\n");
            }

        }

        motor_drive(&motor_command);

        xtimer_periodic_wakeup(&loop_start_time, THREAD_PERIOD_INTERVAL);
    }

    return 0;
}
